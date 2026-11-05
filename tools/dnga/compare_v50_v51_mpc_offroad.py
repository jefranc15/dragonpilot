#!/usr/bin/env python3
"""Read-only OFFROAD native-MPC comparison of pinned DNGA V5.0 vs V5.1-dev.

Uses actual LateralPlanner and native Acados. Feeds the same recorded model
frame and causal car/controls state to both versions, independently in Lane
and E2E modes. No CAN, Panda, car controller, or vehicle simulation.

Use --cases baseline --stride 1 for stateful same-input A/B.
--cases sweep --stride 10 is an exploratory, downsampled offset sensitivity
test; stride changes the MPC warm-start history and is not production replay.
"""
import argparse
import bz2
import csv
import os
import struct
import subprocess
import sys
import types
import zipfile
from collections import Counter, deque
from pathlib import Path
import numpy as np

BASE_SHA = 'ce841812a5b17a6bdcac9b4e9deb5235f06978b9'
SOURCE_PATH = 'selfdrive/controls/lib/lateral_planner.py'

def stream_events(zf, entry, log):
  """Stream a bzip2 rlog without decompressing an entire segment in memory."""
  keepalive = deque(maxlen=2048)
  with zf.open(entry) as member, bz2.BZ2File(member, 'rb') as stream:
    while True:
      header = stream.read(8)
      if not header:
        return
      if len(header) != 8:
        raise ValueError('Truncated Capnp header')
      segments = struct.unpack_from('<I', header)[0] + 1
      if not 1 <= segments <= 256:
        raise ValueError('Invalid Capnp segment count')
      header_bytes = (4 + 4*segments + 7) & ~7
      header += stream.read(header_bytes - 8)
      if len(header) != header_bytes:
        raise ValueError('Truncated Capnp segment table')
      words = struct.unpack_from('<' + 'I'*segments, header, 4)
      payload_bytes = 8*sum(words)
      if payload_bytes > 128*1024*1024:
        raise ValueError('Capnp message too large')
      payload = stream.read(payload_bytes)
      if len(payload) != payload_bytes:
        raise ValueError('Truncated Capnp message')
      message = header + payload
      keepalive.append(message)
      for event in log.Event.read_multiple_bytes(message):
        yield event

def segment_sort(path):
  parts = path.split('/')[-2].split('--')
  try:
    return '--'.join(parts[:2]), int(parts[2].split('-')[0])
  except (IndexError, ValueError):
    return path, 0

def valid_model(md):
  try:
    return (len(md.position.x)==33 and len(md.position.y)==33 and
            len(md.position.z)==33 and len(md.position.t)==33 and
            len(md.position.xStd)==33 and len(md.position.yStd)==33 and
            len(md.position.zStd)==33 and len(md.orientation.z)==33 and
            len(md.laneLines)==4 and len(md.laneLines[0].t)==33 and
            len(md.laneLineProbs)>=3)
  except (AttributeError, ValueError, IndexError):
    return False

def classes():
  import inspect
  from selfdrive.controls.lib.lateral_planner import LateralPlanner as V51
  from selfdrive.controls.lib.lateral_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverFast
  assert AcadosOcpSolverFast is not None
  source = subprocess.check_output(['git','show',BASE_SHA+':'+SOURCE_PATH],cwd='/data/openpilot').decode('utf8')
  if 'build_dnga_paths' in source:
    raise RuntimeError('Unexpected V5.0 source')
  if 'build_dnga_paths' not in inspect.getsource(V51):
    raise RuntimeError('Current imported planner is not V5.1-dev')
  module = types.ModuleType('dnga_pinned_v50_lateral_planner')
  module.__file__='/git/'+BASE_SHA+'/'+SOURCE_PATH
  exec(compile(source,module.__file__,'exec'), module.__dict__)
  return module.LateralPlanner, V51

class Config:
  def __init__(self, camera, path, mode):
    self.dpCameraOffset = camera
    self.dpPathOffset = path
    self.dpLaneLessMode = mode
    self.dpLateralMode = 1
    self.dpLcMinMph = 45
    self.dpLcAutoMinMph = 60
    self.dpLcAutoDelay = 3.0

class ReplaySM:
  def __init__(self, cache, config):
    self.cache, self.config = cache, config
  def __getitem__(self, key):
    return self.config if key == 'dragonConf' else self.cache[key]

def make_cases(which, camera, path, V50, V51, CP):
  cases = []
  if which == 'baseline':
    for version, cls in [('v50',V50),('v51',V51)]:
      for mode in [0,1]:
        cases.append((version,cls,mode,camera,path))
  else:
    configs = list(dict.fromkeys([(camera,path-5),(camera,path),(camera,path+5),
                                  (0,path),(6,path)]))
    for c,p in configs:
      if not (-100<=c<=100 and -100<=p<=100):
        raise ValueError('Sweep exceeds allowed offsets')
      for mode in [0,1]:
        cases.append(('v51',V51,mode,c,p))
  return [(v,mode,c,p,cls(CP),Config(c,p,mode)) for v,cls,mode,c,p in cases]

def recorded_cp(zf, segments, log):
  for entry in segments[:3]:
    for n, event in enumerate(stream_events(zf,entry,log)):
      if event.which() == 'carParams':
        return event.carParams
      if n >= 5000:
        break
  return None

def main():
  ap=argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--archive',required=True)
  ap.add_argument('--out',required=True)
  ap.add_argument('--cases',required=True,choices=['baseline','sweep'])
  ap.add_argument('--camera-cm',type=int,default=4)
  ap.add_argument('--path-cm',type=int,default=0)
  ap.add_argument('--stride',type=int,default=1)
  ap.add_argument('--max-model-frames',type=int,default=0)
  ap.add_argument('--route',default='')
  args=ap.parse_args()
  if args.stride < 1 or args.max_model_frames < 0:
    ap.error('Invalid stride or maximum')
  if not (-100<=args.camera_cm<=100 and -100<=args.path_cm<=100):
    ap.error('Offsets must be in -100..100')

  from cereal import car, log
  from common.params import Params
  status=Params()
  if status.get_bool('IsOnroad') or not status.get_bool('IsOffroad'):
    raise RuntimeError('OFFROAD ONLY; do not start vehicle')
  if os.path.realpath(os.getcwd()) != os.path.realpath('/data/openpilot'):
    raise RuntimeError('Run from /data/openpilot')
  V50,V51=classes()
  print('Read-only native-MPC comparison; no actuator commands',flush=True)
  print('V5.0 pinned:',BASE_SHA,flush=True)
  print('Assumed config: camera %+d cm, path %+d cm' % (args.camera_cm,args.path_cm),flush=True)
  print('Mode:',args.cases,'stride',args.stride,flush=True)

  with zipfile.ZipFile(args.archive) as zf:
    segments=sorted((n for n in zf.namelist()
                     if n.endswith('/rlog.bz2') and
                     (not args.route or args.route in n)),key=segment_sort)
    if not segments:
      raise RuntimeError('No rlogs in archive')
    CP=recorded_cp(zf,segments,log)
    provenance='recorded CarParams'
    if CP is None:
      b=status.get('CarParams')
      if not b:
        raise RuntimeError('Missing CarParams')
      CP=car.CarParams.from_bytes(b)
      provenance='fallback device CarParams'
    if CP.carName!='dnga':
      raise RuntimeError('Expected dnga, got '+str(CP.carName))
    print('CarParams:',provenance,'ratio:',CP.steerRatio,flush=True)
    folder=Path(args.out)
    folder.mkdir(parents=True,exist_ok=True)
    out_csv=folder/('matched_'+args.cases+'.csv')
    fields=['route','segment','logMonoTime','frame','vEgo_mps','lProb','rProb',
            'lane_width_m','raw_model_y0','lane_mid_y0','high_confidence',
            'version','mode','camera_cm','path_cm','candidate_y0',
            'mpc_target_y0','mpc_target_y5','curvature0','curvature5',
            'solver_status','finite']
    counts=Counter()
    route_now=None
    cache={}
    with out_csv.open('w',newline='') as fh:
      writer=csv.DictWriter(fh,fieldnames=fields)
      writer.writeheader()
      for idx,entry in enumerate(segments):
        seg=entry.split('/')[-2]
        route='--'.join(seg.split('--')[:2])
        if route!=route_now:
          route_now=route
          cache={}
          planners=make_cases(args.cases,args.camera_cm,args.path_cm,V50,V51,CP)
        for ev in stream_events(zf,entry,log):
          what=ev.which()
          if what in ('carState','controlsState'):
            cache[what]=getattr(ev,what)
          elif what=='modelV2':
            counts['models_seen']+=1
            if args.max_model_frames and counts['models_seen']>args.max_model_frames:
              break
            if (counts['models_seen']-1)%args.stride:
              continue
            if 'carState' not in cache or 'controlsState' not in cache:
              counts['missing_context']+=1
              continue
            md=ev.modelV2
            if not valid_model(md):
              counts['invalid_model']+=1
              continue
            cache['modelV2']=md
            l=float(md.laneLineProbs[1])
            r=float(md.laneLineProbs[2])
            raw=float(md.position.y[0])
            mid=(float(md.laneLines[1].y[0])+float(md.laneLines[2].y[0]))/2
            width=float(md.laneLines[2].y[0])-float(md.laneLines[1].y[0])
            speed=float(cache['carState'].vEgo)
            hc=(speed>=8 and l>=.8 and r>=.8 and
                2.5<=width<=4 and abs(raw)<1 and abs(mid)<1)
            counts['high_confidence']+=int(hc)
            base=dict(route=route,segment=seg,logMonoTime=int(ev.logMonoTime),
                      frame=counts['models_seen'],vEgo_mps=speed,lProb=l,
                      rProb=r,lane_width_m=width,raw_model_y0=raw,
                      lane_mid_y0=mid,high_confidence=int(hc))
            for version,mode,cam,path,planner,conf in planners:
              try:
                planner.update(ReplaySM(cache,conf))
                if version=='v50':
                  candidate=planner.d_path_w_lines_xyz if mode==0 else planner.path_xyz
                else:
                  candidate=planner.d_path_w_lines_xyz if mode==0 else planner.d_path_e2e_xyz
                good=bool(np.isfinite(planner.lat_mpc.x_sol).all() and np.isfinite(planner.y_pts).all())
                solver_status=int(planner.lat_mpc.solution_status)
                writer.writerow(dict(base,version=version,mode=mode,
                                     camera_cm=cam,path_cm=path,
                                     candidate_y0=float(candidate[0,1]),
                                     mpc_target_y0=float(planner.y_pts[0]),
                                     mpc_target_y5=float(planner.y_pts[5]),
                                     curvature0=float(planner.lat_mpc.x_sol[0,3]),
                                     curvature5=float(planner.lat_mpc.x_sol[5,3]),
                                     solver_status=solver_status,finite=int(good)))
                counts['rows']+=1
                counts['bad_solver_rows']+=int(solver_status!=0 or not good)
              except Exception as exc:
                print('ERROR',seg,ev.logMonoTime,version,mode,cam,path,str(exc),file=sys.stderr,flush=True)
                raise
            counts['replayed_frames']+=1
        print('segment %d/%d %s, replayed=%d'%(idx+1,len(segments),seg,counts['replayed_frames']),flush=True)
        if args.max_model_frames and counts['models_seen']>args.max_model_frames:
          break
    cases_count=len(make_cases(args.cases,args.camera_cm,args.path_cm,V50,V51,CP))
    if counts['rows']!=counts['replayed_frames']*cases_count:
      raise RuntimeError('Incomplete result: frame-to-rows mismatch')
    txt=folder/('summary_'+args.cases+'.txt')
    with txt.open('w') as f:
      f.write('OFFROAD exact-same-event native-MPC comparison\n')
      f.write('Cases=%s stride=%d; model outputs replayed; not vehicle response\n'%(args.cases,args.stride))
      f.write('Assumed config camera=%+d cm path=%+d cm; not verified historical values\n'%(args.camera_cm,args.path_cm))
      f.write('V5.0 source=%s V5.1 source=/data/openpilot\n'%BASE_SHA)
      f.write('CarParams=%s steerRatio=%.4f\n'%(provenance,CP.steerRatio))
      f.write('Counts=%r\n'%dict(counts))
      f.write('CSV=%s\n'%out_csv)
    print('DONE',dict(counts),'CSV',out_csv,flush=True)

if __name__=='__main__':
  main()
