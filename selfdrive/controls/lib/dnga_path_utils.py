"""DNGA E2E/lane path construction in vehicle-centered coordinates.

modelV2.position is camera-relative. DragonPilot's LanePlanner has already
shifted detected lane lines by camera_offset in parse_model(). Apply the same
camera translation to the raw model path before constructing either candidate.
The selected driving-path bias must be applied exactly once to either path.
"""


def build_dnga_paths(raw_path_xyz, lane_planner, v_ego, t_idxs):
  """Return (corrected E2E path, lane-blended path) without mutating modelV2.

  Both paths are expressed in vehicle-centered coordinates and include the
  configured path offset. Mode selection is intentionally left to the caller.
  """
  model_in_vehicle_frame = raw_path_xyz.copy()
  model_in_vehicle_frame[:, 1] += lane_planner.camera_offset

  # get_d_path mutates its supplied ndarray while blending lane lines and
  # adding the path offset, so give it a distinct copy.
  lane_path = lane_planner.get_d_path(v_ego, t_idxs, model_in_vehicle_frame.copy())

  e2e_path = model_in_vehicle_frame.copy()
  e2e_path[:, 1] += lane_planner.path_offset
  return e2e_path, lane_path
