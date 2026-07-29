#!/usr/bin/env python3
import csv, time, sys, os, signal, gzip, shutil
sys.path.insert(0, "/data/openpilot")
os.environ["PYTHONPATH"] = "/data/openpilot:/data/openpilot/cereal"
from cereal import messaging

CSV_PATH = None

def compress_and_exit(signum, frame):
    global CSV_PATH
    print("\nStopping logger...")
    if CSV_PATH and os.path.exists(CSV_PATH):
        gz_path = CSV_PATH + ".gz"
        with open(CSV_PATH, 'rb') as f_in:
            with gzip.open(gz_path, 'wb') as f_out:
                shutil.copyfileobj(f_in, f_out)
        size_kb = os.path.getsize(gz_path) / 1024
        print("DONE: {} ({:.1f} KB)".format(gz_path, size_kb))
    else:
        print("No CSV file found.")
    sys.exit(0)

signal.signal(signal.SIGTERM, compress_and_exit)
signal.signal(signal.SIGINT, compress_and_exit)

def main():
    global CSV_PATH
    sm = messaging.SubMaster(['carState', 'carControl', 'can', 'radarState'])
    CSV_PATH = "/data/openpilot/v25l_{}.csv".format(time.strftime("%m%d_%H%M%S"))
    print("LOG: " + CSV_PATH)
    sys.stdout.flush()

    with open(CSV_PATH, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t','src','addr','data',
                    'vEgo','aEgo','op_accel','enabled','cruise_en',
                    'brakePed','gasPed','standstill',
                    'lead_d','lead_v','lead_prob'])
        n = 0
        while True:
            sm.update(100)
            if not sm.updated['carState']:
                continue
            cs = sm['carState']
            cc = sm['carControl']
            rs = sm['radarState']
            t = time.time()

            ld = lv = lp = 0.0
            try:
                if rs.leadOne.status:
                    ld = rs.leadOne.dRel
                    lv = rs.leadOne.vRel + cs.vEgo
                    lp = rs.leadOne.prob
            except:
                pass

            # Log ALL 0x271, 0x273, 0x274 from ALL sources
            try:
                for msg in sm['can']:
                    if msg.address in (0x271, 0x273, 0x274):
                        d = bytes(msg.dat)
                        w.writerow([
                            round(t, 3), msg.src, hex(msg.address),
                            d.hex(),
                            round(cs.vEgo, 3), round(cs.aEgo, 3),
                            round(cc.actuators.accel, 3),
                            int(cc.enabled), int(cs.cruiseState.enabled),
                            int(cs.brakePressed), int(cs.gasPressed),
                            int(cs.standstill),
                            round(ld, 2), round(lv, 2), round(lp, 3)
                        ])
            except:
                pass

            n += 1
            if n % 500 == 0:
                f.flush()

if __name__ == "__main__":
    main()
