#!/usr/bin/env python3
"""Poll bridge /pose (SLAM) + /gt (ground truth) at ~5Hz for N seconds -> CSV.
READ-ONLY (GET only). For T2 static-stability and T3/T4 motion tests."""
import argparse, csv, json, time, urllib.request

def get(url):
    with urllib.request.urlopen(url, timeout=2.0) as r:
        return json.loads(r.read().decode())

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=120.0)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--base", default="http://127.0.0.1:8042")
    a=ap.parse_args()
    dt=1.0/a.hz; t0=time.time(); n=0
    with open(a.out,"w",newline="") as f:
        w=csv.writer(f)
        w.writerow(["t","px","py","pz","pyaw","gx","gy","gz","gyaw","up_z"])
        while time.time()-t0 < a.secs:
            try:
                p=get(a.base+"/pose"); g=get(a.base+"/gt")
                w.writerow([round(time.time()-t0,3),
                            p.get("x"),p.get("y"),p.get("z"),p.get("yaw"),
                            g.get("x"),g.get("y"),g.get("z"),g.get("yaw"),g.get("up_z")])
                n+=1
                if n % 25 == 0: f.flush(); print(f"  sampled {n} @ {round(time.time()-t0,1)}s", flush=True)
            except Exception as e:
                print(f"  WARN sample fail: {e}", flush=True)
            time.sleep(dt)
    print(f"DONE {n} samples -> {a.out}", flush=True)

if __name__=="__main__": main()
