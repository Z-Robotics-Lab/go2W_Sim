#!/usr/bin/env python3
"""Side-view projection of /registered_scan (map frame) -> PNG.
READ-ONLY. Accumulates a short window, renders X-Z (side) and Y-Z (side)
scatter so walls appear as VERTICAL structure if the map is level.
A tilted map shows walls leaning; a level map shows them plumb.
"""
import argparse, struct, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

def _read_xyz(msg):
    names={f.name:f for f in msg.fields}
    if not all(k in names for k in ("x","y","z")): return np.empty((0,3),np.float32)
    ox,oy,oz=names["x"].offset,names["y"].offset,names["z"].offset
    step=msg.point_step; n=msg.width*msg.height; buf=bytes(msg.data)
    out=np.empty((n,3),np.float32)
    for i in range(n):
        b=i*step
        out[i,0]=struct.unpack_from("<f",buf,b+ox)[0]
        out[i,1]=struct.unpack_from("<f",buf,b+oy)[0]
        out[i,2]=struct.unpack_from("<f",buf,b+oz)[0]
    return out

class C(Node):
    def __init__(self,topic,secs,out):
        super().__init__("cloud_sideview"); self.acc=[]; self.secs=secs; self.out=out
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2,topic,self.cb,qos)
        self.t0=self.get_clock().now()
    def cb(self,msg):
        x=_read_xyz(msg)
        if x.size: self.acc.append(x)
        if (self.get_clock().now()-self.t0).nanoseconds/1e9>self.secs: self.finish()
    def finish(self):
        if not self.acc: print("NO_POINTS",flush=True); rclpy.shutdown(); return
        p=np.concatenate(self.acc,0)
        # subsample for plotting
        if p.shape[0]>200000:
            idx=np.random.default_rng(0).choice(p.shape[0],200000,replace=False); p=p[idx]
        fig,ax=plt.subplots(1,2,figsize=(16,7))
        ax[0].scatter(p[:,0],p[:,2],s=0.3,c=p[:,2],cmap="viridis"); ax[0].set_xlabel("X (m)"); ax[0].set_ylabel("Z (m)")
        ax[0].set_title("Side view X-Z (walls should be VERTICAL)"); ax[0].set_aspect("equal"); ax[0].grid(True,alpha=0.3)
        ax[0].axhline(0,color="r",lw=0.8,alpha=0.6)
        ax[1].scatter(p[:,1],p[:,2],s=0.3,c=p[:,2],cmap="viridis"); ax[1].set_xlabel("Y (m)"); ax[1].set_ylabel("Z (m)")
        ax[1].set_title("Side view Y-Z (walls should be VERTICAL)"); ax[1].set_aspect("equal"); ax[1].grid(True,alpha=0.3)
        ax[1].axhline(0,color="r",lw=0.8,alpha=0.6)
        fig.suptitle(f"/registered_scan side-view  npts={p.shape[0]}  (red line = Z=0 ground)")
        fig.tight_layout(); fig.savefig(self.out,dpi=90)
        print(f"SAVED {self.out} npts={p.shape[0]} z_range=[{p[:,2].min():.2f},{p[:,2].max():.2f}]",flush=True)
        rclpy.shutdown()

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--topic",default="/registered_scan")
    ap.add_argument("--secs",type=float,default=10.0); ap.add_argument("--out",default="/tmp/sideview.png")
    a=ap.parse_args(); rclpy.init(); n=C(a.topic,a.secs,a.out)
    try: rclpy.spin(n)
    except SystemExit: pass
    if rclpy.ok(): rclpy.shutdown()

if __name__=="__main__": sys.exit(main())
