#!/usr/bin/env python3
"""pathDir 振荡诊断分析（宿主侧离线，纯读 CSV）。

区分三个信号：
  slam_yaw  —— localPlanner 输入（SLAM 抖动，H-A 因）
  pathdir_veh —— /path 首 0.5m 方向（vehicle 系；随 slam_yaw 旋转）
  pathdir_world = pathdir_veh + slam_yaw(插值)  —— 世界系"真实规划方向"
去掉 vehicle 系旋转后，pathdir_world 若仍抖 => planner 内因(H-B)；若稳 => 只是 SLAM 姿态传染(H-A)。
用法：python3 pathdir_analyze.py <csv> [label]
"""
import csv
import math
import statistics as st
import sys

path_csv = sys.argv[1]
label = sys.argv[2] if len(sys.argv) > 2 else path_csv


def fl(x):
    try:
        return float(x)
    except (ValueError, TypeError):
        return float("nan")


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


rows = []
with open(path_csv) as f:
    for r in csv.DictReader(f):
        rows.append(r)

slam = [(fl(r["stamp"]), fl(r["slam_x"]), fl(r["slam_y"]), fl(r["slam_yaw"]))
        for r in rows if r["type"] == "slam" and not math.isnan(fl(r["slam_yaw"]))]
path = [(fl(r["stamp"]), fl(r["pathdir_first05"]), int(r["path_size"]) if r["path_size"] else 0)
        for r in rows if r["type"] == "path"]
cmd = [(fl(r["stamp"]), fl(r["vx"]), fl(r["wz"])) for r in rows if r["type"] == "cmd"]

span = cmd[-1][0] - cmd[0][0] if len(cmd) > 1 else 0
print(f"=== {label} ===")
print(f"sim-span={span:.1f}s  slam={len(slam)} path={len(path)} cmd={len(cmd)}")

# interpolate slam_yaw at each path stamp
slam_st = [s[0] for s in slam]
slam_yaw = [s[3] for s in slam]


def interp_yaw(t):
    if not slam_st:
        return float("nan")
    if t <= slam_st[0]:
        return slam_yaw[0]
    if t >= slam_st[-1]:
        return slam_yaw[-1]
    lo, hi = 0, len(slam_st) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if slam_st[mid] <= t:
            lo = mid
        else:
            hi = mid
    dt = slam_st[hi] - slam_st[lo]
    if dt <= 0:
        return slam_yaw[lo]
    f = (t - slam_st[lo]) / dt
    return slam_yaw[lo] + wrap(slam_yaw[hi] - slam_yaw[lo]) * f


# --- H-A: SLAM yaw jitter ---
dyaw = [abs(wrap(slam_yaw[i] - slam_yaw[i - 1])) for i in range(1, len(slam_yaw))]
print(f"[H-A] SLAM yaw |dyaw|/frame: mean={math.degrees(st.mean(dyaw)):.2f}deg "
      f"max={math.degrees(max(dyaw)):.2f}deg  >1deg:{100*sum(1 for d in dyaw if d>math.radians(1))/len(dyaw):.0f}%")

# --- pathSize=1 fraction (no path found / empty fallback) ---
psz = [p[2] for p in path]
frac_empty = 100 * sum(1 for s in psz if s <= 1) / len(psz) if psz else float("nan")
print(f"[plan] pathSize: min={min(psz)} max={max(psz)} mean={st.mean(psz):.0f}  "
      f"pathSize<=1(no-path/empty)={frac_empty:.0f}%")

# --- pathdir vehicle vs world ---
pv = [(p[0], p[1]) for p in path if not math.isnan(p[1]) and p[2] >= 2]
if len(pv) >= 3:
    veh = [x[1] for x in pv]
    world = [wrap(x[1] + interp_yaw(x[0])) for x in pv]
    dveh = [math.degrees(abs(wrap(veh[i] - veh[i - 1]))) for i in range(1, len(veh))]
    dworld = [math.degrees(abs(wrap(world[i] - world[i - 1]))) for i in range(1, len(world))]
    print(f"[H-B] pathdir_VEH  std={math.degrees(st.pstdev(veh)):.1f}deg  "
          f"consec|jump| mean={st.mean(dveh):.1f} max={max(dveh):.1f}  >60deg:{100*sum(1 for d in dveh if d>60)/len(dveh):.0f}%")
    print(f"[H-B] pathdir_WORLD std={math.degrees(st.pstdev(world)):.1f}deg  "
          f"consec|jump| mean={st.mean(dworld):.1f} max={max(dworld):.1f}  >60deg:{100*sum(1 for d in dworld if d>60)/len(dworld):.0f}%")
    print(f"       -> WORLD 抖动 {'仍大(planner内因 H-B)' if st.mean(dworld)>20 else '小(主因=vehicle系旋转/SLAM H-A)'}")
else:
    print(f"[H-B] too few valid pathdir samples ({len(pv)})")

# --- observed wz ---
wz = [c[2] for c in cmd]
vx = [c[1] for c in cmd]
sat = 100 * sum(1 for w in wz if abs(w) > 1.3) / len(wz)
flip = sum(1 for i in range(1, len(wz)) if wz[i] * wz[i - 1] < 0 and abs(wz[i]) > 0.3 and abs(wz[i - 1]) > 0.3)
print(f"[obs] wz range=[{min(wz):.2f},{max(wz):.2f}] sat(|wz|>1.3)={sat:.0f}% signflips={flip}  "
      f"vx nonzero(>.02)={100*sum(1 for v in vx if abs(v)>0.02)/len(vx):.0f}% mean={st.mean(vx):.3f}")
