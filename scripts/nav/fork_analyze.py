#!/usr/bin/env python3
"""叉子实验门判据(宿主侧,纯离线读 CSV)。
门: cmd.x 非零占比 >=70%、GT 实速 >=0.35 m/s(sim)、全程直立、到点能停。
用法: python3 fork_analyze.py <csv> [wx] [wy]
"""
import csv
import math
import sys

csv_path = sys.argv[1]
wx = float(sys.argv[2]) if len(sys.argv) > 2 else None
wy = float(sys.argv[3]) if len(sys.argv) > 3 else None

rows = []
with open(csv_path) as f:
    r = csv.DictReader(f)
    for row in r:
        rows.append(row)

if not rows:
    print("EMPTY CSV")
    sys.exit(2)

n = len(rows)
vx = [float(x["vx"]) for x in rows]
NZ_THRESH = 0.02  # 输出门 maxAccel/100;低于此 lin.x 实际为 0
nonzero = sum(1 for v in vx if abs(v) > NZ_THRESH)
frac_nonzero = nonzero / n
vx_mean = sum(vx) / n
vx_max = max(abs(v) for v in vx)

# GT 实速: 用 GT 位姿的一阶差分(按 gt_stamp = sim 时间)。
gts = []
for x in rows:
    try:
        st = float(x["gt_stamp"]); gx = float(x["gt_x"]); gy = float(x["gt_y"])
        if not (math.isnan(gx) or math.isnan(gy)):
            gts.append((st, gx, gy))
    except (ValueError, KeyError):
        pass
# 去重(GT 5Hz,cmd 50Hz -> 多行同一 GT)
uniq = []
for st, gx, gy in gts:
    if not uniq or st != uniq[-1][0]:
        uniq.append((st, gx, gy))

speeds = []
for i in range(1, len(uniq)):
    dt = uniq[i][0] - uniq[i - 1][0]
    if dt <= 0:
        continue
    d = math.hypot(uniq[i][1] - uniq[i - 1][1], uniq[i][2] - uniq[i - 1][2])
    speeds.append(d / dt)
gt_speed_mean = sum(speeds) / len(speeds) if speeds else float("nan")
gt_speed_max = max(speeds) if speeds else float("nan")
# 净位移(sim 帧,起点->终点)
gt_disp = math.hypot(uniq[-1][1] - uniq[0][1], uniq[-1][2] - uniq[0][2]) if len(uniq) >= 2 else 0.0

# 直立: up_z(本 sim 直立约定 ~ -0.96;摔倒会趋 0/正)。|up_z|>0.7 视直立。
upz = []
for x in rows:
    try:
        u = float(x["up_z"])
        if not math.isnan(u):
            upz.append(u)
    except (ValueError, KeyError):
        pass
upright_frac = sum(1 for u in upz if abs(u) > 0.7) / len(upz) if upz else float("nan")

# 到点: 末位与目标距离
arrive = None
if wx is not None and len(uniq) >= 1:
    arrive = math.hypot(uniq[-1][1] - wx, uniq[-1][2] - wy)
# 末 5 帧位移(停稳判据)
tail_disp = None
if len(uniq) >= 6:
    tail_disp = math.hypot(uniq[-1][1] - uniq[-6][1], uniq[-1][2] - uniq[-6][2])

print(f"rows={n}  gt_frames={len(uniq)}")
print(f"cmd.x nonzero占比 = {frac_nonzero*100:.1f}%   门>=70%  -> "
      f"{'PASS' if frac_nonzero>=0.70 else 'FAIL'}")
print(f"cmd.x mean={vx_mean:.4f}  max={vx_max:.4f}")
print(f"GT 实速 mean = {gt_speed_mean:.4f} m/s(sim)  max={gt_speed_max:.4f}  门>=0.35 -> "
      f"{'PASS' if (not math.isnan(gt_speed_mean) and gt_speed_mean>=0.35) else 'FAIL'}")
print(f"GT 净位移 = {gt_disp:.3f} m (sim)")
print(f"直立占比 = {upright_frac*100:.1f}%  -> {'PASS' if (not math.isnan(upright_frac) and upright_frac>=0.99) else 'FAIL'}")
if arrive is not None:
    print(f"末位->目标距离 = {arrive:.3f} m")
if tail_disp is not None:
    print(f"末5GT帧位移 = {tail_disp:.3f} m (停稳参考)")
