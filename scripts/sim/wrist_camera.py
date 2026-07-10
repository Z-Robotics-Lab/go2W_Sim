# -*- coding: utf-8 -*-
"""Z-Manip M0：腕部 D435i 相机口径 + 三姿态常量与纯助手函数。

设计意图（把 M0 的所有"数字"集中在此，warehouse_nav.py 只做接线，避免散落硬编码）：
  1) NAMED_POSES  —— STOW/LOOKOUT/CARRY 三姿态 8 关节目标（j1..j6 臂 + j7/j8 爪，rad）。
     全部经真实 URDF FK 推导（go2w_sensored.urdf 链），非目测；见文件尾 FK 自检说明。
  2) 相机内参 —— 848x480，fx/fy/cx/cy 从相机 cfg 常量（focal_length/horizontal_aperture）
     实算，保证 CameraInfo.K 与相机张量口径永远自洽（不写死可能与 cfg 漂移的 fx）。
  3) 深度后处理 —— 32FC1(米) → 近裁(<MIN_Z 打空洞) + 无效清零(NaN/inf) + 可选深度相关轻噪
     → 16UC1(mm)。对齐 realsense2_camera / D435i 语义。
  4) optical frame 外参 —— IsaacLab "world" 约定相机 → ROS REP-105 optical(z前x右y下) 的
     常量旋转四元数，及从 d435_link body 世界位姿构造 base→camera_color_optical_frame 动态 TF
     的助手（相机挂运动臂上，动态 TF 是正法；M1 追踪/抓取强制动态，M0 早做省事）。

本模块**只含常量与纯函数/msg 组装**，不 import isaac、不 import rclpy 之外的重物，
可被 py_compile 与单元测试独立加载（warehouse_nav.py 在 isaac 进程内 import 它）。

铁律遵循：绝不触碰 render_interval / fullScan / pc2_to_livox / go2w_policy shim /
vector_sim.lock；本模块只服务 d435 相机块与新增 named_pose 通道。
"""
from __future__ import annotations

import math
import os

import numpy as np

# ======================================================================== 姿态
# 三姿态 8 关节目标表（j1..j6 臂 + j7/j8 爪；单位 rad）。
# 约定：j1=j4=j5=j6=0（保持在臂矢状面内，相机无侧偏，纯 pitch）；爪非抓取时 OPEN。
# 关节顺序 = PiperGraspController.all_ids 顺序 = [j1,j2,j3,j4,j5,j6, j7,j8]
#   （find_joints("piper_joint[1-6]") + find_joints("piper_joint[78]")）。
# 爪 OPEN=(0.035,-0.035) 与 piper_grasp.py:OPEN_POS 一致（j7∈[0,0.035], j8∈[-0.035,0]）。
#
# FK 实证（go2w_sensored.urdf 全链，见文件尾自检）——视轴=相机 prim +X（world 约定）：
#   STOW    (j2=0.80,j3=-1.20): 视轴 pitch=+27.91°（朝上，收臂 park；导航态）。
#   LOOKOUT (j2=0.43,j3=-0.34): 视轴 pitch=-0.16° （水平前视；SCAN/ALIGN；**默认启动姿态**）。
#   CARRY   (j2=1.00,j3=-0.71): 视轴 pitch=-11.62°（胸前握持略俯视；载物导航态）。
# URDF 关节限位核对：j2∈[0,3.14], j3∈[-2.967,0]（三姿态全在域内）；
#                    j7∈[0,0.035], j8∈[-0.035,0]（OPEN/CLOSED 全在域内）。
_GRIP_OPEN = (0.035, -0.035)

# 姿态目标以**关节名→弧度**表达（不用位置元组），因 IsaacLab find_joints 返回顺序未必是
# 数字名序——按名索引杜绝"目标写错关节"的隐患（关键正确性）。缺省的臂关节视为 0。
# 爪：j7=OPEN 上侧(+)，j8=OPEN 下侧(-)，与 piper_grasp.py:OPEN_POS 一致。
NAMED_POSES: dict[str, dict[str, float]] = {
    "STOW":    {"piper_joint2": 0.80, "piper_joint3": -1.20,
                "piper_joint7": _GRIP_OPEN[0], "piper_joint8": _GRIP_OPEN[1]},
    "LOOKOUT": {"piper_joint2": 0.43, "piper_joint3": -0.34,
                "piper_joint7": _GRIP_OPEN[0], "piper_joint8": _GRIP_OPEN[1]},
    "CARRY":   {"piper_joint2": 1.00, "piper_joint3": -0.71,
                "piper_joint7": _GRIP_OPEN[0], "piper_joint8": _GRIP_OPEN[1]},
}
# 默认启动姿态：LOOKOUT（当前 URDF init j2=0.8/j3=-1.2 视轴朝上 +27.9°，过不了 G-b；
# 拉起后由主循环发一次内部 named_pose=LOOKOUT 切到平视，不动 init_state 落地态）。
DEFAULT_POSE = "LOOKOUT"
POSE_NAMES = tuple(NAMED_POSES.keys())


def pose_target_by_names(name: str, joint_names) -> list[float]:
    """按姿态名 + 关节名顺序，返回该顺序的关节目标列表（缺省关节=0.0）。

    joint_names = 执行器写入用的关节名序（如 PiperGraspController.arm_names+grip_names，
    即 set_joint_position_target(joint_ids=...) 对应的名序）。按名索引，顺序无关，杜绝错位。
    未知姿态名 → ValueError（不静默吞，喂回上层）。
    """
    if name not in NAMED_POSES:
        raise ValueError(
            f"unknown named_pose {name!r}; valid={sorted(NAMED_POSES)}")
    tgt = NAMED_POSES[name]
    return [float(tgt.get(jn, 0.0)) for jn in joint_names]


# ==================================================================== 相机内参
# 分辨率：640→848×480（D435i color 常用口径）。focal_length/horizontal_aperture
# 原样不动（现 cfg 值），fx 由三者实算 → HFOV≈69°（对齐 D435 标称 69°H×42°V）。
CAM_WIDTH = 848
CAM_HEIGHT = 480
# 相机 cfg 光学常量（与 warehouse_nav.py PinholeCameraCfg 同源，单侧真源在此）：
CAM_FOCAL_LENGTH_MM = 1.93
CAM_HORIZONTAL_APERTURE_MM = 2.65
CAM_CLIPPING_NEAR = 0.11   # 渲染裁剪（保持）；近裁 0.28 在后处理做，不改此值。
CAM_CLIPPING_FAR = 20.0

# fx=fy 由针孔模型实算（focal/aperture*W）——保证与相机张量口径永远自洽。
# 实测：1.93/2.65*848 = 617.60 px（设计图曾记 616.92 为四舍五入误差，以实算为准并留痕）。
CAM_FX = CAM_FOCAL_LENGTH_MM / CAM_HORIZONTAL_APERTURE_MM * CAM_WIDTH
CAM_FY = CAM_FX  # 方形像素（vertical aperture 随 H/W 等比缩放 → fy==fx）
CAM_CX = CAM_WIDTH / 2.0    # 424.0（主点取几何中心；sim 无偏心）
CAM_CY = CAM_HEIGHT / 2.0   # 240.0

# 帧 id（realsense2_camera 口径；sim=real 同名，见 z-manip §9.3）。
CAM_OPTICAL_FRAME = "camera_color_optical_frame"
# base→optical TF 的 parent frame 名。全链实际机体系名待 Verify 阶段以 TF 树核对
# （SLAM/arise 可能叫 base_link/body）；经环境变量可改，免动代码+配对重启。默认 "base"。
CAM_TF_PARENT_FRAME = os.environ.get("GO2W_CAM_TF_PARENT", "base")

# ROS 话题名（realsense2_camera 对齐口径）。
TOPIC_COLOR = "/camera/color/image_raw"
TOPIC_COLOR_INFO = "/camera/color/camera_info"
TOPIC_DEPTH_ALIGNED = "/camera/aligned_depth_to_color/image_raw"


def camera_hfov_deg() -> float:
    """水平视场角（度）——供日志/自检核对（应≈69）。"""
    return 2.0 * math.degrees(
        math.atan((CAM_HORIZONTAL_APERTURE_MM / 2.0) / CAM_FOCAL_LENGTH_MM))


def fill_camera_info(msg, stamp_sec: int, stamp_nsec: int):
    """把 848x480 内参填进一个 sensor_msgs/CameraInfo（就地修改并返回）。

    K/P/R/D 全部 realsense2_camera 对齐（无畸变 plumb_bob，sim 无镜头畸变）。
    传入 msg 由调用方创建（避免本模块 import rclpy 消息类型）。
    """
    msg.header.stamp.sec = stamp_sec
    msg.header.stamp.nanosec = stamp_nsec
    msg.header.frame_id = CAM_OPTICAL_FRAME
    msg.width = CAM_WIDTH
    msg.height = CAM_HEIGHT
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [CAM_FX, 0.0,    CAM_CX,
             0.0,    CAM_FY, CAM_CY,
             0.0,    0.0,    1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    # P = [fx 0 cx 0; 0 fy cy 0; 0 0 1 0]（单目 Tx=Ty=0）。
    msg.p = [CAM_FX, 0.0,    CAM_CX, 0.0,
             0.0,    CAM_FY, CAM_CY, 0.0,
             0.0,    0.0,    1.0,    0.0]
    return msg


# ==================================================================== 深度后处理
# 近裁：D435i datasheet min-Z≈0.28m；<MIN_Z 的像素在真机是无效返回（0），非不渲染。
# 故保留渲染几何、只把 <MIN_Z 打空洞（比改 clipping near=0.28 更贴真机语义）。
DEPTH_MIN_Z = 0.28          # m；G-e 门（帧内最小非零深度 ≥0.28m）
DEPTH_MAX_MM = 65535        # uint16 上限
# 深度相关轻噪（逼真、防虚假成功）：sigma = A + B*z（米）。M0 默认**小噪**，且可经环境变量
# 关闭/调节——噪声过大会遮住 6cm GraspBox 致后续 M2 抓取拿不到有效点云（先小后调）。
# GO2W_DEPTH_NOISE=0 关闭（verify 需确定性时用）；默认 1 开启小噪。
DEPTH_NOISE_ENABLE = os.environ.get("GO2W_DEPTH_NOISE", "1") == "1"
DEPTH_NOISE_A = float(os.environ.get("GO2W_DEPTH_NOISE_A", "0.001"))   # 常数项 (m)
DEPTH_NOISE_B = float(os.environ.get("GO2W_DEPTH_NOISE_B", "0.002"))   # 深度比例项 (m/m)


def process_depth(dep_m: np.ndarray, rng: np.random.Generator | None = None
                  ) -> np.ndarray:
    """32FC1(米) 深度图 → 16UC1(mm)：近裁 + 无效清零 + 可选轻噪 + 转 mm。

    步骤（顺序要紧）：
      ① 无效清零：NaN/inf/<=0 → 0（真机无效返回）。
      ② 近裁：<DEPTH_MIN_Z 的有效像素 → 0（打空洞；G-e）。
      ③ 轻噪：仅对剩余有效像素加 N(0, A+B*z)（可关）。加噪后可能把值推到 <MIN_Z 或负——
         再夹一次到 [MIN_Z, ∞)/清零，保证输出仍满足 G-e（最小非零 ≥ MIN_Z）。
      ④ 转 mm：round(z*1000) 夹到 [0, 65535]，uint16。无效仍为 0。

    纯函数（无副作用）；返回新数组（不改入参）。rng 可注入以做确定性测试。
    """
    dep = np.asarray(dep_m, dtype=np.float64)
    out = dep.copy()
    # ① 无效清零
    invalid = ~np.isfinite(out) | (out <= 0.0)
    out[invalid] = 0.0
    # ② 近裁（有效且 <MIN_Z → 打空洞）
    near = (out > 0.0) & (out < DEPTH_MIN_Z)
    out[near] = 0.0
    # ③ 轻噪（仅有效像素）
    if DEPTH_NOISE_ENABLE:
        if rng is None:
            rng = np.random.default_rng()
        valid = out > 0.0
        if np.any(valid):
            sigma = DEPTH_NOISE_A + DEPTH_NOISE_B * out[valid]
            out[valid] = out[valid] + rng.normal(0.0, 1.0, size=sigma.shape) * sigma
            # 加噪后守 G-e：被推到 <MIN_Z 的重新打空洞（真机近裁在噪声之后仍成立）。
            pushed = (out > 0.0) & (out < DEPTH_MIN_Z)
            out[pushed] = 0.0
    # ④ 转 mm（uint16），无效=0
    mm = np.rint(out * 1000.0)
    mm = np.clip(mm, 0.0, float(DEPTH_MAX_MM))
    return mm.astype(np.uint16)


# ==================================================================== optical TF
# IsaacLab "world" 约定相机（视轴=prim +X, +Y=左, +Z=上）→ ROS REP-105 optical
# (z前 x右 y下) 的常量旋转。映射（impl-map 1f，已数值核对 det=+1）：
#   optical_z(前) = +cam_X ; optical_x(右) = -cam_Y ; optical_y(下) = -cam_Z
# R_cam2opt（列 = optical 基在 cam 系坐标）:
#   [[ 0, 0, 1],
#    [-1, 0, 0],
#    [ 0,-1, 0]]
# 四元数(wxyz) = (-0.5, 0.5, -0.5, 0.5)（本模块数值验证：mat2quat 复原 R 一致）。
R_CAM2OPT = np.array([
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)
Q_CAM2OPT_WXYZ = (-0.5, 0.5, -0.5, 0.5)


def _quat_wxyz_to_R(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _R_to_quat_wxyz(m: np.ndarray):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m[2, 1] - m[1, 2]) / S
        y = (m[0, 2] - m[2, 0]) / S
        z = (m[1, 0] - m[0, 1]) / S
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        S = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / S
        x = 0.25 * S
        y = (m[0, 1] + m[1, 0]) / S
        z = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / S
        x = (m[0, 1] + m[1, 0]) / S
        y = 0.25 * S
        z = (m[1, 2] + m[2, 1]) / S
    else:
        S = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / S
        x = (m[0, 2] + m[2, 0]) / S
        y = (m[1, 2] + m[2, 1]) / S
        z = 0.25 * S
    return (w, x, y, z)


def base_to_optical_transform(cam_pos_w, cam_quat_w_wxyz,
                              base_pos_w, base_quat_w_wxyz):
    """动态外参：由 d435_link body 与躯干(base)的世界位姿，算 base→optical 的 (pos, quat_wxyz)。

    相机挂在运动臂上，其世界位姿逐拍变；发动态 TF 才在所有姿态正确（M1 追踪/抓取强制）。
    数学：
      T_base_cam   = T_w_base⁻¹ · T_w_cam                （相机 prim 在 base 系位姿）
      T_base_optic = T_base_cam · R_cam2opt              （右乘 world→optical 常量旋转）
    入参均为 world 系 (pos(3,), quat_wxyz(4,))；返回 (pos(3,) list, quat_wxyz(4,) tuple)。
    纯 numpy，无 isaac 依赖（调用方把 torch 张量 .tolist() 传入）。
    """
    cp = np.asarray(cam_pos_w, dtype=np.float64).reshape(3)
    bp = np.asarray(base_pos_w, dtype=np.float64).reshape(3)
    Rwc = _quat_wxyz_to_R(cam_quat_w_wxyz)
    Rwb = _quat_wxyz_to_R(base_quat_w_wxyz)
    # T_base_cam
    Rbc = Rwb.T @ Rwc
    pbc = Rwb.T @ (cp - bp)
    # 右乘 cam→optical 常量旋转（仅旋转，无平移）
    Rbo = Rbc @ R_CAM2OPT
    q = _R_to_quat_wxyz(Rbo)
    return pbc.tolist(), q


def optical_axis_pitch_deg(base_to_optical_quat_wxyz) -> float:
    """给 base→optical 四元数，返回视轴(optical +Z)相对水平面的仰角(度)。G-b 自检用。

    optical +Z 在 base 系 = R_base_optic 的第 3 列；其 z 分量 = sin(仰角)。
    """
    R = _quat_wxyz_to_R(base_to_optical_quat_wxyz)
    z_of_view = float(R[2, 2])
    return math.degrees(math.asin(max(-1.0, min(1.0, z_of_view))))


# ==================================================================== FK 自检
# 独立可跑的 5 行 FK 自检（impl-map 1f/§9.5 铁则）：打印三姿态视轴 pitch，核对 G-b。
# 不依赖 isaac；纯 URDF 关节表 FK。`python3 wrist_camera.py` 即跑。
def _selftest_fk() -> None:
    def rpy_to_R(r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def axis_R(axis, q):
        a = np.array(axis, float)
        a = a / np.linalg.norm(a)
        c, s = math.cos(q), math.sin(q)
        x, y, z = a
        C = 1 - c
        return np.array([
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])

    def Tm(xyz, rpy, axis=None, q=0.0):
        M = np.eye(4)
        M[:3, :3] = rpy_to_R(*rpy)
        M[:3, 3] = xyz
        if axis is not None:
            J = np.eye(4)
            J[:3, :3] = axis_R(axis, q)
            M = M @ J
        return M

    # go2w_sensored.urdf: base_link→j1→..→j6→(fixed)gripper_base→(d435_joint)d435_link
    def fk(j2, j3, j1=0.0, j4=0.0, j5=0.0, j6=0.0):
        M = np.eye(4)
        M = M @ Tm([0, 0, 0.123], [0, 0, 0], [0, 0, 1], j1)
        M = M @ Tm([0, 0, 0], [1.5708, -0.1359, -3.1416], [0, 0, 1], j2)
        M = M @ Tm([0.28503, 0, 0], [0, 0, -1.7939], [0, 0, 1], j3)
        M = M @ Tm([-0.021984, -0.25075, 0], [1.5708, 0, 0], [0, 0, 1], j4)
        M = M @ Tm([0, 0, 0], [-1.5708, 0, 0], [0, 0, 1], j5)
        M = M @ Tm([8.8259e-05, -0.091, 0], [1.5708, 0, 0], [0, 0, 1], j6)
        M = M @ Tm([0, 0, 0], [0, 0, 0])                     # fixed j6→gripper_base
        M = M @ Tm([-0.045, 0.0, 0.02], [0, -1.5708, 0])     # d435_joint
        return M

    print(f"[wrist_camera] fx=fy={CAM_FX:.4f}px  HFOV={camera_hfov_deg():.3f}deg "
          f"({CAM_WIDTH}x{CAM_HEIGHT})")
    print(f"[wrist_camera] R_cam2opt->quat wxyz={Q_CAM2OPT_WXYZ}  "
          f"reproduce_ok={np.allclose(_quat_wxyz_to_R(Q_CAM2OPT_WXYZ), R_CAM2OPT)}")
    for name in POSE_NAMES:
        p = NAMED_POSES[name]
        M = fk(p.get("piper_joint2", 0.0), p.get("piper_joint3", 0.0))
        view = M[:3, 0]  # world 约定视轴 = prim +X
        pitch = math.degrees(math.asin(view[2] / np.linalg.norm(view)))
        gate = "<=5 (G-b PASS)" if abs(pitch) <= 5 else ">5"
        tag = " [default]" if name == DEFAULT_POSE else ""
        print(f"[wrist_camera] {name:8} view+X pitch={pitch:+6.2f}deg  {gate}{tag}")


if __name__ == "__main__":
    _selftest_fk()
