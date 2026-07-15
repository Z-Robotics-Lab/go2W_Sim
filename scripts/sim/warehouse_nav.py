#!/usr/bin/env python3
"""Isaac 当机器人：传感器版 Go2W 在 warehouse 里出真实传感器读数、吃 cmd_vel。

对齐 refs/Navigation-Physical-Experiment（CMU 栈）的接口契约：
  出: /lidar/points  sensor_msgs/PointCloud2（10Hz 全帧；SLAM 要 CustomMsg，由独立转换节点补）
  出: /imu/data      sensor_msgs/Imu 200Hz（挂在 Mid-360 出厂内置 IMU 偏移处，标定文件零改动）
  入: /cmd_vel       geometry_msgs/TwistStamped（vx, vyaw -> 差速轮速；4 定轴轮忽略 vy）

用法（GUI）：  bash scripts/run_gui.sh 不适用本脚本 —— 用:
  docker exec -d -u 0 -e DISPLAY=:0 -e ROS_DISTRO=jazzy -e PYTHONUNBUFFERED=1 go2w-isaac \
    bash -c "cd /workspace/go2w/scripts/sim && /isaac-sim/python.sh warehouse_nav.py"
自检模式： --selftest  发 2s 前进指令，校验底盘位移与轮向符号。
"""
import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--env", choices=["warehouse", "flat"], default="warehouse")
parser.add_argument("--selftest", action="store_true", help="cmd_vel 前进自检后退出")
parser.add_argument("--shot_dir", type=str, default=None,
                    help="每 30s 存一张视口截图到该目录（无人值守自查）")
parser.add_argument("--policy", type=str, default=None,
                    help="robot_lab Go2W 速度策略 checkpoint（.pt）——替代手搓差速，"
                         "wheeled_sport 的仿真等价物，支持 vy")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# 渲染性能模式（GO2W_FAST_RENDER=0 关闭）：RTF = 渲染FPS/100（render_interval=1
# 是雷达时钟正确性的硬约束，坑16），所以帧率就是实时率。关掉视觉特效对导航
# 零影响（SLAM 吃雷达点云；D435 图像仅供 RViz 显示，画质降级可接受）。
import os as _os  # noqa: E402
if _os.environ.get("GO2W_FAST_RENDER", "0") == "1":  # 默认关（实测关特效连累 RTX 雷达：0.7Hz+SLAM z 漂移，坑33）
    import carb.settings  # noqa: E402
    _st = carb.settings.get_settings()
    _st.set("/rtx/reflections/enabled", False)
    _st.set("/rtx/indirectDiffuse/enabled", False)
    _st.set("/rtx/ambientOcclusion/enabled", False)
    _st.set("/rtx/raytracing/subsurface/enabled", False)
    _st.set("/rtx/post/dlss/execMode", 0)          # DLSS performance
    _st.set("/rtx-transient/dlssg/enabled", False)
    _st.set("/app/viewport/grid/enabled", False)
    print("[NAV] FAST_RENDER on: rtx effects off, dlss performance", flush=True)

# C1 视口瘦身（GO2W_VIEWPORT_SLIM=1）：只碰 viewport 层，不碰 /rtx/*（离雷达最远）。
# 视口是 GUI 模式最大单项渲染开销：主视口分辨率 640x360（降 4 倍像素）+ 关网格。
# 雷达 render product 是独立 [1,1] 产品，理论上不受视口分辨率影响（闸门验证）。
# 与 FAST_RENDER 正交：本门开时只降视口渲染分辨率，不动光追特效开关（坑33 嫌疑区）。
if _os.environ.get("GO2W_VIEWPORT_SLIM", "0") == "1":
    import carb.settings  # noqa: E402  # 幂等：FAST_RENDER 已 import 则复用
    _vst = carb.settings.get_settings()
    _vst.set("/app/renderer/resolution/width", 640)
    _vst.set("/app/renderer/resolution/height", 360)
    _vst.set("/app/viewport/grid/enabled", False)
    print("[NAV] VIEWPORT_SLIM on: main viewport 640x360, grid off", flush=True)

# C2 D435 降频（GO2W_CAM_SLOW=1）：常量驱动，三处一致（CameraCfg update_period、
# 相机 update 节拍、发布节拍）。10Hz->2Hz（每 50 物理步一帧）。分辨率不动（640x480）。
# D435 图像仅供 RViz 显示；SLAM 只吃雷达；画质/帧率降级可接受（文件头注释已明示）。
# 保持"仅发布帧才 update"模式不变（每步 update 会打乱物理指令管线，实测轮速恒 0，坑）。
_CAM_SLOW = _os.environ.get("GO2W_CAM_SLOW", "0") == "1"
CAM_STRIDE = 50 if _CAM_SLOW else 10          # 物理步/相机帧：10Hz(=10) or 2Hz(=50)
CAM_UPDATE_PERIOD = 0.5 if _CAM_SLOW else 0.1  # CameraCfg update_period 与 stride 自洽
if _CAM_SLOW:
    print("[NAV] CAM_SLOW on: D435 10Hz->2Hz (stride 50, update_period 0.5)", flush=True)

# C3 DLSS 单项归因（GO2W_DLSS_PERF=1）：拆坑33 的捆——只设 DLSS 两项，四个光追特效
# 开关(reflections/indirectDiffuse/AO/subsurface)保持默认开。检验假设：伤雷达的是光追
# 特效开关(雷达是光追传感器共享 rtx 管线)，DLSS 只是视口上采样、与雷达 render product 无关。
# 若雷达闸失败=实锤 DLSS 也在坑33 伤害链，回退并补记 pitfalls.md 坑33（失败即合格产出）。
if _os.environ.get("GO2W_DLSS_PERF", "0") == "1":
    import carb.settings  # noqa: E402  # 幂等复用
    _dst = carb.settings.get_settings()
    _dst.set("/rtx/post/dlss/execMode", 0)          # DLSS performance（降内部渲染分辨率再上采样）
    _dst.set("/rtx-transient/dlssg/enabled", False)  # frame generation off
    print("[NAV] DLSS_PERF on: dlss performance, dlssg off (rtx effects untouched)", flush=True)

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402
from std_msgs.msg import String  # noqa: E402

# Z-Manip M0：腕相机口径 + 三姿态常量（同目录 sibling；纯常量/助手，无 isaac 依赖）。
import wrist_camera as wc  # noqa: E402
# P2.1 IMU raw 路线帧契约（纯助手，无 isaac 依赖；rotate 路线不调用，零开销）。
import sensor_frame_contract  # noqa: E402
# Z-Manip M3 路线B：外部关节命令面的校验/属主/限速纯核（驻车刹车先例——纯逻辑可
# 单测，主循环只留薄接线；无 isaac/torch 依赖）。
import arm_command_gate as acg  # noqa: E402

# tf2_ros 可选（Jazzy 标配；缺失不应拖垮整条导航拉起——软降级 + 响亮告警，不静默吞）。
try:
    from tf2_ros import TransformBroadcaster  # noqa: E402
    _HAS_TF2 = True
except Exception as _tf_e:  # noqa: BLE001
    TransformBroadcaster = None
    _HAS_TF2 = False
    print(f"[NAV][WARN] tf2_ros 不可用，camera_color_optical_frame TF 不发布 "
          f"(G-b 将无 TF 可核): {_tf_e}", flush=True)

import isaaclab.sim as sim_utils  # noqa: E402
import isaaclab.utils.math as math_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.assets import RigidObject, RigidObjectCfg  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sensors import Imu as IsaacImu  # noqa: E402
from isaaclab.sensors import ImuCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402

# A/B body switch (OOD judgement experiment, 2026-07-07): GO2W_WITH_ARM=0 loads the BARE
# trunk (no PiPER arm / NUC / mounts) = the body the shipped policy was TRAINED on; =1 (default)
# loads the sensored/loaded deployment body. Bare URDF is auto-derived by make_bare_urdf.py.
WITH_ARM = _os.environ.get("GO2W_WITH_ARM", "1") == "1"
ROBOT_URDF = ("/workspace/go2w/assets/urdf/go2w_sensored.urdf" if WITH_ARM
              else "/workspace/go2w/assets/urdf/go2w_bare.urdf")
LIDAR_USD = "/workspace/go2w/assets/lidar_configs/Livox_Mid360_approx.usd"
WAREHOUSE_USD = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd"
OFFICE_USD = f"{ISAAC_NUCLEUS_DIR}/Environments/Office/office.usd"

# 场景注册表（宪法：worlds 是 config 不是 code——一套通用驱动，换场景只换字典）。
# GO2W_SCENE 选场景：**默认=office**（CEO 裁定 2026-07-07：以后一律用 office，两个 CEO 实测发现
# 的现场）。GO2W_SCENE=warehouse 可回旧仓库景（保留为可选值，其 usd/spawn/box/cam 四值原样）。
# 每场景字典：usd（None=纯地面 flat 调试面）、spawn（机器人出生 xyz）、box（红箱 xyz）、
# cam（启动视角 {eye,target}）。cam 让"拉起即看到站立的狗"：eye=出生点斜后上方、
# 高度在天花板以下（office 有顶棚，太高会被挡→看到屋顶/城市外景，坑43+CEO 实测），
# target=出生位姿。spawn/box/cam 选点是场景专属常量（无硬编码散落）；office 值由校准轮写死。
SCENES = {
    "warehouse": {
        "usd": WAREHOUSE_USD,
        "spawn": (0.0, 0.0, 0.42),   # 历史值：贴地生成减小落地冲击
        "box": (2.0, -1.0, 0.031),   # 方形回归验证过的空旷走廊 (2,0)-(2,-2)
        # 历史静态视角（eye=(4,4,3) target=(0,0,0.5)）逐字节保留：出生在原点，旧值即对准狗。
        # follow_dz=3.6 保历史跟随高度（warehouse 无低天花板，高俯视不挡镜头）。
        "cam": {"eye": (4.0, 4.0, 3.0), "target": (0.0, 0.0, 0.5), "follow_dz": 3.6},
    },
    "office": {
        "usd": OFFICE_USD,
        # office 出生点：校准到开阔厅（2026-07-07 校准轮实证）。office 脚印
        # X[-4.3,5.3] Y[-9.3,0.1]；原点 (0,0) 在 reception 顶墙=拥挤角落（净空~1.5m），
        # 开阔厅在 -Y。选 (-2.5,-5.0)=/terrain_map 证实开阔（障碍净空 3m+）、手动驱动
        # origin→此点全程直立可达。z=0.42 同 warehouse 贴地策略。回滚原点见 git 历史。
        #
        # 抓取尾链 shakedown 出生点（CEO 打法调整 2026-07-14，env-gated，默认不变）：
        # 设 GO2W_SPAWN_GRASP=1 时出生点 (-2.08,-5.18,0.42) 朝向 +X（identity quat）——
        # 距 soup_can (-1.18,-5.18) 恰 0.90m、bearing 0.0°。
        # 【0.70m→0.90m 后移，CEO 目击撞台修正 2026-07-14】：初版 0.70m 只算了底盘中心到
        # 罐距离，没算 Go2 机身前伸（体长+臂前悬）+落地前冲——CEO 亲眼目击机器人前部怼进
        # 托盘、soup_can 被撞倒。后移到 0.90m 留出机身余量，最后 ~25cm 由伺服 APPROACH 收口
        # （run1 实证伺服能刹在 0.638-0.664m 不碰）。
        # 目的：出生 standoff（不撞）→ 静止 + TABLE_VIEW 锁定 → 伺服 APPROACH 收到 HOLD 窗 →
        # 打穿尾链（handoff→PRE_GRASP→IK→joint_cmd→CLOSE→LIFT→GT 裁判）。
        # 默认导航出生点 (-2.5,-5.0) 完全不动（sibling nav 场景不受扰）。
        "spawn": (
            (-2.08, -5.18, 0.42)
            if _os.environ.get("GO2W_SPAWN_GRASP", "0") != "0"
            else (-2.5, -5.0, 0.42)
        ),
        # 箱子放出生点 +X 前方 1m 开阔地（6cm 低于障碍阈值不被绕开）。
        "box": (-1.5, -5.0, 0.031),
        # 启动视角：出生点斜后上方 3-4m、eye 高度 2.4m（office 天花板下——高了被顶棚挡、
        # 看到屋顶/自带城市外景，CEO 实测坑）。target=出生位姿，拉起即见站立的狗。
        # follow_dz=2.0：跟随镜头也压在天花板下（历史 3.6 会穿顶棚）。
        "cam": {"eye": (-2.5 + 3.2, -5.0 - 2.4, 0.42 + 2.0),
                "target": (-2.5, -5.0, 0.42), "follow_dz": 2.0},
        # Z-Manip M0.5 抓取测试角（CEO 已批 2026-07-10）：托盘垫台 + 3 个物理 YCB 罐/瓶，
        # 为 M1 find(X)/M2 抓取供靶。摆位纪律（AGENTS §sim-safety + M0.5 铁律）：
        # 布景简化（2026-07-13，全管线直测）：锚从厅深处空角 (-1.5,-6.5) 挪到出生点
        # (-2.5,-5.0) 正前方（+X，机体系朝向；identity quat 出生朝向=世界 +X）1.4m 处
        # (-1.1,-5.0)——托盘近缘（朝向机器人的 -X 缘）距出生基座 ≈1.10m，开机即见目标、
        # 留出 <TABLE_VIEW_DIST_M(0.9m) 切换段仍被走到。前方走廊已由既有证据两次实证清空：
        # (a) BOX_POS 同方向 +X 1m 已跑通(M0 回归)；(b) probe_terrain_map.py FRONT sector
        # (veh +x 0.2-2m,|y|<1m) 实测 cost>0.20 恒 0（var/evidence/terrain_fix/
        # baseline_terrainmap.txt）——1.4m 落在该实证窗内。托盘顶>0.20m 进代价图导航正常
        # 绕，罐高<0.20m 对导航失明（设计意图）故只放停车点侧向、绝不放行进直线上。
        # 每条：name(短语义)/usd/pos(xyz)/physics。physics=True 走 RigidObjectCfg（可抓、
        # 出 GT odom）；physics=False 走静态 UsdFileCfg（垫台，不进 GT）。z 由核验 bbox 算
        # （见 PROPS_OFFICE 上方几何注释）。additive：其它场景无 props 键=[] 向后兼容。
        "props": None,  # 占位；真值在 PROPS_OFFICE 定义后回填（见下）——避免前向引用
    },
}
SCENE_NAME = _os.environ.get("GO2W_SCENE", "office")  # CEO 裁定：默认 office；warehouse 可选回退
if SCENE_NAME not in SCENES:
    raise SystemExit(
        f"[NAV] 未知 GO2W_SCENE={SCENE_NAME!r}；可选：{sorted(SCENES)}")
SCENE = SCENES[SCENE_NAME]
SCENE_USD = SCENE["usd"]
SCENE_SPAWN = SCENE["spawn"]
SCENE_BOX = SCENE["box"]
SCENE_CAM = SCENE["cam"]

# Go2W 轮几何（left_wheel.dae 实测半径 0.086m；轮距待实测校准）
WHEEL_RADIUS = 0.086
TRACK_WIDTH = 0.288  # 自检实测（左右前轮世界系间距）
# 轮-地摩擦（CEO 硬件事实指令 2026-07-07：现实轮摩擦大、不漂移 -> 高值）。
# 覆盖上游 warehouse 隐式默认；配 friction_combine_mode="max" 使场景无关（见 main 绑定处）。
# 训练包络：robot_lab env.yaml 摩擦随机化区间中值附近偏上；向上偏离=安全方向，DEBUG 记录。
WHEEL_FRICTION_STATIC = float(_os.environ.get("GO2W_WHEEL_MU_S", "1.8"))
WHEEL_FRICTION_DYNAMIC = float(_os.environ.get("GO2W_WHEEL_MU_D", "1.6"))
# IMU 发布延迟（秒，仿真时）：SLAM 的 1s 重力 init 窗必须只看到站定机体（真机开机已站稳）。
# 沉降（z=0.42 落地+起立）实测 <1s；1.5s 留裕量。L2 门的 "imu sample" 打印在 step200=2.0s 不受影响。
IMU_SETTLE_S = float(_os.environ.get("GO2W_IMU_SETTLE_S", "1.5"))
# Ry(+20°) wxyz 四元数（w=cos10°, y=sin10°）：发布 orientation 用 q_pub=Q_RY20⊗q_imu_raw，
# 使 arise mapping 模式 init（laserMapping.cpp:487 左乘 q_ext.inverse()）还原雷达真实姿态。
Q_RY20 = torch.tensor([[0.9848077530122081, 0.0, 0.17364817766693033, 0.0]])
# IMU 帧路线开关（P2.1，部署 A/B 基建，CEO 已批 2026-07-13）。发布点单点二选一、互斥，
# 绝不双补偿（否则自洽水平图变真 20°/40° 倾斜）。
#   rotate（默认=main 现状，一字不动）：sim 主动把 acc/gyr 精确旋到躯干水平系 + orientation
#     = Ry20⊗q_imu_raw，lidar frameId="sensor"。navstack 侧 imu_laser_rotation_offset=[0,20,0]
#     声明"雷达相对水平 IMU 前倾 20°"；sim 已预水平化 IMU，两处相消得水平图（A线实测 ~1.7°）。
#   raw（移植 codex sensor_frame_contract.py）：acc/gyr 原样透传（留在 20° 斜的传感器系）+
#     orientation=q_imu_raw，lidar frameId="mid360_raw"，靠 ARISE 下游自重力对齐。此路线要求
#     navstack 侧 imu_laser_rotation_offset 变为单位/0（IMU 与雷达同斜，无需二次旋转）——由
#     sync_navstack_files.sh 在同一 env 驱动下生成期决定，运行期冻结（见 docs/stability-gates.md）。
IMU_ROUTE = _os.environ.get("GO2W_IMU_ROUTE", "rotate")
if IMU_ROUTE not in ("rotate", "raw"):
    raise SystemExit(
        f"[NAV] 未知 GO2W_IMU_ROUTE={IMU_ROUTE!r}；可选：rotate|raw")
LIDAR_FRAME_ID = "sensor" if IMU_ROUTE == "rotate" else "mid360_raw"
# Mid-360 出厂标定: imu^T_laser=[-0.011,-0.02329,0.04412] -> IMU 在雷达系的位置取反
IMU_OFFSET_IN_LIDAR = (0.011, 0.02329, -0.04412)

# 可抓物：6cm 红箱，方形回归验证过的空旷地带（(2,0)-(2,-2) 走廊内）。
# 6cm 低于地形分析的障碍阈值——接近时 planner 不会把它当障碍绕开
BOX_POS = SCENE_BOX  # 场景专属（SCENES[GO2W_SCENE]["box"]）；warehouse 默认=(2.0,-1.0,0.031)
BOX_SIZE = 0.06
BOX_CFG = RigidObjectCfg(
    prim_path="/World/GraspBox",
    spawn=sim_utils.CuboidCfg(
        size=(BOX_SIZE, BOX_SIZE, BOX_SIZE),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.12),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.08, 0.08)),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.5, dynamic_friction=1.3, restitution=0.0),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(pos=BOX_POS),
)

# ===== Z-Manip M0.5 抓取测试角：托盘 + 3 物理 YCB（office 专属 props）=====
# 资产全部复用 ISAAC_NUCLEUS_DIR 前缀（office.usd 同源，零新机制）。真名/质量/尺寸=
# 容器内 headless 核验定案（verify_ycb_assets.py → var/evidence/m05/asset_verification.json，
# meters_per_unit=1.0）。010_potted_meat_can 不存在 → 按预案换 004_sugar_box。
_ITEM_DIR = f"{ISAAC_NUCLEUS_DIR}/Props/YCB/Axis_Aligned_Physics"
_PALLET_DIR = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/Props"
# 摆位锚（2026-07-13 布景简化：出生点 (-2.5,-5.0) 正前方 +X 1.4m，见上 SCENES["office"]["props"]
# 注释；托盘近缘距出生基座 ≈1.10m）。
_ANCHOR_X, _ANCHOR_Y = -1.1, -5.0
# 托盘 SM_PaletteA_01：核验 bbox=[1.213,1.003,0.211]，原点 XY 居中 / Z 基座对齐。
# CEO 2026-07-10 reach 裁定：整托盘当"桌"太大（PiPER 626mm，臂基距鼻端~0.3m ⇒ 只有
# 台缘 ~0.15-0.2m 窄带可达；中线物品离任何边 0.5m+ = 物理不可达）。修法：只缩 XY 不缩 Z
# ——scale(0.5,0.4,1) ⇒ 台面 0.607×0.401m（局部系 X 长/Y 短），顶面 0.21112 不变
# （可抓带一毫米不动）。
_PALLET_SCALE = (0.5, 0.4, 1.0)
# 2026-07-13 布景简化：机器人现从 -X 方向直进近（出生朝向=世界 +X，锚在正前方）。
# 原布局近边=托盘局部 -Y 缘（半宽 0.20057，因彼时机器人从 -Y 方向靠近）。为保持
# "窄带 0.15m reach gate"几何不变，托盘整体绕世界 Z 转 +90°，把局部短轴（Y，半宽
# 0.20057）转到世界 X 上——近缘仍是短轴，只是现在面向 -X（机器人来向）。
_PALLET_YAW90 = (0.70711, 0.0, 0.0, 0.70711)  # quat(w,x,y,z) = Rz(+90°)
_PALLET_TOP_Z = 0.21112  # 托盘顶面（核验 bbox z；Z 不缩故不变，不受 yaw 影响）
# YCB Axis_Aligned_Physics 规范系是 Y 轴朝上——三件的"高度"全在 bbox Y 分量
# （soup [.068,.102,.068] / sugar [.093,.176,.045] / mustard [.096,.191,.058]），
# 默认姿态=躺（CEO 眼见实锤；躺姿圆罐还会滚位）。立正 = 绕 X 转 +90°（体 Y→世界 Z）。
_UPRIGHT = (0.70711, 0.70711, 0.0, 0.0)  # quat(w,x,y,z) = Rx(+90°)
# 抓取轴横置姿态（run50，z-manip DEBUG.md）：在 _UPRIGHT(Rx90) 基础上再绕世界 Z 转 +90°
# = Rz90∘Rx90 = quat(0.5,0.5,0.5,0.5)（120° 绕 (1,1,1)：local x→世界 y, local y→世界 z,
# local z→世界 x）。目的：把 red_block 的 0.04m 抓取短轴从"面向机器人来向(世界 X)"转到
# **横向(世界 Y)**，⊥ 相机视线——唯一 IK 可达的俯抓闭合方向(yaw≈90；沿视线 yaw0 全域 IK 死，
# scripts/reach_puredown_sweep 实证)。保持 local +Y→世界 +Z ⇒ m0.5 立正 gate(up_z=1) 照过。
_LATERAL_GRASP = (0.5, 0.5, 0.5, 0.5)  # quat(w,x,y,z) = Rz(+90°)∘Rx(+90°)
# 落差裕量：0.02 弹跳曾致滚位，立姿收紧 0.01（不穿台、少弹跳）；G-p7 立正 gate 兜底。
_DROP_CLR = 0.01
def _rest_z(bbox_z: float) -> float:
    """物体落托盘顶面后的中心 z（顶面 + 半高 + 落差裕量）。"""
    return _PALLET_TOP_Z + bbox_z / 2.0 + _DROP_CLR
# 入选 3 件的立高 = 核验 bbox Y 分量（立正后即竖直尺寸）：soup 首抓 Ø66×H102 /
# sugar 薄盒立起 176 高、38mm 薄边呈爪 / mustard 瓶 191 高、瓶身 58mm 呈爪。
_H_SOUP, _H_SUGAR, _H_MUSTARD = 0.10185, 0.17625, 0.1913
# ===== 爪子友好的窄抓取物：纯红小盒（CEO 令 2026-07-14；run35 各向异化 2026-07-15）=====
# 缘由：PiPER 平行爪最大张开 GRIPPER_WIDTH_MAX=0.070m；soup_can Ø66mm 仅 4mm 余量，
# 平行爪物理上兜不过最粗处 → CEO 换更窄目标。取景同治：纯色小盒比俯视罐盖好检测/好肉眼分辨。
# ── run35 实测根因（DEBUG.md）：0.04m 立方对 YOLOE-seg 太小，纯红面积不足，任意 prompt 都
#    死在 conf≈0.15 < τ_det 0.4（prompt 不敏感，纯视觉 objectness 封顶），链永不 LOCK；而
#    soup_can(6.6cm) 同距 conf 0.63。⇒ 检测受物体像素面积驱动，须放大可视面。约束张力：爪要
#    宽≤~0.042，YOLOE 要大 → 立方无解 ⇒ 各向异化：抓取/来向轴 X 保持 0.04（爪宽+reach 余量+
#    contract 中心不变），仅横向 Y/高 Z 放大补像素。rot 用单位阵（非 _UPRIGHT Rx90，那会把 Y↔Z
#    对调成竖高易倒）；X 面正对机器人来向（短轴面向机器人，不动 reach 窄带 0.12m 余量）。
_RED_BLOCK_W = 0.04                 # 抓取短轴世界边长 [m]（爪跨此轴；≤0.6×GRIPPER_WIDTH_MAX 0.042）
_RED_BLOCK_LAT = 0.08              # 检测放大轴世界边长 [m]（补 YOLOE 像素；≤邻件空档半宽~0.046）
_RED_BLOCK_H = 0.06               # 高 Z 世界边长 [m]（补检测像素；纵横比 1.5 低倾覆）
# rot=_LATERAL_GRASP(Rz90∘Rx90，run50)：把 0.04m 抓取短轴摆到**横向(世界 Y)**、0.08m 检测轴摆到
# **前后来向(世界 X, 面向机器人)**、0.06m 高保持世界 Z。映射 local(x,y,z)=(W,H,LAT)→世界
# (Y,Z,X)=(0.04,0.06,0.08) ⇒ 世界(X,Y,Z)=(0.08 前后, 0.04 横向抓取, 0.06 高)。俯抓闭合沿横向 Y
# =yaw≈90=唯一 IK 可达向（沿来向 X 的 yaw0 全域 IK 死，见 z-manip DEBUG.md run50）。检测面(顶面
# 0.08×0.04 面积不变，绕 Z 旋不减像素)不受影响；仍保 local+Y→世界+Z 过 m0.5 立正 gate(up_z=1)。
_RED_BLOCK_COLOR = (0.9, 0.05, 0.05)  # 纯红 diffuse（PreviewSurfaceCfg；好检测好分辨）
_RED_BLOCK_MASS = 0.05              # [kg] 轻质塑料小盒量级（YCB soup 罐 ~0.35kg，此更轻）
# 一列贴 -X 近边（托盘转向后短轴面向机器人来向）：列 X = 锚-0.08（距转向后 -X 台缘
# 0.20057-0.08=0.12057m ≤ reach gate 0.15m，与旧布局 0.12m margin 同量级）；
# Y = 锚±0.18 间距（爪进入余量 + 单目标隔离；端件距 Y 台缘 0.30331-0.18=0.12331m）。
_ROW_X = _ANCHOR_X - 0.08
# props 单一同源（z-manip tests/contract.py PROPS 引此为准；改摆位要同步测试常量）。
# 间距沿 Y 展开 0.35m（≥0.15m 门）、均落托盘 Y 跨内留边；X 居锚线 -1.1 前 0.08m。
PROPS_OFFICE = [
    # 垫台：静态托盘（physics=False，不进 GT）。落地 z=0，顶面 _PALLET_TOP_Z；XY 缩放见上，
    # 绕世界 Z 转 +90°（_PALLET_YAW90，短轴面向机器人来向 -X）。
    {"name": "pallet", "usd": f"{_PALLET_DIR}/SM_PaletteA_01.usd",
     "pos": (_ANCHOR_X, _ANCHOR_Y, 0.0), "rot": _PALLET_YAW90,
     "physics": False, "scale": _PALLET_SCALE},
    # 物理物体（physics=True，出 /objects/<name>/odom GT）：全部立正（_UPRIGHT），
    # 一列贴 -X 近边（reach 窄带内，机器人来向），z 按立高算。
    {"name": "soup_can", "usd": f"{_ITEM_DIR}/005_tomato_soup_can.usd",
     "pos": (_ROW_X, _ANCHOR_Y - 0.18, _rest_z(_H_SOUP)), "rot": _UPRIGHT, "physics": True},
    {"name": "sugar_box", "usd": f"{_ITEM_DIR}/004_sugar_box.usd",
     "pos": (_ROW_X, _ANCHOR_Y, _rest_z(_H_SUGAR)), "rot": _UPRIGHT, "physics": True},
    {"name": "mustard", "usd": f"{_ITEM_DIR}/006_mustard_bottle.usd",
     "pos": (_ROW_X, _ANCHOR_Y + 0.18, _rest_z(_H_MUSTARD)), "rot": _UPRIGHT, "physics": True},
    # 爪子友好的窄抓取目标：纯红小立方（CEO 令，抓取目标改此件，soup_can 保留供参照）。
    # 用 spawn 原语（CuboidCfg）而非 usd 路径——立方为程序化基元，spawn 键存在时 spawn 环节
    # 直接用之（见下 spawn loop 的 additive 分支）。摆位：嵌入 soup_can(锚-0.18) 与
    # sugar_box(锚) 之间的空档（Y=锚-0.09），距 GRASP spawn 0.903m/+4.4°（近 soup_can 直进
    # 几何，几乎正前方），reach 窄带内（边距 0.1206m）。距 soup_can 37mm、sugar_box 47mm
    # 不重叠。worst_to_corner=0.4995 < 现最坏 0.55885 ⇒ z-manip KEEPOUT_RADIUS 派生不变。
    # 立方 z=顶面+半边+落差裕量（同 _rest_z 逻辑，实参=边长）。rot=_UPRIGHT 与其它件统一
    # （立方对称，Rx90° 无害）→ 过 G-p7 立正 gate 的同一判据。
    {"name": "red_block",
     "spawn": sim_utils.CuboidCfg(
         # 局部尺寸(W,H,LAT)——经 rot=_UPRIGHT(Rx90) 映射为世界(W,LAT,H)=(0.04,0.08,0.06)。
         size=(_RED_BLOCK_W, _RED_BLOCK_H, _RED_BLOCK_LAT),
         rigid_props=sim_utils.RigidBodyPropertiesCfg(),
         mass_props=sim_utils.MassPropertiesCfg(mass=_RED_BLOCK_MASS),
         collision_props=sim_utils.CollisionPropertiesCfg(),
         visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=_RED_BLOCK_COLOR),
         physics_material=sim_utils.RigidBodyMaterialCfg(
             static_friction=1.5, dynamic_friction=1.3, restitution=0.0)),
     "pos": (_ROW_X, _ANCHOR_Y - 0.09, _rest_z(_RED_BLOCK_H)),
     "rot": _LATERAL_GRASP, "physics": True},
]
SCENES["office"]["props"] = PROPS_OFFICE  # 回填占位（前向引用规避）
# 其它场景无 props ⇒ 取 [] 向后兼容（main 用 SCENE.get("props") or []）。
SCENE_PROPS = SCENE.get("props") or []

GO2W_NAV_CFG = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UrdfFileCfg(
        asset_path=ROBOT_URDF,
        fix_base=False,
        merge_fixed_joints=False,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=1.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=1,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=SCENE_SPAWN,  # 场景专属出生位姿；warehouse 默认=(0,0,0.42) 贴地减冲击
        joint_pos={
            ".*_hip_joint": 0.0, ".*_thigh_joint": 0.8, ".*_calf_joint": -1.5,
            ".*_foot_joint": 0.0,
            "piper_joint2": 0.8, "piper_joint3": -1.2,
            "piper_joint[1456]": 0.0, "piper_joint[78]": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=["(FL|FR|RL|RR)_(hip|thigh|calf)_joint"],
            # 100Hz 物理下 60/2 会站不稳摔倒（实测截图确认）；100/5 稳
            effort_limit_sim=23.5, velocity_limit_sim=30.0, stiffness=100.0, damping=5.0),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*_foot_joint"],
            # 滑移转向要克服四轮横向摩擦：阻尼 2 时转向力矩不足（回归实测直线 OK
            # 转弯全丢），提到 8
            effort_limit_sim=60.0, velocity_limit_sim=30.0, stiffness=0.0, damping=8.0),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["piper_joint[1-6]"],
            # Z-Manip M0 姿态保持：K=100/D=5 太软，j2/j3 扛臂重力矩下 j3 稳态下垂
            # ~0.049rad、j2 欠阻尼振荡 ptp=0.114rad → G-c(关节误差<0.05)/G-b(视轴≤5°)
            # 双双失守（对照夹爪 K=800 稳态误差仅 0.0001rad）。稳态 PD 误差 ∝ τ/K：
            # K 100→400 把 j3 下垂压到 ~0.012rad（4× 余量过 0.05 门）；阻尼比 ∝ D/√K，
            # K 翻 4×(√=2×) 需 D 至少翻倍，取 D=15 对齐夹爪 D/√K=0.707≈临界阻尼，
            # 压掉 j2 振荡。effort 30N·m 对稳态需求(~5N·m)余量充足。不放宽任何 gate 阈值。
            effort_limit_sim=30.0, velocity_limit_sim=5.0, stiffness=400.0, damping=15.0),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["piper_joint[78]"],
            effort_limit_sim=50.0, velocity_limit_sim=1.0, stiffness=800.0, damping=20.0),
    },
)


def setup_lidar_ros2():
    """OmniLidar prim 挂到 mid360_link，经 ROS2 helper 发布 PointCloud2 全帧。"""
    stage = get_current_stage()
    lidar_prim = stage.DefinePrim("/World/Robot/mid360_link/lidar", "OmniLidar")
    lidar_prim.GetReferences().AddReference(LIDAR_USD)
    simulation_app.update()
    rp = rep.create.render_product("/World/Robot/mid360_link/lidar", [1, 1],
                                   name="mid360_rp")
    og.Controller.edit(
        {"graph_path": "/World/ros2_lidar_graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("lidar_pub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("lidar_pub.inputs:renderProductPath", rp.path),
                ("lidar_pub.inputs:topicName", "/lidar/points"),
                # frameId 由 IMU 路线开关决定（P2.1）：rotate="sensor"(main 现状)；
                # raw="mid360_raw"（ARISE 下游自对齐，见 IMU_ROUTE 注释）。
                ("lidar_pub.inputs:frameId", LIDAR_FRAME_ID),
                ("lidar_pub.inputs:type", "point_cloud"),
                ("lidar_pub.inputs:fullScan", False),  # 增量模式：每拍点云=该拍扫过的方位片，
                # 到达即时序（坑34：fullScan 整帧的点序非时序，索引铺 offset_time
                # = 随机时间戳，运动中去畸变毁灭性出错 -> SLAM z 俯冲）
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "lidar_pub.inputs:execIn"),
            ],
        },
    )
    print("[NAV] RTX lidar attached ->", "/lidar/points")


def main():
    # dt=1/100 + render_interval=1：GUI 模式下 Kit 时间线每个物理步前进 rendering_dt，
    # 只有两者相等时雷达时钟才与物理/IMU 时钟一致（曾实测雷达 stamp 跑到 2 倍速）
    sim = SimulationContext(SimulationCfg(dt=1 / 100, render_interval=1, device=args_cli.device))
    # 启动视角=场景专属（SCENE_CAM）：warehouse 保历史 eye=(4,4,3) target=(0,0,0.5)；
    # office 对准开阔厅出生点、eye 在天花板下——拉起即见站立的狗（非屋顶/城市外景，CEO 实测）。
    sim.set_camera_view(eye=SCENE_CAM["eye"], target=SCENE_CAM["target"])

    if args_cli.env == "warehouse":
        # 场景 USD 由 GO2W_SCENE 选（SCENE_USD）；默认 warehouse=full_warehouse.usd 等价。
        # prim 路径沿用 /World/Warehouse 作通用场景容器名（仅 stage 路径，不绑场景语义）。
        print(f"[NAV] scene={SCENE_NAME} usd={SCENE_USD}", flush=True)
        env_cfg = sim_utils.UsdFileCfg(usd_path=SCENE_USD)
        env_cfg.func("/World/Warehouse", env_cfg)
    else:
        ground = sim_utils.GroundPlaneCfg(); ground.func("/World/Ground", ground)
        light = sim_utils.DomeLightCfg(intensity=2000.0); light.func("/World/Light", light)

    if args_cli.policy:
        # 训练态增益（必须与 robot_lab UNITREE_GO2W_CFG 一致，策略才有效）。
        # 载荷态腿增益环境覆盖（P1.1，CEO 已批 2026-07-13）：训练态腿增益 25/0.5 是裸机身
        # 定的，扛不住臂+NUC 载荷（部署实测狗向后倒，audit 定罪）。GO2W_POLICY_LEG_STIFFNESS/
        # DAMPING 允许在 A/B 门实验里把 B 臂调到 4×（100/5）而**不改文件**。默认严格保持 main
        # 现状 25.0/0.5 不翻——过门后才另行提交翻默认（红线纪律：只动增益读取处，diff 最小）。
        _leg_k = float(_os.environ.get("GO2W_POLICY_LEG_STIFFNESS", "25.0"))
        _leg_d = float(_os.environ.get("GO2W_POLICY_LEG_DAMPING", "0.5"))
        GO2W_NAV_CFG.actuators["legs"].stiffness = _leg_k
        GO2W_NAV_CFG.actuators["legs"].damping = _leg_d
        GO2W_NAV_CFG.actuators["legs"].effort_limit_sim = 23.5
        print(f"[NAV] policy leg gains=({_leg_k:.1f},{_leg_d:.1f}) "
              f"(env GO2W_POLICY_LEG_STIFFNESS/DAMPING; default 25.0/0.5)", flush=True)
        # 轮驱动增益（速度模式：stiffness=0）。命名成变量供驻车刹车(P1.2)读作 drive 基准，
        # 咬合时从此 0/0.5 混合到 park Kp/D，释放时再混回——单一真源，杜绝漂移。
        _wheel_drive_stiffness = 0.0
        _wheel_drive_damping = 0.5
        GO2W_NAV_CFG.actuators["wheels"].stiffness = _wheel_drive_stiffness
        GO2W_NAV_CFG.actuators["wheels"].damping = _wheel_drive_damping
        GO2W_NAV_CFG.actuators["wheels"].effort_limit_sim = 23.5
    if not WITH_ARM:
        # 裸机 A/B 对照：bare URDF 无 piper 关节，去掉臂/夹爪执行器与臂 init_state
        # （IsaacLab 对零匹配的 actuator 正则会报 ValueError；对不存在关节的 init_state 同理）。
        GO2W_NAV_CFG.actuators.pop("arm", None)
        GO2W_NAV_CFG.actuators.pop("gripper", None)
        GO2W_NAV_CFG.init_state.joint_pos = {
            k: v for k, v in GO2W_NAV_CFG.init_state.joint_pos.items()
            if not k.startswith("piper_joint")
        }
    robot = Articulation(GO2W_NAV_CFG)
    box = RigidObject(BOX_CFG)
    # Z-Manip M0.5 抓取测试角：静态垫台直落 prim，物理物体建 RigidObject（后续出 GT odom）。
    # SCENE_PROPS 为场景专属（office 有托盘+3 YCB；其它场景=[] 向后兼容）。物理物体收进
    # phys_props（name→RigidObject），供 GT 多路发布循环沿用 box 的组装逻辑（抽 _publish_odom）。
    phys_props = {}
    for _pd in SCENE_PROPS:
        _pname, _ppos = _pd["name"], _pd["pos"]
        _prot = _pd.get("rot", (1.0, 0.0, 0.0, 0.0))   # quat(w,x,y,z)；YCB 立正用
        _pscale = _pd.get("scale")                      # None=原尺寸；托盘缩 XY 用
        # spawn 源二选一（additive）：给 usd 路径走 UsdFileCfg（YCB 自带质量/collider）；
        # 给 spawn 配置对象（如 CuboidCfg）则直接用之（程序化基元，如 red_block 窄抓取物）。
        _pspawn = _pd.get("spawn")
        _pusd = _pd.get("usd")
        if _pspawn is None:
            _pspawn = sim_utils.UsdFileCfg(usd_path=_pusd, scale=_pscale)
        _psrc = _pusd if _pusd is not None else type(_pspawn).__name__
        if _pd["physics"]:
            # 参照 BOX_CFG：spawn 源 = usd(YCB) 或 CuboidCfg 等基元；init_state 落位/立正统一。
            _cfg = RigidObjectCfg(
                prim_path=f"/World/Props/{_pname}",
                spawn=_pspawn,
                init_state=RigidObjectCfg.InitialStateCfg(pos=_ppos, rot=_prot),
            )
            phys_props[_pname] = RigidObject(_cfg)
            print(f"[NAV][M05] physics prop {_pname} @ {_ppos} rot={_prot} <- {_psrc}", flush=True)
        else:
            # 静态垫台：直落 prim（参照场景 USD 落法 env_cfg.func）。translate=落位。
            _pspawn.func(f"/World/Props/{_pname}", _pspawn, translation=_ppos, orientation=_prot)
            print(f"[NAV][M05] static prop {_pname} @ {_ppos} scale={_pscale} <- {_psrc}", flush=True)
    imu = IsaacImu(ImuCfg(
        prim_path="/World/Robot/mid360_link",
        # 注意（2026-07-08 更正）：OffsetCfg 从未传 rot（历史注释描述过"-20°反转装平"设计但
        # 代码缺行=编排者烟枪一）。现设计不靠 OffsetCfg.rot：发布环节做三件套精确变换
        # （acc/gyr 运行时精确旋到躯干系 + orientation=Ry20⊗raw + 沉降延迟），见 IMU 发布块注释。
        # 真机形态依据：Mid-360 内置 IMU 与雷达同斜 20°，真机靠 arise init_pitch 声明初始姿态；
        # mapping 模式该 yaml 键被 local_mode:false 门死，等价通道=IMU orientation（实证）。
        # 雷达相对"水平 IMU"的 20° 俯仰由标定文件 imu_laser_rotation_offset=[0,20,0] 声明。
        offset=ImuCfg.OffsetCfg(pos=IMU_OFFSET_IN_LIDAR),
        update_period=1 / 100,
        gravity_bias=(0.0, 0.0, 0.0),  # 纯运动学加速度；重力在发布时按姿态正确投影
    ))
    # 注意：isaaclab 默认 gravity_bias=(0,0,9.81) 是在传感器本体系直接加常量，
    # 只对水平安装成立；我们的 Mid-360 前倾 20°，必须按姿态投影（否则 SLAM 拿到错误重力方向）
    # D435 RGB+深度（挂手眼 d435_link，X 前向=convention world；69deg HFOV）。
    # Z-Manip M0：分辨率 640→848×480（D435i color 口径）；focal/aperture 原样不动，
    # 仅改 W → fx=fy=617.6px、HFOV≈69°（对齐 D435 标称）。内参源在 wrist_camera.py。
    d435 = Camera(CameraCfg(
        prim_path="/World/Robot/d435_link/d435_cam",
        update_period=CAM_UPDATE_PERIOD, height=wc.CAM_HEIGHT, width=wc.CAM_WIDTH,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=wc.CAM_FOCAL_LENGTH_MM,
            horizontal_aperture=wc.CAM_HORIZONTAL_APERTURE_MM,
            clipping_range=(wc.CAM_CLIPPING_NEAR, wc.CAM_CLIPPING_FAR)),
        offset=CameraCfg.OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0),
                                   convention="world"),
    ))
    # 轮胎高摩擦材质：滑移转向的横摆力矩来自轮-地纵向抓地力。
    # CEO 硬件事实指令(2026-07-07)：现实轮子摩擦很大，不会漂移 -> 轮摩擦设高值。
    # friction_combine_mode="max" 是关键一步：PhysX 合成两材质时取"优先级更高"的模式
    # (max>multiply>min>average)，轮设 max 后无论场景地面材质(office 大理石 μ 低、经默认
    # average/multiply 会吃掉轮的高摩擦)都以轮的高 μ 为准 -> 换景免调、场景无关。
    # 训练包络偏离(env.yaml 摩擦随机化中值附近)照 DEBUG 记录：向上偏离=安全方向
    # (滑移减少让轮式运动学更接近训练的理想意图)，非谎报。
    wheel_mat = sim_utils.RigidBodyMaterialCfg(
        static_friction=WHEEL_FRICTION_STATIC, dynamic_friction=WHEEL_FRICTION_DYNAMIC,
        restitution=0.0, friction_combine_mode="max", restitution_combine_mode="max")
    wheel_mat.func("/World/Materials/wheel_rubber", wheel_mat)
    for foot in ("FL", "FR", "RL", "RR"):
        sim_utils.bind_physics_material(f"/World/Robot/{foot}_foot",
                                        "/World/Materials/wheel_rubber")
    # B线取证（2026-07-08）：回读轮 collider 实际生效的 physics material 绑定 + combine mode。
    # 编排者烟枪二怀疑 URDF 导入的 collider 自带材质覆盖绑定；bind_physics_material 是
    # apply_nested + strongerThanDescendants，理应覆盖——此处打印实值定案（一次性，进 nav_bridge.log）。
    try:
        from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade
        _stage = get_current_stage()
        _mat_prim = _stage.GetPrimAtPath("/World/Materials/wheel_rubber")
        _papi = UsdPhysics.MaterialAPI(_mat_prim)
        _pxapi = PhysxSchema.PhysxMaterialAPI(_mat_prim)
        print(f"[MAT] wheel_rubber staticF={_papi.GetStaticFrictionAttr().Get()} "
              f"dynamicF={_papi.GetDynamicFrictionAttr().Get()} "
              f"combine={_pxapi.GetFrictionCombineModeAttr().Get()}", flush=True)
        for foot in ("FL", "FR", "RL", "RR"):
            _link = _stage.GetPrimAtPath(f"/World/Robot/{foot}_foot")
            for _p in Usd.PrimRange(_link):
                if _p.HasAPI(UsdPhysics.CollisionAPI):
                    _bound = UsdShade.MaterialBindingAPI(_p).ComputeBoundMaterial("physics")[0]
                    _bpath = _bound.GetPath() if _bound else "NONE"
                    print(f"[MAT] {foot} collider={_p.GetPath()} bound_physics_material={_bpath}",
                          flush=True)
    except Exception as _e:  # 取证失败不阻断拉起，但必须留痕（编码规范：不静默吞）
        print(f"[MAT] introspection FAILED: {_e}", flush=True)
    setup_lidar_ros2()
    sim.reset()
    print(f"[NAV] joints({robot.num_joints}) ready")

    # ===== 冻结根治（坑40）：timeline 停摆自愈守卫 + 焊死回调旁路 =====
    # 根因：kit 内 timeline STOP/PAUSE → IsaacLab 把主线程焊死在同步 render 循环
    #   - STOP:  simulation_context._app_control_on_stop_handle_fn  `while not is_playing(): render()`（无 stop-break）
    #   - PAUSE: simulation_context.step()  `while not is_playing(): render()`（旋等）
    #   - end-of-range: replicator orchestrator 把「到端点的 PAUSE」升格成 timeline.stop()
    # 机制层三出口全堵死，且不依赖触发源判定。
    import traceback  # noqa: E402
    import time as _time  # noqa: E402

    # 【1】anti-wedge：把 STOP 焊死回调变 no-op（IsaacLab 自身 reset():513 同款用法）。
    #      必须在 sim.reset() 之后——reset 尾部会把 flag 回置 False。
    assert hasattr(sim, "_disable_app_control_on_stop_handle"), \
        "[NAV][FATAL] IsaacLab 上游改名 _disable_app_control_on_stop_handle —— anti-wedge 静默失效，停手"
    sim._disable_app_control_on_stop_handle = True
    print("[NAV] anti-wedge armed: _disable_app_control_on_stop_handle=True", flush=True)

    # 【2】拆升格器（防御纵深）+ 取证默认值。orchestrator.py:327 的升格条件是
    #   `current>=end AND NOT is_looping()`——实测默认 looping=True，故升格本就不触发；
    #   但外部输入可能 set_looping(False)，故仍把 end_time 推到不可达兜住。
    #   注：set_end_time 写 USD stage，需 commit() 落盘才生效（否则 get 读回旧值）。
    #   即便本项失效，守卫【3】在 step 前对任何 PAUSE/STOP 已兜底，不依赖本项。
    import omni.timeline  # noqa: E402
    _tl = omni.timeline.get_timeline_interface()
    print(f"[NAV] timeline defaults: end_time={_tl.get_end_time()} "
          f"looping={_tl.is_looping()} start={_tl.get_start_time()}", flush=True)
    _tl.set_end_time(1.0e9)  # orchestrator.py 的 current>=end 永假
    _tl.commit()             # 落盘 USD stage 写入
    print(f"[NAV] timeline de-promoted: end_time={_tl.get_end_time()} "
          f"looping={_tl.is_looping()}", flush=True)

    # PiPER 抓取控制器（臂 8 关节目标的唯一属主；README 抓取管线见 sim-plan M5）
    # A/B 判决实验（GO2W_WITH_ARM=0，裸机对照）：裸机 URDF 无臂关节，跳过抓取控制器
    # 与臂目标写入——臂/抓取管线全程 None-guarded，其余导航链一字不动。
    if WITH_ARM:
        from piper_grasp import PiperGraspController
        grasp = PiperGraspController(robot, args_cli.device or "cuda:0")
        arm_ids_t = grasp.all_ids
    else:
        grasp = None
        arm_ids_t = None
        print("[NAV] GO2W_WITH_ARM=0: bare trunk, PiPER grasp controller SKIPPED (A/B control)", flush=True)

    # rclpy: IMU 发布 + cmd_vel 订阅（桥扩展已带 jazzy 内部库）
    rclpy.init()
    node = rclpy.create_node("go2w_isaac_bridge")
    imu_pub = node.create_publisher(Imu, "/imu/data", 50)
    # 地面真值位姿：来自 SIM 而非任何执行者——vector_os_nano verify 谓词的 GT 源
    gt_pub = node.create_publisher(PoseStamped, "/ground_truth/pose", 10)
    gt_msg = PoseStamped()
    # 直立度 GT（摔倒可观测性，2026-07-07）：projected_gravity_b 的 z 分量——机体系
    # 重力方向的 z。站立时 body-z 朝上、重力沿 -body-z → up_z≈-1；翻倒/前塌则明显
    # 偏离 -1（劈叉/侧翻 up_z→0，四脚朝天 up_z→+1）。独立 Float32 话题=加性演进，
    # /ground_truth/pose 的 PoseStamped schema 一字不动（桥与旧订阅者全兼容）。
    up_z_pub = node.create_publisher(Float32, "/ground_truth/up_z", 10)
    up_z_msg = Float32()
    rgb_pub = node.create_publisher(Image, "/camera/image", 5)
    depth_pub = node.create_publisher(Image, "/camera/depth", 5)
    # Z-Manip M0：realsense2_camera 对齐口径（新增，与旧 /camera/image·/camera/depth 并存——
    # 旧话题被 refs CMU 栈 RViz 消费，不可删；新话题供 z-manip / M0 gate）。sim=real 同名同编码。
    color_pub = node.create_publisher(Image, wc.TOPIC_COLOR, 5)
    cam_info_pub = node.create_publisher(CameraInfo, wc.TOPIC_COLOR_INFO, 5)
    depth_aligned_pub = node.create_publisher(Image, wc.TOPIC_DEPTH_ALIGNED, 5)
    # base→camera_color_optical_frame 动态 TF（相机挂运动臂，逐拍位姿变；G-b 数值核对源）。
    tf_broadcaster = TransformBroadcaster(node) if _HAS_TF2 else None
    # d435_link body 索引（发动态 TF 用；prim 名 d435_link）。找不到不阻断，软降级留痕。
    _d435_body_ids, _ = robot.find_bodies("d435_link")
    _d435_body_idx = _d435_body_ids[0] if _d435_body_ids else None
    if _d435_body_idx is None:
        print("[NAV][WARN] d435_link body 未找到，optical TF 不发布", flush=True)
    _depth_rng = np.random.default_rng(0)  # 深度噪声固定种子（可复现；GO2W_DEPTH_NOISE=0 关噪）
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    # 抓取管线话题：箱子 GT、EE GT、臂关节态/目标、抓取指令与状态
    box_pub = node.create_publisher(Odometry, "/objects/box/odom", 5)
    # Z-Manip M0.5 GT 多路：每个物理 prop 一路 /objects/<name>/odom（沿用 box 5Hz 与组装
    # 逻辑，抽 _publish_odom 一处；/objects/box/odom 保持原样不动=既有 M0 测试依赖）。
    prop_pubs = {
        _pname: node.create_publisher(Odometry, f"/objects/{_pname}/odom", 5)
        for _pname in phys_props
    }
    if prop_pubs:
        print(f"[NAV][M05] GT odom publishers: {sorted(prop_pubs)}", flush=True)

    def _publish_odom(pub, obj, child, sec_, nsec_):
        """把一个 RigidObject 的 SIM GT（pose+twist，世界系）发到 /objects/<child>/odom。
        box 与所有 M0.5 物理 prop 共用此一处（sec_/nsec_=同拍 sim_stamp，5Hz 分支调用）。
        obj.update() 必须已由调用方在同拍先行（读 root_pos_w/quat_w/lin_vel_w 前刷新）。"""
        pp = obj.data.root_pos_w[0].tolist()
        qq = obj.data.root_quat_w[0].tolist()
        vv = obj.data.root_lin_vel_w[0].tolist()
        od = Odometry()
        od.header.stamp.sec, od.header.stamp.nanosec = sec_, nsec_
        od.header.frame_id = "world"
        od.child_frame_id = child
        (od.pose.pose.position.x, od.pose.pose.position.y,
         od.pose.pose.position.z) = pp
        (od.pose.pose.orientation.w, od.pose.pose.orientation.x,
         od.pose.pose.orientation.y, od.pose.pose.orientation.z) = qq
        (od.twist.twist.linear.x, od.twist.twist.linear.y,
         od.twist.twist.linear.z) = vv
        pub.publish(od)
    ee_pub = node.create_publisher(PoseStamped, "/piper/ee_pose", 10)
    js_pub = node.create_publisher(JointState, "/piper/state", 10)
    jc_pub = node.create_publisher(JointState, "/piper/cmd", 10)
    gs_pub = node.create_publisher(String, "/piper/grasp_status", 5)
    grasp_req = {"pending": False}

    def on_grasp_cmd(msg: String):
        grasp_req["pending"] = True
        print(f"[GRASP] cmd received: {msg.data!r}", flush=True)

    node.create_subscription(String, "/piper/grasp_cmd", on_grasp_cmd, 5)

    # Z-Manip M0：三姿态切换通道 /piper/named_pose(std_msgs/String ∈ {STOW,LOOKOUT,CARRY})。
    # sim 内部 String 通道（与 /piper/grasp_cmd 同级），不经 HTTP 桥、不加 nav_owner 态。
    # 回调只置目标姿态名（校验合法），主循环按属主协调写臂目标（见抓取块 if/else 排他分支）。
    # 默认目标=LOOKOUT：拉起后主循环把臂切到平视（当前 URDF init 视轴朝上 +27.9° 过不了 G-b）。
    named_pose_req = {"name": wc.DEFAULT_POSE}

    def on_named_pose(msg: String):
        name = msg.data.strip()
        if name not in wc.NAMED_POSES:
            print(f"[POSE][WARN] 未知 named_pose {name!r}；合法={sorted(wc.NAMED_POSES)}，忽略",
                  flush=True)
            return
        named_pose_req["name"] = name
        print(f"[POSE] named_pose -> {name}", flush=True)

    node.create_subscription(String, "/piper/named_pose", on_named_pose, 5)
    # 姿态目标张量缓存（按需构建，避免每拍新建）：名 → (arm_names+grip_names) 名序的 8 关节 tensor。
    # 按 grasp 的关节名序装配（= arm_ids_t 顺序），彻底规避 find_joints 返回序≠数字名序的错位。
    _pose_cache: dict = {}
    _pose_joint_names = (list(grasp.arm_names) + list(grasp.grip_names)
                         if grasp is not None else None)

    def _pose_tensor(name: str):
        if _pose_joint_names is None:
            return None
        if name not in _pose_cache:
            _pose_cache[name] = torch.tensor(
                wc.pose_target_by_names(name, _pose_joint_names),
                dtype=torch.float32, device=robot.data.joint_pos.device)
        return _pose_cache.get(name)
    cmd = {"vx": 0.0, "wz": 0.0, "t": 0.0}
    # 近段直控通道（Z-Manip M1）：伺服节点排他发 /manip/cmd_vel(Twist)；消费端仲裁——
    # manip 新鲜(<0.5 sim-s)则优先、否则回落 pathFollower 的 /cmd_vel。单一逻辑生产者由
    # 消费端保证，绝不双写 /cmd_vel 原话题。t 用本地 sim 时钟在收到帧时打戳（不读 header，
    # 与 /cmd_vel 新鲜度判据同源、同系）。
    manip_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0, "t": -1.0}
    manip_src = {"active": False, "last_log_t": -1e9}  # 仲裁源迟滞日志（限频）
    sim_t = {"now": 0.0}  # 全链路用仿真时钟（墙钟慢于实时会让 SLAM 数据破碎）

    # 【4】触发取证监听（坑40）：对 PLAY/PAUSE/STOP 打印带栈痕迹的事件——闭环触发源。
    #   进程内调用→栈给出调用者；栈只到事件泵→证明外部输入/UI 注入。放在 sim_t 定义后。
    _TL_NAMES = {0: "PLAY", 1: "PAUSE", 2: "STOP"}
    def _on_timeline_event(e):
        name = _TL_NAMES.get(e.type)
        if name is None:
            return  # 只关心 PLAY/PAUSE/STOP，忽略 tick/time-changed 洪流
        print(f"[NAV][TIMELINE] {name} at sim_t={sim_t['now']:.2f} wall={_time.time():.3f}",
              flush=True)
        if e.type in (1, 2):  # PAUSE/STOP 附栈痕迹
            traceback.print_stack()
    _tl_event_sub = _tl.get_timeline_event_stream().create_subscription_to_pop(  # noqa: F841
        _on_timeline_event)  # 强引用保活（勿 GC，否则订阅静默失效）

    def sim_stamp():
        t = sim_t["now"]
        sec = int(t)
        return sec, int((t - sec) * 1e9)

    def on_cmd(msg: TwistStamped):
        cmd["vx"] = msg.twist.linear.x
        vy_cmd["v"] = msg.twist.linear.y
        cmd["wz"] = msg.twist.angular.z
        cmd["t"] = sim_t["now"]

    node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, 10)

    def on_manip_cmd(msg: Twist):
        # 近段直控（M1 伺服节点）：Twist（无 header）——收到即以本地 sim 时钟打戳。
        manip_cmd["vx"] = msg.linear.x
        manip_cmd["vy"] = msg.linear.y
        manip_cmd["wz"] = msg.angular.z
        manip_cmd["t"] = sim_t["now"]

    node.create_subscription(Twist, "/manip/cmd_vel", on_manip_cmd, 10)

    # Z-Manip M3 路线B（[RULING] CEO 2026-07-14）：外部关节命令面 /piper/joint_cmd
    # (sensor_msgs/JointState，8 关节：j1..j6 rad + j7/j8 m)。校验/属主/限速纯逻辑在
    # arm_command_gate.py；回调只做 校验+缓存（名按 _pose_joint_names 重排、限位/NaN
    # 拒绝、拒绝必打日志——响亮失败绝不半应用），真正写入在主循环单一属主排他块
    # （优先级：内置 grasp running > external 新鲜(<GO2W_ARM_EXT_FRESH_S=0.5 sim-s) >
    # named_pose——与 /manip/cmd_vel 仲裁同构，发布方须 >2Hz 续持，静默即回落）。
    # 裸机 A/B（grasp None，无臂关节）整面跳过。
    ext_arm = {"q": None, "t": -1.0}
    if grasp is not None:
        _gate_params = acg.gate_params_from_environ(_os.environ)
        _ext_lim = robot.data.joint_pos_limits[0]
        _ext_lims_lo = _ext_lim[arm_ids_t, 0].tolist()
        _ext_lims_hi = _ext_lim[arm_ids_t, 1].tolist()

        def on_joint_cmd(msg: JointState):
            q, why = acg.validate_joint_command(
                list(msg.name), list(msg.position), _pose_joint_names,
                _ext_lims_lo, _ext_lims_hi)
            if q is None:
                print(f"[ARM][WARN] joint_cmd rejected: {why}", flush=True)
                return
            ext_arm["q"] = q
            ext_arm["t"] = sim_t["now"]

        node.create_subscription(JointState, "/piper/joint_cmd", on_joint_cmd, 10)
        print(f"[ARM] /piper/joint_cmd face up: fresh={_gate_params.fresh_sim_s}s "
              f"arm_vel={_gate_params.arm_vel_rad_s}rad/s "
              f"grip_vel={_gate_params.grip_vel_m_s}m/s", flush=True)

    # 运维复位通道（仿真专属，真机无此语义——真机没有"传送回出生点"）：桥 POST /reset
    # → /sim/reset(Bool true) → 主循环把 root 状态写回出生位姿 + 清零所有速度。用于摔倒/
    # 失稳后无需成对重启即可复位机器人再验证。回调只置 flag，真正写状态在主循环做
    # （与物理步同线程，避免回调里改物理态的竞态）。
    reset_req = {"pending": False}

    def on_reset(msg: Bool):
        if msg.data:
            reset_req["pending"] = True
            print("[NAV][RESET] request received -> will teleport to birth pose", flush=True)

    node.create_subscription(Bool, "/sim/reset", on_reset, 5)
    # 出生态锁存（reset 目标）：root 13 维=pos3+quat4+linvel3+angvel3；关节=default 位/零速。
    _birth_root = robot.data.default_root_state.clone()
    _birth_jpos = robot.data.default_joint_pos.clone()
    _birth_jvel = robot.data.default_joint_vel.clone()

    default_pos = robot.data.default_joint_pos.clone()
    wheel_ids, wheel_names = robot.find_joints(".*_foot_joint")
    # 轮向符号：左侧(FL/RL)与右侧(FR/RR)前进同向性由 URDF 轴向决定，先全 +1，自检校准
    signs = {n: 1.0 for n in wheel_names}
    left = [i for i, n in zip(wheel_ids, wheel_names) if n.startswith(("FL", "RL"))]
    right = [i for i, n in zip(wheel_ids, wheel_names) if n.startswith(("FR", "RR"))]

    yaw0 = {"v": 0.0}
    arc_tq_peak = {"v": 0.0}   # selftest 弧线段轮 effort 峰值（转向回归，Finding B）
    arc_up_min = {"v": 0.0}    # selftest 弧线段最不直立 up_z（越负越直立）
    st = {"vx": 0.4, "wz": 0.0}  # 自检指令（独立于 cmd：外部 pathFollower 会持续发
    # cmd_vel=0 把 cmd 字典覆盖——第7轮自检取证的教训）
    if args_cli.selftest:
        start_pos = None

    policy_cache = {}
    policy = None
    if args_cli.policy:
        from go2w_policy import Go2WPolicy
        policy = Go2WPolicy(args_cli.policy, robot, args_cli.device or "cuda:0")
    vy_cmd = {"v": 0.0}
    # M1 断崖取证（GO2W_OBS_DUMP 存在才启用；未设=零开销、零行为改变，纯旁路）。
    # 记 shim 每拍实际吃到的 57 维 obs + 吐出的 16 维 act，供 live vs 评估态对拍。
    from obs_dump import ObsDumper
    obs_dumper = ObsDumper.maybe("live", lambda: sim_t["now"])

    # ===== 零指令死区（孪生保真度补丁，CEO 已批）=====
    # 病理（DEBUG.md E0 实锤）：部署策略在零/小指令区永不真正站定——cmd(0,0,0) 下
    #   仍以 0.075 m/s(sim) 向机头爬行；nav 到点/yaw 门压制 vx/路径间隙的每一刻，
    #   CEO 看到的就是"蠕动、没有明显前进"。真机宇树步态零指令本就站定。
    # 修复：策略喂入路径加死区——命令范数连续 N 拍低于阈值 → 不喂策略，改站姿保持
    #   （腿=default_pos、轮速=0）；命令回升即恢复喂策略。
    # 阈值对齐训练：robot_lab UniformThresholdVelocityCommand(_resample):47 用
    #   `norm(vel_command_b[:2]) > 0.2` 把小的 (vx,vy) 归零（wz 另计）；训练还有
    #   rel_standing_envs=0.02 把 2% 环境全维置零逼策略学"站定"。故策略本会站定，
    #   部署却爬行=分布边缘/观测失配。这里用 CEO 批的 3D 范数 norm(vx,vy,wz)<0.2：
    #   ①比训练 2D 阈值对 (vx,vy) 更严（3D≥2D）；②额外兜住 wz，纯自转指令
    #   (vx=vy=0,wz=1.4，pathFollower 转弯爆发) 范数=1.4>0.2 不触发死区 → 正常导航
    #   的原地转/起步段一律不被吃掉（回归硬约束）。
    # ---- 死区 v2（迟滞 + 柔性过渡）----
    # v1 病理（DEBUG 2026-07-07 A/B 实锤）：单阈值 0.2 + 单向 25 拍 debounce，被
    #   pathFollower 无目标 |wz|=1.396 爆发 chatter 每拍清零 → 25 连拍永远凑不满 →
    #   死区在 idle 下 enter 计数=0（等于没开）；且站姿硬钉 default_pos std=4.6cm 抖振
    #   （非自稳点），切换处满幅突变 → 失稳劈叉（win_b）。
    # v2 三件套：
    #   ① 迟滞双阈值——进入 norm<ENTER_THRESH 连续 ENTER_DEBOUNCE 拍；退出 norm>EXIT_THRESH
    #      连续 EXIT_DEBOUNCE 拍。0.15/0.25 分离带 0.10 → 0.2 附近抖动不再来回切。
    #      注：真实持续自转爆发（norm=1.4，非 chatter）仍会满足退出 → 正常导航放行（G2）。
    #   ② 柔性进入——腿目标从"进入拍的当前腿位"线性混合到 default，BLEND_TICKS 拍到位，
    #      杜绝站姿硬钉的瞬时突跳。
    #   ③ 柔性退出——喂给策略的 cmd 从 0 斜坡到实际值，RAMP_TICKS 拍到位，杜绝满幅突变。
    #   站姿保持期间轮速目标恒 0（不变）。
    STANDSTILL_ENABLE = _os.environ.get("GO2W_STANDSTILL", "1") == "1"
    STANDSTILL_ENTER_THRESH = 0.15  # 进入迟滞下阈（比 v1 0.2 更严，只有真近零才进）
    STANDSTILL_EXIT_THRESH = 0.25   # 退出迟滞上阈（分离带 0.10，抗 0.2 边界抖动）
    STANDSTILL_ENTER_DEBOUNCE = 25  # 连续低命令拍数才进入（0.5s@50Hz）
    STANDSTILL_EXIT_DEBOUNCE = 5    # 连续高命令拍数才退出（0.1s，真自转/起步立即放行）
    STANDSTILL_BLEND_TICKS = 10     # 进入：腿 当前位→default 线性混合拍数（0.2s）
    STANDSTILL_RAMP_TICKS = 10      # 退出：喂策略 cmd 0→实际 斜坡拍数（0.2s）
    standstill_low_count = 0        # 连续低命令计数（策略拍）
    standstill_high_count = 0       # 连续高命令计数（退出迟滞用）
    standstill_active = False       # 当前是否处于站姿保持
    standstill_blend = 0            # 进入混合剩余拍（BLEND..0 递减；0=已到 default）
    standstill_ramp = 0             # 退出斜坡剩余拍（RAMP..0 递减；0=已喂满幅）
    _blend_from = None              # 进入拍锁存的当前腿位（混合起点）
    # 站姿目标：腿=default（前 12 关节），轮速=0（后 4 关节）。用 policy 的 id/序对齐。
    # 驻车刹车（WheelParkingBrake，P1.2 移植自 codex 分支 standstill_control.py，CEO 已批）：
    #   病理——站姿保持只把轮**速度**目标钉 0，但轮执行器是纯速度模式(Kp=0)，无位置锁；
    #   载荷斜面/推力下轮子仍会缓慢自转打滑，基座 XY 漂移（分支实锤零指令蠕动 0.075 m/s）。
    #   修法——咬合时锁存当前轮编码器位置，把轮执行器增益从 drive(Kp=0/D=0.5) 混合到
    #   park(Kp=20/D=8)，并给一个位置目标=锁存编码器角，令轮子物理位置保持不动=真"刹车"。
    #   释放时把位置目标重新对齐到当前实测编码器角（recenter），令残余 Kp 不与速度斜坡对抗。
    #   反馈只用本体轮编码器(robot.data.joint_pos)，绝不读 root/GT——GT 仍是纯输出真值台。
    parking_brake = None
    _parking_gains = None
    if policy is not None:
        _stand_leg_ids = policy.leg_ids
        _stand_leg_tgt = policy.default_pos[:1, :12].clone()
        _stand_wheel_ids = policy.wheel_ids
        _stand_wheel_vel = torch.zeros(1, len(policy.wheel_ids), device=policy.device)
        from standstill_control import ParkingBrakeConfig, WheelParkingBrake
        _parking_config = ParkingBrakeConfig.from_environ(
            _os.environ,
            drive_stiffness=_wheel_drive_stiffness,
            drive_damping=_wheel_drive_damping,
        )
        parking_brake = WheelParkingBrake(_parking_config)
        _parking_gains = (
            _parking_config.drive_stiffness,
            _parking_config.drive_damping,
        )

        def _apply_wheel_gains(stiffness, damping):
            """把驻车混合增益写进 PhysX + IsaacLab 缓存（两处一致，扭矩诊断才不漂）。"""
            robot.write_joint_stiffness_to_sim(
                stiffness, joint_ids=_stand_wheel_ids)
            robot.write_joint_damping_to_sim(
                damping, joint_ids=_stand_wheel_ids)
            wheel_actuator = robot.actuators["wheels"]
            wheel_actuator.stiffness.fill_(float(stiffness))
            wheel_actuator.damping.fill_(float(damping))

        print("[NAV][STANDSTILL] encoder parking brake "
              f"Kp={_parking_config.park_stiffness:.1f} "
              f"D={_parking_config.park_damping:.1f} "
              f"transition={_parking_config.transition_ticks} policy ticks", flush=True)

    step = 0
    # 臂有效目标（属主协调后实际写入的 8 关节目标）——/piper/cmd 报此，杜绝与写入漂移。
    # 缺省 = grasp 初始 q_tgt（idle 收臂态）；每拍在臂写入块按属主更新
    # （named_pose / 内置 grasp / external 三选一，单一属主排他）。
    eff_arm_tgt = grasp.q_tgt if grasp is not None else None
    imu_msg = Imu()
    clock_msg = Clock()
    physics_dt = sim.get_physics_dt()
    # 外部臂命令每拍步长上限（j1..j6 rad/tick + j7/j8 m/tick）——纯核由速率上限×
    # physics_dt 换算；跳变命令只产生有界运动（与内置 DQ_MAX≈1 rad/s 同族）。
    _ext_dq_max = (acg.per_tick_dq_limits(
        physics_dt, len(grasp.arm_ids), len(grasp.grip_ids),
        arm_vel_rad_s=_gate_params.arm_vel_rad_s,
        grip_vel_m_s=_gate_params.grip_vel_m_s)
        if grasp is not None else None)
    # 【3】主循环自愈守卫状态（坑40）：PAUSE→自动 play；STOP→响亮退出；限速防与人工暂停拉锯。
    _resume_ts = []          # 最近一分钟的 auto-resume wall 时间戳
    _RESUME_MAX_PER_MIN = 5  # >5 次/分钟 → 升级 FATAL 退出
    stopped = False
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        sim_t["now"] += physics_dt
        sec, nsec = sim_stamp()
        # 运维复位：把 root 传送回出生位姿 + 清零所有速度（仿真专属通道）。写在 step 前。
        if reset_req["pending"]:
            reset_req["pending"] = False
            robot.write_root_state_to_sim(_birth_root.clone())
            robot.write_joint_state_to_sim(_birth_jpos.clone(), _birth_jvel.clone())
            robot.reset()
            if policy is not None:
                policy.last_action = torch.zeros_like(policy.last_action)
            # 清死区态：复位后从"刚落地站姿"重新起算迟滞，避免带入摔倒前的计数/斜坡。
            standstill_low_count = standstill_high_count = 0
            standstill_active = False
            standstill_blend = standstill_ramp = 0
            _blend_from = None
            # 驻车刹车复位：复位是"停"边界，立刻放掉锁存编码器 + 把轮增益写回 drive 基准，
            # 避免带入摔倒前的 park 增益/锚点（否则复位后轮子被钉在旧角度打架）。
            if parking_brake is not None:
                parking_brake.reset()
                _apply_wheel_gains(
                    parking_brake.config.drive_stiffness,
                    parking_brake.config.drive_damping,
                )
                _parking_gains = (
                    parking_brake.config.drive_stiffness,
                    parking_brake.config.drive_damping,
                )
                policy_cache.pop("wheel_position", None)
            print(f"[NAV][RESET] done at sim_t={sim_t['now']:.2f} "
                  f"root->birth {SCENE_SPAWN} (scene={SCENE_NAME}), vel=0", flush=True)
        # cmd_vel 看门狗：0.5s（仿真时）无新指令则停
        manip_active_nonzero = False  # 驻车 rule 2 用：CMDMUX 选中 MANIP 且命令非零
        if args_cli.selftest:
            vx, wz = st["vx"], st["wz"]
            # selftest 不走 manip 仲裁（vy 沿用既有 /cmd_vel 门控读法，语义不变）。
            vy = vy_cmd["v"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
        else:
            # ---- 近段直控仲裁（M1）：manip 新鲜(<0.5 sim-s)则排他优先，否则回落 pathFollower ----
            # 单一逻辑生产者由此消费端保证；/cmd_vel 原订阅语义不变（仍是回落面）。
            manip_fresh = (sim_t["now"] - manip_cmd["t"]) < 0.5
            if manip_fresh:
                vx, vy, wz = manip_cmd["vx"], manip_cmd["vy"], manip_cmd["wz"]
            else:
                vx = cmd["vx"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
                wz = cmd["wz"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
                vy = vy_cmd["v"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
            # 仲裁源切换打一行日志（限频：源变化 且 距上次日志 ≥1 sim-s）。
            if manip_fresh != manip_src["active"] \
                    and (sim_t["now"] - manip_src["last_log_t"]) >= 1.0:
                print(f"[NAV][CMDMUX] source -> "
                      f"{'MANIP(/manip/cmd_vel)' if manip_fresh else 'PATHFOLLOWER(/cmd_vel)'} "
                      f"at sim_t={sim_t['now']:.2f} "
                      f"(vx={vx:.3f} vy={vy:.3f} wz={wz:.3f})", flush=True)
                manip_src["last_log_t"] = sim_t["now"]
            manip_src["active"] = manip_fresh
            # 驻车 rule 2（CEO 裁定 2026-07-13）：CMDMUX 选中 MANIP 源且命令非零时，精伺服的
            # 微弧命令（~0.1 m/s 量级）可能低于死区 EXIT=0.25 迟滞带——若放任死区/驻车按迟滞
            # 慢慢退出，驻车会跟精伺服打架毁掉抓取管线。MANIP 源的**任何**非零命令 = 无条件
            # "有意运动"：立即释放驻车 + 本拍禁止咬合（绕过 ENTER debounce），无迟滞延迟。
            manip_active_nonzero = manip_fresh and (
                vx != 0.0 or vy != 0.0 or wz != 0.0)

        if policy is not None:
            # vy 已在上方仲裁/门控确定（manip 优先或 pathFollower 回落）。
            if step % 2 == 0:  # 策略 50Hz（sim 100Hz）
                # ---- 死区 v2 迟滞状态机：命令 3D 范数 + 双阈值 + 柔性过渡 ----
                cmd_norm = math.sqrt(vx * vx + vy * vy + wz * wz)
                if manip_active_nonzero:
                    # 驻车 rule 2：MANIP 精伺服有意运动——无条件立即退出+禁咬合（绕迟滞）。
                    standstill_low_count = 0
                    standstill_high_count = STANDSTILL_EXIT_DEBOUNCE
                    if standstill_active:
                        standstill_active = False
                        parking_brake.release()
                        policy.last_action = torch.zeros_like(policy.last_action)
                        standstill_ramp = STANDSTILL_RAMP_TICKS
                        standstill_blend = 0
                        print(f"[NAV][STANDSTILL] MANIP-override release at "
                              f"sim_t={sim_t['now']:.2f} cmd_norm={cmd_norm:.4f} "
                              f"(精伺服有意运动，绕迟滞立即放行)", flush=True)
                elif STANDSTILL_ENABLE:
                    # 进入迟滞：norm<ENTER_THRESH 连续 ENTER_DEBOUNCE 拍。
                    if cmd_norm < STANDSTILL_ENTER_THRESH:
                        standstill_low_count += 1
                        standstill_high_count = 0
                        if standstill_low_count >= STANDSTILL_ENTER_DEBOUNCE \
                                and not standstill_active:
                            standstill_active = True
                            # 咬合驻车：锁存当前轮编码器位置（本体反馈，不读 GT）。
                            parking_brake.engage(
                                robot.data.joint_pos[0, _stand_wheel_ids].tolist())
                            # 柔性进入：锁存当前腿位为混合起点，BLEND 拍内 →default。
                            _blend_from = robot.data.joint_pos[:1, _stand_leg_ids].clone()
                            standstill_blend = STANDSTILL_BLEND_TICKS
                            standstill_ramp = 0
                            print(f"[NAV][STANDSTILL] enter at sim_t={sim_t['now']:.2f} "
                                  f"cmd_norm={cmd_norm:.4f} (blend {STANDSTILL_BLEND_TICKS})",
                                  flush=True)
                    # 退出迟滞：norm>EXIT_THRESH 连续 EXIT_DEBOUNCE 拍（真自转/起步放行）。
                    elif cmd_norm > STANDSTILL_EXIT_THRESH:
                        standstill_high_count += 1
                        standstill_low_count = 0
                        if standstill_high_count >= STANDSTILL_EXIT_DEBOUNCE \
                                and standstill_active:
                            standstill_active = False
                            parking_brake.release()
                            # 柔性退出：last_action 复位（站姿⟺a=0，物理诚实）+ 起 cmd 斜坡。
                            policy.last_action = torch.zeros_like(policy.last_action)
                            standstill_ramp = STANDSTILL_RAMP_TICKS
                            standstill_blend = 0
                            print(f"[NAV][STANDSTILL] exit at sim_t={sim_t['now']:.2f} "
                                  f"cmd_norm={cmd_norm:.4f} (reset+ramp "
                                  f"{STANDSTILL_RAMP_TICKS})", flush=True)
                    else:
                        # 分离带内（0.15~0.25）：既不累进入也不累退出，维持当前态（迟滞核心）。
                        standstill_low_count = 0
                        standstill_high_count = 0
                else:
                    # 死区关闭：确保退出并清态。
                    if standstill_active:
                        standstill_active = False
                        parking_brake.release()
                        policy.last_action = torch.zeros_like(policy.last_action)
                    standstill_low_count = standstill_high_count = 0
                    standstill_blend = standstill_ramp = 0

                # 驻车刹车推进一拍：只用轮编码器。过渡拍混合增益；释放期纯逻辑把位置目标
                # recenter 到当前编码器角，令残余 Kp 不与速度斜坡对抗。增益变化才写 sim。
                parking_command = parking_brake.step(
                    robot.data.joint_pos[0, _stand_wheel_ids].tolist())
                next_parking_gains = (
                    parking_command.stiffness, parking_command.damping)
                if next_parking_gains != _parking_gains:
                    _apply_wheel_gains(
                        parking_command.stiffness, parking_command.damping)
                    _parking_gains = next_parking_gains
                if parking_command.position_target is None:
                    policy_cache.pop("wheel_position", None)
                else:
                    parking_target = torch.tensor(
                        [parking_command.position_target],
                        dtype=robot.data.joint_pos.dtype,
                        device=robot.data.joint_pos.device,
                    )
                    policy_cache["wheel_position"] = (
                        _stand_wheel_ids, parking_target)

                if standstill_active:
                    # 站姿保持：腿目标（柔性进入混合）、轮速目标恒 0（不推进策略）。
                    if standstill_blend > 0 and _blend_from is not None:
                        # 线性混合：alpha 从 (1-1/BLEND) 递减到 0（0=纯 default）。
                        alpha = standstill_blend / STANDSTILL_BLEND_TICKS
                        leg_tgt = (alpha * _blend_from
                                   + (1.0 - alpha) * _stand_leg_tgt)
                        standstill_blend -= 1
                        policy_cache["legs"] = (_stand_leg_ids, leg_tgt)
                    else:
                        policy_cache["legs"] = (_stand_leg_ids, _stand_leg_tgt)
                    policy_cache["wheels"] = (_stand_wheel_ids, _stand_wheel_vel)
                else:
                    # 柔性退出斜坡：喂策略的 cmd 从 0 线性升到实际值，杜绝满幅突变。
                    if standstill_ramp > 0:
                        scale = 1.0 - standstill_ramp / STANDSTILL_RAMP_TICKS
                        standstill_ramp -= 1
                    else:
                        scale = 1.0
                    # 速度斜坡不得快于驻车增益释放：残余 park Kp(blend>0) 仍在钉轮位时，
                    # 把喂进策略的 cmd 幅度压到 (1-blend)，令位置锁与速度命令不对抗。
                    scale = min(scale, 1.0 - parking_command.blend)
                    # M1 取证：obs 在 act() 之前采（此刻 last_action=本拍旧值，逐位对齐 shim）；
                    # act 在 act() 之后补。两段式保证 dump 的 obs = shim 真喂进网络的 obs。
                    _dump_pending = (obs_dumper.begin(
                        policy, vx * scale, vy * scale, wz * scale,
                        extra={"scale": round(scale, 4), "step": step})
                        if obs_dumper else None)
                    leg_ids, leg_tgt, wheel_ids_p, wheel_vel = policy.act(
                        vx * scale, vy * scale, wz * scale)
                    if obs_dumper:
                        obs_dumper.finish(_dump_pending, policy.last_action)
                    policy_cache["legs"] = (leg_ids, leg_tgt)
                    policy_cache["wheels"] = (wheel_ids_p, wheel_vel)
            robot.set_joint_position_target(default_pos)  # 臂/夹爪保持
            if "legs" in policy_cache:
                robot.set_joint_position_target(policy_cache["legs"][1],
                                                joint_ids=policy_cache["legs"][0])
                robot.set_joint_velocity_target(policy_cache["wheels"][1],
                                                joint_ids=policy_cache["wheels"][0])
            # 驻车咬合/释放期：给轮**位置**目标=锁存/recenter 编码器角（park Kp>0 才生效）。
            # 写在轮速目标之后：速度模式(Kp=0)下位置目标无效应，混合到 park Kp 后位置锁才咬。
            if "wheel_position" in policy_cache:
                robot.set_joint_position_target(
                    policy_cache["wheel_position"][1],
                    joint_ids=policy_cache["wheel_position"][0])
        else:
            robot.set_joint_position_target(default_pos)
            vel_t = robot.data.default_joint_vel.clone()
            wl = (vx - wz * TRACK_WIDTH / 2) / WHEEL_RADIUS
            wr = (vx + wz * TRACK_WIDTH / 2) / WHEEL_RADIUS
            for i in left:
                vel_t[:, i] = wl
            for i in right:
                vel_t[:, i] = wr
            robot.set_joint_velocity_target(vel_t)
        # 抓取 + 三姿态 + 外部命令：臂 8 关节目标的**单一属主排他**（Z-Manip M0/M3）。
        # 规则（arm_command_gate.resolve_owner，优先级 CEO 2026-07-14 定案）：
        #   内置抓取激活(status 以 "running:" 开头，遗留调试面，显式触发才活) → 属主=
        #   PiperGraspController，写 grasp.q_tgt（现有语义一字不动）；
        #   否则 external 新鲜(已校验 /piper/joint_cmd，<fresh_sim_s) → 属主=external，
        #   由上一拍有效目标向命令目标限速逼近（每拍 dq 上限）；
        #   否则 → 属主=named_pose，写当前姿态表。
        # 任一拍臂目标只有一个逻辑写者，杜绝多写者互相覆盖抖动。
        # 裸机 A/B 对照（grasp is None）跳过全部臂/抓取写入（无臂关节）。
        if grasp is not None:
            if grasp_req["pending"]:
                grasp_req["pending"] = False
                box.update(physics_dt)
                grasp.start(box.data.root_pos_w[0])
            if step % 2 == 0:
                grasp.step(2 * physics_dt)
            grasp_active = grasp.status.startswith("running:")
            _owner = acg.resolve_owner(
                grasp_active,
                ext_arm["t"] if ext_arm["q"] is not None else None,
                sim_t["now"], _gate_params.fresh_sim_s)
            if _owner == acg.OWNER_GRASP:
                # 属主=抓取：臂目标由伺服 q_tgt 提供（姿态让位）。
                eff_arm_tgt = grasp.q_tgt
            elif _owner == acg.OWNER_EXTERNAL:
                # 属主=external（z_manip 抓取执行器）：从上一拍有效目标向已校验命令
                # 限速逼近——跳变只产生有界运动；断供 >fresh_sim_s 由 resolve_owner
                # 自动回落 named_pose（发布方需 >2Hz 续持所有权）。
                eff_arm_tgt = torch.tensor(
                    acg.rate_limit_step(eff_arm_tgt.tolist(), ext_arm["q"],
                                        _ext_dq_max),
                    dtype=torch.float32, device=eff_arm_tgt.device)
            else:
                # 属主=named_pose：写当前姿态 8 关节目标（arm_ids_t 名序=arm_names+grip_names）。
                _ptgt = _pose_tensor(named_pose_req["name"])
                eff_arm_tgt = _ptgt if _ptgt is not None else grasp.q_tgt
            robot.set_joint_position_target(
                eff_arm_tgt.unsqueeze(0), joint_ids=arm_ids_t)
        robot.write_data_to_sim()
        # 【3】自愈守卫：必须在 sim.step() 之前——step():565 的 PAUSE 旋等会先于守卫焊死主线程。
        if not sim.is_playing():
            if sim.is_stopped():
                print(f"[NAV][FATAL] timeline STOPPED at sim_t={sim_t['now']:.2f} "
                      f"— exit for supervised restart", flush=True)
                stopped = True
                break
            else:  # PAUSED：自动恢复
                now_wall = _time.time()
                _resume_ts[:] = [t for t in _resume_ts if now_wall - t < 60.0]
                _resume_ts.append(now_wall)
                if len(_resume_ts) > _RESUME_MAX_PER_MIN:
                    print(f"[NAV][FATAL] timeline PAUSED {len(_resume_ts)}x/min at "
                          f"sim_t={sim_t['now']:.2f} — 与人工暂停拉锯，exit for supervised restart",
                          flush=True)
                    stopped = True
                    break
                print(f"[NAV][WARN] timeline PAUSED at sim_t={sim_t['now']:.2f} "
                      f"— auto-resume ({len(_resume_ts)}/min)", flush=True)
                _tl.play()
                _tl.set_end_time(1.0e9)  # play() 可能复位 end_time；重申不可达端点
                _tl.commit()
        sim.step()  # 渲染节拍由 SimulationCfg.render_interval 管理
        robot.update(physics_dt)
        imu.update(physics_dt)
        if step % 20 == 0:  # GT 位姿 5Hz
            p = robot.data.root_pos_w[0].tolist()
            q = robot.data.root_quat_w[0].tolist()
            gt_msg.header.stamp.sec, gt_msg.header.stamp.nanosec = sec, nsec
            gt_msg.header.frame_id = "world"
            gt_msg.pose.position.x, gt_msg.pose.position.y, gt_msg.pose.position.z = p
            (gt_msg.pose.orientation.w, gt_msg.pose.orientation.x,
             gt_msg.pose.orientation.y, gt_msg.pose.orientation.z) = q
            gt_pub.publish(gt_msg)
            # 直立度：机体系重力 z 分量（站立≈-1，翻倒偏离）。同 5Hz 与 GT 位姿同步。
            up_z_msg.data = float(robot.data.projected_gravity_b[0, 2].item())
            up_z_pub.publish(up_z_msg)
            # 箱子 GT（pose+twist，verify oracle 的 get_object_positions/velocities 源）。
            # 与 M0.5 物理 prop 共用 _publish_odom；/objects/box/odom 语义/字段一字不变。
            box.update(physics_dt)
            _publish_odom(box_pub, box, "box", sec, nsec)
            # Z-Manip M0.5 GT 多路：每个物理 prop 同拍刷新后发 /objects/<name>/odom。
            for _pname, _pobj in phys_props.items():
                _pobj.update(physics_dt)
                _publish_odom(prop_pubs[_pname], _pobj, _pname, sec, nsec)
            if grasp is not None:
                gs = String()
                gs.data = f"{grasp.status};aperture={grasp.aperture():.4f};" \
                          f"cmd_closed={int(grasp.cmd_closed())}"
                gs_pub.publish(gs)

        if grasp is not None and step % 5 == 0:  # 臂状态 20Hz：EE GT 位姿（=夹持中心）+ 关节实测/目标
            ep = grasp.grip_center().tolist()
            _, eq = grasp.ee_pose()
            eq = eq.tolist()
            em = PoseStamped()
            em.header.stamp.sec, em.header.stamp.nanosec = sec, nsec
            em.header.frame_id = "world"
            em.pose.position.x, em.pose.position.y, em.pose.position.z = ep
            (em.pose.orientation.w, em.pose.orientation.x,
             em.pose.orientation.y, em.pose.orientation.z) = eq
            ee_pub.publish(em)
            js = JointState()
            js.header.stamp.sec, js.header.stamp.nanosec = sec, nsec
            js.name = grasp.arm_names + grasp.grip_names
            js.position = robot.data.joint_pos[0, arm_ids_t].tolist()
            js_pub.publish(js)
            jc = JointState()
            jc.header = js.header
            jc.name = js.name
            # 报**有效**臂目标（属主协调后实际写入的目标）——G-c 用 /piper/state vs /piper/cmd
            # 逐关节比对；若此处报 grasp.q_tgt 而实际写 named_pose，会假报大误差。
            jc.position = eff_arm_tgt.tolist()
            jc_pub.publish(jc)

        # 相机 update 只在发布帧做：每步 update 会打乱物理指令写入管线
        # （实测开相机后轮速目标恒为 0、施加力矩变刹车向）。CAM_STRIDE：10Hz or 2Hz(CAM_SLOW)。
        if step % CAM_STRIDE == 0:
            d435.update(physics_dt)

        # /clock：仿真时钟广播（导航栈开 use_sim_time 对齐）
        clock_msg.clock.sec, clock_msg.clock.nanosec = sec, nsec
        clock_pub.publish(clock_msg)

        # IMU 发布（A线修复 2026-07-08，CEO 真机处方"init pitch=20"的 mapping 模式落地）：
        # 真机形态：Mid-360 内置 IMU 与雷达刚性同斜 20°，真机部署在 arise localization 模式
        # 用 yaml init_pitch 声明初始姿态（laserMapping.cpp:498 setRPY，弧度）。本栈跑 mapping
        # 模式，init_pitch 被 local_mode:false 门死（:493/:514；:188-196 无先验地图自动回落）。
        # mapping 模式初始姿态的唯一活通道 = IMU orientation：laserMapping.cpp:487
        #   q_w_curr = q_extrinsic(Ry20).inverse() ⊗ q_imu_pub
        # 实测定罪（2026-07-08，office 静止 fresh init）：旧实现地图斜 11.26°，机理=
        # 初始姿态误判(~-1..-3°) vs 雷达真实 +18.8° + init 窗沉降残差，算术闭合。三件套修复：
        # 1) acc/gyr：运行时精确相对旋转表达到躯干系（等效"IMU 平装车体"，与标定
        #    imu_laser_rotation_offset=[0,20,0] 的水平 IMU 假设一致）。替换旧硬编码
        #    cos/sin(20°)（prim 实际姿态≠精确 20°，曾留 -2.95° 稳态残差，实测）。
        # 2) orientation：q_pub = Ry(+20°) ⊗ q_imu_raw —— 代数上使 :487 还原出
        #    T_w_lidar.rot = q_imu_raw = 雷达真实世界姿态（prim 误差一并吸收，零残差）。
        # 3) 发布延迟 IMU_SETTLE_S：SLAM 1s 重力 init 窗（imuPreintegration.cpp:886）只看
        #    站定机体（旧 init 窗含沉降瞬态，重力多偏 -1.35°）；真机开机时本来就已站稳。
        if sim_t["now"] >= IMU_SETTLE_S:
            q_imu = imu.data.quat_w                  # (1,4) wxyz：imu prim 世界姿态
            if IMU_ROUTE == "rotate":
                # ---- rotate 路线（默认=main 现状，一字不动）----
                q_trunk = robot.data.root_quat_w[0:1]    # (1,4) wxyz：躯干世界姿态
                f_imu = imu.data.lin_acc_b + math_utils.quat_apply_inverse(
                    q_imu, torch.tensor([[0.0, 0.0, 9.81]], device=q_imu.device))  # 比力（prim 系）
                acc = math_utils.quat_apply_inverse(
                    q_trunk, math_utils.quat_apply(q_imu, f_imu))[0].tolist()      # 躯干系=水平IMU
                gyr = math_utils.quat_apply_inverse(
                    q_trunk, math_utils.quat_apply(q_imu, imu.data.ang_vel_b))[0].tolist()
                quat = math_utils.quat_mul(Q_RY20.to(q_imu.device), q_imu)[0].tolist()
            else:
                # ---- raw 路线（P2.1，移植 codex sensor_frame_contract.py）----
                # acc/gyr 原样透传（留在 20° 斜的传感器系，含重力），orientation=q_imu_raw；
                # ARISE 下游自重力对齐。绝不在此再乘 Ry20/躯干旋转（否则双补偿）。
                acc_raw = imu.data.lin_acc_b[0].tolist()
                gyr_raw = imu.data.ang_vel_b[0].tolist()
                acc, gyr = sensor_frame_contract.to_navigation_imu(acc_raw, gyr_raw)
                quat = q_imu[0].tolist()
            imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec = sec, nsec
            imu_msg.header.frame_id = "imu"
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyr
            imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z = quat
            imu_pub.publish(imu_msg)

        # 相机发布：CAM_STRIDE 步一帧（10Hz 默认，2Hz 若 CAM_SLOW）。
        # Z-Manip M0：分辨率 848×480；同一帧发两套——
        #   旧口径（refs CMU 栈 RViz 消费，保留不删）：/camera/image(rgb8)、/camera/depth(32FC1 米)。
        #   新口径（realsense2_camera 对齐，供 z-manip / M0 gate）：/camera/color/image_raw(rgb8)、
        #     /camera/color/camera_info、/camera/aligned_depth_to_color/image_raw(16UC1 mm)、
        #     base→camera_color_optical_frame 动态 TF。
        # 尺寸/step 全用 wrist_camera 常量，杜绝与实际张量口径漂移（旧块曾写死 640，现 848）。
        if step % CAM_STRIDE == 0 and "rgb" in d435.data.output:
            W, H = wc.CAM_WIDTH, wc.CAM_HEIGHT
            rgb = d435.data.output["rgb"][0]
            if rgb.shape[-1] == 4:
                rgb = rgb[..., :3]
            rgb_np = rgb.to("cpu", non_blocking=False).numpy().tobytes() if hasattr(rgb, "to") else rgb.tobytes()
            # 旧口径 RGB（frame_id=d435 不变，尺寸随实际 848×480）
            im = Image()
            im.header.stamp.sec, im.header.stamp.nanosec = sec, nsec
            im.header.frame_id = "d435"
            im.height, im.width = H, W
            im.encoding, im.step = "rgb8", W * 3
            im.data = rgb_np
            rgb_pub.publish(im)
            # 新口径 color（frame_id=camera_color_optical_frame）
            cim = Image()
            cim.header.stamp.sec, cim.header.stamp.nanosec = sec, nsec
            cim.header.frame_id = wc.CAM_OPTICAL_FRAME
            cim.height, cim.width = H, W
            cim.encoding, cim.step = "rgb8", W * 3
            cim.data = rgb_np
            color_pub.publish(cim)
            # camera_info（与 color 同 stamp/frame；K/D/P/R 由 wrist_camera 填）
            ci = CameraInfo()
            wc.fill_camera_info(ci, sec, nsec)
            cam_info_pub.publish(ci)
            # 深度：一次取张量转 numpy(米)
            dep_t = d435.data.output["distance_to_image_plane"][0]
            dep_m = dep_t.to("cpu").numpy() if hasattr(dep_t, "to") else np.asarray(dep_t)
            # 旧口径 depth（32FC1 米，frame_id=d435 不变；尺寸随 848×480）
            dm = Image()
            dm.header = im.header
            dm.height, dm.width = H, W
            dm.encoding, dm.step = "32FC1", W * 4
            dm.data = dep_m.astype("float32").tobytes()
            depth_pub.publish(dm)
            # 新口径 aligned depth（16UC1 mm；近裁 0.28 + 无效清零 + 可选轻噪，见 wrist_camera）
            dep_mm = wc.process_depth(dep_m, _depth_rng)
            da = Image()
            da.header.stamp.sec, da.header.stamp.nanosec = sec, nsec
            da.header.frame_id = wc.CAM_OPTICAL_FRAME
            da.height, da.width = H, W
            da.encoding, da.step = "16UC1", W * 2
            da.data = dep_mm.tobytes()
            depth_aligned_pub.publish(da)
            # base→camera_color_optical_frame 动态 TF（读 d435_link body 世界位姿实时算）
            if tf_broadcaster is not None and _d435_body_idx is not None:
                cam_p = robot.data.body_pos_w[0, _d435_body_idx].tolist()
                cam_q = robot.data.body_quat_w[0, _d435_body_idx].tolist()  # wxyz
                base_p = robot.data.root_pos_w[0].tolist()
                base_q = robot.data.root_quat_w[0].tolist()                 # wxyz
                opt_p, opt_q = wc.base_to_optical_transform(
                    cam_p, cam_q, base_p, base_q)
                tfm = TransformStamped()
                tfm.header.stamp.sec, tfm.header.stamp.nanosec = sec, nsec
                tfm.header.frame_id = wc.CAM_TF_PARENT_FRAME
                tfm.child_frame_id = wc.CAM_OPTICAL_FRAME
                tfm.transform.translation.x = float(opt_p[0])
                tfm.transform.translation.y = float(opt_p[1])
                tfm.transform.translation.z = float(opt_p[2])
                (tfm.transform.rotation.w, tfm.transform.rotation.x,
                 tfm.transform.rotation.y, tfm.transform.rotation.z) = (
                    float(opt_q[0]), float(opt_q[1]),
                    float(opt_q[2]), float(opt_q[3]))
                tf_broadcaster.sendTransform(tfm)

        step += 1
        if args_cli.selftest:
            if step == 100:
                start_pos = robot.data.root_pos_w[0].clone()
            if step == 150:
                # 实测轮距（左右前轮世界系 y 距离）
                fl = robot.body_names.index("FL_foot"); fr = robot.body_names.index("FR_foot")
                track = abs(robot.data.body_pos_w[0, fl, 1] - robot.data.body_pos_w[0, fr, 1])
                print(f"[SELFTEST] 实测轮距 track={track:.3f}m (脚本用 {TRACK_WIDTH})")
            if step == 250:
                z = robot.data.root_pos_w[0, 2].item()
                leg_ids, leg_names = robot.find_joints("FL_(hip|thigh|calf)_joint")
                lp = robot.data.joint_pos[0, leg_ids].tolist()
                lt = default_pos[0, leg_ids].tolist()
                tq = robot.data.applied_torque[0, wheel_ids].tolist()
                print(f"[DIAG] 身高z={z:.3f} 轮力矩={[round(v,1) for v in tq]}")
            if step == 300:
                wv = robot.data.joint_vel[0, wheel_ids].tolist()
                print(f"[SELFTEST] 轮速实测 {dict(zip(wheel_names, [round(v,2) for v in wv]))} "
                      f"(目标 {round((0.4)/WHEEL_RADIUS,2)})")
            if step == 300:  # 前进段结束，切纯旋转
                dp = (robot.data.root_pos_w[0] - start_pos).tolist()
                # 滑移率（Finding B）：v_body = dx / 2.0s（step100->300 @100Hz）；
                # ω·r = mean(|轮角速度|)·WHEEL_RADIUS。slip = 1 - v_body/(ω·r)。
                # 摩擦修复前后对照的核心量：CEO 大理石地面轮转身体不进=高滑移率。
                _wv_mean = sum(abs(v) for v in wv) / max(len(wv), 1)
                _v_body = dp[0] / 2.0
                _v_wheel = _wv_mean * WHEEL_RADIUS
                _slip = 1.0 - (_v_body / _v_wheel) if _v_wheel > 1e-6 else float("nan")
                _tq_fwd = robot.data.applied_torque[0, wheel_ids].tolist()
                _tq_max = max(abs(t) for t in _tq_fwd)
                print(f"[SELFTEST] 滑移率 slip={_slip:.3f} "
                      f"(v_body={_v_body:.3f}m/s ω·r={_v_wheel:.3f}m/s; <0.15 GOOD, ≈0 理想) "
                      f"轮effort_max={_tq_max:.1f}(限23.5,{'饱和!' if _tq_max > 23.0 else 'ok'})")
                print(f"[SELFTEST] 前进 dx={dp[0]:.3f}m dy={dp[1]:.3f} "
                      f"({'PASS' if dp[0] > 0.3 else 'FAIL'})")
                st["vx"], st["wz"] = 0.3, 0.5  # 行进弧线段（planner 的真实指令形态）
                q = robot.data.root_quat_w[0].tolist()
                import math as _m
                yaw0["v"] = _m.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            if 300 < step <= 600:  # 弧线段：跟踪轮 effort 峰值（转向回归，高抓地负载）
                _tq = robot.data.applied_torque[0, wheel_ids].abs().max().item()
                arc_tq_peak["v"] = max(arc_tq_peak["v"], _tq)
                _uz = robot.data.projected_gravity_b[0, 2].item()
                arc_up_min["v"] = min(arc_up_min["v"], _uz)  # 最不直立时刻（越负越直立）
            if step == 600:  # 3s 旋转结束：测 yaw 响应
                import math as _m
                q = robot.data.root_quat_w[0].tolist()
                yaw1 = _m.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
                dyaw = (yaw1 - yaw0["v"] + _m.pi) % (2*_m.pi) - _m.pi
                print(f"[SELFTEST] 3s 旋转 dyaw={_m.degrees(dyaw):.1f}deg "
                      f"(指令 0.5rad/s x3s = 86deg; >45 PASS): "
                      f"{'PASS' if _m.degrees(dyaw) > 45 else 'FAIL'}")
                # 转向回归（Finding B 高摩擦副作用检查）：弧线段轮 effort 峰值不得顶 23.5、
                # 全程直立（up_z<-0.8=四轮着地直立）。高抓地改变差速+腿协调负载。
                print(f"[SELFTEST] 弧线转向回归 轮effort峰值={arc_tq_peak['v']:.1f}"
                      f"(限23.5,{'饱和!' if arc_tq_peak['v'] > 23.0 else 'ok'}) "
                      f"最不直立up_z={arc_up_min['v']:.2f}"
                      f"({'直立ok' if arc_up_min['v'] < -0.8 else '倾覆risk!'})")
                break
        if step == 200:
            print(f"[NAV] imu sample: acc={[round(a,2) for a in acc]} (水平化后期望 ~[0,0,9.8])")
        if step == 800:
            # P2.2 isaac_first 就绪标记：step=800=8 sim-s IMU 沉降。ARISE 只在启动时估一次
            # 重力——绝不让它从落地/出生瞬态 init（会把倾态固化进地图）。isaac_first 顺序下
            # bringup.sh 等此标记后才起 navstack，令 SLAM 首帧重力估计看到已站定的机体。
            print(f"[NAV] imu settled: step={step} acc={[round(a,2) for a in acc]}", flush=True)
        if args_cli.shot_dir and step % 3000 == 0:  # 30s @100Hz
            import os
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            os.makedirs(args_cli.shot_dir, exist_ok=True)
            capture_viewport_to_file(get_active_viewport(),
                                     f"{args_cli.shot_dir}/nav_{step//6000:04d}.png")
            # 跟随机器人视角（斜后上方俯视）。高度偏移=场景专属 follow_dz：
            # warehouse 3.6（无低顶）、office 2.0（压在天花板下，历史 3.6 会穿顶棚）。
            p = robot.data.root_pos_w[0].tolist()
            sim.set_camera_view(eye=(p[0] + 1.6, p[1] - 1.2, p[2] + SCENE_CAM["follow_dz"]),
                                target=(p[0], p[1], p[2] + 0.2))
            print(f"[POSE] step={step} root=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")

    # 收尾照旧；stopped=True 时响亮死给 status.sh/监管看，绝不再留 527% CPU 焊死僵尸。
    if obs_dumper:
        obs_dumper.close()
    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
    if stopped:
        import sys
        sys.exit(3)


if __name__ == "__main__":
    main()
