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
from pathlib import Path

from sensor_frame_contract import to_navigation_imu
from standstill_control import (
    ParkingBrakeConfig,
    StandstillCommandGate,
    StandstillGateConfig,
    WheelParkingBrake,
    select_joint_complement,
)

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
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped  # noqa: E402
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
from trajectory_msgs.msg import JointTrajectory  # noqa: E402

# Z-Manip M0：腕相机口径 + 三姿态常量（同目录 sibling；纯常量/助手，无 isaac 依赖）。
import wrist_camera as wc  # noqa: E402
from manip_scene import load_manip_scene  # noqa: E402
from piper_trajectory import (  # noqa: E402
    format_execution_status,
    GripperCommandBuffer,
    GripperValidationError,
    JointTrajectoryBuffer,
)

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
# GO2W_SCENE 选场景：默认 "warehouse" 与历史逐字节等价（usd/spawn/box/cam 四值原样）。
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
        "cam": {"eye": (4.0, 4.0, 3.0), "target": (0.0, 0.0, 0.5),
                "follow_eye": (1.6, -1.2), "follow_target": (0.0, 0.0),
                "follow_dz": 3.6},
    },
    "office": {
        "usd": OFFICE_USD,
        # office 出生点：校准到开阔厅（2026-07-07 校准轮实证）。office 脚印
        # X[-4.3,5.3] Y[-9.3,0.1]；原点 (0,0) 在 reception 顶墙=拥挤角落（净空~1.5m），
        # 开阔厅在 -Y。选 (-2.5,-5.0)=/terrain_map 证实开阔（障碍净空 3m+）、手动驱动
        # origin→此点全程直立可达。z=0.42 与训练/warehouse 基线一致，避免带载荷落地冲击。
        "spawn": (-2.5, -5.0, 0.42),
        # 箱子放出生点 +X 前方 1m 开阔地（6cm 低于障碍阈值不被绕开）。
        "box": (-1.5, -5.0, 0.031),
        # 启动视角：出生点斜后上方 3-4m、eye 高度 2.4m（office 天花板下——高了被顶棚挡、
        # 看到屋顶/自带城市外景，CEO 实测坑）。target=出生位姿，拉起即见站立的狗。
        # follow_dz=2.0：跟随镜头也压在天花板下（历史 3.6 会穿顶棚）。
        # View from the robot side of the shelf so the front opening and objects
        # are visible; the previous +X view looked through the opaque back panel.
        "cam": {"eye": (-4.4, -7.0, 1.85),
                "target": (-0.95, -5.0, 0.48),
                "follow_eye": (-2.8, -2.5), "follow_target": (1.45, 0.0),
                "follow_dz": 1.55},
    },
}
SCENE_NAME = _os.environ.get("GO2W_SCENE", "office")
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

# Office manipulation fixture is data, not algorithm code.  Operators may point at
# another validated JSON file without rebuilding the simulator.  Other navigation
# scenes remain unchanged and do not spawn this fixture.
_DEFAULT_MANIP_CONFIG = Path(__file__).resolve().parents[2] / "configs/manip_office_scene.json"
_MANIP_CONFIG_PATH = Path(_os.environ.get(
    "GO2W_MANIP_SCENE_CONFIG", str(_DEFAULT_MANIP_CONFIG)))
if SCENE_NAME == "office":
    MANIP_SCENE = load_manip_scene(
        _MANIP_CONFIG_PATH, {"ISAAC_NUCLEUS_DIR": ISAAC_NUCLEUS_DIR})
    SCENE_SHELF_PARTS = MANIP_SCENE["shelf"]["parts"]
    SCENE_OBJECTS = MANIP_SCENE["objects"]
else:
    MANIP_SCENE = None
    SCENE_SHELF_PARTS = []
    SCENE_OBJECTS = []

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
    """Publish RTX points in the physical, pitched Mid-360 measurement frame."""
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
                ("lidar_pub.inputs:frameId", "mid360_raw"),
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
    print("[NAV] RTX lidar attached -> /lidar/points [mid360_raw]")


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
        # 带 PiPER/NUC 的部署形态质量与训练本体不同。默认使用经过静止载荷
        # 稳定性校准的刚度/阻尼；保留环境覆盖，便于在无载荷或新策略上复现训练态。
        _leg_k = float(_os.environ.get("GO2W_POLICY_LEG_STIFFNESS", "100.0"))
        _leg_d = float(_os.environ.get("GO2W_POLICY_LEG_DAMPING", "5.0"))
        GO2W_NAV_CFG.actuators["legs"].stiffness = _leg_k
        GO2W_NAV_CFG.actuators["legs"].damping = _leg_d
        GO2W_NAV_CFG.actuators["legs"].effort_limit_sim = 23.5
        _wheel_drive_stiffness = 0.0
        _wheel_drive_damping = float(
            _os.environ.get("GO2W_POLICY_WHEEL_DAMPING", "5.0"))
        GO2W_NAV_CFG.actuators["wheels"].stiffness = _wheel_drive_stiffness
        GO2W_NAV_CFG.actuators["wheels"].damping = _wheel_drive_damping
        GO2W_NAV_CFG.actuators["wheels"].effort_limit_sim = 23.5
        print(f"[NAV] policy gains legs=({_leg_k:.1f},{_leg_d:.1f}) "
              f"wheels damping={_wheel_drive_damping:.1f}", flush=True)
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
    # Configured Office fixture: every shelf part is a static collider.  Object
    # positions and assets are read only from JSON; none are an execution input.
    for _part in SCENE_SHELF_PARTS:
        _part_cfg = sim_utils.CuboidCfg(
            size=_part["size"],
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=_part["color"]),
        )
        _part_cfg.func(
            f"/World/ManipFixture/{_part['name']}", _part_cfg,
            translation=_part["position"],
            orientation=_part["orientation_wxyz"],
        )
        print(f"[MANIP_SCENE] static collider {_part['name']} "
              f"size={_part['size']} pos={_part['position']}", flush=True)

    def _proxy_shape_cfg(proxy):
        common = {"collision_props": sim_utils.CollisionPropertiesCfg()}
        if proxy["shape"] == "cuboid":
            return sim_utils.CuboidCfg(size=proxy["size"], **common)
        if proxy["shape"] == "cylinder":
            return sim_utils.CylinderCfg(
                radius=proxy["radius"], height=proxy["height"], **common)
        if proxy["shape"] == "capsule":
            return sim_utils.CapsuleCfg(
                radius=proxy["radius"], height=proxy["height"], **common)
        raise RuntimeError(f"unsupported validated proxy shape {proxy['shape']!r}")

    # Physical objects are retained solely for simulation and external scoring.
    # Physics-baked assets use their authored colliders.  Visual-only irregular YCB
    # assets receive a rigid Xform and configurable compound primitive colliders.
    phys_props = {}
    for _obj in SCENE_OBJECTS:
        _name = _obj["name"]
        _root = f"/World/ManipObjects/{_name}"
        if _obj["physics_mode"] == "asset":
            _cfg = RigidObjectCfg(
                prim_path=_root,
                spawn=sim_utils.UsdFileCfg(
                    usd_path=_obj["asset"], scale=_obj["scale"]),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=_obj["position"], rot=_obj["orientation_wxyz"]),
            )
            phys_props[_name] = RigidObject(_cfg)
        else:
            sim_utils.create_prim(
                _root, prim_type="Xform", position=_obj["position"],
                orientation=_obj["orientation_wxyz"])
            _visual_cfg = sim_utils.UsdFileCfg(
                usd_path=_obj["asset"], scale=_obj["scale"])
            _visual_cfg.func(
                f"{_root}/Visual", _visual_cfg,
                translation=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0))
            sim_utils.define_rigid_body_properties(
                _root, sim_utils.RigidBodyPropertiesCfg())
            sim_utils.define_mass_properties(
                _root, sim_utils.MassPropertiesCfg(mass=_obj["mass_kg"]))
            for _proxy_index, _proxy in enumerate(_obj["collision_proxies"]):
                _proxy_path = f"{_root}/Collision_{_proxy_index}"
                _proxy_cfg = _proxy_shape_cfg(_proxy)
                _proxy_cfg.func(
                    _proxy_path, _proxy_cfg,
                    translation=_proxy["position"],
                    orientation=_proxy["orientation_wxyz"])
                sim_utils.set_prim_visibility(
                    get_current_stage().GetPrimAtPath(_proxy_path), False)
            phys_props[_name] = RigidObject(RigidObjectCfg(
                prim_path=_root,
                spawn=None,
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=_obj["position"], rot=_obj["orientation_wxyz"]),
            ))
        print(f"[MANIP_SCENE] object {_name} family={_obj['shape_family']} "
              f"physics={_obj['physics_mode']} pos={_obj['position']} "
              f"asset={_obj['asset']}", flush=True)
    if MANIP_SCENE is not None:
        print(f"[MANIP_SCENE] loaded name={MANIP_SCENE['name']} "
              f"shelf_parts={len(SCENE_SHELF_PARTS)} objects={len(SCENE_OBJECTS)} "
              f"source={MANIP_SCENE['source']}", flush=True)
    imu = IsaacImu(ImuCfg(
        prim_path="/World/Robot/mid360_link",
        # Keep the physical IMU location and publish its raw vectors unchanged.
        # ARISE gravity-levels them together with the pitched Mid-360 cloud;
        # applying the mount rotation here would double-compensate both streams.
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
    # 轮胎高摩擦材质：滑移转向的横摆力矩来自轮-地纵向抓地力
    wheel_mat = sim_utils.RigidBodyMaterialCfg(
        static_friction=1.6, dynamic_friction=1.4, restitution=0.0)
    wheel_mat.func("/World/Materials/wheel_rubber", wheel_mat)
    for foot in ("FL", "FR", "RL", "RR"):
        sim_utils.bind_physics_material(f"/World/Robot/{foot}_foot",
                                        "/World/Materials/wheel_rubber")
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

    # Dedicated liveness evidence. Container state and boot log markers remain
    # true after Kit leaves the simulation loop, so they cannot prove that
    # /clock and sensors are advancing. Refresh this only after a complete
    # physics/publish iteration; status.sh treats it as fail-closed.
    _heartbeat_path = Path(_os.environ.get(
        "GO2W_ISAAC_HEARTBEAT_FILE",
        "/workspace/go2w/logs/.isaac_heartbeat",
    ))
    _heartbeat_path.parent.mkdir(parents=True, exist_ok=True)
    _heartbeat_path.unlink(missing_ok=True)
    _heartbeat_period_s = max(
        0.5, float(_os.environ.get("GO2W_ISAAC_HEARTBEAT_PERIOD_S", "2.0")))
    _heartbeat_next_wall = _time.monotonic()

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

    # PiPER execution boundary.  IK, grasp-pose generation, collision checking and
    # planning live outside Isaac and send a named JointTrajectory.  This bridge
    # validates limits/timing, interpolates on simulation time, and never reads an
    # object's simulator pose to decide arm motion.
    if WITH_ARM:
        arm_joint_ids, arm_joint_names = robot.find_joints("piper_joint[1-6]")
        grip_joint_ids, grip_joint_names = robot.find_joints("piper_joint[78]")
        piper_joint_ids = list(arm_joint_ids) + list(grip_joint_ids)
        ee_body_ids, _ = robot.find_bodies("piper_gripper_base")
        if len(arm_joint_ids) != 6 or len(grip_joint_ids) != 2 or not ee_body_ids:
            raise RuntimeError(
                f"invalid PiPER contract arm={arm_joint_names} grip={grip_joint_names} "
                f"ee={ee_body_ids}")
        _position_limits = {
            name: tuple(float(v) for v in robot.data.joint_pos_limits[0, joint_id].tolist())
            for joint_id, name in zip(arm_joint_ids, arm_joint_names)
        }
        _velocity_limits = {
            name: float(robot.data.joint_vel_limits[0, joint_id].item())
            for joint_id, name in zip(arm_joint_ids, arm_joint_names)
        }
        trajectory_executor = JointTrajectoryBuffer(
            arm_joint_names, _position_limits, _velocity_limits)
        _max_aperture = (
            float(robot.data.joint_pos_limits[0, grip_joint_ids[0], 1])
            - float(robot.data.joint_pos_limits[0, grip_joint_ids[1], 0]))
        gripper_executor = GripperCommandBuffer(0.0, _max_aperture)
        ee_body_idx = ee_body_ids[0]
        print(f"[PIPER_EXEC] ready arm={arm_joint_names} grip={grip_joint_names} "
              f"limits={_position_limits}", flush=True)
    else:
        arm_joint_ids = arm_joint_names = None
        grip_joint_ids = grip_joint_names = None
        piper_joint_ids = None
        trajectory_executor = None
        gripper_executor = None
        ee_body_idx = None
        print("[NAV] GO2W_WITH_ARM=0: bare trunk, PiPER executor SKIPPED (A/B control)", flush=True)

    # robot.joint_names contains movable articulation DOFs.  Excluding the
    # dynamically discovered PiPER IDs gives the complete platform state
    # without maintaining another model-specific list of leg and wheel names.
    _platform_ids, platform_joint_names = select_joint_complement(
        robot.joint_names, piper_joint_ids or ())
    platform_joint_ids = list(_platform_ids)
    print(f"[NAV] platform joint state ready ({len(platform_joint_ids)} joints): "
          f"{list(platform_joint_names)}", flush=True)

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
    # Camera payloads are high-bandwidth sensor streams. Reliable writers can
    # block the single physics/publish loop while a dead RViz or perception
    # reader remains in DDS discovery, freezing /clock with the images. Match
    # real camera drivers and every critical consumer with SensorDataQoS.
    rgb_pub = node.create_publisher(
        Image, "/camera/image", qos_profile_sensor_data)
    depth_pub = node.create_publisher(
        Image, "/camera/depth", qos_profile_sensor_data)
    # Z-Manip M0：realsense2_camera 对齐口径（新增，与旧 /camera/image·/camera/depth 并存——
    # 旧话题被 refs CMU 栈 RViz 消费，不可删；新话题供 z-manip / M0 gate）。sim=real 同名同编码。
    color_pub = node.create_publisher(
        Image, wc.TOPIC_COLOR, qos_profile_sensor_data)
    cam_info_pub = node.create_publisher(
        CameraInfo, wc.TOPIC_COLOR_INFO, qos_profile_sensor_data)
    depth_aligned_pub = node.create_publisher(
        Image, wc.TOPIC_DEPTH_ALIGNED, qos_profile_sensor_data)
    # base→camera_color_optical_frame 动态 TF（相机挂运动臂，逐拍位姿变；G-b 数值核对源）。
    tf_broadcaster = TransformBroadcaster(node) if _HAS_TF2 else None
    # d435_link body 索引（发动态 TF 用；prim 名 d435_link）。找不到不阻断，软降级留痕。
    _d435_body_ids, _ = robot.find_bodies("d435_link")
    _d435_body_idx = _d435_body_ids[0] if _d435_body_ids else None
    if _d435_body_idx is None:
        print("[NAV][WARN] d435_link body 未找到，optical TF 不发布", flush=True)
    _depth_rng = np.random.default_rng(0)  # 深度噪声固定种子（可复现；GO2W_DEPTH_NOISE=0 关噪）
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    # Simulation-only truth outputs are scoring oracles, never command inputs.
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
    platform_js_pub = node.create_publisher(
        JointState, "/go2w/joint_states", qos_profile_sensor_data)
    exec_status_pub = node.create_publisher(String, "/piper/execution_status", 5)
    trajectory_req = {"message": None, "cancel": False, "aperture": None}

    def on_joint_trajectory(msg: JointTrajectory):
        trajectory_req["message"] = msg
        print(f"[PIPER_EXEC] trajectory received joints={list(msg.joint_names)} "
              f"points={len(msg.points)}", flush=True)

    def on_gripper_aperture(msg: Float32):
        trajectory_req["aperture"] = float(msg.data)

    def on_execution_cancel(msg: Bool):
        if msg.data:
            trajectory_req["cancel"] = True

    node.create_subscription(
        JointTrajectory, "/piper/joint_trajectory", on_joint_trajectory, 5)
    node.create_subscription(
        Float32, "/piper/gripper_aperture", on_gripper_aperture, 5)
    node.create_subscription(Bool, "/piper/cancel", on_execution_cancel, 5)

    # Z-Manip M0：三姿态切换通道 /piper/named_pose(std_msgs/String ∈ {STOW,LOOKOUT,CARRY})。
    # sim 内部 String 通道，不经 HTTP 桥、不加 nav_owner 态。
    # 回调只置目标姿态名（校验合法），主循环按属主协调写臂目标（见抓取块 if/else 排他分支）。
    # 默认目标=LOOKOUT：拉起后主循环把臂切到平视（当前 URDF init 视轴朝上 +27.9° 过不了 G-b）。
    named_pose_req = {"name": wc.DEFAULT_POSE, "pending": True}

    def on_named_pose(msg: String):
        name = msg.data.strip()
        if name not in wc.NAMED_POSES:
            print(f"[POSE][WARN] 未知 named_pose {name!r}；合法={sorted(wc.NAMED_POSES)}，忽略",
                  flush=True)
            return
        named_pose_req["name"] = name
        named_pose_req["pending"] = True
        print(f"[POSE] named_pose -> {name}", flush=True)

    node.create_subscription(String, "/piper/named_pose", on_named_pose, 5)
    # Named poses remain an operator convenience.  Planned trajectories control
    # arm joints only; gripper aperture is an independent channel.
    _pose_cache: dict = {}

    def _pose_tensor(name: str):
        if arm_joint_names is None:
            return None
        if name not in _pose_cache:
            _pose_cache[name] = (
                torch.tensor(
                    wc.pose_target_by_names(name, arm_joint_names),
                    dtype=torch.float32, device=robot.data.joint_pos.device),
                torch.tensor(
                    wc.pose_target_by_names(name, grip_joint_names),
                    dtype=torch.float32, device=robot.data.joint_pos.device),
            )
        return _pose_cache.get(name)
    cmd = {"vx": 0.0, "wz": 0.0, "t": 0.0, "valid": True}
    vy_cmd = {"v": 0.0}
    invalid_cmd_log = {"not_before": 0.0}
    sim_t = {"now": 0.0}  # 全链路用仿真时钟（墙钟慢于实时会让 SLAM 数据破碎）
    timeline_stop = {"hit": False}

    # 【4】触发取证监听（坑40）：对 PLAY/PAUSE/STOP 打印带栈痕迹的事件——闭环触发源。
    #   进程内调用→栈给出调用者；栈只到事件泵→证明外部输入/UI 注入。放在 sim_t 定义后。
    _TL_NAMES = {0: "PLAY", 1: "PAUSE", 2: "STOP"}
    def _on_timeline_event(e):
        name = _TL_NAMES.get(e.type)
        if name is None:
            return  # 只关心 PLAY/PAUSE/STOP，忽略 tick/time-changed 洪流
        print(f"[NAV][TIMELINE] {name} at sim_t={sim_t['now']:.2f} wall={_time.time():.3f}",
              flush=True)
        if e.type == 2:
            timeline_stop["hit"] = True
        if e.type in (1, 2):  # PAUSE/STOP 附栈痕迹
            traceback.print_stack()
    _tl_event_sub = _tl.get_timeline_event_stream().create_subscription_to_pop(  # noqa: F841
        _on_timeline_event)  # 强引用保活（勿 GC，否则订阅静默失效）

    def sim_stamp():
        t = sim_t["now"]
        sec = int(t)
        return sec, int((t - sec) * 1e9)

    def on_cmd(msg: TwistStamped):
        values = (
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.angular.z),
        )
        if not all(math.isfinite(value) for value in values):
            cmd.update(vx=0.0, wz=0.0, t=sim_t["now"], valid=False)
            vy_cmd["v"] = 0.0
            now = _time.monotonic()
            if now >= invalid_cmd_log["not_before"]:
                print("[NAV][CMD][WARN] non-finite cmd_vel rejected; forcing parking",
                      flush=True)
                invalid_cmd_log["not_before"] = now + 1.0
            return
        vx, vy, wz = values
        cmd.update(vx=vx, wz=wz, t=sim_t["now"], valid=True)
        vy_cmd["v"] = vy

    node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, 10)

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
    st = {"vx": 0.4, "wz": 0.0}  # 自检指令（独立于 cmd：外部 pathFollower 会持续发
    # cmd_vel=0 把 cmd 字典覆盖——第7轮自检取证的教训）
    if args_cli.selftest:
        start_pos = None

    policy_cache = {}
    policy = None
    if args_cli.policy:
        from go2w_policy import Go2WPolicy
        policy = Go2WPolicy(args_cli.policy, robot, args_cli.device or "cuda:0")
    # ===== 零指令停车（孪生保真度补丁，CEO 已批）=====
    # 病理（DEBUG.md E0 实锤）：部署策略在零/小指令区永不真正站定——cmd(0,0,0) 下
    #   仍以 0.075 m/s(sim) 向机头爬行；nav 到点/yaw 门压制 vx/路径间隙的每一刻，
    #   CEO 看到的就是"蠕动、没有明显前进"。真机宇树步态零指令本就站定。
    # 修复：策略喂入路径加停车门——线速度与角速度分别持续近零时不再喂策略，
    #   改站姿保持（腿=default_pos、轮速=0）；任一通道表达有效运动意图即恢复策略。
    # metres/second 与 radians/second 不可直接拼成一个范数。旧统一阈值 0.15 会把
    #   manipulation visual search 的 0.1087 rad/s 有效 yaw 当成停车命令。
    # ---- 停车 v3（SI 双通道迟滞 + 柔性过渡）----
    # v1 病理（DEBUG 2026-07-07 A/B 实锤）：单阈值 0.2 + 单向 25 拍 debounce，被
    #   pathFollower 无目标 |wz|=1.396 爆发 chatter 每拍清零 → 25 连拍永远凑不满 →
    #   死区在 idle 下 enter 计数=0（等于没开）；且站姿硬钉 default_pos std=4.6cm 抖振
    #   （非自稳点），切换处满幅突变 → 失稳劈叉（win_b）。
    # v3 三件套：
    #   ① 线/角速度各自有带 SI 单位的进入、退出阈值和共享 debounce。只有两者都近零才
    #      进入；任一超过退出阈值就释放。默认退出阈值低于 manipulation 控制器在容差外
    #      的最小命令；时间滞回不依赖策略控制频率。
    #   ② 柔性进入——腿目标从"进入拍的当前腿位"线性混合到 default，BLEND_TICKS 拍到位，
    #      杜绝站姿硬钉的瞬时突跳。
    #   ③ 柔性退出——喂给策略的 cmd 从 0 斜坡到实际值，RAMP_TICKS 拍到位，杜绝满幅突变。
    #   站姿保持期间轮速目标恒 0（不变）。
    STANDSTILL_ENABLE = _os.environ.get("GO2W_STANDSTILL", "1") == "1"
    STANDSTILL_BLEND_TICKS = 10     # policy config 后与停车增益过渡拍数对齐
    STANDSTILL_RAMP_TICKS = 10      # policy config 后与停车增益过渡拍数对齐
    standstill_active = False       # 当前是否处于站姿保持
    standstill_blend = 0            # 进入混合剩余拍（BLEND..0 递减；0=已到 default）
    standstill_ramp = 0             # 退出斜坡剩余拍（RAMP..0 递减；0=已喂满幅）
    _blend_from = None              # 进入拍锁存的当前腿位（混合起点）
    # 站姿目标：腿=default（前 12 关节），轮速=0（后 4 关节）。用 policy 的 id/序对齐。
    parking_brake = None
    standstill_gate = None
    _parking_gains = None
    if policy is not None:
        _stand_leg_ids = policy.leg_ids
        _stand_leg_tgt = policy.default_pos[:1, :12].clone()
        _stand_wheel_ids = policy.wheel_ids
        _stand_wheel_vel = torch.zeros(1, len(policy.wheel_ids), device=policy.device)
        _parking_config = ParkingBrakeConfig.from_environ(
            _os.environ,
            drive_stiffness=_wheel_drive_stiffness,
            drive_damping=_wheel_drive_damping,
        )
        _standstill_config = StandstillGateConfig.from_environ(_os.environ)
        parking_brake = WheelParkingBrake(_parking_config)
        standstill_gate = StandstillCommandGate(_standstill_config)
        STANDSTILL_BLEND_TICKS = _parking_config.transition_ticks
        STANDSTILL_RAMP_TICKS = _parking_config.transition_ticks
        _parking_gains = (
            _parking_config.drive_stiffness,
            _parking_config.drive_damping,
        )

        def _apply_wheel_gains(stiffness, damping):
            """Keep PhysX control and IsaacLab's torque diagnostics consistent."""
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
              f"transition={_parking_config.transition_ticks} policy ticks; "
              f"linear={_standstill_config.linear_enter_mps:.3f}/"
              f"{_standstill_config.linear_exit_mps:.3f}m/s "
              f"angular={_standstill_config.angular_enter_rps:.3f}/"
              f"{_standstill_config.angular_exit_rps:.3f}rad/s "
              f"debounce={_standstill_config.enter_duration_s:.2f}/"
              f"{_standstill_config.exit_duration_s:.2f}s", flush=True)

    step = 0
    # Effective targets are also published on /piper/cmd.  A completed trajectory
    # holds its final point; cancel/rejection holds measured state; a named pose is
    # used only when explicitly selected (LOOKOUT is the startup selection).
    if trajectory_executor is not None:
        _startup_arm, _startup_grip = _pose_tensor(wc.DEFAULT_POSE)
        eff_arm_tgt = _startup_arm.clone()
        gripper_tgt = _startup_grip.clone()
        arm_owner = {"mode": "named_pose"}
        gripper_executor.note_external_owner(
            f"named_pose:{wc.DEFAULT_POSE}", sim_t["now"])
    else:
        eff_arm_tgt = None
        gripper_tgt = None
        arm_owner = {"mode": "disabled"}
    imu_msg = Imu()
    clock_msg = Clock()
    physics_dt = sim.get_physics_dt()
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
                # A reset is a stop boundary. Do not replay a cached gait target
                # or a pre-reset command on the next odd physics step.
                policy_cache.clear()
                policy_cache["legs"] = (_stand_leg_ids, _stand_leg_tgt.clone())
                policy_cache["wheels"] = (_stand_wheel_ids, _stand_wheel_vel.clone())
                cmd.update(vx=0.0, wz=0.0, t=float("-inf"), valid=False)
                vy_cmd["v"] = 0.0
            if parking_brake is not None:
                parking_brake.reset()
                standstill_gate.reset()
                _apply_wheel_gains(
                    parking_brake.config.drive_stiffness,
                    parking_brake.config.drive_damping,
                )
                _parking_gains = (
                    parking_brake.config.drive_stiffness,
                    parking_brake.config.drive_damping,
                )
            # 清死区态：复位后从"刚落地站姿"重新起算迟滞，避免带入摔倒前的时长/斜坡。
            standstill_active = False
            standstill_blend = standstill_ramp = 0
            _blend_from = None
            if trajectory_executor is not None:
                reset_arm = _birth_jpos[0, arm_joint_ids].tolist()
                trajectory_executor.cancel(reset_arm, reason="canceled:sim_reset")
                named_pose_req["name"] = wc.DEFAULT_POSE
                named_pose_req["pending"] = True
                trajectory_req.update(message=None, cancel=False, aperture=None)
            print(f"[NAV][RESET] done at sim_t={sim_t['now']:.2f} "
                  f"root->birth {SCENE_SPAWN} (scene={SCENE_NAME}), vel=0", flush=True)

        # Process arm commands in the physics thread.  Cancel has highest priority;
        # a newly planned trajectory supersedes a pending named pose.  Every start
        # state comes from measured joints, never an object truth topic.
        if trajectory_executor is not None:
            measured_arm = robot.data.joint_pos[0, arm_joint_ids].tolist()
            if trajectory_req["cancel"]:
                trajectory_req["cancel"] = False
                trajectory_req["message"] = None
                trajectory_executor.cancel(measured_arm)
                arm_owner["mode"] = "hold"
                eff_arm_tgt = torch.tensor(
                    measured_arm, dtype=torch.float32,
                    device=robot.data.joint_pos.device)
                print("[PIPER_EXEC] canceled -> measured-state hold", flush=True)
            elif trajectory_req["message"] is not None:
                _msg = trajectory_req["message"]
                trajectory_req["message"] = None
                try:
                    _times = [
                        float(point.time_from_start.sec)
                        + float(point.time_from_start.nanosec) * 1.0e-9
                        for point in _msg.points
                    ]
                    _command_id = trajectory_executor.submit(
                        _msg.joint_names,
                        [point.positions for point in _msg.points],
                        _times,
                        measured_arm,
                        sim_t["now"],
                        segment=_msg.header.frame_id,
                    )
                    named_pose_req["pending"] = False
                    arm_owner["mode"] = "trajectory"
                    print(f"[PIPER_EXEC] accepted command_id={_command_id} "
                          f"segment={trajectory_executor.segment} "
                          f"duration={_times[-1]:.3f}s points={len(_times)}", flush=True)
                except Exception as exc:  # callback boundary must fail closed
                    trajectory_executor.cancel(
                        measured_arm, reason=f"rejected:{type(exc).__name__}:{exc}")
                    arm_owner["mode"] = "hold"
                    eff_arm_tgt = torch.tensor(
                        measured_arm, dtype=torch.float32,
                        device=robot.data.joint_pos.device)
                    print(f"[PIPER_EXEC][REJECT] {exc}", flush=True)
            elif named_pose_req["pending"]:
                named_pose_req["pending"] = False
                if trajectory_executor.active:
                    trajectory_executor.cancel(
                        measured_arm, reason="preempted:named_pose")
                _named_arm, _named_grip = _pose_tensor(named_pose_req["name"])
                eff_arm_tgt = _named_arm.clone()
                gripper_tgt = _named_grip.clone()
                arm_owner["mode"] = "named_pose"
                gripper_executor.note_external_owner(
                    f"named_pose:{named_pose_req['name']}", sim_t["now"])

            if trajectory_req["aperture"] is not None:
                aperture = trajectory_req["aperture"]
                trajectory_req["aperture"] = None
                try:
                    accepted = gripper_executor.submit(aperture, sim_t["now"])
                    gripper_tgt = torch.tensor(
                        [0.5 * accepted.aperture, -0.5 * accepted.aperture],
                        dtype=torch.float32, device=robot.data.joint_pos.device)
                    print(f"[PIPER_EXEC] gripper command_id={accepted.command_id} "
                          f"aperture={accepted.aperture:.4f}m", flush=True)
                except GripperValidationError as exc:
                    gripper_executor.reject(exc, sim_t["now"])
                    print(f"[PIPER_EXEC][REJECT] gripper aperture={aperture!r}: {exc}",
                          flush=True)
        # cmd_vel 看门狗：0.5s（仿真时）无新指令则停
        if args_cli.selftest:
            vx, wz = st["vx"], st["wz"]
            command_fail_closed = False
        else:
            command_fresh = (sim_t["now"] - cmd["t"]) < 0.5
            command_fail_closed = not command_fresh or not cmd["valid"]
            vx = cmd["vx"] if not command_fail_closed else 0.0
            wz = cmd["wz"] if not command_fail_closed else 0.0

        if policy is not None:
            vy = vy_cmd["v"] if not command_fail_closed else 0.0
            if step % 2 == 0:  # 策略 50Hz（sim 100Hz）
                # ---- 停车 v3：SI 双通道迟滞 + 柔性过渡 ----
                if STANDSTILL_ENABLE:
                    decision = (
                        standstill_gate.force_park()
                        if command_fail_closed
                        else standstill_gate.update(
                            vx, vy, wz, dt_s=2.0 * physics_dt,
                        )
                    )
                    if decision.engage:
                        parking_brake.engage(
                            robot.data.joint_pos[0, _stand_wheel_ids].tolist())
                        # 柔性进入：锁存当前腿位为混合起点，BLEND 拍内 →default。
                        _blend_from = robot.data.joint_pos[:1, _stand_leg_ids].clone()
                        standstill_blend = STANDSTILL_BLEND_TICKS
                        standstill_ramp = 0
                        print(f"[NAV][STANDSTILL] enter at sim_t={sim_t['now']:.2f} "
                              f"linear={decision.linear_speed_mps:.4f}m/s "
                              f"angular={decision.angular_speed_rps:.4f}rad/s "
                              f"(blend {STANDSTILL_BLEND_TICKS})", flush=True)
                    elif decision.release:
                        parking_brake.release()
                        # 柔性退出：last_action 复位（站姿⟺a=0，物理诚实）+ 起 cmd 斜坡。
                        policy.last_action = torch.zeros_like(policy.last_action)
                        standstill_ramp = STANDSTILL_RAMP_TICKS
                        standstill_blend = 0
                        print(f"[NAV][STANDSTILL] exit at sim_t={sim_t['now']:.2f} "
                              f"linear={decision.linear_speed_mps:.4f}m/s "
                              f"angular={decision.angular_speed_rps:.4f}rad/s "
                              f"(reset+ramp {STANDSTILL_RAMP_TICKS})", flush=True)
                    standstill_active = decision.parked
                else:
                    # 停车门关闭：确保退出并清态。
                    if standstill_active:
                        parking_brake.release()
                        policy.last_action = torch.zeros_like(policy.last_action)
                    standstill_gate.reset()
                    standstill_active = False
                    standstill_blend = standstill_ramp = 0

                # Parking uses only wheel encoders.  Blend gains on transitions;
                # while releasing, the pure controller recenters the position
                # target so residual Kp cannot oppose the velocity command ramp.
                parking_command = parking_brake.step(
                    robot.data.joint_pos[0, _stand_wheel_ids].tolist())
                next_parking_gains = (
                    parking_command.stiffness,
                    parking_command.damping,
                )
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
                    # Never ramp velocity faster than the parking gain releases.
                    scale = min(scale, 1.0 - parking_command.blend)
                    leg_ids, leg_tgt, wheel_ids_p, wheel_vel = policy.act(
                        vx * scale, vy * scale, wz * scale)
                    policy_cache["legs"] = (leg_ids, leg_tgt)
                    policy_cache["wheels"] = (wheel_ids_p, wheel_vel)
            robot.set_joint_position_target(default_pos)  # 臂/夹爪保持
            if "legs" in policy_cache:
                robot.set_joint_position_target(policy_cache["legs"][1],
                                                joint_ids=policy_cache["legs"][0])
                robot.set_joint_velocity_target(policy_cache["wheels"][1],
                                                joint_ids=policy_cache["wheels"][0])
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
        # Generic arm execution: interpolate the externally planned trajectory on
        # sim time.  Gripper target is deliberately independent so approach,
        # closure, retreat and re-grasp can be composed by the same stack in sim
        # and on hardware.
        if trajectory_executor is not None:
            if arm_owner["mode"] == "trajectory":
                _sample = trajectory_executor.sample(sim_t["now"])
                eff_arm_tgt = torch.tensor(
                    _sample.positions, dtype=torch.float32,
                    device=robot.data.joint_pos.device)
                if _sample.done:
                    arm_owner["mode"] = "trajectory_hold"
                    print("[PIPER_EXEC] trajectory succeeded -> final hold", flush=True)
            _piper_target = torch.cat([eff_arm_tgt, gripper_tgt])
            robot.set_joint_position_target(
                _piper_target.unsqueeze(0), joint_ids=piper_joint_ids)
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
        if timeline_stop["hit"] or sim.is_stopped():
            print(f"[NAV][FATAL] timeline STOPPED during sim.step at sim_t={sim_t['now']:.2f} "
                  f"— exit before touching invalid PhysX views", flush=True)
            stopped = True
            break
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
            if trajectory_executor is not None:
                _measured_grip = robot.data.joint_pos[0, grip_joint_ids]
                _measured_aperture = float(
                    (_measured_grip[0] - _measured_grip[1]).item())
                _status = String()
                _status.data = format_execution_status(
                    trajectory_executor,
                    gripper_executor,
                    physical_owner=arm_owner["mode"],
                    measured_aperture=_measured_aperture,
                )
                exec_status_pub.publish(_status)

        if step % 5 == 0:
            # Complete platform proprioception for state assembly and MoveIt.
            # Names and indices come from the live articulation DOFs, not a
            # duplicated 16-joint model list.
            platform_js = JointState()
            platform_js.header.stamp.sec = sec
            platform_js.header.stamp.nanosec = nsec
            platform_js.name = list(platform_joint_names)
            platform_js.position = robot.data.joint_pos[
                0, platform_joint_ids].tolist()
            platform_js.velocity = robot.data.joint_vel[
                0, platform_joint_ids].tolist()
            platform_js_pub.publish(platform_js)

        if trajectory_executor is not None and step % 5 == 0:
            # Robot proprioception/command state.  /piper/ee_pose is a sim scoring
            # convenience; task-object truth never flows back into this executor.
            ep = robot.data.body_pos_w[0, ee_body_idx].tolist()
            eq = robot.data.body_quat_w[0, ee_body_idx].tolist()
            em = PoseStamped()
            em.header.stamp.sec, em.header.stamp.nanosec = sec, nsec
            em.header.frame_id = "world"
            em.pose.position.x, em.pose.position.y, em.pose.position.z = ep
            (em.pose.orientation.w, em.pose.orientation.x,
             em.pose.orientation.y, em.pose.orientation.z) = eq
            ee_pub.publish(em)
            js = JointState()
            js.header.stamp.sec, js.header.stamp.nanosec = sec, nsec
            js.name = list(arm_joint_names) + list(grip_joint_names)
            js.position = robot.data.joint_pos[0, piper_joint_ids].tolist()
            js_pub.publish(js)
            jc = JointState()
            jc.header = js.header
            jc.name = js.name
            jc.position = torch.cat([eff_arm_tgt, gripper_tgt]).tolist()
            jc_pub.publish(jc)

        # 相机 update 只在发布帧做：每步 update 会打乱物理指令写入管线
        # （实测开相机后轮速目标恒为 0、施加力矩变刹车向）。CAM_STRIDE：10Hz or 2Hz(CAM_SLOW)。
        if step % CAM_STRIDE == 0:
            d435.update(physics_dt)

        # /clock：仿真时钟广播（导航栈开 use_sim_time 对齐）
        clock_msg.clock.sec, clock_msg.clock.nanosec = sec, nsec
        clock_pub.publish(clock_msg)

        # Raw LiDAR and IMU samples share the physical +20 degree Mid-360
        # measurement frame.  ARISE estimates one gravity alignment for both
        # streams; rotating either stream again here would double-compensate
        # the mount and tilt registered_scan.
        g_b = math_utils.quat_apply_inverse(
            imu.data.quat_w, torch.tensor([[0.0, 0.0, 9.81]], device=imu.data.quat_w.device))
        ax, ay, az = (imu.data.lin_acc_b + g_b)[0].tolist()
        gx_, gy_, gz_ = imu.data.ang_vel_b[0].tolist()
        acc, gyr = to_navigation_imu((ax, ay, az), (gx_, gy_, gz_))
        quat = imu.data.quat_w[0].tolist()  # wxyz（arise use_imu_roll_pitch=false，仅参考）
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
                print(f"[SELFTEST] 前进 dx={dp[0]:.3f}m dy={dp[1]:.3f} "
                      f"({'PASS' if dp[0] > 0.3 else 'FAIL'})")
                st["vx"], st["wz"] = 0.3, 0.5  # 行进弧线段（planner 的真实指令形态）
                q = robot.data.root_quat_w[0].tolist()
                import math as _m
                yaw0["v"] = _m.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
            if step == 600:  # 3s 旋转结束：测 yaw 响应
                import math as _m
                q = robot.data.root_quat_w[0].tolist()
                yaw1 = _m.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
                dyaw = (yaw1 - yaw0["v"] + _m.pi) % (2*_m.pi) - _m.pi
                print(f"[SELFTEST] 3s 旋转 dyaw={_m.degrees(dyaw):.1f}deg "
                      f"(指令 0.5rad/s x3s = 86deg; >45 PASS): "
                      f"{'PASS' if _m.degrees(dyaw) > 45 else 'FAIL'}")
                break
        if step == 200:
            print(f"[NAV] imu sample: acc={[round(a,2) for a in acc]} (水平化后期望 ~[0,0,9.8])")
        if step == 800:
            # ARISE estimates gravity once. Do not let it initialize from the
            # spawn/landing transient; bringup waits for this 8 s sim-time marker.
            print(f"[NAV] imu settled: step={step} acc={[round(a,2) for a in acc]}")
        if args_cli.shot_dir and step % 3000 == 0:  # 30s @100Hz
            import os
            os.makedirs(args_cli.shot_dir, exist_ok=True)
            if _os.environ.get("GO2W_CAPTURE_VIEWPORT", "0") == "1":
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                capture_viewport_to_file(get_active_viewport(),
                                         f"{args_cli.shot_dir}/nav_{step//3000:04d}.png")
            # 跟随机器人视角（斜后上方俯视）。高度偏移=场景专属 follow_dz：
            # warehouse 3.6（无低顶）、office 2.0（压在天花板下，历史 3.6 会穿顶棚）。
            p = robot.data.root_pos_w[0].tolist()
            eye_dx, eye_dy = SCENE_CAM.get("follow_eye", (1.6, -1.2))
            target_dx, target_dy = SCENE_CAM.get("follow_target", (0.0, 0.0))
            sim.set_camera_view(
                eye=(p[0] + eye_dx, p[1] + eye_dy, p[2] + SCENE_CAM["follow_dz"]),
                target=(p[0] + target_dx, p[1] + target_dy, p[2] + 0.2),
            )
            print(f"[POSE] step={step} root=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")

        # A fresh heartbeat proves sim.step and every ROS publisher block
        # returned. It deliberately uses wall time so low RTF cannot look dead.
        _heartbeat_wall = _time.monotonic()
        if _heartbeat_wall >= _heartbeat_next_wall:
            _heartbeat_path.write_text(
                f"pid={_os.getpid()} sim_time={sim_t['now']:.6f} "
                f"wall_time={_time.time():.6f}\n",
                encoding="ascii",
            )
            _heartbeat_next_wall = _heartbeat_wall + _heartbeat_period_s

    # A GUI/editor quit makes SimulationApp.is_running() false without entering
    # the timeline STOP branch. Treat it as failure rather than a clean exit so
    # bringup can reconcile Isaac and RViz together on the next `up`.
    if not stopped:
        stopped = True
        print(f"[NAV][FATAL] SimulationApp left run loop at sim_t={sim_t['now']:.2f} "
              "— paired restart required", flush=True)

    # 收尾照旧；stopped=True 时响亮死给 status.sh/监管看，绝不再留 527% CPU 焊死僵尸。
    print("[NAV][CLEANUP] destroy ROS node", flush=True)
    node.destroy_node()
    print("[NAV][CLEANUP] shutdown rclpy", flush=True)
    rclpy.shutdown()
    print("[NAV][CLEANUP] close SimulationApp", flush=True)
    simulation_app.close()
    _heartbeat_path.unlink(missing_ok=True)
    if stopped:
        import sys
        sys.exit(3)


if __name__ == "__main__":
    main()
