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

import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
from geometry_msgs.msg import PoseStamped, TwistStamped  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402
from std_msgs.msg import String  # noqa: E402

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
# GO2W_SCENE 选场景：默认 "warehouse" 与历史逐字节等价（usd/spawn/box 三值原样）。
# 每场景字典：usd（None=纯地面 flat 调试面）、spawn（机器人出生 xyz）、box（红箱 xyz）。
# spawn/box 的选点是场景专属常量（无硬编码散落）；office 值由出生点校准轮写死（见 DEBUG）。
SCENES = {
    "warehouse": {
        "usd": WAREHOUSE_USD,
        "spawn": (0.0, 0.0, 0.42),   # 历史值：贴地生成减小落地冲击
        "box": (2.0, -1.0, 0.031),   # 方形回归验证过的空旷走廊 (2,0)-(2,-2)
    },
    "office": {
        "usd": OFFICE_USD,
        # office 出生点：校准到开阔厅（2026-07-07 校准轮实证）。office 脚印
        # X[-4.3,5.3] Y[-9.3,0.1]；原点 (0,0) 在 reception 顶墙=拥挤角落（净空~1.5m），
        # 开阔厅在 -Y。选 (-2.5,-5.0)=/terrain_map 证实开阔（障碍净空 3m+）、手动驱动
        # origin→此点全程直立可达。z=0.42 同 warehouse 贴地策略。回滚原点见 git 历史。
        "spawn": (-2.5, -5.0, 0.42),
        # 箱子放出生点 +X 前方 1m 开阔地（6cm 低于障碍阈值不被绕开）。
        "box": (-1.5, -5.0, 0.031),
    },
}
SCENE_NAME = _os.environ.get("GO2W_SCENE", "warehouse")
if SCENE_NAME not in SCENES:
    raise SystemExit(
        f"[NAV] 未知 GO2W_SCENE={SCENE_NAME!r}；可选：{sorted(SCENES)}")
SCENE = SCENES[SCENE_NAME]
SCENE_USD = SCENE["usd"]
SCENE_SPAWN = SCENE["spawn"]
SCENE_BOX = SCENE["box"]

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
            effort_limit_sim=30.0, velocity_limit_sim=5.0, stiffness=100.0, damping=5.0),
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
                ("lidar_pub.inputs:frameId", "sensor"),
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
    sim.set_camera_view(eye=(4.0, 4.0, 3.0), target=(0.0, 0.0, 0.5))

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
        # 训练态增益（必须与 robot_lab UNITREE_GO2W_CFG 一致，策略才有效）
        GO2W_NAV_CFG.actuators["legs"].stiffness = 25.0
        GO2W_NAV_CFG.actuators["legs"].damping = 0.5
        GO2W_NAV_CFG.actuators["legs"].effort_limit_sim = 23.5
        GO2W_NAV_CFG.actuators["wheels"].stiffness = 0.0
        GO2W_NAV_CFG.actuators["wheels"].damping = 0.5
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
    imu = IsaacImu(ImuCfg(
        prim_path="/World/Robot/mid360_link",
        # rot: -20° 俯仰抵消雷达前倾 -> IMU 帧水平（等效"IMU 平装在车体"）。
        # CMU 栈的 imu_acc_x_limit 限幅假设 IMU 水平：斜装 IMU 的重力 x 分量会被
        # 剪掉导致重力初始化错 13°、vehicle 系歪（RViz 路径扇面竖起，已实锤）。
        # 雷达相对 IMU 的 20° 俯仰改由标定文件 imu_laser_rotation_offset 声明。
        offset=ImuCfg.OffsetCfg(pos=IMU_OFFSET_IN_LIDAR),
        update_period=1 / 100,
        gravity_bias=(0.0, 0.0, 0.0),  # 纯运动学加速度；重力在发布时按姿态正确投影
    ))
    # 注意：isaaclab 默认 gravity_bias=(0,0,9.81) 是在传感器本体系直接加常量，
    # 只对水平安装成立；我们的 Mid-360 前倾 20°，必须按姿态投影（否则 SLAM 拿到错误重力方向）
    # D435 RGB+深度（挂手眼 d435_link，X 前向=convention world；69deg HFOV）
    d435 = Camera(CameraCfg(
        prim_path="/World/Robot/d435_link/d435_cam",
        update_period=CAM_UPDATE_PERIOD, height=480, width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=1.93, horizontal_aperture=2.65, clipping_range=(0.11, 20.0)),
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
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    # 抓取管线话题：箱子 GT、EE GT、臂关节态/目标、抓取指令与状态
    box_pub = node.create_publisher(Odometry, "/objects/box/odom", 5)
    ee_pub = node.create_publisher(PoseStamped, "/piper/ee_pose", 10)
    js_pub = node.create_publisher(JointState, "/piper/state", 10)
    jc_pub = node.create_publisher(JointState, "/piper/cmd", 10)
    gs_pub = node.create_publisher(String, "/piper/grasp_status", 5)
    grasp_req = {"pending": False}

    def on_grasp_cmd(msg: String):
        grasp_req["pending"] = True
        print(f"[GRASP] cmd received: {msg.data!r}", flush=True)

    node.create_subscription(String, "/piper/grasp_cmd", on_grasp_cmd, 5)
    cmd = {"vx": 0.0, "wz": 0.0, "t": 0.0}
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
    vy_cmd = {"v": 0.0}

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
    if policy is not None:
        _stand_leg_ids = policy.leg_ids
        _stand_leg_tgt = policy.default_pos[:1, :12].clone()
        _stand_wheel_ids = policy.wheel_ids
        _stand_wheel_vel = torch.zeros(1, len(policy.wheel_ids), device=policy.device)

    step = 0
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
            # 清死区态：复位后从"刚落地站姿"重新起算迟滞，避免带入摔倒前的计数/斜坡。
            standstill_low_count = standstill_high_count = 0
            standstill_active = False
            standstill_blend = standstill_ramp = 0
            _blend_from = None
            print(f"[NAV][RESET] done at sim_t={sim_t['now']:.2f} "
                  f"root->birth {SCENE_SPAWN} (scene={SCENE_NAME}), vel=0", flush=True)
        # cmd_vel 看门狗：0.5s（仿真时）无新指令则停
        if args_cli.selftest:
            vx, wz = st["vx"], st["wz"]
        else:
            vx = cmd["vx"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
            wz = cmd["wz"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0

        if policy is not None:
            vy = vy_cmd["v"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
            if step % 2 == 0:  # 策略 50Hz（sim 100Hz）
                # ---- 死区 v2 迟滞状态机：命令 3D 范数 + 双阈值 + 柔性过渡 ----
                cmd_norm = math.sqrt(vx * vx + vy * vy + wz * wz)
                if STANDSTILL_ENABLE:
                    # 进入迟滞：norm<ENTER_THRESH 连续 ENTER_DEBOUNCE 拍。
                    if cmd_norm < STANDSTILL_ENTER_THRESH:
                        standstill_low_count += 1
                        standstill_high_count = 0
                        if standstill_low_count >= STANDSTILL_ENTER_DEBOUNCE \
                                and not standstill_active:
                            standstill_active = True
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
                        policy.last_action = torch.zeros_like(policy.last_action)
                    standstill_low_count = standstill_high_count = 0
                    standstill_blend = standstill_ramp = 0

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
        # 抓取：指令接收 -> 状态机启动；50Hz 伺服；臂目标最后写覆盖 default 保持
        # 裸机 A/B 对照（grasp is None）跳过全部臂/抓取写入。
        if grasp is not None:
            if grasp_req["pending"]:
                grasp_req["pending"] = False
                box.update(physics_dt)
                grasp.start(box.data.root_pos_w[0])
            if step % 2 == 0:
                grasp.step(2 * physics_dt)
            robot.set_joint_position_target(
                grasp.q_tgt.unsqueeze(0), joint_ids=arm_ids_t)
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
            # 箱子 GT（pose+twist，verify oracle 的 get_object_positions/velocities 源）
            box.update(physics_dt)
            bp = box.data.root_pos_w[0].tolist()
            bq = box.data.root_quat_w[0].tolist()
            bv = box.data.root_lin_vel_w[0].tolist()
            bo = Odometry()
            bo.header.stamp.sec, bo.header.stamp.nanosec = sec, nsec
            bo.header.frame_id = "world"
            bo.child_frame_id = "box"
            (bo.pose.pose.position.x, bo.pose.pose.position.y,
             bo.pose.pose.position.z) = bp
            (bo.pose.pose.orientation.w, bo.pose.pose.orientation.x,
             bo.pose.pose.orientation.y, bo.pose.pose.orientation.z) = bq
            (bo.twist.twist.linear.x, bo.twist.twist.linear.y,
             bo.twist.twist.linear.z) = bv
            box_pub.publish(bo)
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
            jc.position = grasp.q_tgt.tolist()
            jc_pub.publish(jc)

        # 相机 update 只在发布帧做：每步 update 会打乱物理指令写入管线
        # （实测开相机后轮速目标恒为 0、施加力矩变刹车向）。CAM_STRIDE：10Hz or 2Hz(CAM_SLOW)。
        if step % CAM_STRIDE == 0:
            d435.update(physics_dt)

        # /clock：仿真时钟广播（导航栈开 use_sim_time 对齐）
        clock_msg.clock.sec, clock_msg.clock.nanosec = sec, nsec
        clock_pub.publish(clock_msg)

        # IMU 发布：比力（斜帧）-> 恒定 Ry(+20°) 旋到水平帧。
        # isaaclab OffsetCfg.rot 实测不作用于测量值（样本仍斜帧），故自己旋。
        # 与 SLAM 标定 imu_laser_rotation_offset=[0,20,0]（雷达相对水平 IMU 俯仰 20°）自洽。
        g_b = math_utils.quat_apply_inverse(
            imu.data.quat_w, torch.tensor([[0.0, 0.0, 9.81]], device=imu.data.quat_w.device))
        ax, ay, az = (imu.data.lin_acc_b + g_b)[0].tolist()
        gx_, gy_, gz_ = imu.data.ang_vel_b[0].tolist()
        CY, SY = 0.9396926, 0.3420201  # cos/sin(20°)
        acc = (CY * ax + SY * az, ay, -SY * ax + CY * az)
        gyr = (CY * gx_ + SY * gz_, gy_, -SY * gx_ + CY * gz_)
        quat = imu.data.quat_w[0].tolist()  # wxyz（arise use_imu_roll_pitch=false，仅参考）
        imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec = sec, nsec
        imu_msg.header.frame_id = "imu"
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyr
        imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z = quat
        imu_pub.publish(imu_msg)

        # 相机发布 RGB + 深度：CAM_STRIDE 步一帧（10Hz 默认，2Hz 若 CAM_SLOW）
        if step % CAM_STRIDE == 0 and "rgb" in d435.data.output:
            rgb = d435.data.output["rgb"][0]
            if rgb.shape[-1] == 4:
                rgb = rgb[..., :3]
            rgb_np = rgb.to("cpu", non_blocking=False).numpy().tobytes() if hasattr(rgb, "to") else rgb.tobytes()
            im = Image()
            im.header.stamp.sec, im.header.stamp.nanosec = sec, nsec
            im.header.frame_id = "d435"
            im.height, im.width = 480, 640
            im.encoding, im.step = "rgb8", 640 * 3
            im.data = rgb_np
            rgb_pub.publish(im)
            dep = d435.data.output["distance_to_image_plane"][0]
            dm = Image()
            dm.header = im.header
            dm.height, dm.width = 480, 640
            dm.encoding, dm.step = "32FC1", 640 * 4
            dm.data = dep.to("cpu").numpy().astype("float32").tobytes() if hasattr(dep, "to") else dep.astype("float32").tobytes()
            depth_pub.publish(dm)

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
        if args_cli.shot_dir and step % 3000 == 0:  # 30s @100Hz
            import os
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            os.makedirs(args_cli.shot_dir, exist_ok=True)
            capture_viewport_to_file(get_active_viewport(),
                                     f"{args_cli.shot_dir}/nav_{step//6000:04d}.png")
            # 跟随机器人视角
            p = robot.data.root_pos_w[0].tolist()
            # 高位斜俯视：货架区不易挡镜头
            sim.set_camera_view(eye=(p[0] + 1.6, p[1] - 1.2, p[2] + 3.6),
                                target=(p[0], p[1], p[2] + 0.2))
            print(f"[POSE] step={step} root=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})")

    # 收尾照旧；stopped=True 时响亮死给 status.sh/监管看，绝不再留 527% CPU 焊死僵尸。
    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()
    if stopped:
        import sys
        sys.exit(3)


if __name__ == "__main__":
    main()
