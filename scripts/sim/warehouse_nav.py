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
if _os.environ.get("GO2W_FAST_RENDER", "1") != "0":
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

ROBOT_URDF = "/workspace/go2w/assets/urdf/go2w_sensored.urdf"
LIDAR_USD = "/workspace/go2w/assets/lidar_configs/Livox_Mid360_approx.usd"
WAREHOUSE_USD = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd"

# Go2W 轮几何（left_wheel.dae 实测半径 0.086m；轮距待实测校准）
WHEEL_RADIUS = 0.086
TRACK_WIDTH = 0.288  # 自检实测（左右前轮世界系间距）
# Mid-360 出厂标定: imu^T_laser=[-0.011,-0.02329,0.04412] -> IMU 在雷达系的位置取反
IMU_OFFSET_IN_LIDAR = (0.011, 0.02329, -0.04412)

# 可抓物：6cm 红箱，方形回归验证过的空旷地带（(2,0)-(2,-2) 走廊内）。
# 6cm 低于地形分析的障碍阈值——接近时 planner 不会把它当障碍绕开
BOX_POS = (2.0, -1.0, 0.031)
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
        pos=(0.0, 0.0, 0.42),  # 贴地生成减小落地冲击
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
                ("lidar_pub.inputs:fullScan", True),
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
        env_cfg = sim_utils.UsdFileCfg(usd_path=WAREHOUSE_USD)
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
        update_period=0.1, height=480, width=640,
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

    # PiPER 抓取控制器（臂 8 关节目标的唯一属主；README 抓取管线见 sim-plan M5）
    from piper_grasp import PiperGraspController
    grasp = PiperGraspController(robot, args_cli.device or "cuda:0")
    arm_ids_t = grasp.all_ids

    # rclpy: IMU 发布 + cmd_vel 订阅（桥扩展已带 jazzy 内部库）
    rclpy.init()
    node = rclpy.create_node("go2w_isaac_bridge")
    imu_pub = node.create_publisher(Imu, "/imu/data", 50)
    # 地面真值位姿：来自 SIM 而非任何执行者——vector_os_nano verify 谓词的 GT 源
    gt_pub = node.create_publisher(PoseStamped, "/ground_truth/pose", 10)
    gt_msg = PoseStamped()
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

    step = 0
    imu_msg = Imu()
    clock_msg = Clock()
    physics_dt = sim.get_physics_dt()
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        sim_t["now"] += physics_dt
        sec, nsec = sim_stamp()
        # cmd_vel 看门狗：0.5s（仿真时）无新指令则停
        if args_cli.selftest:
            vx, wz = st["vx"], st["wz"]
        else:
            vx = cmd["vx"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
            wz = cmd["wz"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0

        if policy is not None:
            vy = vy_cmd["v"] if (sim_t["now"] - cmd["t"]) < 0.5 else 0.0
            if step % 2 == 0:  # 策略 50Hz（sim 100Hz）
                leg_ids, leg_tgt, wheel_ids_p, wheel_vel = policy.act(vx, vy, wz)
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
        if grasp_req["pending"]:
            grasp_req["pending"] = False
            box.update(physics_dt)
            grasp.start(box.data.root_pos_w[0])
        if step % 2 == 0:
            grasp.step(2 * physics_dt)
        robot.set_joint_position_target(
            grasp.q_tgt.unsqueeze(0), joint_ids=arm_ids_t)
        robot.write_data_to_sim()
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
            gs = String()
            gs.data = f"{grasp.status};aperture={grasp.aperture():.4f};" \
                      f"cmd_closed={int(grasp.cmd_closed())}"
            gs_pub.publish(gs)

        if step % 5 == 0:  # 臂状态 20Hz：EE GT 位姿（=夹持中心）+ 关节实测/目标
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
        # （实测开相机后轮速目标恒为 0、施加力矩变刹车向）
        if step % 10 == 0:
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

        # 相机 10Hz（每 10 个物理步）发布 RGB + 深度
        if step % 10 == 0 and "rgb" in d435.data.output:
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

    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
