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
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import omni.graph.core as og  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import rclpy  # noqa: E402
import torch  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
import isaaclab.utils.math as math_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sensors import Imu as IsaacImu  # noqa: E402
from isaaclab.sensors import ImuCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402

ROBOT_URDF = "/workspace/go2w/assets/urdf/go2w_sensored.urdf"
LIDAR_USD = "/workspace/go2w/assets/lidar_configs/Livox_Mid360_approx.usd"
WAREHOUSE_USD = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd"

# Go2W 轮几何（left_wheel.dae 实测半径 0.086m；轮距待实测校准）
WHEEL_RADIUS = 0.086
TRACK_WIDTH = 0.42
# Mid-360 出厂标定: imu^T_laser=[-0.011,-0.02329,0.04412] -> IMU 在雷达系的位置取反
IMU_OFFSET_IN_LIDAR = (0.011, 0.02329, -0.04412)

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
            effort_limit_sim=23.5, velocity_limit_sim=30.0, stiffness=0.0, damping=2.0),
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

    robot = Articulation(GO2W_NAV_CFG)
    imu = IsaacImu(ImuCfg(
        prim_path="/World/Robot/mid360_link",
        offset=ImuCfg.OffsetCfg(pos=IMU_OFFSET_IN_LIDAR),
        update_period=1 / 100,
        gravity_bias=(0.0, 0.0, 0.0),  # 纯运动学加速度；重力在发布时按姿态正确投影
    ))
    # 注意：isaaclab 默认 gravity_bias=(0,0,9.81) 是在传感器本体系直接加常量，
    # 只对水平安装成立；我们的 Mid-360 前倾 20°，必须按姿态投影（否则 SLAM 拿到错误重力方向）
    setup_lidar_ros2()
    sim.reset()
    print(f"[NAV] joints({robot.num_joints}) ready")

    # rclpy: IMU 发布 + cmd_vel 订阅（桥扩展已带 jazzy 内部库）
    rclpy.init()
    node = rclpy.create_node("go2w_isaac_bridge")
    imu_pub = node.create_publisher(Imu, "/imu/data", 50)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    cmd = {"vx": 0.0, "wz": 0.0, "t": 0.0}
    sim_t = {"now": 0.0}  # 全链路用仿真时钟（墙钟慢于实时会让 SLAM 数据破碎）

    def sim_stamp():
        t = sim_t["now"]
        sec = int(t)
        return sec, int((t - sec) * 1e9)

    def on_cmd(msg: TwistStamped):
        cmd["vx"] = msg.twist.linear.x
        cmd["wz"] = msg.twist.angular.z
        cmd["t"] = sim_t["now"]

    node.create_subscription(TwistStamped, "/cmd_vel", on_cmd, 10)

    default_pos = robot.data.default_joint_pos.clone()
    wheel_ids, wheel_names = robot.find_joints(".*_foot_joint")
    # 轮向符号：左侧(FL/RL)与右侧(FR/RR)前进同向性由 URDF 轴向决定，先全 +1，自检校准
    signs = {n: 1.0 for n in wheel_names}
    left = [i for i, n in zip(wheel_ids, wheel_names) if n.startswith(("FL", "RL"))]
    right = [i for i, n in zip(wheel_ids, wheel_names) if n.startswith(("FR", "RR"))]

    if args_cli.selftest:
        cmd["vx"], cmd["t"] = 0.4, 1e18  # 恒定前进指令
        start_pos = None

    step = 0
    imu_msg = Imu()
    clock_msg = Clock()
    physics_dt = sim.get_physics_dt()
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        sim_t["now"] += physics_dt
        # cmd_vel 看门狗：0.5s（仿真时）无新指令则停
        vx = cmd["vx"] if (sim_t["now"] - cmd["t"]) < 0.5 or args_cli.selftest else 0.0
        wz = cmd["wz"] if (sim_t["now"] - cmd["t"]) < 0.5 or args_cli.selftest else 0.0

        robot.set_joint_position_target(default_pos)
        vel_t = robot.data.default_joint_vel.clone()
        wl = (vx - wz * TRACK_WIDTH / 2) / WHEEL_RADIUS
        wr = (vx + wz * TRACK_WIDTH / 2) / WHEEL_RADIUS
        for i in left:
            vel_t[:, i] = wl
        for i in right:
            vel_t[:, i] = wr
        robot.set_joint_velocity_target(vel_t)
        robot.write_data_to_sim()
        sim.step()  # 渲染节拍由 SimulationCfg.render_interval 管理
        robot.update(physics_dt)
        imu.update(physics_dt)

        # /clock：仿真时钟广播（导航栈开 use_sim_time 对齐）
        sec, nsec = sim_stamp()
        clock_msg.clock.sec, clock_msg.clock.nanosec = sec, nsec
        clock_pub.publish(clock_msg)

        # IMU 200Hz 发布（比力 = 本体系运动学加速度 + R^T·(0,0,9.81)，真实 IMU 物理）
        g_b = math_utils.quat_apply_inverse(
            imu.data.quat_w, torch.tensor([[0.0, 0.0, 9.81]], device=imu.data.quat_w.device))
        acc = (imu.data.lin_acc_b + g_b)[0].tolist()
        gyr = imu.data.ang_vel_b[0].tolist()
        quat = imu.data.quat_w[0].tolist()  # wxyz
        imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec = sec, nsec
        imu_msg.header.frame_id = "imu"
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = acc
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyr
        imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z = quat
        imu_pub.publish(imu_msg)

        step += 1
        if args_cli.selftest:
            if step == 100:
                start_pos = robot.data.root_pos_w[0].clone()
            if step == 300:
                wv = robot.data.joint_vel[0, wheel_ids].tolist()
                print(f"[SELFTEST] 轮速实测 {dict(zip(wheel_names, [round(v,2) for v in wv]))} "
                      f"(目标 {round((0.4)/WHEEL_RADIUS,2)})")
            if step == 300:  # 2s+ @100Hz
                dp = (robot.data.root_pos_w[0] - start_pos).tolist()
                fwd = dp[0]
                print(f"[SELFTEST] 2s 前进位移 dx={fwd:.3f}m dy={dp[1]:.3f} "
                      f"(期望 ~0.8m; 为负则轮向符号要翻)")
                print(f"[SELFTEST] {'PASS' if fwd > 0.4 else 'FAIL'}")
                break
        if step == 200:
            print(f"[NAV] imu sample: acc={[round(a,2) for a in acc]} (期望 x~-3.3 z~9.2, 前倾20°)")
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
