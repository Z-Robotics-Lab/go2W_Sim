#!/usr/bin/env python3
"""传感器版 Go2W（PiPER + Mid-360 + D435 + NUC 配重）落入 Isaac Sim 室内场景。

用法（容器内）：
  GUI 看场景:    /isaac-sim/python.sh warehouse_scene.py
  headless 截图: /isaac-sim/python.sh warehouse_scene.py --headless --enable_cameras \
                   --screenshot /workspace/go2w/logs/scene.png
  云资产不可用:  加 --env flat 用本地平地代替仓库
  轮子转起来:    加 --drive 2.0  （rad/s，看物理是否鲜活）
"""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--env", choices=["warehouse", "flat"], default="warehouse",
                    help="warehouse=Nucleus 云端仓库资产; flat=本地平地(无网络依赖)")
parser.add_argument("--screenshot", type=str, default=None,
                    help="步进若干帧后保存视口截图到该路径并退出")
parser.add_argument("--drive", type=float, default=0.0, help="轮关节速度目标 rad/s")
parser.add_argument("--cam_eye", type=float, nargs=3, default=[3.5, 3.5, 2.5],
                    help="相机位置 x y z")
parser.add_argument("--cam_target", type=float, nargs=3, default=[0.0, 0.0, 0.5],
                    help="相机看向 x y z")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR  # noqa: E402

ROBOT_URDF = "/workspace/go2w/assets/urdf/go2w_sensored.urdf"
WAREHOUSE_USD = f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/full_warehouse.usd"

# 与 robot_lab UNITREE_GO2W_CFG 同源的底盘参数；臂/夹爪为位置保持（可调）
GO2W_SENSORED_CFG = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UrdfFileCfg(
        asset_path=ROBOT_URDF,
        fix_base=False,
        # 保留固定 link（mid360/d435/nuc 的 frame 后续挂传感器要用）
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
        pos=(0.0, 0.0, 0.50),
        joint_pos={
            ".*_hip_joint": 0.0,
            ".*_thigh_joint": 0.8,
            ".*_calf_joint": -1.5,
            ".*_foot_joint": 0.0,      # 4 个轮
            "piper_joint2": 0.8,       # 臂收拢姿态，避免前伸翻车
            "piper_joint3": -1.2,
            "piper_joint[1456]": 0.0,
            "piper_joint[78]": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=["(FL|FR|RL|RR)_(hip|thigh|calf)_joint"],
            effort_limit_sim=23.5, velocity_limit_sim=30.0,
            # 静态展示场景用高刚度撑住 26kg 整机（RL 训练配置是 25/0.5，勿混用）
            stiffness=60.0, damping=2.0,
        ),
        "wheels": ImplicitActuatorCfg(
            joint_names_expr=[".*_foot_joint"],
            effort_limit_sim=23.5, velocity_limit_sim=30.0,
            stiffness=0.0, damping=0.5,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["piper_joint[1-6]"],
            effort_limit_sim=30.0, velocity_limit_sim=5.0,
            stiffness=100.0, damping=5.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["piper_joint[78]"],
            effort_limit_sim=50.0, velocity_limit_sim=1.0,
            stiffness=800.0, damping=20.0,
        ),
    },
)


def main():
    sim = SimulationContext(SimulationCfg(dt=1 / 200, device=args_cli.device))
    sim.set_camera_view(eye=tuple(args_cli.cam_eye), target=tuple(args_cli.cam_target))

    if args_cli.env == "warehouse":
        env_cfg = sim_utils.UsdFileCfg(usd_path=WAREHOUSE_USD)
        env_cfg.func("/World/Warehouse", env_cfg)
    else:
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/Ground", ground)
        light = sim_utils.DomeLightCfg(intensity=2000.0)
        light.func("/World/Light", light)

    robot = Articulation(GO2W_SENSORED_CFG)
    sim.reset()
    print(f"[INFO] robot joints ({robot.num_joints}):", robot.joint_names)
    # 关键 frame 位姿（校验挂载位置/朝向用）
    for name in ("mid360_link", "d435_link", "piper_gripper_base", "nuc_weight", "regulator"):
        if name in robot.body_names:
            i = robot.body_names.index(name)
            p = robot.data.body_pos_w[0, i].tolist()
            q = robot.data.body_quat_w[0, i].tolist()
            print(f"[FRAME] {name}: pos=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) quat_wxyz=({q[0]:.2f},{q[1]:.2f},{q[2]:.2f},{q[3]:.2f})")
        else:
            print(f"[FRAME] {name}: NOT FOUND in body_names!")

    default_pos = robot.data.default_joint_pos.clone()
    wheel_ids, _ = robot.find_joints(".*_foot_joint")

    step = 0
    while simulation_app.is_running():
        robot.set_joint_position_target(default_pos)
        if args_cli.drive != 0.0:
            vel_target = robot.data.default_joint_vel.clone()
            vel_target[:, wheel_ids] = args_cli.drive
            robot.set_joint_velocity_target(vel_target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        step += 1

        if step == 100:
            h = robot.data.root_pos_w[0, 2].item()
            print(f"[CHECK] 100 步后机身高度 z={h:.3f} m（期望 ~0.4，若 <0.25 说明翻了/塌了）")
        if args_cli.screenshot and step == 150:
            from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
            capture_viewport_to_file(get_active_viewport(), args_cli.screenshot)
            for _ in range(20):  # 等异步写盘
                simulation_app.update()
            print(f"[INFO] screenshot -> {args_cli.screenshot}")
            if args_cli.headless:
                break  # GUI 模式下继续跑，窗口留给用户

    simulation_app.close()


if __name__ == "__main__":
    main()
