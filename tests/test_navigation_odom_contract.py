import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_REFERENCES = (
    ROOT / "scripts/nav/system_isaac_sim.launch.py.reference",
    ROOT / "scripts/nav/system_isaac_sim_with_exploration.launch.py.reference",
)


def test_sim_launches_separate_raw_mount_from_level_arise_frame_and_publish_base_odometry():
    for launch_path in LAUNCH_REFERENCES:
        source = launch_path.read_text(encoding="utf-8")
        ast.parse(source)

        assert "'local_planner'), 'launch', 'local_planner.launch'" in source
        assert "default_value='0.27'" in source
        assert "default_value='0.10'" in source
        assert "default_value='0.3490658503988659'" in source
        for offset in ("sensorOffsetX", "sensorOffsetY", "sensorOffsetZ"):
            assert f"PythonExpression(['str(-float(', {offset}, '))'])" in source

        assert "package='odom_transformer'" in source
        assert "executable='odom_transformer_node'" in source
        assert "'use_sim_time': True" in source
        assert "'source_frame': 'sensor'" in source
        assert "'target_frame': 'base_link'" in source
        assert "'odom_topic_in': '/state_estimation'" in source
        assert "'odom_topic_out': '/odom_base_link'" in source
        assert source.count("start_odom_transformer = Node(") == 1
        assert source.count("ld.add_action(start_odom_transformer)") == 1

        # /lidar/points has its own physical mount TF. Named tf2 arguments avoid
        # the positional Euler-order ambiguity and keep it out of ARISE sensor.
        assert "package='tf2_ros'" in source
        assert "executable='static_transform_publisher'" in source
        assert "'--x', sensorOffsetX" in source
        assert "'--y', sensorOffsetY" in source
        assert "'--z', sensorOffsetZ" in source
        assert "'--roll', '0.0'" in source
        assert "'--pitch', sensorMountPitch" in source
        assert "'--yaw', '0.0'" in source
        assert "'--frame-id', 'base_link'" in source
        assert "'--child-frame-id', 'mid360_raw'" in source
        assert source.count("start_mid360_raw_tf = Node(") == 1
        assert source.count("ld.add_action(start_mid360_raw_tf)") == 1

        local_planner_block = source[
            source.index("start_local_planner = IncludeLaunchDescription("):
            source.index("start_odom_transformer = Node(")
        ]
        assert "sensorMountPitch" not in local_planner_block


def test_odom_stream_is_a_fail_closed_bringup_contract():
    gate = (ROOT / "scripts/nav/ros_stream_gate.py").read_text(encoding="utf-8")
    bringup = (ROOT / "scripts/nav/bringup.sh").read_text(encoding="utf-8")
    ast.parse(gate)

    assert '"/odom_base_link"' in gate
    assert 'Odometry, "/odom_base_link", self.on_odom' in gate
    assert 'msg.header.frame_id == "map"' in gate
    assert 'msg.child_frame_id == "base_link"' in gate
    assert 'failures.append("odom_stamp_not_advancing")' in gate
    assert re.search(r'add_argument\("--min-odom".+default=2', gate)
    assert "/state_estimation /odom_base_link" in bringup
    assert "--min-odom 2" in bringup
