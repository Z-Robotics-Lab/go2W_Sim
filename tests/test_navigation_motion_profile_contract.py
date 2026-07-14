import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
CHILD = ROOT / "scripts/nav/local_planner.launch.reference"
PYTHON_CHILD = ROOT / "scripts/nav/local_planner.launch.py.reference"
VALIDATOR = ROOT / "scripts/nav/validate_nav_profile.py"
SYSTEM_LAUNCHES = (
    ROOT / "scripts/nav/system_isaac_sim.launch.py.reference",
    ROOT / "scripts/nav/system_isaac_sim_with_exploration.launch.py.reference",
)


def test_xml_child_is_only_a_python_compatibility_wrapper():
    root = ET.parse(CHILD).getroot()
    arguments = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    assert arguments["robot_config"] == "$(env NAV_ROBOT_CONFIG unitree/unitree_go2)"
    assert arguments["maxSpeed"] == "$(env NAV_MAX_SPEED 0.25)"
    assert arguments["autonomySpeed"] == "$(env NAV_AUTONOMY_SPEED 0.20)"
    assert arguments["maxAccel"] == "$(env NAV_MAX_ACCEL 0.35)"
    assert arguments["maxYawRate"] == "$(env NAV_MAX_YAW_RATE_DEG_S 15.0)"
    assert arguments["goalReachedThreshold"] == (
        "$(env LOCAL_PLANNER_GOAL_REACHED_THRESHOLD 0.15)"
    )
    assert arguments["obstacleHeightThre"] == (
        "$(env LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE 0.20)"
    )
    assert arguments["use_sim_time"] == "true"
    assert arguments["startOdomTransformer"] == "false"
    assert arguments["overrideMountOffsets"] == "true"

    assert root.findall("node") == []
    includes = root.findall("include")
    assert len(includes) == 1
    include = includes[0]
    assert include.attrib["file"].endswith("/launch/local_planner.launch.py")
    forwarded = {item.attrib["name"]: item.attrib["value"] for item in include.findall("arg")}
    for name in (
        "config", "robot_config", "twoWayDrive", "realRobot", "autonomyMode",
        "use_sim_time", "startOdomTransformer", "overrideMountOffsets",
        "maxSpeed", "autonomySpeed",
        "maxAccel", "maxYawRate", "goalReachedThreshold", "obstacleHeightThre",
        "sensorOffsetX", "sensorOffsetY", "sensorOffsetZ",
    ):
        assert forwarded[name] == f"$(var {name})"
    assert arguments["cameraOffsetRoll"] == "-1.5707963"
    assert arguments["cameraOffsetYaw"] == "-1.5707963"
    assert arguments["cameraLink"] == "camera"


def test_system_launches_forward_every_motion_argument_instead_of_hardcoding_speed():
    names = (
        "maxSpeed", "autonomySpeed", "maxAccel", "maxYawRate",
        "goalReachedThreshold", "obstacleHeightThre",
    )
    for path in SYSTEM_LAUNCHES:
        source = path.read_text(encoding="utf-8")
        ast.parse(source)
        local_block = source[
            source.index("start_local_planner = IncludeLaunchDescription("):
            source.index("start_odom_transformer = Node(")
        ]
        assert "robotConfig = LaunchConfiguration('robotConfig')" in source
        assert "'robot_config': robotConfig" in local_block
        assert "'use_sim_time': 'true'" in local_block
        assert "'startOdomTransformer': 'false'" in local_block
        assert "'overrideMountOffsets': 'true'" in local_block
        assert "unitree/unitree_go2" in source
        for name in names:
            assert f"{name} = LaunchConfiguration('{name}')" in source
            declaration = source[source.index(f"declare_{name} ="):]
            assert f"'{name}'" in declaration[:240]
            assert f"'{name}': {name}" in local_block
            assert f"ld.add_action(declare_{name})" in source
        assert "'maxSpeed': '0.6'" not in local_block
        assert "'autonomySpeed': '0.6'" not in local_block


def test_office_profile_is_resolved_once_and_passed_to_launch_and_speed_recovery():
    bringup = (ROOT / "scripts/nav/bringup.sh").read_text(encoding="utf-8")
    supervisor = (ROOT / "scripts/nav/run_all_forever.sh").read_text(encoding="utf-8")
    bridge = (ROOT / "scripts/nav/agent_bridge.py").read_text(encoding="utf-8")

    assert 'GO2W_NAV_PROFILE="${GO2W_NAV_PROFILE:-manipulation_tracking}"' in bringup
    assert "manipulation_tracking)" in bringup
    for value in ("0.25", "0.20", "0.35", "15.0"):
        assert value in bringup
    for name in (
        "NAV_MAX_SPEED", "NAV_AUTONOMY_SPEED", "NAV_MAX_ACCEL",
        "NAV_MAX_YAW_RATE_DEG_S", "NAV_ROBOT_CONFIG",
        "LOCAL_PLANNER_GOAL_REACHED_THRESHOLD",
    ):
        assert f'-e {name}="${name}"' in bringup
        assert name in supervisor
    assert 'NAV_SPEED="$NAV_AUTONOMY_SPEED"' in bringup
    assert '-e NAV_SPEED="$NAV_SPEED"' in bringup
    assert '"NAV_SPEED=$NAV_SPEED"' in bringup
    assert 'NAV_ROBOT_CONFIG:-unitree/unitree_go2' in bringup
    assert 'robotConfig:="$NAV_ROBOT_CONFIG"' in supervisor
    assert 'maxSpeed:="$NAV_MAX_SPEED"' in supervisor
    assert 'autonomySpeed:="$NAV_AUTONOMY_SPEED"' in supervisor
    assert 'maxAccel:="$NAV_MAX_ACCEL"' in supervisor
    assert 'maxYawRate:="$NAV_MAX_YAW_RATE_DEG_S"' in supervisor
    assert "ros2 param set /localPlanner" not in supervisor
    assert "if ! python3 /ws/validate_nav_profile.py" in supervisor
    assert "invalid navigation profile; refusing to start" in supervisor
    assert "exit 2" in supervisor

    assert 'os.environ.get("NAV_MAX_SPEED", "0.25")' in bridge
    assert "_nav_speed / _nav_max_speed" in bridge
    assert "/ 0.875" not in bridge


def test_sync_installs_the_tracked_child_launch_contract():
    sync = (ROOT / "scripts/nav/sync_navstack_files.sh").read_text(encoding="utf-8")
    assert '"$HERE/local_planner.launch.reference"' in sync
    assert 'src/base_autonomy/local_planner/launch/local_planner.launch' in sync
    assert '"$HERE/local_planner.launch.py.reference"' in sync
    assert 'src/base_autonomy/local_planner/launch/local_planner.launch.py' in sync
    assert "validate_nav_profile.py" in sync
    assert "refresh_installed_launch" in sync
    assert "install/local_planner/share/local_planner/launch" in sync
    assert "install/share/local_planner/launch" in sync


def test_legacy_orchestrator_delegates_to_the_validated_supervisor():
    legacy = (ROOT / "scripts/nav/run_navstack.sh").read_text(encoding="utf-8")
    assert "exec bash /ws/run_all_forever.sh" in legacy
    assert "ros2 launch /ws/system_isaac_sim.launch.py" not in legacy
    assert "pgrep -x localPlanner" in legacy
    assert "pgrep -x pathFollower" in legacy
    assert "navigation children are not healthy" in legacy


def test_python_child_structurally_merges_yaml_before_final_motion_profile():
    source = PYTHON_CHILD.read_text(encoding="utf-8")
    ast.parse(source)
    for name in (
        "NAV_MAX_SPEED",
        "NAV_AUTONOMY_SPEED",
        "NAV_MAX_ACCEL",
        "NAV_MAX_YAW_RATE_DEG_S",
        "LOCAL_PLANNER_GOAL_REACHED_THRESHOLD",
        "LOCAL_PLANNER_OBSTACLE_HEIGHT_THRE",
        "NAV_ROBOT_CONFIG",
    ):
        assert name in source
    assert '"unitree/unitree_go2"' in source
    assert 'parameters=[local_parameters]' in source
    assert 'parameters=[path_parameters]' in source
    assert 'str(generic_path)' not in source
    assert 'str(robot_path)' not in source
    assert '"goalReachedThreshold": goal_threshold' in source
    assert '"obstacleHeightThre": obstacle_height' in source
    assert '"maxAccel": command_ramp' in source
    assert '"maxYawRate": yaw_rate' in source
    assert '"realRobot": real_robot' in source
    assert '_bool_value(context, "realRobot")' in source
    assert '_bool_value(context, "overrideMountOffsets")' in source
    assert 'DeclareLaunchArgument("overrideMountOffsets", default_value="false")' in source
    assert "if override_mount_offsets:" in source
    assert '_safe_config_path(config_root' in source
    assert 'candidate.resolve()' not in source

    local_generic = source.index(
        '_node_parameters(generic_data, "localPlanner", "local planner config")'
    )
    local_robot = source.index(
        '_node_parameters(robot_data, "localPlanner", "robot config")'
    )
    local_final = source.index('"goalReachedThreshold": goal_threshold')
    assert local_generic < local_robot < local_final

    follower_generic = source.index(
        '_node_parameters(generic_data, "pathFollower", "local planner config")'
    )
    follower_robot = source.index(
        '_node_parameters(robot_data, "pathFollower", "robot config")'
    )
    follower_final = source.index('"maxAccel": command_ramp')
    assert follower_generic < follower_robot < follower_final

    real_launches = list(
        (ROOT / "refs/Navigation-Physical-Experiment/src/base_autonomy/vehicle_simulator/launch").glob(
            "system_real_robot*.launch.py"
        )
    )
    assert real_launches
    assert all("local_planner.launch.py" in path.read_text() for path in real_launches)


def _validator_module():
    spec = importlib.util.spec_from_file_location("validate_nav_profile", VALIDATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _valid_profile():
    return {
        "maximum_speed": 0.25,
        "autonomy_speed": 0.20,
        "bridge_speed": 0.20,
        "command_ramp_coefficient": 0.35,
        "yaw_rate_deg_s": 15.0,
        "goal_threshold": 0.15,
        "obstacle_height": 0.20,
        "robot_config": "unitree/unitree_go2",
    }


@pytest.mark.parametrize(
    ("field", "dangerous_value"),
    (
        ("maximum_speed", 1.01),
        ("autonomy_speed", 0.26),
        ("bridge_speed", 0.21),
        ("command_ramp_coefficient", 2.01),
        ("yaw_rate_deg_s", 60.01),
        ("goal_threshold", 0.20),
        ("obstacle_height", 0.201),
        ("robot_config", "../../unsafe"),
    ),
)
def test_shared_validator_rejects_dangerous_profile_boundaries(field, dangerous_value):
    values = _valid_profile()
    values[field] = dangerous_value
    with pytest.raises(ValueError):
        _validator_module().validate_profile(**values)


def test_shared_validator_accepts_office_profile_and_existing_go2_config():
    values = _valid_profile()
    values["config_root"] = (
        ROOT / "refs/Navigation-Physical-Experiment/src/base_autonomy/local_planner/config"
    )
    _validator_module().validate_profile(**values)


def test_shared_validator_accepts_documented_root_level_robot_config():
    values = _valid_profile()
    values["robot_config"] = "mechanum_drive"
    values["config_root"] = (
        ROOT / "refs/Navigation-Physical-Experiment/src/base_autonomy/local_planner/config"
    )
    _validator_module().validate_profile(**values)


def test_shared_validator_rejects_algorithm_yaml_as_robot_config():
    values = _valid_profile()
    values["robot_config"] = "omniDir"
    values["config_root"] = (
        ROOT / "refs/Navigation-Physical-Experiment/src/base_autonomy/local_planner/config"
    )
    with pytest.raises(ValueError, match="sensorMountingOffsets"):
        _validator_module().validate_profile(**values)


@pytest.mark.parametrize(
    "config_name",
    ("/absolute/config", "../escape", "unitree/../escape", "unitree/go2.yaml", "bad name"),
)
def test_shared_validator_rejects_unsafe_robot_config_paths(config_name):
    values = _valid_profile()
    values["robot_config"] = config_name
    with pytest.raises(ValueError):
        _validator_module().validate_profile(**values)


def test_shared_validator_allows_package_owned_symlink_install_config(tmp_path):
    source = tmp_path / "source.yaml"
    source.write_text(
        """sensorMountingOffsets:
  ros__parameters: {sensorOffsetX: 0.0, sensorOffsetY: 0.0, sensorOffsetZ: 0.0}
localPlanner:
  ros__parameters: {vehicleLength: 0.5, vehicleWidth: 0.5}
pathFollower:
  ros__parameters: {}
""",
        encoding="utf-8",
    )
    config_root = tmp_path / "install" / "config"
    config_root.mkdir(parents=True)
    (config_root / "mechanum_drive.yaml").symlink_to(source)
    values = _valid_profile()
    values["robot_config"] = "mechanum_drive"
    values["config_root"] = config_root
    _validator_module().validate_profile(**values)


def test_max_accel_is_documented_as_wall_loop_command_ramp_not_sim_time_acceleration():
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (CHILD, PYTHON_CHILD, *SYSTEM_LAUNCHES)
    )
    assert "100 Hz" in sources or "100Hz" in sources
    assert "m/s^2 linear acceleration" not in sources
