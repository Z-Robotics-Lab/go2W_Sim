import importlib.util
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
BUILDER_PATH = ROOT / "scripts/nav/build_rviz_config.py"
CONTRACT_PATH = ROOT / "configs/rviz/manipulation_topics.json"


def _load_builder():
    spec = importlib.util.spec_from_file_location("build_rviz_config", BUILDER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _minimal_stock():
    return {
        "Panels": [
            {"Class": "rviz_common/Displays", "Name": "Displays"},
            {"Class": "teleop_rviz_plugin/TeleopPanel", "Name": "TeleopPanel"},
        ],
        "Visualization Manager": {
            "Displays": [
                {
                    "Class": "rviz_default_plugins/PointCloud2",
                    "Name": "Navigation map",
                    "Enabled": True,
                    "Topic": {"Value": "/overall_map"},
                },
                {
                    "Class": "rviz_default_plugins/Image",
                    "Name": "legacy image",
                    "Enabled": True,
                    "Topic": {"Value": "/camera/image"},
                },
            ],
            "Global Options": {"Fixed Frame": "map"},
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit",
                    "Name": "Current View",
                    "Target Frame": "map",
                },
            },
        },
    }


def _flatten(displays):
    for display in displays:
        yield display
        yield from _flatten(display.get("Displays", []))


def test_combined_config_preserves_navigation_and_adds_safe_debug_views():
    builder = _load_builder()
    contract = builder.load_contract(CONTRACT_PATH)
    result = builder.augment_config(_minimal_stock(), contract)
    panels = result["Panels"]
    assert not any("teleop" in panel["Class"] for panel in panels)

    displays = list(_flatten(result["Visualization Manager"]["Displays"]))
    by_name = {display["Name"]: display for display in displays}
    assert by_name["Navigation map"]["Enabled"] is True
    assert "legacy image" not in by_name
    assert by_name["Local LiDAR [LIVE, accumulated]"]["Enabled"] is True
    assert by_name["Local LiDAR [LIVE, accumulated]"]["Decay Time"] == 0.2
    assert by_name["Local LiDAR [LIVE, accumulated]"]["Style"] == "Points"
    assert by_name["Local LiDAR [LIVE, accumulated]"]["Topic"]["Value"] == "/lidar/points"
    assert by_name["Local LiDAR [LIVE, accumulated]"]["Topic"]["Reliability Policy"] == "Best Effort"
    assert by_name["Perception | Wrist RGB [LIVE]"]["Enabled"] is True
    assert by_name["Perception | Aligned Depth [LIVE]"]["Enabled"] is True
    assert by_name["Perception | Detections + Mask [upstream required]"]["Enabled"] is False
    assert by_name["Validated Target Cloud [upstream required]"]["Enabled"] is False
    assert by_name["Scene Collision Cloud [upstream required]"]["Enabled"] is False
    assert by_name["6DoF Grasp Candidates [upstream required]"]["Enabled"] is False
    assert by_name["Octomap Occupied Cells [octomap_server required]"]["Enabled"] is False
    assert by_name["Perception Contract Status"]["Enabled"] is False
    assert by_name["PiPER Execution Status"]["Enabled"] is False
    assert "QMainWindow State" not in result["Window Geometry"]

    saved = result["Visualization Manager"]["Views"]["Saved"]
    assert {view["Name"] for view in saved} == {
        "Navigation Overview",
        "Mobile Manipulation",
        "Wrist Camera TF",
    }
    navigation = next(view for view in saved if view["Name"] == "Navigation Overview")
    assert navigation["Target Frame"] == "map"

    current = result["Visualization Manager"]["Views"]["Current"]
    assert result["Visualization Manager"]["Global Options"]["Fixed Frame"] == "base_link"
    assert current["Name"] == "Mobile Manipulation"
    assert current["Target Frame"] == "base_link"
    assert current["Distance"] == 3.0
    assert current["Focal Point"] == {"X": 0.45, "Y": 0.0, "Z": 0.75}


def test_topic_contract_is_absolute_parameterized_and_contains_no_gt():
    contract = json.loads(CONTRACT_PATH.read_text())
    assert contract["schema_version"] == 1
    assert contract["fixed_frame"] == "map"
    assert all(topic.startswith("/") for topic in contract["topics"].values())
    assert "ground_truth" not in CONTRACT_PATH.read_text().lower()

    builder_text = BUILDER_PATH.read_text()
    for environment_name in (
        "GO2W_RVIZ_WRIST_RGB_TOPIC",
        "GO2W_RVIZ_RAW_LIDAR_TOPIC",
        "GO2W_RVIZ_ALIGNED_DEPTH_TOPIC",
        "GO2W_RVIZ_TARGET_CLOUD_TOPIC",
        "GO2W_RVIZ_SCENE_CLOUD_TOPIC",
        "GO2W_RVIZ_GRASP_CANDIDATES_TOPIC",
        "GO2W_RVIZ_PLANNED_TCP_PATH_TOPIC",
    ):
        assert environment_name in builder_text


def test_only_standard_jazzy_display_plugins_are_required():
    builder = _load_builder()
    result = builder.augment_config(_minimal_stock(), builder.load_contract(CONTRACT_PATH))
    classes = {
        display["Class"]
        for display in _flatten(result["Visualization Manager"]["Displays"])
        if display["Name"] != "Navigation map"
    }
    assert classes <= {
        "rviz_common/Group",
        "rviz_default_plugins/CameraInfo",
        "rviz_default_plugins/Image",
        "rviz_default_plugins/Marker",
        "rviz_default_plugins/MarkerArray",
        "rviz_default_plugins/Path",
        "rviz_default_plugins/PointCloud2",
        "rviz_default_plugins/Pose",
        "rviz_default_plugins/RobotModel",
        "rviz_default_plugins/TF",
    }


def test_runtime_bridge_is_visualization_only_and_supervised():
    bridge = (ROOT / "scripts/nav/manip_rviz_bridge.py").read_text()
    supervisor = (ROOT / "scripts/nav/run_all_forever.sh").read_text()
    sync = (ROOT / "scripts/nav/sync_navstack_files.sh").read_text()
    assert "ground_truth" not in bridge.lower()
    assert "create_publisher" in bridge
    assert "create_subscription" in bridge
    assert "Twist" not in bridge
    assert "manip_rviz_bridge.py" in supervisor
    assert "manip_rviz_bridge.py" in sync


def test_builder_writes_loadable_yaml(tmp_path):
    builder = _load_builder()
    stock = tmp_path / "stock.rviz"
    output = tmp_path / "combined.rviz"
    stock.write_text(yaml.safe_dump(_minimal_stock()))
    builder.build(stock, CONTRACT_PATH, output)
    parsed = yaml.safe_load(output.read_text())
    assert parsed["Visualization Manager"]["Global Options"]["Fixed Frame"] == "base_link"
