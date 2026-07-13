import argparse
import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/nav/ros_stream_gate.py"
SPEC = importlib.util.spec_from_file_location("ros_stream_gate_logic", MODULE_PATH)
GATE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GATE
SPEC.loader.exec_module(GATE)


def policies():
    return {
        "clock": GATE.StreamPolicy(2, 0.25),
        "imu": GATE.StreamPolicy(2, 0.25, 0.25),
        "raw_cloud": GATE.StreamPolicy(2, 0.25, 0.25, True),
        "custom_scan": GATE.StreamPolicy(2, 1.0, 1.0, True),
        "regscan": GATE.StreamPolicy(2, 1.5, 1.5, True),
        "state": GATE.StreamPolicy(2, 0.5, 0.5, False, ("map", "sensor")),
        "odom": GATE.StreamPolicy(2, 0.5, 0.5, False, ("map", "base_link")),
    }


def healthy_stats(*, wall_base=1_000.0):
    stats = {name: GATE.StreamStats() for name in GATE.TOPICS}
    for stamp_s in (10.0, 10.1):
        stamp = int(stamp_s * 1e9)
        for name, stream in stats.items():
            # Non-clock arrivals may be arbitrarily slow in wall time at low RTF;
            # their source stamps remain close to the simulation clock.
            wall = wall_base if name != "clock" else wall_base + stamp_s - 10.0
            stream.observe(stamp, wall, nonempty=True, valid_frames=True)
    return stats


def evaluate(stats, *, now_wall=1_000.2):
    publishers = {topic: 1 for topic in GATE.CRITICAL_TOPICS}
    return GATE.evaluate_streams(
        stats,
        policies(),
        publishers,
        now_wall_s=now_wall,
        max_future_skew_s=0.1,
        max_clock_wall_age_s=2.0,
    )


def test_healthy_streams_pass_even_at_low_real_time_factor():
    stats = healthy_stats(wall_base=100.0)
    # Only the clock's final wall arrival is relevant. Non-clock messages can
    # have old wall arrivals as long as they remain fresh in simulation time.
    for name, stream in stats.items():
        if name != "clock":
            stream.last_wall_s = 1.0

    result = evaluate(stats, now_wall=100.2)

    assert result["ok"], result["failures"]


def test_burst_then_stall_is_rejected_by_sim_time_age():
    stats = healthy_stats()
    stats["clock"].observe(int(12.0e9), 1_000.2)

    result = evaluate(stats, now_wall=1_000.3)

    assert not result["ok"]
    assert any("imu_sim_age_s=" in failure for failure in result["failures"])
    assert any("regscan_sim_age_s=" in failure for failure in result["failures"])


def test_duplicate_timestamp_is_rejected():
    stats = healthy_stats()
    stats["imu"].observe(int(10.1e9), 1_000.15)

    result = evaluate(stats)

    assert not result["ok"]
    assert "imu_duplicate_stamps=1" in result["failures"]


def test_future_dated_stream_is_rejected():
    stats = healthy_stats()
    stats["odom"].observe(int(10.25e9), 1_000.15, valid_frames=True)

    result = evaluate(stats)

    assert not result["ok"]
    assert any("odom_future_skew_s=" in failure for failure in result["failures"])


def test_excessive_adjacent_sim_time_gap_is_rejected():
    stats = healthy_stats()
    stats["custom_scan"].observe(int(11.2e9), 1_000.15, nonempty=True)
    stats["clock"].observe(int(11.2e9), 1_000.2)

    result = evaluate(stats, now_wall=1_000.3)

    assert not result["ok"]
    assert any("custom_scan_max_gap_s=" in failure for failure in result["failures"])


def test_frozen_clock_is_rejected_by_wall_freshness():
    stats = healthy_stats()

    result = evaluate(stats, now_wall=1_003.0)

    assert not result["ok"]
    assert any("clock_wall_age_s=" in failure for failure in result["failures"])


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf", "-inf"])
def test_thresholds_reject_nonpositive_or_nonfinite_values(value):
    with pytest.raises(argparse.ArgumentTypeError):
        GATE.positive_float(value)
