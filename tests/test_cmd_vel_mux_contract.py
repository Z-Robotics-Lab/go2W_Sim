from decimal import Decimal
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/nav/cmd_vel_mux_params.yaml"
SYNC = ROOT / "scripts/nav/sync_navstack_files.sh"


def _subscribers():
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return document["cmd_vel_mux"]["ros__parameters"]["subscribers"]


def test_priorities_are_unique_and_safety_preempts_every_other_input():
    subscribers = _subscribers()
    priorities = [entry["priority"] for entry in subscribers.values()]

    assert len(priorities) == len(set(priorities))
    assert subscribers["safety"]["priority"] == max(priorities)
    assert all(
        subscribers["safety"]["priority"] > entry["priority"]
        for name, entry in subscribers.items()
        if name != "safety"
    )


def test_safety_and_local_topics_have_jitter_tolerant_timeouts():
    subscribers = _subscribers()
    ten_hz_period_seconds = Decimal("0.1")

    assert subscribers["safety"]["topic"] == "/safety_cmd_vel"
    assert subscribers["local_movement"]["topic"] == "/local_movement_cmd_vel"
    assert subscribers["safety"]["timeout"] == 0.3
    assert subscribers["local_movement"]["timeout"] == 0.3
    assert Decimal(str(subscribers["safety"]["timeout"])) >= 3 * ten_hz_period_seconds
    assert (
        Decimal(str(subscribers["local_movement"]["timeout"]))
        >= 3 * ten_hz_period_seconds
    )


def test_sync_deploys_tracked_contract_to_navstack_source_tree():
    sync = SYNC.read_text(encoding="utf-8")

    assert 'MUX_CONTRACT="$HERE/../../configs/nav/cmd_vel_mux_params.yaml"' in sync
    assert (
        'MUX_RUNTIME="$NAV/src/utilities/cmd_vel_mux/config/'
        'cmd_vel_mux_params.yaml"'
    ) in sync
    assert 'install -D -m 0644 "$MUX_CONTRACT" "$MUX_RUNTIME"' in sync
