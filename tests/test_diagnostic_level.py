import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "scripts/nav/diagnostic_level.py"
)


def _load_module():
    spec = importlib.util.spec_from_file_location("diagnostic_level", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("level", "expected"),
    [
        (0, 0),
        (3, 3),
        (b"\x00", 0),
        (b"\x03", 3),
        (bytearray([2]), 2),
        (memoryview(b"\x01"), 1),
    ],
)
def test_diagnostic_level_accepts_integer_and_jazzy_bytes(level, expected):
    assert _load_module().diagnostic_level_to_int(level) == expected


@pytest.mark.parametrize("level", [b"", b"\x00\x01", -1, 256, "2"])
def test_diagnostic_level_rejects_malformed_values(level):
    with pytest.raises((TypeError, ValueError)):
        _load_module().diagnostic_level_to_int(level)
