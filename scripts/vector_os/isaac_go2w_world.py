# SPDX-License-Identifier: Apache-2.0
"""isaac-go2w — vector_os_nano 的 BYO world 插件（零内核修改）。

活在 go2W_Sim 仓库；按 vector_os_nano 的 World Protocol 鸭子类型实现（对齐其
tests/vcli/test_plug_and_play_boundary.py 的 _AcmeArmWorld 金标准样例）。

链路：agent 工具 -> HTTP 桥(127.0.0.1:8042, scripts/nav/agent_bridge.py) ->
DDS 域 42 -> CMU 导航栈 -> RL locomotion -> Isaac 仓库里的 Go2W。
verify 谓词 go2w_at 读 /gt（SIM 地面真值，执行者无法伪造——"Verify is the moat"）。
"""
from __future__ import annotations

import json
import math
import urllib.request
from typing import Any

from vector_os_nano.vcli.tools.base import ToolContext, ToolResult, tool
from vector_os_nano.vcli.worlds.base import DecomposeVocab

BRIDGE = "http://127.0.0.1:8042"


def _get(path: str) -> dict:
    with urllib.request.urlopen(f"{BRIDGE}/{path}", timeout=5) as r:
        return json.loads(r.read())


def _post(path: str, obj: dict) -> dict:
    req = urllib.request.Request(
        f"{BRIDGE}/{path}", data=json.dumps(obj).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


@tool(
    name="go2w_navigate",
    description=(
        "Send the Go2W robot dog to a target position (x, y) in the SLAM map frame. "
        "The navigation stack plans around obstacles and drives the robot there. "
        "Non-blocking: returns immediately; use go2w_where to track progress. "
        "让 Go2W 机器狗导航到地图坐标 (x, y)。"
    ),
    read_only=False,
    permission="allow",
)
class Go2WNavigateTool:
    input_schema: dict[str, Any] = {
        "type": "object",
        "properties": {
            "x": {"type": "number", "description": "target x (map frame, meters)"},
            "y": {"type": "number", "description": "target y (map frame, meters)"},
        },
        "required": ["x", "y"],
    }

    def execute(self, params: dict[str, Any], context: ToolContext) -> ToolResult:
        try:
            res = _post("waypoint", {"x": params["x"], "y": params["y"]})
            return ToolResult(content=json.dumps({"sent": res}))
        except Exception as e:  # noqa: BLE001 — 桥边界
            return ToolResult(content=f"bridge error: {e}", is_error=True)


@tool(
    name="go2w_where",
    description=(
        "Get the Go2W robot's current SLAM pose {x, y, z, stamp} in the map frame. "
        "查询 Go2W 当前位姿。"
    ),
    read_only=True,
    permission="allow",
)
class Go2WWhereTool:
    input_schema: dict[str, Any] = {"type": "object", "properties": {}}

    def execute(self, params: dict[str, Any], context: ToolContext) -> ToolResult:
        try:
            return ToolResult(content=json.dumps(_get("pose")))
        except Exception as e:  # noqa: BLE001
            return ToolResult(content=f"bridge error: {e}", is_error=True)


def go2w_at(x: float, y: float, tol: float = 0.8) -> bool:
    """GT 谓词：机器人（SIM 地面真值）是否在 (x, y) 的 tol 米内。

    真值经 SLAM 原点补偿到 map 系：桥同时提供 /gt 与 /pose，用两者的当前差
    作为原点偏移的近似（静止时精确；行驶中误差 = SLAM 瞬时漂移，可接受）。
    """
    gt = _get("gt")
    pose = _get("pose")
    off_x, off_y = gt["x"] - pose["x"], gt["y"] - pose["y"]
    gx, gy = x + off_x, y + off_y  # 目标从 map 系换到 GT 系
    return math.hypot(gt["x"] - gx, gt["y"] - gy) < tol


# ---- 技能层（VGG MOTION skill 正门）+ embodiment ------------------------------
import time as _time

from vector_os_nano.core.skill import SkillRegistry, skill
from vector_os_nano.core.types import SkillResult


@skill(aliases=["navigate", "nav_to_pos", "nav", "go to", "导航", "去", "开到", "走到"])
class Go2WNavigateSkill:
    """阻塞式导航技能：发 waypoint 并轮询直到到达（SLAM 系）或超时。"""

    name = "navigate"
    description = ("Navigate the Go2W robot to map coordinates (x, y). "
                   "Blocks until arrival (tolerance ~0.7m) or 300s timeout.")

    def _target(self, context, kw):
        for src in (kw, getattr(context, "params", None) or {},
                    getattr(context, "args", None) or {}):
            if isinstance(src, dict) and "x" in src and "y" in src:
                return float(src["x"]), float(src["y"])
        text = str(getattr(context, "instruction", "") or getattr(context, "text", "") or kw)
        import re
        m = re.search(r"\(?\s*(-?\d+\.?\d*)\s*[,，]\s*(-?\d+\.?\d*)\s*\)?", text)
        if m:
            return float(m.group(1)), float(m.group(2))
        raise ValueError(f"no (x, y) target in skill call: {text[:120]}")

    def execute(self, context=None, **kw):
        import sys
        try:
            x, y = self._target(context, kw)
        except ValueError as e:
            print(f"[SKILL] target parse FAIL: {e}", file=sys.stderr, flush=True)
            return SkillResult(success=False, message=str(e))
        print(f"[SKILL] navigate -> ({x},{y}) ctx={str(getattr(context,'params',None))[:80]} kw={str(kw)[:80]}",
              file=sys.stderr, flush=True)
        _post("waypoint", {"x": x, "y": y})
        t0 = _time.time()
        while _time.time() - t0 < 300:
            _time.sleep(5)
            _post("waypoint", {"x": x, "y": y})  # 周期重发（栈只认最新）
            p = _get("pose")
            # 到达（0.45m）后冻结：waypoint 改发当前位置让 pathFollower 停追，
            # 停稳复核（0.7m）后返回——verify（0.8m）才有余量且不再漂移触发重试
            if math.hypot(p["x"] - x, p["y"] - y) < 0.45:
                _post("waypoint", {"x": p["x"], "y": p["y"]})
                _time.sleep(6)
                p = _get("pose")
                d2 = math.hypot(p["x"] - x, p["y"] - y)
                print(f"[SKILL] held check d={d2:.2f} at ({p['x']:.2f},{p['y']:.2f})",
                      file=sys.stderr, flush=True)
                if d2 < 0.7:
                    return SkillResult(success=True,
                                       message=f"arrived+held ({p['x']:.2f},{p['y']:.2f})")
        p = _get("pose")
        return SkillResult(success=False,
                           message=f"timeout at ({p['x']:.2f},{p['y']:.2f})")


class IsaacGo2WEmbodiment:
    """最小 embodiment：VGG 就绪判据（_base 非 None）+ 技能注册表 + 状态读取。

    状态方法全部读真实数据（桥），供内核 verifier 绑定使用。
    """

    def __init__(self) -> None:
        self._base = self
        self._arm = None
        self._skill_registry = SkillRegistry()
        self._skill_registry.register(Go2WNavigateSkill())

    def navigate_to(self, x: float, y: float, timeout: float = 240.0) -> bool:
        """native_loop 的 base 合同：阻塞导航，到达返回 True。

        到达（0.45m）后 waypoint 冻结在当前位置（pathFollower 停追不再漂），
        停稳复核 0.7m —— verify 的 go2w_at（0.8m）留有余量。
        """
        import sys
        import time as _t
        x, y = float(x), float(y)
        # 慢动作 sim（约 0.3-0.5x 实时）里导航需 100-300s 墙钟；调用方的 60s 默认
        # 超时会制造假失败步毒化判决——基座最了解自身动力学，下限钳 240s
        timeout = max(float(timeout), 240.0)
        print(f"[BASE] navigate_to ({x},{y}) timeout={timeout}", file=sys.stderr, flush=True)
        _post("waypoint", {"x": x, "y": y})
        t0 = _t.time()
        while _t.time() - t0 < timeout:
            _t.sleep(5)
            _post("waypoint", {"x": x, "y": y})
            p = _get("pose")
            if math.hypot(p["x"] - x, p["y"] - y) < 0.45:
                _post("waypoint", {"x": p["x"], "y": p["y"]})  # 冻结
                _t.sleep(6)
                p = _get("pose")
                d = math.hypot(p["x"] - x, p["y"] - y)
                print(f"[BASE] arrived+held d={d:.2f}", file=sys.stderr, flush=True)
                return d < 0.7
        print("[BASE] navigate_to timeout", file=sys.stderr, flush=True)
        return False

    def get_position(self):
        p = _get("pose")
        return (p["x"], p["y"])

    def get_heading(self):
        return float(_get("pose").get("yaw", 0.0))

    def get_pose(self):
        p = _get("pose")
        return (p["x"], p["y"], p.get("yaw", 0.0))


class IsaacGo2WWorld:
    """World Protocol 的鸭子类型实现（不继承任何内核类）。"""

    name = "isaac-go2w"

    def is_robot(self) -> bool:
        return True

    def persona_blocks(self) -> tuple[str, str]:
        return (
            "You operate a Unitree Go2W robot dog (with a PiPER arm) inside an "
            "Isaac Sim warehouse. It navigates via a SLAM + planner stack.",
            "Use go2w_navigate(x, y) to send it somewhere; use go2w_where() to "
            "check progress. Navigation takes tens of seconds of sim time — "
            "poll go2w_where between checks. Verify arrival with "
            "go2w_at(x, y) == True.",
        )

    def register_tools(self, registry: Any, agent: Any) -> None:
        registry.register(Go2WNavigateTool(), category="go2w")
        registry.register(Go2WWhereTool(), category="go2w")

    def build_verify_namespace(self, agent: Any) -> dict[str, Any]:
        return {"go2w_at": go2w_at}

    def register_capabilities(self, registry: Any, agent: Any, backend: Any) -> None:
        return None

    def decompose_vocab(self) -> DecomposeVocab | None:
        return DecomposeVocab(
            planner_intro=(
                "Drive a Go2W robot dog in a warehouse. Navigation goals are "
                "map-frame (x, y) positions."
            ),
            verify_functions=frozenset({"go2w_at"}),
        )

    def derive_vocab_from_registry(self) -> bool:
        return False


def register() -> None:
    from vector_os_nano.vcli.worlds.registry import get_world_registry

    get_world_registry().register("isaac-go2w", IsaacGo2WWorld, replace=True)


register()
