#!/usr/bin/env python3
"""生成 Livox Mid-360 的 RTX Lidar 近似配置 JSON（Isaac Sim 5.x 转换器 schema）。

近似方式：rotary 扫描、64 束覆盖 [-7°, +52°]、reportRate 3125Hz x 64 = 20 万点/秒
（对齐真机吞吐）、0.5° 角度抖动模拟非重复扫描的覆盖特性。数值来自 Livox 公开规格书。
生成后用官方工具转 USDA：
  /isaac-sim/python.sh /isaac-sim/tools/isaacsim.sensors.rtx/convert_lidar_json_to_usda.py \
      -f <本文件输出> -m "Livox Mid360 Approx" -n Mid360Approx -v Livox
"""
import json
from pathlib import Path

N = 64
OUT = Path(__file__).resolve().parents[2] / "assets/lidar_configs/Livox_Mid360_approx.json"

elev = [round(-7.0 + i * (52.0 - (-7.0)) / (N - 1), 3) for i in range(N)]
cfg = {
    "class": "sensor",
    "type": "lidar",
    "name": "Livox Mid360 (rotary approximation)",
    "driveWorksId": "GENERIC",
    "comment": "Mid-360 近似: 360x[-7,+52]deg, 64 beams, 200k pts/s, 70m max, "
               "0.5deg jitter approximates non-repetitive pattern. Specs from public datasheet.",
    "profile": {
        "scanType": "rotary",
        "intensityProcessing": "normalization",
        "rayType": "IDEALIZED",
        "nearRangeM": 0.1,
        "farRangeM": 70.0,
        "startAzimuthDeg": 0.0,
        "endAzimuthDeg": 360.0,
        "upElevationDeg": 52.0,
        "downElevationDeg": -7.0,
        "rangeResolutionM": 0.002,
        "rangeAccuracyM": 0.02,
        "minReflectance": 0.1,
        "minReflectanceRange": 40.0,
        "wavelengthNm": 905.0,
        "pulseTimeNs": 6,
        "azimuthErrorMean": 0.0,
        "azimuthErrorStd": 0.5,
        "elevationErrorMean": 0.0,
        "elevationErrorStd": 0.5,
        "maxReturns": 2,
        "scanRateBaseHz": 10.0,
        "reportRateBaseHz": 3125.0,
        "numberOfEmitters": N,
        "numberOfChannels": N,
        "emitterStateCount": 1,
        "emitterStates": [
            {
                "azimuthDeg": [0.0] * N,
                "elevationDeg": elev,
                "fireTimeNs": [i * 500 for i in range(N)],
                "channelId": list(range(1, N + 1)),
            }
        ],
        "intensityMappingType": "LINEAR",
    },
}
OUT.write_text(json.dumps(cfg, indent=2))
print(f"OK -> {OUT} ({N} beams, elev {elev[0]}..{elev[-1]})")
