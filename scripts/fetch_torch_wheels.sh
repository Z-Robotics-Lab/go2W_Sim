#!/usr/bin/env bash
# 断点续传下载 torch/torchvision cu128 wheel 到 wheels/（对付不稳定网络：
# pip 不支持续传，1GB 的 torch 一断就前功尽弃；wget -c 可以无限接力）。
# 下载完成后做 zip CRC 完整性校验。
set -uo pipefail
cd "$(dirname "$0")/.." && mkdir -p wheels && cd wheels

BASE="https://download.pytorch.org/whl/cu128"   # 国内可换 https://mirrors.aliyun.com/pytorch-wheels/cu128
FILES=(
  "torch-2.7.0%2Bcu128-cp311-cp311-manylinux_2_28_x86_64.whl"
  "torchvision-0.22.0%2Bcu128-cp311-cp311-manylinux_2_28_x86_64.whl"
)

for f in "${FILES[@]}"; do
  for i in $(seq 1 300); do
    wget -c -q --timeout=60 --tries=1 "$BASE/$f" && break
    echo "retry $i: $f"; sleep 5
  done
done

for w in *.whl; do
  python3 -m zipfile -t "$w" >/dev/null && echo "OK: $w" || { echo "CORRUPT: $w"; exit 1; }
done
