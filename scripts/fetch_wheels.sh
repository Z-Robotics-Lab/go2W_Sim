#!/usr/bin/env bash
# 断点续传预下载安装需要的大 wheel（不稳定网络下 pip 在线下载必断流且不能续传）。
# torch 不需要下载 —— Isaac Sim 镜像 prebundle 自带 2.7.0+cu128。
set -uo pipefail
cd "$(dirname "$0")/.." && mkdir -p wheels && cd wheels

URLS=(
  "https://mirrors.aliyun.com/pypi/packages/3c/c5/4874a81131cc9e934d88377fbc9d24319ae1fb540f3333b4e9c696ebc607/triton-3.3.0-cp311-cp311-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl"
  "https://mirrors.aliyun.com/pypi/packages/9a/32/784829665b0cc6fadada337f103c811b7bf92a951a69c08f7e3cd6ab2580/warp_lang-1.14.0-py3-none-manylinux_2_28_x86_64.whl"
)
for u in "${URLS[@]}"; do
  for i in $(seq 1 200); do
    wget -c -q --timeout=60 --tries=1 "$u" && break
    echo "retry $i: $(basename "$u")"; sleep 5
  done
done
for w in *.whl; do
  python3 -m zipfile -t "$w" >/dev/null && echo "OK: $w" || { echo "CORRUPT: $w"; exit 1; }
done
