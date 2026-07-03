#!/usr/bin/env bash
# 创建并初始化 go2w-isaac 容器（Isaac Sim 5.1 + Isaac Lab v2.3.2 + robot_lab v2.3.2）。
# 前置：docker + nvidia-container-toolkit + 本地已有 nvcr.io/nvidia/isaac-sim:5.1.0 镜像；
#       先跑 scripts/clone_deps.sh 和 scripts/fetch_wheels.sh（大 wheel 断点续传预下载）。
# 全部安装完成后建议固化：docker commit go2w-isaac go2w-isaac:ready
set -euo pipefail
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$REPO_DIR"/docker-cache/{kit,ov,pip,glcache,computecache,logs,data} "$REPO_DIR"/logs

IMAGE="${GO2W_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"   # 已有 go2w-isaac:ready 时可覆盖

docker rm -f go2w-isaac 2>/dev/null || true
docker run -d --name go2w-isaac --entrypoint bash \
  --gpus all --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES \
  -e DISPLAY="${DISPLAY:-:0}" \
  --memory=40g \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$REPO_DIR":/workspace/go2w \
  -v "$REPO_DIR"/docker-cache/kit:/isaac-sim/kit/cache \
  -v "$REPO_DIR"/docker-cache/ov:/root/.cache/ov \
  -v "$REPO_DIR"/docker-cache/pip:/root/.cache/pip \
  -v "$REPO_DIR"/docker-cache/glcache:/root/.cache/nvidia/GLCache \
  -v "$REPO_DIR"/docker-cache/computecache:/root/.nv/ComputeCache \
  -v "$REPO_DIR"/docker-cache/logs:/root/.nvidia-omniverse/logs \
  -v "$REPO_DIR"/docker-cache/data:/root/.local/share/ov/data \
  "$IMAGE" -c "sleep infinity"

# 已是就绪镜像则跳过安装
if [ "$IMAGE" != "nvcr.io/nvidia/isaac-sim:5.1.0" ]; then
  echo "使用就绪镜像 $IMAGE，跳过安装"; exit 0
fi

# 系统工具（rsl_rl 日志器需要 git；aria2 备用）+ 国内 pip 镜像
docker exec -u 0 go2w-isaac bash -c "
  apt-get update -q && apt-get install -y -q git aria2 &&
  printf '[global]\nindex-url = https://mirrors.aliyun.com/pypi/simple/\ntimeout = 60\nretries = 10\n' > /etc/pip.conf"

# 坑 1：kit python 缺 setuptools（pkg_resources）
# 坑 2：pip 运行中自升级会留下新旧混合的损坏 pip —— 删干净重装
docker exec -u 0 go2w-isaac bash -c "
  curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py &&
  SP=/isaac-sim/kit/python/lib/python3.11/site-packages &&
  rm -rf \$SP/pip \$SP/pip-*.dist-info &&
  /isaac-sim/kit/python/bin/python3 /tmp/get-pip.py -q &&
  /isaac-sim/python.sh -m pip install -q 'setuptools>=70' wheel"

# 坑 3（最大坑）：绝对不要 pip 安装/卸载 torch —— 镜像 prebundle 自带
# torch/torchvision/torchaudio 2.7.0+cu128（CUDA 配好）；pip 动它必坏。
# 大依赖 triton/warp 用预下载的 wheel 本地装，避免 pip 在线下大文件断流：
if ls "$REPO_DIR"/wheels/triton-*.whl >/dev/null 2>&1; then
  docker exec -u 0 go2w-isaac bash -c \
    "/isaac-sim/python.sh -m pip install --no-deps /workspace/go2w/wheels/triton-*.whl /workspace/go2w/wheels/warp_lang-*.whl"
fi

# 坑 4：不要用 isaaclab.sh --install（会强制在线下载 torch + 自升级 pip）。
# 手动装六个源码包（--no-build-isolation 用已装 setuptools，避免联网拉构建依赖）：
docker exec -u 0 go2w-isaac bash -c "
  set -e; ln -sfn /isaac-sim /workspace/go2w/IsaacLab/_isaac_sim
  cd /workspace/go2w/IsaacLab/source
  for pkg in isaaclab isaaclab_assets isaaclab_contrib isaaclab_rl isaaclab_tasks isaaclab_mimic; do
    /isaac-sim/python.sh -m pip install --no-build-isolation -e \$pkg
  done
  # 坑 5：rsl-rl-lib 必须钉 3.1.2（isaaclab_rl 官方 pin；装最新 5.x 会 KeyError 'actor'）
  /isaac-sim/python.sh -m pip install --no-build-isolation 'rsl-rl-lib==3.1.2'
  /isaac-sim/python.sh -m pip install --no-build-isolation -e /workspace/go2w/robot_lab/source/robot_lab"

docker exec -u 0 go2w-isaac bash -c "chown -R $(id -u):$(id -g) /workspace/go2w" || true
docker exec go2w-isaac bash -c \
  "/isaac-sim/python.sh -c 'from isaaclab.app import AppLauncher; import torch; print(\"setup OK, cuda:\", torch.cuda.is_available())'"
echo "建议固化: docker commit go2w-isaac go2w-isaac:ready"
