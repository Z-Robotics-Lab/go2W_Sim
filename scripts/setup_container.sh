#!/usr/bin/env bash
# 创建并初始化 go2w-isaac 容器（Isaac Sim 5.1 + Isaac Lab v2.3.2 + robot_lab v2.3.2）。
# 前置：docker + nvidia-container-toolkit + 本地已有 nvcr.io/nvidia/isaac-sim:5.1.0 镜像；
#       先运行 scripts/clone_deps.sh。
# 网络差时：先用 scripts/fetch_torch_wheels.sh 断点续传 torch/torchvision 到 wheels/，
#           本脚本检测到后会离线安装，跳过 pip 在线下载 1GB 大包。
set -euo pipefail
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$REPO_DIR"/docker-cache/{kit,ov,pip,glcache,computecache,logs,data} "$REPO_DIR"/logs

docker rm -f go2w-isaac 2>/dev/null || true
docker run -d --name go2w-isaac --entrypoint bash \
  --gpus all --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES \
  --memory=40g \
  -v "$REPO_DIR":/workspace/go2w \
  -v "$REPO_DIR"/docker-cache/kit:/isaac-sim/kit/cache \
  -v "$REPO_DIR"/docker-cache/ov:/root/.cache/ov \
  -v "$REPO_DIR"/docker-cache/pip:/root/.cache/pip \
  -v "$REPO_DIR"/docker-cache/glcache:/root/.cache/nvidia/GLCache \
  -v "$REPO_DIR"/docker-cache/computecache:/root/.nv/ComputeCache \
  -v "$REPO_DIR"/docker-cache/logs:/root/.nvidia-omniverse/logs \
  -v "$REPO_DIR"/docker-cache/data:/root/.local/share/ov/data \
  nvcr.io/nvidia/isaac-sim:5.1.0 -c "sleep infinity"

# 坑 1：kit python 缺 setuptools（pkg_resources），源码构建的包会挂
# 坑 2：isaaclab.sh 的 pip 自升级会把运行中的 pip 弄成新旧混合损坏态 —— 先装干净的最新 pip
docker exec -u 0 go2w-isaac bash -c "
  curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py &&
  SP=/isaac-sim/kit/python/lib/python3.11/site-packages &&
  rm -rf \$SP/pip \$SP/pip-*.dist-info &&
  /isaac-sim/kit/python/bin/python3 /tmp/get-pip.py &&
  /isaac-sim/python.sh -m pip install 'setuptools>=70' wheel"

# torch 离线安装（wheels/ 里有就用，没有则交给 isaaclab.sh 在线装）
if ls "$REPO_DIR"/wheels/torch-2.7.0+cu128-*.whl >/dev/null 2>&1; then
  docker exec -u 0 go2w-isaac bash -c \
    "/isaac-sim/python.sh -m pip install --no-index /workspace/go2w/wheels/torch-2.7.0+cu128-*.whl /workspace/go2w/wheels/torchvision-0.22.0+cu128-*.whl"
fi

# Isaac Lab（rsl_rl 方案）+ robot_lab
# 坑 3：TERM=dumb 会让 isaaclab.sh 直接退出，必须给正常终端类型
docker exec -u 0 go2w-isaac bash -c "
  ln -sfn /isaac-sim /workspace/go2w/IsaacLab/_isaac_sim &&
  cd /workspace/go2w/IsaacLab && TERM=xterm ./isaaclab.sh --install rsl_rl &&
  /isaac-sim/python.sh -m pip install -e /workspace/go2w/robot_lab/source/robot_lab"

# 挂载目录里 root 写入的文件还给宿主用户
docker exec -u 0 go2w-isaac bash -c "chown -R $(id -u):$(id -g) /workspace/go2w" || true

docker exec go2w-isaac bash -c \
  "/isaac-sim/python.sh -c 'import isaaclab, isaaclab_tasks, robot_lab; print(\"setup OK\")'"
