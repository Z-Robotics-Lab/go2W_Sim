# 踩坑清单（scripts 已内置规避；改任何脚本前必读）

> 从 README 迁出的完整坑表。每条都是实测踩过的，新坑继续追加编号。


1. pip 直装 isaacsim 不可行（pypi.nvidia.com 大 wheel 断流且 pip 不能续传）→ 用 Docker 镜像。
2. kit python 缺 setuptools（pkg_resources）→ setup 脚本已补。
3. pip 运行中自升级会把 pip 弄成新旧混合损坏态 → get-pip 全新重装。
4. `TERM=dumb` 会让 isaaclab.sh 直接退出 → 一律 `TERM=xterm`。
5. **不要用 isaaclab.sh --install**：会强制在线下载 torch → 手动 pip 装六个源码包。
6. **绝不 pip install/uninstall torch**：镜像 prebundle 自带 torch/torchvision/torchaudio
   2.7.0+cu128（CUDA 配好）；pip 碰它必坏（亲测卸载会把 prebundle 拆成空壳）。
7. 大 wheel（triton 150M/warp 133M）用 `wget -c` 预下载再本地装；aria2 多连接会被
   阿里云 403。
8. **rsl-rl-lib 必须钉 3.1.2**（isaaclab_rl 官方 pin）；装最新 5.x 报 `KeyError: 'actor'`。
9. 容器缺 git → rsl_rl 日志器 `ImportError: Bad git executable`。
10. **headless + --enable_cameras 会静默死**（两次复现：卡在启动 10 秒处然后无声消失）
    → 截图/迭代一律用 GUI 模式（`run_gui.sh --screenshot ...`）。
11. 容器内 Python 输出有缓冲，日志看不到 print → `PYTHONUNBUFFERED=1`。
12. 轮关节命名：URDF/USD 是 `*_foot_joint`，MuJoCo 官方模型是 `*_wheel_joint`（sim2sim 要映射）。
13. mid360.stl 是毫米单位+角点原点（scale 0.001 + 平移居中）；d435.dae 是米单位
    （姿态沿用 realsense2_description 官方 xacro）。
14. GitHub 上不少 mesh 走 Git-LFS，raw 链接只给指针 → 用 media.githubusercontent.com。
15. 一次只跑一个 sim 实例（GPU/内存都吃紧）；重启场景前 `pkill -9 -f "kit/pytho[n]"`。
16. **GUI 模式下 Kit 时间线 = 物理时间 × (rendering_dt/物理dt)**：每个物理步都触发 app
    更新且时间线前进 rendering_dt。必须 `render_interval=1` 且 dt 一致，否则 RTX 雷达
    时间戳跑倍速，SLAM 直接发散（实测 2 倍速 → z 漂到 -7.5m）。
17. RTX 雷达 helper 按扫描**完成时刻**打帧戳：转 CustomMsg 时必须回溯一个扫描周期
    （否则每帧都"来自未来"，SLAM 等 IMU 覆盖等到 buffer 爆）。实测旋转周期 0.2 sim-s
    （配置写 10Hz 也一样，内部系数），转换器按 0.2s 铺 offset_time。
18. isaaclab 的 `Imu.gravity_bias` 是本体系加常量，只对水平安装正确——斜装传感器必须
    自己按姿态投影重力（quat_apply_inverse），否则 SLAM 拿到错误重力方向。
19. 物理 100Hz 时腿部 PD 60/2 站不稳会摔（截图实锤）；100/5 稳。
20. DDS 必须隔离域（本仓库约定 ROS_DOMAIN_ID=42）：域 0 会和主机上其他 ROS 项目串台。
21. 手动 cmake install 到 /usr/local 后要 `ldconfig`，否则节点起不来（libgtsam 找不到）。
22. 编排脚本别用 `set -u`（ROS setup.bash 有 unbound 变量，直接静默死）。
23. **宿主机上其他项目的定时清理会跨命名空间杀容器内同 uid 进程**（本机实证：
    vector_os_nano 循环每轮末的 `rosm nuke` 定点清除我们 navstack 里的 ROS 进程，
    时间戳 ±1s 吻合）。防御：关键容器用 root 运行（`--user 0`，非 root 清理器 EPERM）+
    PID-1 supervisor 自愈（run_all_forever.sh）+ 配对重启脚本（restart_all.sh）。
24. fastdds 大消息（900KB 点云）+ 频繁进程死亡环境：禁 SHM 强制 UDP
    （FASTDDS_BUILTIN_TRANSPORTS=UDPv4），/dev/shm 僵尸段会让 SHM 静默瘫痪。
25. **自检必须与生产数据通路隔离**：navstack 的 pathFollower 无目标时持续发
    cmd_vel=0，会把自检的注入指令每帧覆盖（第 7 轮自检取证）。
26. 手搓四轮差速在 Go2W 上物理不可行（轴距>轮距的定轴四轮滑移转向：3s 只转 2-4°）。
    正解 = robot_lab 训练速度跟踪 RL 策略（真机 wheeled_sport 的仿真等价物）。
27. isaaclab `Imu` 的 OffsetCfg.rot 不作用于测量值——斜装传感器的水平化在发布器里
    用常量旋转自己做。
28. **navstack:ready 镜像不含 tare_planner**（原始编译只做了 21 个包）——NAV_MODE=explore
    首启会 PackageNotFoundError 崩溃循环、连带 SLAM 起不来。补编一次即可（or-tools 是
    vendored 预编译库，3 秒）：bringup.sh 已加前置检查并打印补编命令。
29. **RViz 的 TeleopPanel 会误杀自主模式**：它发 sensor_msgs/Joy 到 /joy，而 pathFollower
    的 joystickHandler 按“手柄扳机”语义解读——axes[2] 不压死(-1) 就 autonomyMode=false。
    鼠标点面板产生的轴值与扳机语义不匹配 → 一点就可能把 agent/TARE 的自主导航踢掉且难恢复
    （恢复需要一条 axes[2]<-0.1 的 /joy）。agent 跑任务时不要碰 TeleopPanel；sim 里人工
    遥控不在产品路径（真机人工接管=宇树遥控器+官方步态，绕开本栈）。
30. **探索完成后 nav_owner 卡死**：TARE finished 后不再发航点，桥的 explore 占用若不
    释放，手动导航被 409 锁到永远。已修：/exploration_finish=true 时 owner 自动归 idle。
31. **僵尸桥（护城河级）**：rclpy.init 会接管 SIGTERM/SIGINT——若 rclpy.spin 跑在子线程，
    kill/pkill 只杀 ROS 线程，HTTP 主线程带冻结 STATE 继续 200 应答（实测 55s 陈旧位姿），
    verify 谓词可能读到陈旧 GT。已修：spin 回主线程（信号=全进程退出=supervisor 重生）
    + /pose /gt 超 5s 未更新返回 503。教训：rclpy 进程的存活判定永远看数据新鲜度，
    不看进程在不在。
