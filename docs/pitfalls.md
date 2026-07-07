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
32. **pathFollower 速度会被任何 /joy 重写且栈内无 /speed 自愈源**：TeleopPanel 一碰
    joySpeed 锁死（甚至归零）、autonomyMode 被关。修：桥 1Hz 发 /speed=NAV_SPEED +
    每航点注入矫正 joy（axes[2]=-1 保自主）+ RViz 用去掉 TeleopPanel 的 go2w.rviz。
33. **给 Isaac 关渲染特效可能连累 RTX 雷达**（它本身是光追传感器）——FAST_RENDER 观测到
    雷达频率异常后回退为默认关闭；提速要重新单独验证雷达质量。
34. **fullScan 整帧点云的点序完全非时序**（方位角前向率 ~48%=随机，发射器状态交织），
    按索引铺 offset_time = 逐点随机时戳：静止无感、一运动 SLAM z 俯冲、地形被打空、
    空路径、cmd_vel 全零、走走停停。正解 = fullScan=False 增量模式 + 转换器按增量片
    真实时戳聚合组帧（pc2_to_livox 增量聚合版）。数字孪生级坑：真机 Livox 驱动原生
    逐点时戳，此坑仅存在于仿真侧。
35. 策略小指令特性（非坑，特征记录）：hip scale 修复后 vx 指令 0.15/0.30/0.60 跟踪
    171%/128%/107%——存在 ~0.25 m/s 输出地板；“训练死区 0.2”假设已证伪。
36. **Isaac "杀不死" 其实是"没发对信号 + 不复核"**（2026-07-06 活体僵尸取证）：用户
    报 GUI-quit / zeno-stop / rosm 都杀不死 Isaac。真因**不是** -9 无效、**不是** D 态、
    **不是** 模式失配（取证：kit-python STAT=Rl 活锁可被 -9 收割，实测一发 SIGTERM 就死；
    宿主 `pgrep -f "kit/pytho[n]"` 命中它）。真因链：
    - **GUI-quit 无效**：sim 逻辑冻结时渲染循环不服务窗口输入，quit 设不了
      `simulation_app.is_running()=False`，clean-shutdown 路径永不触发。
    - **zeno "stop" 无操作**：go2w 世界 `disable("sim")` 禁掉了内核 `stop_simulation`
      （见 z-agent go2w.py），"stop" 在此世界无 tool 可路由。唯一能发容器级 -9 的路径
      是 `go2w_bringup(action="teardown")` → `bringup.sh teardown`。
    - **那条唯一路径旧版不复核 + 静默假成功**：`pkill…||true` 恒 0 退出、status.sh 退出
      码被 `||true` 吞、zeno 层不读 returncode——"工具说关了实际没关"。
    **正确姿势**：拆栈只走 `bash scripts/nav/bringup.sh teardown`（或 zeno
    `go2w_bringup(teardown)`）。加固后它：先 `docker rm -f navstack` → kit-python
    TERM→KILL→仍活(D 态兜底)则 `docker restart go2w-isaac`（容器级重启终结整个 PID
    namespace，**保留容器**满足重建代价高）→ 逐级复核，判据 = 宿主 `pgrep -f
    "kit/pytho[n]"` 为空 且 status.sh l0=false，失败退非零 + 打印残留表。杀灭矩阵见
    `scripts/nav/teardown_matrix.md`。
37. **`rosm clean` / `rosm nuke` 对 go2w 栈无效——别用**：rosm 是 vector_os_nano 时代的
    **宿主**进程清理器，目标模式是宿主上的 MuJoCo/ROS 进程；Isaac 跑在 go2w-isaac 容器
    **内部**，navstack 跑在 navstack 容器内部（ROS_DOMAIN_ID=42、`--user 0`）。rosm 既
    定位不到容器内 kit-python（不是它的目标模式），非 root 清理器对 root 容器进程还
    EPERM。历史上 rosm 对本栈的唯一效果是**跨命名空间误伤** navstack 里同 uid 的 ROS
    进程（坑 23）——是要防的对象，不是清理工具。正确姿势同坑 36：走 `bringup.sh
    teardown`（scoped 容器内 exec / 容器级 docker，绝不宿主无差别 pkill）。
38. **手动关掉的 RViz 会被 supervisor 拉回，是自愈设计不是 bug**：RViz 挂在 navstack
    容器 PID-1 的 `run_all_forever.sh` supervisor 下、崩溃后自动重生（软能力保活）。
    直接 `docker exec navstack pkill rviz2` 或点窗口关闭 → supervisor 3s 内把它拉回。
    要真正关掉 RViz，必须走 teardown（`bringup.sh teardown` 先 `docker rm -f navstack`
    整体移除容器，supervisor 随之消亡，RViz 无处可回弹）。teardown 的 step [1/3] 先杀
    navstack 就是为此——先杀 supervisor 再动 Isaac，窗口不再弹回。
39. **Isaac 会在 sim 时间 ≈187s 自发冻结（timeline 无声停 + isaaclab step() 卡死）**
    （2026-07-06 僵尸 #1/#2 验尸，var/evidence/freeze_watch/zombie{1,2}_kit.log）：
    两具僵尸均在 **sim ≈187s / step ≈18.6k** 全话题一瞬同冻，kit 日志唯一签名 =
    `Replicator Stop`（timeline 停）且前 13 分钟零 Error、无人交互——确定性累积触发。
    机制：IsaacLab `simulation_context.py step()` L567 `while not self.is_playing():
    self.render()` —— timeline 一停/暂停，主循环在此**热旋**（527% CPU、不发布、不
    spin rclpy），且循环判 `is_playing` 而非 `is_running` → **GUI quit 都解不开**；
    SIGTERM 反而能秒杀（信号中断 render 旋 → close() 1s 走完，两具实测）。
    首要嫌疑：timeline endTime 自动停（待冻结 #3 哨兵取证一锤定音）。
    哨兵：`scripts/nav/freeze_watch.sh`（nohup 部署守栈；age>15s 自动抓 py-spy/gdb
    双栈 + GPU/内核证据落 var/evidence/freeze_watch/<ts>/ 然后停住留标本，不自动杀）。
    py-spy 判读：冻结栈在 isaaclab step():568 = PAUSED 旋；卡 :579/isaacsim.core:710
    内 = STOPPED 路径阻塞。健康基线栈 = warehouse_nav.py:351 → :579 → :710。
40. **【坑39 修正+根治】Isaac 冻结根因=timeline STOP/PAUSE 焊死主线程，触发=外部
    UI/输入注入 pause/stop，非"187s 确定性自停"**（2026-07-06 深夜真跑验收，
    DEBUG.md 坑40 假设环 + var/evidence/freeze_fix/）：
    - **坑39 两处判断被推翻**：(a) "sim≈187s 确定性自停" REFUTED——同构栈实测跑到
      sim_t 589s（>3× 187s）零冻；本会话 idle 栈存活>2h。两具僵尸都落在步态调试员
      xdotool windowactivate 压焦点 + 录屏活跃窗内 → 触发是**外部 X11 输入/UI 注入
      pause/stop**（Phase2a 单次 space 注入的 PAUSE traceback 栈顶=_app.update() 事件
      泵、无进程内 pause 调用，直证外部注入）。(b) "527% CPU=冻结签名" 是**误导**——
      健康运行态本就 507% CPU（RTX+RL策略+传感器多线程）；冻结的真判据是**时钟停摆**
      （/pose stamp 不推进），不是 CPU 高。凭 CPU 判冻会误诊。
    - **机制（源码直证）**：STOP→simulation_context.py:1027 `_app_control_on_stop_
      handle_fn` while not is_playing():render() **无 stop-break** 焊死；PAUSE→step():565
      同款旋等；replicator orchestrator.py:327 把"到端点 PAUSE"升成 timeline.stop()
      （仅 not is_looping 时；默认 looping=True 故此路默认关）。
    - **根治（scripts/sim/warehouse_nav.py 应用层四件套，零 IsaacLab/kit 改动）**：
      [1] sim.reset() 后 `sim._disable_app_control_on_stop_handle=True` 把 STOP 焊死回调
      变 no-op（IsaacLab 自身 reset():513 同款）；[2] `set_end_time(1e9)+commit()`
      拆升格器（**必须 commit() 落 USD stage，否则 get 读回旧值 1.0**）；[3] 主循环
      `sim.step()` **之前**插守卫：is_stopped→[FATAL]+sys.exit(3) 干净退出（响亮死给
      status.sh 看，不留僵尸）、else PAUSED→auto-resume(play+commit)、限速 >5/min 升
      FATAL 防人工暂停拉锯；[4] timeline 事件监听打 PLAY/PAUSE/STOP + traceback 闭环
      触发源。守卫**必须放 step 之前**——step():565 的旋等会先于守卫焊死。
    - **验收**：40.4min 浸泡零冻（243 行 CSV 全 ok、age<5s、freeze_watch 零捕获、全程
      goto 驾驶）；space 注入 35ms 内 auto-resume；快连 6 次触发 FATAL 干净退出无僵尸；
      RTF 0.206 vs 基线 0.222（守卫每步一次 is_playing C++ 调用开销可忽略）。
    - **回归测试**：`xdotool key --window <isaac_wid> space` 注入 pause，nav_bridge.log
      应现 `[NAV][WARN]…auto-resume` 且 /health age 不阶跃（固化 phase2a_pause_inject.log）。
