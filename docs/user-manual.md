# Go2W 使用手册（从开机到对话）

> 面向操作者（不需要懂代码）。两种形态：**仿真模式**（开发机，今天可用）与
> **真机模式**（NUC 在狗背上，硬件到位后启用——前置条件是
> [nuc-setup.md](nuc-setup.md) 的验收自检全部打钩）。
> 标注【P5.x】的环节正在落地，当前的替代做法都写明了。

---

## A. 仿真模式（开发机）

### A1. 启动全链

```bash
cd ~/Desktop/go2w
bash scripts/nav/restart_all.sh     # 拉起 Isaac + 导航栈，2-6 分钟
# 看到 "ALL-GREEN: 全链就绪" 才算好；GATE-FAILED 就再跑一次
```

【P5.1 落地后】改为 `bash scripts/nav/bringup.sh`（幂等：环境已好会直接返回，
不会重启正在跑的仿真），健康检查用 `bash scripts/nav/status.sh`。

### A2. 和机器狗对话

```bash
bash scripts/vector_os/run_agent.sh            # 交互 REPL
# 或单发一句：
bash scripts/vector_os/run_agent.sh --no-permission -p "导航到 (2.0, 0.0)"
```

【F2 落地后】改为 `za --world go2w`（z-agent 一等公民入口，同样的话术）。

能说的话（示例）：
- `去 (2, 0)` / `导航到坐标 (1.5, -1)` —— 开到地图坐标
- `现在在哪` —— 报当前位姿
- `explore` / `探索这个仓库`【P5.2】—— 自主探索建图
- agent 完成后会输出 `VECTOR_VERDICT verified=true/false`——**true 才是真到了**
  （判定读仿真地面真值，agent 自己说了不算）

### A3. 看画面

- Isaac Sim GUI 窗口：机器人本体与场景（restart_all 自动打开）
- RViz【P5.1】：SLAM 点云 / 地形图 / 规划路径 / 探索区域
- 截图自查：`logs/shots/` 每 30 秒一张

### A4. 结束

不用管它（常驻无害），或【P5.1】`bash scripts/nav/bringup.sh teardown`。
手动等价操作：`docker rm -f navstack` + 在 go2w-isaac 容器里
`pkill -9 -f "kit/pytho[n]"`。**绝不要**裸 `pkill mujoco/python`（会误杀别的项目）。

---

## B. 真机模式（Go2W + NUC）

### B1. 上电顺序（每次固定这么做）

1. **遥控器**满电、开机、确认在手——它是硬件急停，全程不离手
2. 狗趴平地，周围 3m 无人无障碍
3. Go2W 开机（宇树标准流程），等自检完成、站立（出厂 wheeled_sport 步态）
4. **先用遥控器手动走两步**——确认底盘/轮子正常，再交给 agent
5. NUC 上电（背部稳压器供电），等 ~1 分钟启动完成

### B2. PC 连接 NUC（你的电脑只是遥控终端）

推荐：**同一 WiFi + ssh**（NUC 在路由器上绑定固定 IP，例如 192.168.31.50）：

```bash
ssh go2w@192.168.31.50
```

- 没有 WiFi 的场地：手机开热点让 NUC 和 PC 都连上；或网线直连 PC↔NUC
  （临时调试用，注意别拔错——网口1 是狗、网口2 是雷达）
- 传文件：`scp` 或 VS Code 的 Remote-SSH（推荐，等于直接在 NUC 上开编辑器）
- 不需要给狗本体 ssh——我们不碰宇树主控，一切经 NUC

### B3. 启动软件栈（在 ssh 会话里）

```bash
cd ~/go2W_Sim
bash scripts/nav/bringup.sh        # 【P5.4 真机版】拉起 导航栈+桥+宇树桥
bash scripts/nav/status.sh         # green 才继续
```

启动后狗进入"听 agent 的"状态；遥控器仍随时可以接管（硬件优先级更高）。

### B4. 对话（和仿真里一模一样）

```bash
cd ~/z-agent && source .venv/bin/activate
vector-cli --world go2w            # 【F2 后：za --world go2w】
```

同样的话术：`去门口`、`explore`、`现在在哪`。sim-to-real 的设计目标就是
**你在仿真里学会的每一句话，真机上原样能用**。

### B5. 看画面（三选一）

1. **PC 上跑 RViz**（PC 装 ROS2 Jazzy，或用 docker）：
   ```bash
   export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
   rviz2 -d vehicle_simulator.rviz    # 配置文件从仓库拷
   ```
   经 WiFi 收点云会有点卡，看轨迹/地图够用
2. **远程桌面**（NoMachine/VNC）连 NUC，在 NUC 桌面开 RViz——流畅，推荐演示用
3. 不看图：只信 agent 输出的 verified 结果与位姿播报

### B6. 停止与关机（倒序）

- **随时急停**：遥控器接管（最高优先级）；或 agent 里输入 `停` / Ctrl+C
- 正常结束：
  1. agent CLI 退出
  2. `bash scripts/nav/bringup.sh teardown`
  3. 遥控器让狗趴下（damp）
  4. NUC：`sudo poweroff`，等指示灯灭
  5. Go2W 关机

### B7. 安全铁律（违反任何一条就不要开跑）

- 遥控器全程在手、有电、已确认能接管
- 首次/改代码后的首跑：空旷场地，速度上限保持 0.6 m/s，不载重
- explore 模式下 3m 内不站人；狗和楼梯/坡沿之间必须有物理隔挡
- 桥的 watchdog（0.4s 断令即停）与限幅是底线，**不许绕过**
- 电量 <20% 不开新任务

---

## C. 快速排障（先看这里再喊人）

| 症状 | 动作 |
|---|---|
| `桥不在线` | `status.sh` 看哪层红：L0 容器没起→bringup；L3 红→等 30s 再看；L4 红→SLAM 没收敛，雷达口/网段检查 |
| agent 说 verified=false | 它真没到/没做成——不是 bug 是诚实。看 RViz 位姿，重新下指令 |
| SLAM 漂移/画面乱 | 配对重启：`bringup.sh`（它会两侧一起重启，单侧重启没用） |
| 狗不动但一切绿 | 真机：查宇树桥进程与遥控器是否在手动模式；仿真：查 RL 策略加载日志 |
| explore 和 goto 打架 | 【P5.2】桥会拒绝并提示（409）；说 `停止探索` 再下导航 |
| 其它诡异问题 | 翻 [pitfalls.md](pitfalls.md)——27+ 条前人踩过的坑 |
