# DEBUG — OFAT ground-friction knob silently not applied (policy_acceptance.py)

日期 2026-07-13. 取证代理 (Opus, 亲自执行). 红线 go2w_policy.py / 部署 warehouse_nav.py 摩擦指令不动.

## OBSERVE
- 真源 `~/Desktop/z-manip/var/evidence/m1/yaw_ofat_matrix.md` R3 行标 KNOB-NOT-APPLIED:
  R3 (only-friction, model_7494) 数值 byte-for-byte 等于 R1 (yaw_rate=0.9857 两 wz).
- R3 run log 报 `Could not perform 'bind_physics_material' on any prims under
  '/World/Robot/{FL,FR,RL,RR}_foot' ... (2) The desired attribute exists on an instanced prim.`
- harness `policy_acceptance.py:287-299` 的绑定逻辑是 nav `warehouse_nav.py:449-455` 的逐行复刻:
  两者都 `bind_physics_material(f"/World/Robot/{foot}_foot", "/World/Materials/wheel_rubber")`.
- **决定性反证**: `logs/nav_bridge.log:881-897` 显示 **部署链 nav 里同一 bind 对全部四轮
  也静默失败**（同样的 instanced-prim 错误），且后续逐轮回读循环 `[MAT] {foot} collider=...
  bound_physics_material=...` **一行都没打印** —— 只打了材质属性行 (:897)，无绑定确认行.

## HYPOTHESIZE
| # | 假设 | 证据 | 类别 |
|---|------|------|------|
| H1 | nav 用别的机制（articulation spawn physics_material / GroundPlane material）真绑上了摩擦，只有 harness 漏 | 案情前提；vx 部署面正常、真机不漂移说明摩擦某处生效 | 机制 |
| H2 | nav 和 harness 一样，bind_physics_material 对 instanced foot prim 都静默失败，部署摩擦从未生效 | nav_bridge.log:881-896 四轮全失败；逐轮回读零行 | 机制 |
| H3 | 正确施加法 = 绑地面 collision prim（非 instanced）+ combine=max，等效轮-地有效摩擦 | IsaacLab spawn_ground_plane 官方就是绑 TypeName=="Plane" 的地面 collider | 修复 |

## EXPERIMENT
- H1: 读 nav :270-287 ArticulationCfg spawn — 是 UrdfFileCfg，spawn 段**无** `physics_material=`
  参数（physics_material 只在抓取箱 :216、垫台上有，不在 Robot 上）. 整条 nav 里机器人轮-地
  摩擦唯一施加尝试就是 :454 的 bind_physics_material.
  → nav_bridge.log:897 只有 wheel_rubber 材质属性行，**无任何 bound_physics_material 回读行**.
  **H1 REJECTED**: nav 无第二条生效路径.
- H2: nav_bridge.log:881-896 逐行四轮 `Could not perform 'bind_physics_material' ... instanced prim`.
  **H2 CONFIRMED**: 部署链 μ1.8/1.6 从未真正绑到任何 collider；轮子跑 URDF 导入默认材质摩擦.
- H3: IsaacLab `sim/spawners/from_files/from_files.py:spawn_ground_plane` — 当 `cfg.physics_material`
  非空时，它 `get_first_matching_child_prim(predicate: TypeName=="Plane")` 找地面 collision prim
  再 `bind_physics_material(collision_prim_path, ...)`. 地面 Plane 非 instanced → 必然生效.
  **H3 CONFIRMED (机制)**: 把 μ+combine=max 作为 GroundPlaneCfg.physics_material 传入即正确施加.

## CONCLUDE
- 根因: `policy_acceptance.py:287-299`（复刻自 `warehouse_nav.py:449-455`）用
  `bind_physics_material("/World/Robot/{foot}_foot", ...)` 施加轮摩擦，但 foot 是 instanced USD prim，
  bind 静默失败（只 WARNING，不 raise）→ mu_s/mu_d 写进 meta.ofat 但从未落到物理.
- 附带更正 R3 note 的隐含假设: **部署链 nav 里这个 bind 同样失败**（nav_bridge.log:881-896），
  部署摩擦 μ1.8/1.6 从未生效 —— 这是比"仅 harness bug"更大的发现，改变判词方向.
- 修复 (file:line policy_acceptance.py:277 & :287-299): 弃手工绑 foot，改用 IsaacLab 官方
  `GroundPlaneCfg(physics_material=RigidBodyMaterialCfg(mu_s,mu_d,combine=max))` —— 绑到地面
  Plane collider（非 instanced），combine=max 使有效轮-地摩擦=max(轮默认,1.8/1.6)=1.8/1.6，
  物理等效. 留验证打印: 回读 /World/Ground 下 Plane collider 的 bound physics material + 属性值，
  若未绑成功则打印 NONE（绝不再静默）.
- 回归: 重跑 R3'（只摩擦）/R5'（全态）/R6'（5495 全态），每行 append yaw_ofat_matrix.md，含绑定验证行.
