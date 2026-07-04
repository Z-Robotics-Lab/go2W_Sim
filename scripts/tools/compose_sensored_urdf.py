#!/usr/bin/env python3
"""合成传感器版 Go2W URDF：go2w + PiPER 机械臂 + Mid-360 + D435 + NUC 配重。

输出 assets/urdf/go2w_sensored.urdf（mesh 用相对路径，Isaac URDF importer 可直接解析）。
挂载位姿常量见下方 MOUNTS —— 在 GUI 里目检后微调这里即可，重跑本脚本再转 USD。
质量预算：PiPER 4.66 + Mid-360 0.265 + D435 0.072 + NUC 0.5 + 安装板 1.003 ≈ 6.5 kg。
"""
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GO2W_URDF = ROOT / "assets/unitree_ros/robots/go2w_description/urdf/go2w_description.urdf"
PIPER_URDF = ROOT / "assets/piper_ros/src/piper_description/urdf/piper_description.urdf"
OUT = ROOT / "assets/urdf/go2w_sensored.urdf"

# ---- 挂载常量（相对 go2w 根 link "base"；躯干碰撞盒顶面 z=+0.057）--------------
# 布局依据参考图（真机效果图）：背板上从前到后 = 手臂立柱（中前）-> NUC -> 稳压器，
# 全部在背板 x∈[-0.18, +0.18] 内，不悬出压到后腿电机；Mid-360 在头顶前倾 20°；
# D435 装在腕部顶面、镜头朝前（手眼）。
MOUNTS = {
    # 安装板：铺满背部，质量兜底补足 6.5kg 预算（含支架/线缆等杂项）
    "plate":  dict(xyz=(0.0, 0.0, 0.062), size=(0.36, 0.14, 0.010), mass=0.803),
    # PiPER 基座：直接装在背板上（用户确认不要垫高柱）
    "piper":  dict(xyz=(0.06, 0.0, 0.067), rpy=(0, 0, 0)),
    # NUC 配重块：手臂正后方（用户指定 0.5kg 代替真 NUC）
    "nuc":    dict(xyz=(-0.10, 0.0, 0.092), size=(0.11, 0.10, 0.05), mass=0.5),
    # 稳压器：NUC 再往后，仍在背板内
    "reg":    dict(xyz=(-0.155, 0.0, 0.082), size=(0.08, 0.06, 0.03), mass=0.20),
    # Mid-360：头顶，前倾 20°（pitch=+0.349rad）。视觉用真实网格
    # （mid360.stl 为毫米单位、角点原点 -> scale 0.001 + 平移居中），碰撞保持圆柱
    "mid360": dict(xyz=(0.27, 0.0, 0.10), rpy=(0, math.radians(20), 0),
                   radius=0.0325, length=0.065, mass=0.265,
                   # STL 圆柱轴是 mesh-Y：绕 X 转 90° 立正；角点原点 -> 旋转后平移居中
                   mesh="../sensor_meshes/mid360.stl", mesh_scale="0.001 0.001 0.001",
                   mesh_xyz=(-0.0364, 0.0334, -0.0301), mesh_rpy=(1.5708, 0, 0)),
    # D435 手眼：装腕顶（gripper_base -X 侧），镜头沿逼近轴（gripper_base +Z）。
    # FK 依据：手指沿 gripper_base +Z 安装（joint7 origin z=0.1358），腕顶=-X。
    # rpy=(0,-pi/2,0) 使 d435_link X 轴（RealSense 前向约定）= gripper_base +Z
    "d435":   dict(xyz=(-0.045, 0.0, 0.02), rpy=(0, -1.5708, 0),
                   size=(0.025, 0.090, 0.025), mass=0.072,
                   mesh="../sensor_meshes/d435.dae", mesh_scale="1 1 1",
                   mesh_xyz=(0.0043, -0.0175, 0), mesh_rpy=(1.5708, 0, 1.5708)),
}

def box_inertia(m, x, y, z):
    return dict(ixx=m/12*(y*y+z*z), iyy=m/12*(x*x+z*z), izz=m/12*(x*x+y*y),
                ixy=0.0, ixz=0.0, iyz=0.0)

def cyl_inertia(m, r, h):
    return dict(ixx=m/12*(3*r*r+h*h), iyy=m/12*(3*r*r+h*h), izz=m/2*r*r,
                ixy=0.0, ixz=0.0, iyz=0.0)

def make_link(name, geom_xml, mass, inertia, rgba="0.3 0.3 0.3 1",
              visual_xml=None, v_xyz=(0, 0, 0), v_rpy=(0, 0, 0)):
    """visual_xml 缺省与碰撞同形；传真实网格时碰撞仍用简单几何体（物理更稳更快）。"""
    i = " ".join(f'{k}="{v:.6g}"' for k, v in inertia.items())
    vis = visual_xml if visual_xml is not None else geom_xml
    return ET.fromstring(f"""
  <link name="{name}">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="{mass}"/><inertia {i}/></inertial>
    <visual><origin xyz="{v_xyz[0]} {v_xyz[1]} {v_xyz[2]}" rpy="{v_rpy[0]} {v_rpy[1]} {v_rpy[2]}"/>
      <geometry>{vis}</geometry>
      <material name="{name}_mat"><color rgba="{rgba}"/></material></visual>
    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry>{geom_xml}</geometry></collision>
  </link>""")

def make_fixed_joint(name, parent, child, xyz, rpy=(0, 0, 0)):
    return ET.fromstring(f"""
  <joint name="{name}" type="fixed">
    <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>
    <parent link="{parent}"/><child link="{child}"/>
  </joint>""")

def main():
    go2w = ET.parse(GO2W_URDF).getroot()
    piper = ET.parse(PIPER_URDF).getroot()

    # go2w mesh 路径: package:// -> 相对 assets/urdf/ 的路径
    for mesh in go2w.iter("mesh"):
        fn = mesh.get("filename", "")
        mesh.set("filename", fn.replace("package://go2w_description",
                                        "../unitree_ros/robots/go2w_description"))

    # piper：全部 link/joint 加前缀，mesh 换相对路径
    def pfx(n):
        return f"piper_{n}"
    for link in piper.iter("link"):
        link.set("name", pfx(link.get("name")))
    for joint in piper.iter("joint"):
        joint.set("name", pfx(joint.get("name")))
        for tag in ("parent", "child"):
            el = joint.find(tag)
            if el is not None:
                el.set("link", pfx(el.get("link")))
        # mimic 关节引用的关节名同样要加前缀
        mim = joint.find("mimic")
        if mim is not None:
            mim.set("joint", pfx(mim.get("joint")))
    for mesh in piper.iter("mesh"):
        fn = mesh.get("filename", "")
        mesh.set("filename", fn.replace("package://piper_description",
                                        "../piper_ros/src/piper_description"))

    out = ET.Element("robot", name="go2w_sensored")
    for child in list(go2w):
        out.append(child)
    for child in list(piper):
        out.append(child)

    # 轮子碰撞体：网格凸包 -> 圆柱（速度控制下多边形轮滚动拖滞；实测 r=0.086 w=0.052,
    # 网格沿轮轴(Y)外偏 0.048，左轮 +y 右轮 -y）
    WHEEL_R, WHEEL_W, WHEEL_YOFF = 0.086, 0.052, 0.0481
    for link in out.iter("link"):
        n = link.get("name", "")
        if n in ("FL_foot", "FR_foot", "RL_foot", "RR_foot"):
            side = 1.0 if n.startswith(("FL", "RL")) else -1.0
            col = link.find("collision")
            origin = col.find("origin")
            origin.set("xyz", f"0 {side * WHEEL_YOFF:.4f} 0")
            origin.set("rpy", "1.5708 0 0")
            geom = col.find("geometry")
            for c in list(geom):
                geom.remove(c)
            ET.SubElement(geom, "cylinder", radius=str(WHEEL_R), length=str(WHEEL_W))

    m = MOUNTS
    sx, sy, sz = m["plate"]["size"]
    out.append(make_link("mount_plate", f'<box size="{sx} {sy} {sz}"/>',
                         m["plate"]["mass"], box_inertia(m["plate"]["mass"], sx, sy, sz)))
    out.append(make_fixed_joint("mount_plate_joint", "base", "mount_plate", m["plate"]["xyz"]))

    out.append(make_fixed_joint("piper_mount_joint", "base", "piper_base_link",
                                m["piper"]["xyz"], m["piper"]["rpy"]))

    sx, sy, sz = m["nuc"]["size"]
    out.append(make_link("nuc_weight", f'<box size="{sx} {sy} {sz}"/>',
                         m["nuc"]["mass"], box_inertia(m["nuc"]["mass"], sx, sy, sz),
                         rgba="0.1 0.1 0.15 1"))
    out.append(make_fixed_joint("nuc_weight_joint", "base", "nuc_weight", m["nuc"]["xyz"]))

    sx, sy, sz = m["reg"]["size"]
    out.append(make_link("regulator", f'<box size="{sx} {sy} {sz}"/>',
                         m["reg"]["mass"], box_inertia(m["reg"]["mass"], sx, sy, sz),
                         rgba="0.15 0.15 0.2 1"))
    out.append(make_fixed_joint("regulator_joint", "base", "regulator", m["reg"]["xyz"]))

    r, h, mm = m["mid360"]["radius"], m["mid360"]["length"], m["mid360"]["mass"]
    out.append(make_link(
        "mid360_link", f'<cylinder radius="{r}" length="{h}"/>',
        mm, cyl_inertia(mm, r, h), rgba="0.85 0.85 0.9 1",
        visual_xml=f'<mesh filename="{m["mid360"]["mesh"]}" scale="{m["mid360"]["mesh_scale"]}"/>',
        v_xyz=m["mid360"]["mesh_xyz"], v_rpy=m["mid360"]["mesh_rpy"]))
    out.append(make_fixed_joint("mid360_joint", "base", "mid360_link",
                                m["mid360"]["xyz"], m["mid360"]["rpy"]))

    sx, sy, sz = m["d435"]["size"]
    out.append(make_link(
        "d435_link", f'<box size="{sx} {sy} {sz}"/>',
        m["d435"]["mass"], box_inertia(m["d435"]["mass"], sx, sy, sz),
        rgba="0.9 0.9 0.9 1",
        visual_xml=f'<mesh filename="{m["d435"]["mesh"]}" scale="{m["d435"]["mesh_scale"]}"/>',
        v_xyz=m["d435"]["mesh_xyz"], v_rpy=m["d435"]["mesh_rpy"]))
    out.append(make_fixed_joint("d435_joint", "piper_gripper_base", "d435_link",
                                m["d435"]["xyz"], m["d435"]["rpy"]))

    ET.indent(out)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(out).write(OUT, encoding="unicode")

    # 质量审计
    txt = OUT.read_text()
    payload = sum(float(v) for v in re.findall(r'<mass value="([^"]+)"', txt)) - 6.921 \
        - sum(float(v) for v in re.findall(r'<mass value="([^"]+)"',
              GO2W_URDF.read_text())) + 6.921
    go2w_total = sum(float(v) for v in re.findall(r'<mass value="([^"]+)"',
                                                  GO2W_URDF.read_text()))
    total = sum(float(v) for v in re.findall(r'<mass value="([^"]+)"', txt))
    links = len(re.findall(r"<link ", txt))
    joints = len(re.findall(r"<joint ", txt))
    print(f"OK -> {OUT}")
    print(f"links={links} joints={joints}")
    print(f"go2w 本体 {go2w_total:.3f} kg, 背部载荷 {total - go2w_total:.3f} kg, 整机 {total:.3f} kg")

if __name__ == "__main__":
    main()
