#!/usr/bin/env python3
# Copyright 2026 COD
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""把 gz-sim 仿真世界的地图模型直接采样成 PCD 点云地图（离线，不需要起仿真）。

用途：给 small_gicp / NDT 重定位、pcd2pgm 生成 2D 栅格等提供地图点云，
省掉“开仿真 + FAST-LIO 建图”的流程。

数据流向：
    命令行 -> 解析 world.sdf（含 include -> model.sdf）-> 展开每个 <visual> 的
    几何 + 世界位姿 -> 表面采样 -> 体素降采样 -> 写出 .pcd

示例：
    # 当前 gz_world.yaml 里选中的世界，输出到 resource/maps/<world>.pcd
    ros2 run rmu_gazebo_simulator mesh_to_pcd.py

    # 指定世界，并把原点平移到机器人出生点（对齐 "odom 原点 = 出生点" 的约定）
    ros2 run rmu_gazebo_simulator mesh_to_pcd.py \
        --world resource/worlds/rmul_2026_world.sdf --align-to-spawn

    # 只要 0.1 m 以上的墙面，0.02 m 体素，顺带出个俯视预览图
    python3 scripts/mesh_to_pcd.py --z-min 0.1 --voxel 0.02 --preview

注意：
  * 采到的是“完整表面”，包含雷达看不到的墙背面、底面；喂给 pcd2pgm 或
    重定位前一般先用 --z-min 裁掉地面。
  * 只支持 mesh(stl/obj) / box / cylinder / sphere / plane；.dae 等格式会
    跳过并提示（Open3D 和 VTK 都没有 Collada 读取器）。
  * 生成的 PCD 默认在 Gazebo 世界坐标系；是否要对齐到 map 帧见 --align-to-spawn。
"""

import argparse
import glob
import os
import re
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)  # .../rmu_gazebo_simulator
PROJECT_NAME = "rmu_gazebo_simulator"


def _resolve_pkg_dir():
    """ros2 run + symlink-install 时 __file__ 是 install 里的软链，realpath 指回源码树。"""
    for cand in (PKG_DIR, os.path.dirname(os.path.dirname(os.path.realpath(__file__)))):
        if os.path.isdir(os.path.join(cand, "resource")):
            return cand
    return PKG_DIR


PKG_DIR = _resolve_pkg_dir()


def share_dir():
    """源码树里是 PKG_DIR，安装后（ros2 run）在 ../share/<pkg>。"""
    cand = os.path.join(PKG_DIR, "resource")
    if os.path.isdir(cand):
        return PKG_DIR
    for rel in (("..", "share", PROJECT_NAME), ("..", "..", "share", PROJECT_NAME)):
        share = os.path.normpath(os.path.join(PKG_DIR, *rel))
        if os.path.isdir(share):
            return share
    return PKG_DIR


def resource_dir():
    return os.path.join(share_dir(), "resource")


def config_file():
    return os.path.join(share_dir(), "config", "gz_world.yaml")

try:
    import open3d as o3d  # noqa: F401  (可选依赖，仅 --sampler open3d/poisson 用)

    HAS_OPEN3D = True
except Exception:  # pragma: no cover - 环境相关
    HAS_OPEN3D = False


# --------------------------------------------------------------------------- #
# 资源路径与 URI 解析
# --------------------------------------------------------------------------- #
def collect_resource_paths(extra=()):
    """收集 model:// 的搜索路径（优先环境变量，其次源码/install 目录）。"""
    paths = []

    def add(path):
        if path and os.path.isdir(path) and path not in paths:
            paths.append(path)

    for env_name in ("GZ_SIM_RESOURCE_PATH", "SDF_PATH"):
        for p in os.environ.get(env_name, "").split(os.pathsep):
            add(p)
    for p in extra:
        add(p)

    candidates = [PKG_DIR, os.getcwd()]
    for base in candidates:
        d = base
        for _ in range(6):
            add(os.path.join(d, "resource", "models"))
            for m in glob.glob(os.path.join(d, "share", "*", "resource", "models")):
                add(m)
            for m in glob.glob(os.path.join(d, "install", "*", "share", "*", "resource", "models")):
                add(m)
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    return paths


def resolve_uri(uri, search_paths, base_dir=None):
    """把 SDF 里的 uri（model:// / package:// / file:// / 相对路径）解析成本地路径。"""
    if not uri:
        return None
    if "://" in uri:
        scheme, rel = uri.split("://", 1)
        if scheme == "file":
            return rel if os.path.exists(rel) else None
        if scheme in ("model", "package"):
            for base in search_paths:
                for cand in (os.path.join(base, rel), os.path.join(base, "models", rel)):
                    if os.path.exists(cand):
                        return cand
            # 兜底：按第一段目录名在整个资源树里找
            name = rel.split("/", 1)[0]
            rest = rel.split("/", 1)[1] if "/" in rel else ""
            for base in search_paths:
                for cand in glob.glob(os.path.join(base, "**", name), recursive=True):
                    full = os.path.join(cand, rest) if rest else cand
                    if os.path.exists(full):
                        return full
            return None
        return None
    cand = os.path.join(base_dir, uri) if base_dir else uri
    return cand if os.path.exists(cand) else None


def model_sdf_file(model_dir):
    """给定 model 目录，返回它的 sdf 文件。"""
    if os.path.isfile(model_dir):
        return model_dir
    cfg = os.path.join(model_dir, "model.config")
    if os.path.isfile(cfg):
        try:
            root = ET.parse(cfg).getroot()
            el = root.find("sdf")
            if el is not None and (el.text or "").strip():
                cand = os.path.join(model_dir, el.text.strip())
                if os.path.isfile(cand):
                    return cand
        except ET.ParseError:
            pass
    default = os.path.join(model_dir, "model.sdf")
    if os.path.isfile(default):
        return default
    sdfs = sorted(glob.glob(os.path.join(model_dir, "*.sdf")))
    return sdfs[0] if sdfs else None


# --------------------------------------------------------------------------- #
# SDF 位姿 / 几何解析
# --------------------------------------------------------------------------- #
def parse_numbers(el, count):
    if el is None or not (el.text or "").strip():
        return None
    vals = [float(v) for v in el.text.replace(",", " ").split()]
    if len(vals) < count:
        vals += [0.0] * (count - len(vals))
    return vals[:count]


def parse_pose(el, what=""):
    """<pose>x y z roll pitch yaw</pose> -> 4x4 矩阵（SDF 旋转顺序 Rz*Ry*Rx）。"""
    vals = parse_numbers(el, 6)
    if vals is None:
        return np.eye(4)
    if el is not None and el.get("relative_to"):
        print(f"[warn] {what}: 忽略 pose 的 relative_to='{el.get('relative_to')}'", file=sys.stderr)
    x, y, z, roll, pitch, yaw = vals
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = (x, y, z)
    return t


def parse_scale(el, what=""):
    """<scale>x y z</scale>，允许只写一个值。"""
    vals = parse_numbers(el, 3)
    if vals is None:
        return np.ones(3)
    if len([v for v in (el.text or "").replace(",", " ").split()]) == 1:
        return np.full(3, vals[0])
    return np.asarray(vals, dtype=float)


# --------------------------------------------------------------------------- #
# 网格读取与表面采样
# --------------------------------------------------------------------------- #
def load_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = int.from_bytes(data[80:84], "little")
        if n > 0 and 84 + n * 50 == len(data):
            rec = np.frombuffer(data, dtype=np.uint8, count=n * 50, offset=84).reshape(n, 50)
            verts = rec[:, 12:48].copy().view(np.float32).reshape(n, 3, 3).reshape(-1, 3)
            tris = np.arange(n * 3, dtype=np.int64).reshape(n, 3)
            return verts.astype(np.float64), tris
    # ASCII STL
    verts = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        s = line.strip()
        if s[:6].lower() == "vertex":
            parts = s.split()
            verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if len(verts) < 3:
        raise ValueError(f"无法解析 STL: {path}")
    v = np.asarray(verts[: len(verts) // 3 * 3], dtype=np.float64).reshape(-1, 3, 3)
    return v.reshape(-1, 3), np.arange(v.size // 3, dtype=np.int64).reshape(-1, 3)


def load_obj(path):
    verts, tris = [], []
    with open(path, errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
            elif line.startswith("f "):
                idx = []
                for tok in line.split()[1:]:
                    i = int(tok.split("/")[0])
                    idx.append(i - 1 if i > 0 else len(verts) + i)
                for k in range(1, len(idx) - 1):
                    tris.append((idx[0], idx[k], idx[k + 1]))
    if not verts or not tris:
        raise ValueError(f"无法解析 OBJ: {path}")
    return np.asarray(verts, dtype=np.float64), np.asarray(tris, dtype=np.int64)


MESH_LOADERS = {".stl": load_stl, ".obj": load_obj}


def load_mesh(path):
    ext = os.path.splitext(path)[1].lower()
    loader = MESH_LOADERS.get(ext)
    if loader is None:
        raise ValueError(f"不支持的网格格式 {ext}（只有 stl/obj）")
    return loader(path)


def sample_triangles(verts, tris, n, rng):
    """按三角形面积做均匀采样。"""
    if n <= 0 or len(tris) == 0:
        return np.zeros((0, 3))
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    total = areas.sum()
    if total <= 0:
        return np.zeros((0, 3))
    cum = np.cumsum(areas / total)
    idx = np.searchsorted(cum, rng.random(n), side="right")
    idx = np.clip(idx, 0, len(tris) - 1)
    u = rng.random(n)
    v = rng.random(n)
    flip = (u + v) > 1.0
    u[flip] = 1.0 - u[flip]
    v[flip] = 1.0 - v[flip]
    pts = a[idx] * (1 - u - v)[:, None] + b[idx] * u[:, None] + c[idx] * v[:, None]
    return pts


def mesh_area(verts, tris):
    if len(tris) == 0:
        return 0.0
    a, b, c = verts[tris[:, 0]], verts[tris[:, 1]], verts[tris[:, 2]]
    return float(0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1).sum())


def box_area(size):
    sx, sy, sz = size
    return 2.0 * (sx * sy + sy * sz + sz * sx)


def sample_box(size, n, rng):
    sx, sy, sz = np.asarray(size, dtype=float)
    areas = np.array([sx * sy, sx * sy, sy * sz, sy * sz, sx * sz, sx * sz])
    p = areas / areas.sum()
    face = np.searchsorted(np.cumsum(p), rng.random(n), side="right")
    face = np.clip(face, 0, 5)
    u = rng.random(n) - 0.5
    v = rng.random(n) - 0.5
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    pts = np.empty((n, 3))
    for i, (ax, sign) in enumerate([(2, 1), (2, -1), (0, 1), (0, -1), (1, 1), (1, -1)]):
        m = face == i
        if not m.any():
            continue
        half = (hx, hy, hz)[ax]
        pts[m, ax] = sign * half
        other = [k for k in range(3) if k != ax]
        halfs = (hx, hy, hz)
        pts[m, other[0]] = u[m] * 2 * halfs[other[0]]
        pts[m, other[1]] = v[m] * 2 * halfs[other[1]]
    return pts


def sample_cylinder(radius, length, n, rng):
    side = 2 * np.pi * radius * length
    caps = 2 * np.pi * radius * radius
    p_side = side / (side + caps)
    m_side = rng.random(n) < p_side
    pts = np.empty((n, 3))
    ns = int(m_side.sum())
    if ns:
        theta = rng.random(ns) * 2 * np.pi
        pts[m_side, 0] = radius * np.cos(theta)
        pts[m_side, 1] = radius * np.sin(theta)
        pts[m_side, 2] = (rng.random(ns) - 0.5) * length
    nc = n - ns
    if nc:
        r = radius * np.sqrt(rng.random(nc))
        theta = rng.random(nc) * 2 * np.pi
        pts[~m_side, 0] = r * np.cos(theta)
        pts[~m_side, 1] = r * np.sin(theta)
        pts[~m_side, 2] = np.where(rng.random(nc) < 0.5, length / 2, -length / 2)
    return pts


def sample_sphere(radius, n, rng):
    # 球面均匀采样
    z = rng.uniform(-1, 1, n)
    theta = rng.random(n) * 2 * np.pi
    r = np.sqrt(np.maximum(0.0, 1 - z * z))
    return np.stack([radius * r * np.cos(theta), radius * r * np.sin(theta), radius * z], axis=1)


def sample_plane(size, n, rng):
    sx, sy = size
    return np.stack([(rng.random(n) - 0.5) * sx, (rng.random(n) - 0.5) * sy, np.zeros(n)], axis=1)


# --------------------------------------------------------------------------- #
# 遍历 SDF，收集 (世界位姿, 几何, 缩放)
# --------------------------------------------------------------------------- #
class Geometry:
    __slots__ = ("transform", "scale", "kind", "element", "source", "mesh_path", "label")

    def __init__(self, transform, scale, kind, element, source, mesh_path=None, label=None):
        self.transform = transform
        self.scale = scale  # 3 维缩放向量（mesh scale * include scale）
        self.kind = kind
        self.element = element
        self.source = source
        self.mesh_path = mesh_path
        self.label = label

    def area(self):
        """表面积（用于按面积分配采样点数），失败返回 0。"""
        try:
            if self.kind == "mesh":
                verts, tris = load_mesh(self.mesh_path)
                return mesh_area(verts * self.scale, tris)
            if self.kind == "box":
                return box_area(self.element)
            if self.kind == "cylinder":
                r, l = self.element
                return 2 * np.pi * r * l + 2 * np.pi * r * r
            if self.kind == "sphere":
                return 4 * np.pi * self.element ** 2
            if self.kind == "plane":
                return float(self.element[0] * self.element[1])
        except Exception as exc:
            print(f"[warn] 计算面积失败 {self.source}: {exc}", file=sys.stderr)
        return 0.0

    def sample(self, n, rng, sampler):
        if n <= 0:
            return np.zeros((0, 3))
        if self.kind == "mesh":
            verts, tris = load_mesh(self.mesh_path)
            if sampler in ("open3d", "poisson") and HAS_OPEN3D:
                mesh = o3d.geometry.TriangleMesh(
                    o3d.utility.Vector3dVector(verts * self.scale),
                    o3d.utility.Vector3iVector(tris.astype(np.int32)),
                )
                if sampler == "poisson":
                    pcd = mesh.sample_points_poisson_disk(number_of_points=n, init_factor=5)
                else:
                    pcd = mesh.sample_points_uniformly(number_of_points=n)
                return np.asarray(pcd.points)
            return sample_triangles(verts * self.scale, tris, n, rng)
        if self.kind == "box":
            return sample_box(self.element, n, rng)
        if self.kind == "cylinder":
            return sample_cylinder(self.element[0], self.element[1], n, rng)
        if self.kind == "sphere":
            return sample_sphere(self.element, n, rng)
        if self.kind == "plane":
            return sample_plane(self.element, n, rng)
        return np.zeros((0, 3))


def geometry_from_element(geo_el, scale, source, search_paths):
    """解析 <geometry>，返回 Geometry（mesh / box / cylinder / sphere / plane）。"""
    mesh = geo_el.find("mesh")
    if mesh is not None:
        uri = (mesh.findtext("uri") or "").strip()
        path = resolve_uri(uri, search_paths, os.path.dirname(source))
        if path is None:
            raise FileNotFoundError(f"找不到网格 {uri}（来自 {source}）")
        ext = os.path.splitext(path)[1].lower()
        if ext not in MESH_LOADERS:
            raise ValueError(f"不支持的网格格式 {ext}: {uri}")
        return Geometry(np.eye(4), scale * parse_scale(mesh.find("scale")), "mesh", None, source,
                        mesh_path=path)
    box = geo_el.find("box")
    if box is not None:
        return Geometry(np.eye(4), scale, "box", np.asarray(parse_numbers(box.find("size"), 3)), source)
    cyl = geo_el.find("cylinder")
    if cyl is not None:
        r = parse_numbers(cyl.find("radius"), 1)[0]
        l = parse_numbers(cyl.find("length"), 1)[0]
        return Geometry(np.eye(4), scale, "cylinder", (r, l), source)
    sph = geo_el.find("sphere")
    if sph is not None:
        r = parse_numbers(sph.find("radius"), 1)[0]
        return Geometry(np.eye(4), scale, "sphere", r, source)
    plane = geo_el.find("plane")
    if plane is not None:
        size = parse_numbers(plane.find("size"), 2)
        return Geometry(np.eye(4), scale, "plane", size, source)
    return None


def walk_model(model_el, t_parent, scale_parent, source, out, search_paths):
    """递归展开 model：nested model / link / visual。"""

    def add_visual(vis, t_parent_vis, scale_vis):
        t_vis = t_parent_vis @ parse_pose(vis.find("pose"), f"visual '{vis.get('name')}'")
        geo_el = vis.find("geometry")
        if geo_el is None:
            return
        try:
            g = geometry_from_element(geo_el, scale_vis, source, search_paths)
        except Exception as exc:
            print(f"[warn] 跳过 {source}: {exc}", file=sys.stderr)
            return
        if g is not None:
            g.transform = t_vis
            g.label = f"{model_el.get('name', 'model')}/{vis.get('name', 'visual')}"
            out.append(g)

    t_model = t_parent @ parse_pose(model_el.find("pose"), f"model '{model_el.get('name')}'")
    scale = scale_parent * parse_scale(model_el.find("scale"))
    for nested in model_el.findall("model"):
        walk_model(nested, t_model, scale, source, out, search_paths)
    for vis in model_el.findall("visual"):  # 少见但合法
        add_visual(vis, t_model, scale)
    for link in model_el.findall("link"):
        t_link = t_model @ parse_pose(link.find("pose"), f"link '{link.get('name')}'")
        scale_link = scale * parse_scale(link.find("scale"))
        for vis in link.findall("visual"):
            add_visual(vis, t_link, scale_link)


def walk_world(sdf_path, search_paths):
    """解析 world.sdf（或 model.sdf），返回 Geometry 列表。"""
    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    container = world if world is not None else root
    base_dir = os.path.dirname(os.path.abspath(sdf_path))
    out = []

    for model_el in container.findall("model"):
        t_model = parse_pose(model_el.find("pose"), f"model '{model_el.get('name')}'")
        include = model_el.find("include")
        if include is not None:
            uri = (include.findtext("uri") or "").strip()
            path = resolve_uri(uri, search_paths, base_dir)
            if path is None:
                print(f"[warn] 找不到 include 的模型: {uri}", file=sys.stderr)
                continue
            sdf_file = model_sdf_file(path)
            if sdf_file is None:
                print(f"[warn] {path} 下没有 sdf 文件", file=sys.stderr)
                continue
            t_include = t_model @ parse_pose(include.find("pose"), f"include '{uri}'")
            scale_include = parse_scale(include.find("scale"))
            mroot = ET.parse(sdf_file).getroot()
            mmodel = mroot.find("model")
            if mmodel is None:
                print(f"[warn] {sdf_file} 里没有 <model>", file=sys.stderr)
                continue
            walk_model(mmodel, t_include, scale_include, sdf_file, out, search_paths)
        else:
            walk_model(model_el, np.eye(4), np.ones(3), sdf_path, out, search_paths)

    for include in container.findall("include"):  # <world><include>（SDF 1.10+）
        uri = (include.findtext("uri") or "").strip()
        path = resolve_uri(uri, search_paths, base_dir)
        if path is None:
            print(f"[warn] 找不到 include 的模型: {uri}", file=sys.stderr)
            continue
        sdf_file = model_sdf_file(path)
        mroot = ET.parse(sdf_file).getroot()
        mmodel = mroot.find("model")
        if mmodel is not None:
            walk_model(mmodel, parse_pose(include.find("pose")), parse_scale(include.find("scale")),
                       sdf_file, out, search_paths)
    return out


# --------------------------------------------------------------------------- #
# 后处理与写出
# --------------------------------------------------------------------------- #
def voxel_downsample(points, voxel):
    if voxel <= 0 or len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    uniq, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    sums = np.empty((len(uniq), 3))
    for d in range(3):
        sums[:, d] = np.bincount(inv, weights=points[:, d], minlength=len(uniq))
    return sums / counts[:, None]


def write_pcd(path, points, ascii_mode=False):
    n = len(points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA {'ascii' if ascii_mode else 'binary'}\n"
    )
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        if ascii_mode:
            np.savetxt(f, points.astype(np.float32), fmt="%.4f %.4f %.4f")
        else:
            f.write(np.ascontiguousarray(points, dtype="<f4").tobytes())


def _occupancy(points, res, x0, y0, nx, ny):
    ix = np.clip(((points[:, 0] - x0) / res).astype(np.int64), 0, nx - 1)
    iy = np.clip(((points[:, 1] - y0) / res).astype(np.int64), 0, ny - 1)
    occ = np.full((ny, nx), 255, np.uint8)  # 白=空闲
    occ[iy, ix] = 0  # 黑=有点（与 nav2 pgm 习惯一致）
    return occ[::-1]  # 图像行从上到下 = y 从大到小


def write_preview(points, path, res=0.05, wall_z=0.1):
    """俯视占据图（PNG）：左=全部点，右=墙面带（z >= wall_z），用来肉眼检查成果。"""
    from PIL import Image

    if len(points) == 0:
        return
    x0, y0 = points[:, 0].min(), points[:, 1].min()
    nx = int(np.ceil((points[:, 0].max() - x0) / res)) + 1
    ny = int(np.ceil((points[:, 1].max() - y0) / res)) + 1
    panels = [_occupancy(points, res, x0, y0, nx, ny)]
    walls = points[points[:, 2] >= wall_z]
    panels.append(_occupancy(walls, res, x0, y0, nx, ny) if len(walls) else np.full((ny, nx), 255, np.uint8))
    gap = np.full((ny, 10), 90, np.uint8)
    Image.fromarray(np.hstack([panels[0], gap, panels[1]])).save(path)


# --------------------------------------------------------------------------- #
# 世界选择与命令行
# --------------------------------------------------------------------------- #
def world_from_config():
    """读 config/gz_world.yaml 里当前选中的世界名。"""
    cfg = config_file()
    if not os.path.isfile(cfg):
        return None
    try:
        import yaml

        with open(cfg) as f:
            return (yaml.safe_load(f) or {}).get("world")
    except Exception:
        text = open(cfg).read()
        m = re.search(r"^world:\s*['\"]?([\w.-]+)", text, re.M)
        return m.group(1) if m else None


def world_sdf_path(world_name):
    cand = os.path.join(resource_dir(), "worlds", f"{world_name}_world.sdf")
    return os.path.abspath(cand) if os.path.isfile(cand) else None


def spawn_pose(world_name):
    """从 config/gz_world.yaml 取该世界第一台机器人的出生位姿 (x, y)。"""
    cfg = config_file()
    try:
        import yaml

        with open(cfg) as f:
            data = yaml.safe_load(f) or {}
        robots = (data.get("robots") or {}).get(world_name) or []
        if robots:
            return float(robots[0]["x_pose"]), float(robots[0]["y_pose"])
    except Exception as exc:
        print(f"[warn] 读取出生点失败: {exc}", file=sys.stderr)
    return None


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="把 gz-sim 世界地图模型直接采样成 PCD 点云地图",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("示例：")[-1],
    )
    p.add_argument("--world", help="world 名称（如 rmul_2026）或 world.sdf 路径（默认取 config/gz_world.yaml 里选中的世界）")
    p.add_argument("--out", help="输出 pcd 路径（默认 resource/maps/<world>.pcd）")
    p.add_argument("--density", type=float, default=1000.0, help="采样密度，点/m^2（默认 1000）")
    p.add_argument("--points", type=int, help="直接指定总点数（覆盖 --density）")
    p.add_argument("--voxel", type=float, default=0.02, help="体素降采样尺寸 m，0 表示不降（默认 0.02）")
    p.add_argument("--z-min", type=float, help="只保留 z 大于该值的点")
    p.add_argument("--z-max", type=float, help="只保留 z 小于该值的点")
    p.add_argument("--offset", nargs=3, type=float, metavar=("X", "Y", "Z"), default=(0.0, 0.0, 0.0),
                   help="输出点云整体平移（默认 0 0 0）")
    p.add_argument("--align-to-spawn", action="store_true",
                   help="把原点平移到机器人出生点（对齐 odom 原点=出生点的约定）")
    p.add_argument("--sampler", choices=["numpy", "open3d", "poisson"], default="numpy",
                   help="mesh 采样器：numpy 面积均匀（默认）/ open3d 均匀 / poisson 泊松盘")
    p.add_argument("--seed", type=int, default=0, help="随机种子（默认 0）")
    p.add_argument("--preview", action="store_true", help="额外输出俯视密度预览 PNG")
    p.add_argument("--ascii", action="store_true", help="输出 ascii 格式 pcd")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    t0 = time.time()

    world_path = args.world
    world_name = None
    if world_path:
        if os.path.isfile(world_path) or os.sep in world_path:
            world_name = os.path.basename(world_path).replace("_world.sdf", "").replace(".sdf", "")
            world_path = os.path.abspath(world_path)
        else:  # 只给了世界名，去 resource/worlds 里找
            world_name = world_path.strip().removesuffix("_world")
            world_path = world_sdf_path(world_name)
            if not world_path:
                print(f"[error] 找不到世界: {world_name}", file=sys.stderr)
                return 1
    else:
        world_name = world_from_config()
        if not world_name:
            print("[error] 无法从 config/gz_world.yaml 读取世界名，请用 --world 指定", file=sys.stderr)
            return 1
        world_path = world_sdf_path(world_name)
        if not world_path:
            print(f"[error] 找不到世界文件: {world_name}_world.sdf", file=sys.stderr)
            return 1

    if args.sampler in ("open3d", "poisson") and not HAS_OPEN3D:
        print("[warn] 没装 open3d，回退到 numpy 采样器", file=sys.stderr)
        args.sampler = "numpy"

    search_paths = collect_resource_paths([os.path.join(os.path.dirname(world_path))])
    print(f"[info] world    : {world_path}")
    print(f"[info] 资源路径 : {len(search_paths)} 个")
    geoms = walk_world(world_path, search_paths)
    if not geoms:
        print("[error] 没有解析到任何几何体", file=sys.stderr)
        return 1

    areas = [g.area() for g in geoms]
    total_area = sum(areas)
    rng = np.random.default_rng(args.seed)
    chunks = []
    print(f"[info] 几何体   : {len(geoms)} 个，总表面积 {total_area:.2f} m^2")
    for g, area in zip(geoms, areas):
        if args.points:
            n = int(round(args.points * (area / total_area))) if total_area > 0 else 0
        else:
            n = int(round(area * args.density))
        if area <= 0:
            print(f"[warn] 跳过零面积几何体: {g.source}")
            continue
        pts = g.sample(n, rng, args.sampler)
        if len(pts) == 0:
            continue
        pts = pts @ g.transform[:3, :3].T + g.transform[:3, 3]
        chunks.append(pts)
        name = g.label or os.path.basename(g.source)
        print(f"  - {name:35s} {g.kind:8s} area={area:8.2f} m^2  points={len(pts)}")

    if not chunks:
        print("[error] 采样结果为空", file=sys.stderr)
        return 1

    points = np.vstack(chunks)
    if args.voxel and args.voxel > 0:
        before = len(points)
        points = voxel_downsample(points, args.voxel)
        print(f"[info] 体素 {args.voxel} m: {before} -> {len(points)} 点")

    offset = np.asarray(args.offset, dtype=float)
    if args.align_to_spawn:
        spawn = spawn_pose(world_name)
        if spawn is None:
            print("[warn] 拿不到出生点，忽略 --align-to-spawn", file=sys.stderr)
        else:
            offset = offset - np.array([spawn[0], spawn[1], 0.0])
            print(f"[info] 对齐出生点: 平移 {offset}")
    points = points + offset

    if args.z_min is not None:
        points = points[points[:, 2] >= args.z_min]
    if args.z_max is not None:
        points = points[points[:, 2] <= args.z_max]
    if len(points) == 0:
        print("[error] 裁剪后没有点了", file=sys.stderr)
        return 1

    out = args.out or os.path.join(resource_dir(), "maps", f"{world_name}.pcd")
    write_pcd(out, points, ascii_mode=args.ascii)
    mins, maxs = points.min(axis=0), points.max(axis=0)
    print(f"[info] 输出     : {out}  ({os.path.getsize(out) / 1e6:.1f} MB, {len(points)} 点)")
    print(f"[info] 包围盒   : x {mins[0]:.2f}..{maxs[0]:.2f}  y {mins[1]:.2f}..{maxs[1]:.2f}  z {mins[2]:.2f}..{maxs[2]:.2f}")
    print(f"[info] 耗时     : {time.time() - t0:.2f} s")

    if args.preview:
        png = os.path.splitext(out)[0] + "_preview.png"
        wall_z = args.z_min if args.z_min is not None else 0.1
        write_preview(points, png, wall_z=wall_z)
        print(f"[info] 预览     : {png}（左=全部点，右=z>={wall_z:g} 的墙面带）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
