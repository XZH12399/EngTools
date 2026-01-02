import json
import numpy as np
import os
from build123d import *
from typing import Union, Dict, Optional


# ==========================================
# 1. 自定义异常
# ==========================================
class MechanismError(Exception):
    pass


# ==========================================
# 2. 核心类定义
# ==========================================

class MechanismParser:
    """负责解析 JSON 或 字典格式的机构数据"""

    def __init__(self, source: Union[str, Dict]):
        self.raw_data = {}
        self.joints = {}
        self.edges = {}
        self._load_source(source)

    def _load_source(self, source):
        if isinstance(source, str):
            if not os.path.exists(source):
                raise FileNotFoundError(f"File not found: {source}")
            with open(source, 'r', encoding='utf-8') as f:
                self.raw_data = json.load(f)
        elif isinstance(source, dict):
            self.raw_data = source
        else:
            raise ValueError("Input must be a file path or a dictionary.")

        # 兼容输入是完整 JSON 还是仅 data 部分
        data = self.raw_data.get('data', self.raw_data)
        if 'joints' not in data or 'edges' not in data:
            raise ValueError("Invalid Data: Input must contain 'joints' and 'edges' keys.")

        self.joints = data.get('joints', {})
        self.edges = data.get('edges', {})


class KinematicsSolver:
    """负责运动学求解，计算各关节的全局位姿矩阵"""

    def __init__(self, parser: MechanismParser):
        self.parser = parser
        self.global_poses = {}
        self.adjacency = self._build_adjacency()

    def _build_adjacency(self):
        adj = {j: [] for j in self.parser.joints}
        for edge_key in self.parser.edges.keys():
            try:
                u, v = edge_key.split('_')
                if u in adj: adj[u].append(v)
                if v in adj: adj[v].append(u)
            except ValueError:
                continue
        return adj

    def _get_edge_data(self, u, v):
        key_fwd, key_rev = f"{u}_{v}", f"{v}_{u}"
        edge = self.parser.edges.get(key_fwd) or self.parser.edges.get(key_rev)
        if not edge: raise ValueError(f"No edge found between {u} and {v}")
        is_forward = (edge == self.parser.edges.get(key_fwd))
        state = edge.get('state_source', 0.0) if is_forward else edge.get('state_target', 0.0)
        offset = edge.get('offset_source', 0.0) if is_forward else edge.get('offset_target', 0.0)
        return edge, state, offset

    def _get_dh_matrix(self, a, alpha, d, theta):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def solve(self, start_node=None) -> Dict[str, np.ndarray]:
        if start_node is None:
            start_node = list(self.parser.joints.keys())[0]
        self._compute_multi_path_states_numpy(start_node)
        return self.global_poses

    def _compute_multi_path_states_numpy(self, base_node):
        T_base = np.eye(4)
        self.global_poses = {base_node: T_base}
        expanded_nodes = {base_node}
        stack = [(base_node, None, T_base)]

        while stack:
            u, p, T_u = stack.pop()
            q_in, off_in = (0, 0)
            if p is not None: _, q_in, off_in = self._get_edge_data(u, p)

            for v in self.adjacency.get(u, []):
                if v == p: continue
                edge_out, q_out, off_out = self._get_edge_data(u, v)
                a, alpha = edge_out.get('a', 0.0), edge_out.get('alpha', 0.0)

                # 计算步进变换
                theta = (q_out - q_in) - np.pi if self.parser.joints.get(u) == 'R' else 0
                d = off_out - off_in

                T_v = T_u @ self._get_dh_matrix(a, alpha, d, theta)
                if v not in expanded_nodes:
                    self.global_poses[v] = T_v
                    expanded_nodes.add(v)
                    stack.append((v, u, T_v))


# ==========================================
# 3. 绘图与导出函数 (build123d 封装)
# ==========================================

def export_to_step(data: Dict, output_path: str):
    """
    一键导出接口：解析数据、运动学求解并生成包含关节轴的 STEP 文件
    """
    # 自动创建输出文件夹目录
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"[System] Created directory: {output_dir}")

    # 1. 解析与求解
    parser = MechanismParser(data)
    solver = KinematicsSolver(parser)
    poses = solver.solve()

    print(f"[Build123d] 正在生成几何模型...")

    # 2. 自动计算尺寸参考
    all_points = [p[:3, 3] for p in poses.values()]
    if not all_points:
        print("[Error] No pose data found.")
        return

    # 【核心修复】：增加最小值保护，防止球面机构（点全部重合）导致计算结果为0
    raw_max_dim = np.ptp(all_points, axis=0).max() if len(all_points) > 1 else 0.0

    # 如果是球面机构 (max_dim 接近 0)，给一个默认参考尺寸（例如 10.0）
    max_dim = raw_max_dim if raw_max_dim > 1e-3 else 10.0
    link_radius = max_dim / 60.0
    axis_radius = link_radius * 1.2
    axis_length = max_dim / 5.0

    shapes = []

    # 3. 生成杆件 (Links)
    for edge_key in parser.edges.keys():
        u, v = edge_key.split('_')
        if u in poses and v in poses:
            p1 = Vector(tuple(poses[u][:3, 3]))
            p2 = Vector(tuple(poses[v][:3, 3]))
            dist = (p2 - p1).length
            if dist < 1e-6: continue

            with BuildPart() as p:
                with BuildSketch(Plane(origin=p1, z_dir=p2 - p1)):
                    Circle(radius=link_radius)
                extrude(amount=dist)
            shapes.append(p.part)

    # 4. 生成关节轴 (Joint Axes)
    for j_id, matrix in poses.items():
        origin = Vector(tuple(matrix[:3, 3]))
        z_axis = Vector(tuple(matrix[:3, 2]))  # Z轴方向

        with BuildPart() as a:
            with BuildSketch(Plane(origin=origin, z_dir=z_axis).offset(-axis_length / 2)):
                Circle(radius=axis_radius)
            extrude(amount=axis_length)
        shapes.append(a.part)

    # 5. 合并并导出
    if shapes:
        assembly = Compound(shapes)
        assembly.export_step(output_path)
        print(f"[Finished] STEP 文件已保存至: {output_path}")
    else:
        print("[Error] 几何体生成失败。")