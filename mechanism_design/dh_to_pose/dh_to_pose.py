import json
import argparse
import numpy as np
import sys
import os
from collections import deque
from typing import Union, Dict, Optional


# ==========================================
# 1. 核心转换类
# ==========================================

class DHToPoseConverter:
    def __init__(self, json_data: Dict):
        if 'data' not in json_data:
            raise ValueError("Input JSON must contain a 'data' field.")

        self.raw_data = json_data['data']
        # 【新增】读取 settings 配置
        self.settings = json_data.get('settings', {})

        self.joints = self.raw_data.get('joints', {})
        self.edges = self.raw_data.get('edges', {})

        self.adj = {jid: [] for jid in self.joints}
        self.edge_lookup = {}

        for key, params in self.edges.items():
            try:
                u, v = key.split('_')
                if u in self.adj: self.adj[u].append(v)
                if v in self.adj: self.adj[v].append(u)
                self.edge_lookup[tuple(sorted((u, v)))] = params
            except ValueError:
                continue

    def get_edge_params(self, u, v):
        """
        获取边 u->v 的 DH 参数 (a, alpha) 和状态变量 (offset, q)
        """
        key = tuple(sorted((u, v)))
        params = self.edge_lookup.get(key)
        if not params:
            return 0.0, 0.0, 0.0, 0.0

        is_forward = (key == (u, v))

        a = params.get('a', 0.0)
        alpha = params.get('alpha', 0.0)

        if is_forward:
            q = params.get('state_source', 0.0)
            offset = params.get('offset_source', 0.0)
        else:
            q = params.get('state_target', 0.0)
            offset = params.get('offset_target', 0.0)

        return a, alpha, offset, q

    def _calculate_dh_matrix(self, a, alpha, d, theta):
        """标准 DH 参数转 4x4 矩阵"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def compute(self):
        """
        基于链路上下文 (Chain Context) 计算位姿。
        支持通过 settings 指定根节点。
        """
        if not self.joints:
            return {}, []

        # 【新增】确定根节点 (Fixed Base)
        # 1. 尝试从 settings 读取
        # 2. 否则默认取第一个定义的关节
        user_root = str(self.settings.get('root_node', ''))
        if user_root and user_root in self.joints:
            root_id = user_root
            print(f"📍 Root Node set to: {root_id} (from settings)")
        else:
            root_id = list(self.joints.keys())[0]
            # print(f"📍 Root Node defaulted to: {root_id}")

        neighbors = self.adj.get(root_id, [])

        if not neighbors:
            return {root_id: np.eye(4)}, []

        # 根节点的参考邻居 (用于定义 "Input" 状态，确立基座坐标系)
        # 注意：改变根节点会改变整个机构在空间中的绝对位置和姿态，
        # 但不会改变关节之间的相对关系（形状）。
        ref_node = neighbors[0]

        global_poses = {}
        global_poses[root_id] = np.eye(4)

        # 队列存储: (当前节点 u, 父节点 p)
        # p = -1 表示根节点
        queue = deque([(root_id, -1)])
        visited = {root_id}

        while queue:
            u, p = queue.popleft()

            # 1. 确定 "入边" (Input Edge) 的参数
            if p == -1:
                logical_parent = ref_node
                prev_node = ref_node
            else:
                logical_parent = p
                prev_node = p

            # 获取入边状态
            _, _, off_in, q_in = self.get_edge_params(u, prev_node)

            T_u = global_poses[u]

            # 2. 遍历邻居寻找 "出边" (Output Edge)
            for v in self.adj.get(u, []):
                if v == logical_parent:
                    continue

                if v in visited:
                    continue

                    # 获取出边状态
                a, alpha, off_out, q_out = self.get_edge_params(u, v)
                j_type = self.joints.get(u, 'R')

                # 3. 计算 DH 变量
                if j_type == 'R':
                    theta = (q_out - q_in) - np.pi
                    d = (off_out - off_in)
                else:  # 'P'
                    theta = (off_out - off_in) - np.pi
                    d = (q_out - q_in)

                T_step = self._calculate_dh_matrix(a, alpha, d, theta)
                T_v = T_u @ T_step

                global_poses[v] = T_v
                visited.add(v)
                queue.append((v, u))

        # B. 提取完整拓扑
        full_topology = []
        for key in self.edges.keys():
            try:
                u, v = key.split('_')
                full_topology.append([u, v])
            except ValueError:
                continue

        return global_poses, full_topology


# ==========================================
# 2. 封装函数
# ==========================================

def run_dh_to_pose(
        input_source: Union[str, Dict],
        output_path: Optional[str] = None,
        return_memory: bool = False,
        verbose: bool = True
) -> Optional[Dict]:
    if isinstance(input_source, str):
        if not os.path.exists(input_source):
            base_dir = os.path.dirname(os.path.abspath(__file__))
            alt_path = os.path.join(base_dir, input_source)
            if os.path.exists(alt_path):
                input_source = alt_path
            else:
                raise FileNotFoundError(f"Input file not found: {input_source}")
        with open(input_source, 'r', encoding='utf-8') as f:
            json_data = json.load(f)
        input_filename_base = os.path.splitext(os.path.basename(input_source))[0]
    else:
        json_data = input_source
        input_filename_base = "in_memory_data"

    if verbose:
        print(f"🔄 Calculating pose for: {input_filename_base}...")

    converter = DHToPoseConverter(json_data)
    matrix_poses, topology = converter.compute()

    simplified_pose = {}
    for jid, mat in matrix_poses.items():
        p = mat[:3, 3].tolist()
        z = mat[:3, 2].tolist()

        simplified_pose[jid] = {
            "p": p,
            "z": z
        }

    # 【新增】将 settings 透传给输出结果
    # 这样后续的积木 (如 pose_to_kinematics) 可以直接继承这些配置
    result_data = {
        "joints": converter.joints,
        "pose": simplified_pose,
        "topology": topology,
        "settings": converter.settings
    }

    if return_memory:
        if verbose: print(f"✅ Calculation done (returned in memory).")
        return result_data
    else:
        if not output_path:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            default_output_dir = os.path.join(base_dir, "output")

            if input_filename_base.endswith("_dh"):
                clean_name = input_filename_base[:-3]
                auto_filename = f"{clean_name}_pose.json"
            else:
                auto_filename = f"{input_filename_base}_pose.json"

            output_path = os.path.join(default_output_dir, auto_filename)

        output_dir = os.path.dirname(os.path.abspath(output_path))
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(result_data, f, indent=2)

        if verbose: print(f"✅ [DH->Pose] Saved to: {output_path}")
        return None


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_input = os.path.join(base_dir, "data", "Bennett_dh.json")

    parser = argparse.ArgumentParser(description="[Module] DH to Spatial Pose (Axis+Point)")
    parser.add_argument('-i', '--input', default=default_input, help=f"Input JSON")
    parser.add_argument('-o', '--output', default=None, help="Output JSON")
    parser.add_argument('-q', '--quiet', action='store_true', help="Suppress output")
    args = parser.parse_args()

    try:
        run_dh_to_pose(args.input, args.output, verbose=not args.quiet)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()