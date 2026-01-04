import json
import argparse
import os
import sys
import numpy as np
from typing import Union, Dict, Optional


class PoseToDHConverter:
    def __init__(self, json_data: Dict, tolerance: float = 1e-4):
        if 'pose' not in json_data or 'topology' not in json_data:
            raise ValueError("Input JSON must contain 'pose' and 'topology'.")

        self.pose_data = json_data['pose']
        self.topology = json_data['topology']
        self.joints_info = json_data.get('joints', {})
        self.nodes = list(self.pose_data.keys())

        # 【新增】数值清洗阈值
        self.tolerance = tolerance

    def _get_vector(self, node_id, key):
        return np.array(self.pose_data[node_id][key], dtype=np.float64)

    def _clean_float(self, val: float) -> float:
        """【核心】数值清洗：绝对值小于阈值则置为 0.0"""
        if abs(val) < self.tolerance:
            return 0.0
        return float(val)

    def _compute_geometry_params(self, p1, z1, p2, z2):
        """计算公垂线参数 (a, alpha, n, Pa, Pb)"""
        n = np.cross(z1, z2)
        n_norm = np.linalg.norm(n)

        # 判定平行或接近平行
        if n_norm < 1e-6:
            alpha = 0.0
            vec = p2 - p1
            # 平行时，公垂线方向取垂直于 Z1 的连线分量
            n = np.cross(z1, np.cross(vec, z1))
            if np.linalg.norm(n) < 1e-6:
                # 共线情况：任意取一个垂直方向
                n = np.cross(z1, np.array([1, 0, 0]))
                if np.linalg.norm(n) < 1e-6: n = np.cross(z1, np.array([0, 1, 0]))

            n = n / np.linalg.norm(n)
            a = np.abs(np.dot(p2 - p1, n))

            # 平行时的垂足计算
            t1 = np.dot(p2 - p1, z1)
            Pa = p1 + t1 * z1
            Pb = p2 + z2 * np.dot(Pa - p2, z2)

        else:  # 异面
            n = n / n_norm
            alpha = np.arctan2(n_norm, np.dot(z1, z2))
            a = np.abs(np.dot(p2 - p1, n))

            dot_12 = np.dot(z1, z2)
            denom = 1 - dot_12 ** 2

            # 【增强】防止 denom 过小导致数值不稳定
            if abs(denom) < 1e-9:
                denom = 1e-9

            vec = p2 - p1
            t1 = (np.dot(vec, z1) - np.dot(vec, z2) * dot_12) / denom
            t2 = (np.dot(vec, z1) * dot_12 - np.dot(vec, z2)) / denom

            Pa = p1 + z1 * t1
            Pb = p2 + z2 * t2

        return a, alpha, n, Pa, Pb

    def _normalize_node_params(self, node_id, attachments, axis_vec):
        """节点参数归一化 (计算相对 Offset 和 State)"""
        if not attachments: return {}

        # 1. 计算平均 Offset (用于中心化 d)
        d_values = [item['d_raw'] for item in attachments]
        d_mean = np.mean(d_values) if d_values else 0.0

        # 2. 建立局部坐标系
        base_n = attachments[0]['n_vec']
        local_x = base_n
        local_y = np.cross(axis_vec, local_x)

        # 3. 计算相对角度
        q_values = []
        for item in attachments:
            n = item['n_vec']
            angle = np.arctan2(np.dot(n, local_y), np.dot(n, local_x))
            q_values.append(angle)

        # 【新增】计算角度均值 (用于中心化 State)
        q_mean = np.mean(q_values) if q_values else 0.0

        normalized_map = {}
        for i, item in enumerate(attachments):
            edge_key = item['edge_key']

            # 执行中心化
            d_final = d_values[i] - d_mean
            q_final = q_values[i] - q_mean

            # 将角度规范化到 (-pi, pi]
            q_final = (q_final + np.pi) % (2 * np.pi) - np.pi

            normalized_map[edge_key] = {
                "geo_d": d_final,
                "geo_theta": q_final
            }

        return normalized_map

    def compute(self):
        node_attachments = {nid: [] for nid in self.nodes}
        edges_geo_data = {}

        # 1. 几何重建
        for edge in self.topology:
            u, v = edge[0], edge[1]
            if u not in self.pose_data or v not in self.pose_data: continue

            p_u, z_u = self._get_vector(u, 'p'), self._get_vector(u, 'z')
            p_v, z_v = self._get_vector(v, 'p'), self._get_vector(v, 'z')

            a, alpha, n_vec, Pa, Pb = self._compute_geometry_params(p_u, z_u, p_v, z_v)

            edge_key = f"{u}_{v}"
            # 记录几何参数
            edges_geo_data[tuple(sorted((u, v)))] = {"a": a, "alpha": alpha}

            # 记录连接点信息
            d_u_raw = np.dot(Pa - p_u, z_u)
            node_attachments[u].append({
                "edge_key": edge_key, "d_raw": d_u_raw, "n_vec": n_vec
            })

            d_v_raw = np.dot(Pb - p_v, z_v)
            node_attachments[v].append({
                "edge_key": edge_key, "d_raw": d_v_raw, "n_vec": n_vec
            })

        # 2. 节点参数归一化
        node_params_map = {}
        for nid in self.nodes:
            z_axis = self._get_vector(nid, 'z')
            node_params_map[nid] = self._normalize_node_params(nid, node_attachments[nid], z_axis)

        # 3. 组装结果 (含数值清洗和 R/P 映射)
        final_edges = {}
        for edge in self.topology:
            u, v = edge[0], edge[1]
            edge_id = f"{u}_{v}"

            geo = edges_geo_data.get(tuple(sorted((u, v))))
            if not geo: continue

            p_src = node_params_map[u].get(edge_id)
            p_tgt = node_params_map[v].get(edge_id)

            if p_src and p_tgt:
                # 映射 R/P 副逻辑
                type_u = self.joints_info.get(u, 'R')
                if type_u == 'P':
                    off_src, state_src = p_src['geo_theta'], p_src['geo_d']
                else:
                    off_src, state_src = p_src['geo_d'], p_src['geo_theta']

                type_v = self.joints_info.get(v, 'R')
                if type_v == 'P':
                    off_tgt, state_tgt = p_tgt['geo_theta'], p_tgt['geo_d']
                else:
                    off_tgt, state_tgt = p_tgt['geo_d'], p_tgt['geo_theta']

                # 应用数值清洗
                final_edges[edge_id] = {
                    "a": self._clean_float(geo['a']),
                    "alpha": self._clean_float(geo['alpha']),
                    "offset_source": self._clean_float(off_src),
                    "offset_target": self._clean_float(off_tgt),
                    "state_source": self._clean_float(state_src),
                    "state_target": self._clean_float(state_tgt)
                }

        return {
            "data": {
                "joints": self.joints_info,
                "edges": final_edges
            }
        }


# ==========================================
# CLI / API
# ==========================================

def run_pose_to_dh(input_source, output_path=None, return_memory=False, verbose=True, tolerance=1e-4):
    if isinstance(input_source, str):
        with open(input_source, 'r', encoding='utf-8') as f:
            json_data = json.load(f)
        input_name = os.path.splitext(os.path.basename(input_source))[0]
    else:
        json_data = input_source
        input_name = "memory_data"

    if verbose: print(f"📐 DH Extraction (tol={tolerance}) for: {input_name}...")

    # 传入 tolerance
    converter = PoseToDHConverter(json_data, tolerance=tolerance)
    result = converter.compute()

    if return_memory: return result

    if not output_path:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        clean_name = input_name.replace("_pose", "")
        output_path = os.path.join(base_dir, "output", f"{clean_name}_dh.json")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2)

    if verbose: print(f"✅ Saved to: {output_path}")
    return None


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_in = os.path.join(base_dir, "data", "Bennett_pose.json")

    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', default=default_in)
    parser.add_argument('-o', '--output', default=None)
    # 允许用户在命令行调整阈值
    parser.add_argument('-t', '--tol', type=float, default=1e-3, help="Zero threshold (default 1e-3)")
    parser.add_argument('-q', '--quiet', action='store_true')
    args = parser.parse_args()

    if args.input == default_in and not os.path.exists(default_in):
        alt = os.path.join(base_dir, "data", "pose.json")
        if os.path.exists(alt): args.input = alt

    run_pose_to_dh(args.input, args.output, False, not args.quiet, args.tol)


if __name__ == "__main__":
    main()