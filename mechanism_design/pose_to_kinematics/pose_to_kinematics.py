import json
import argparse
import os
import sys
import numpy as np
import networkx as nx
import itertools
from typing import Union, Dict, Optional


# ==============================================================================
# PART 1: 数学核心 (原 dof_analysis.py)
# ==============================================================================

def _lie_bracket(twist1, twist2):
    w1, v1 = twist1[:3], twist1[3:]
    w2, v2 = twist2[:3], twist2[3:]
    w_new = np.cross(w1, w2)
    v_new = np.cross(w1, v2) - np.cross(w2, v1)
    return np.concatenate([w_new, v_new])


def _build_extended_path_nx(G, raw_path):
    """
    (旧版兜底逻辑) 构建扩展路径 [ghost_prev, start, ..., end, ghost_next]
    """
    if not raw_path: return None
    start, end = raw_path[0], raw_path[-1]
    path_set = set(raw_path)
    ghost_prev, ghost_next = None, None
    try:
        nbrs = list(G.neighbors(start))
        valid = [n for n in nbrs if n not in path_set]
        if valid: ghost_prev = min(valid)
    except:
        pass
    try:
        nbrs = list(G.neighbors(end))
        valid = [n for n in nbrs if n not in path_set]
        if valid: ghost_next = max(valid)
    except:
        pass
    return [ghost_prev] + raw_path + [ghost_next]


def augment_k_matrix_to_remove_modes(K, bad_modes, weight=10.0):
    if not bad_modes: return K
    rows_to_add = []
    for mode in bad_modes:
        mode_norm = mode / (np.linalg.norm(mode) + 1e-12)
        rows_to_add.append(mode_norm * weight)
    return np.vstack((K, np.array(rows_to_add)))


def detect_instantaneous_modes(K_func_builder, candidate_modes, loops, edge_to_col, node_screw_map):
    dt = 1e-3
    idof_vectors = []
    K_curr = K_func_builder(node_screw_map)
    K_pinv = np.linalg.pinv(K_curr, rcond=1e-3)

    for i, mode_vec in enumerate(candidate_modes):
        mode_vec = mode_vec / (np.linalg.norm(mode_vec) + 1e-9)
        loop_drifts_list = []
        for loop in loops:
            loop_drift = np.zeros(6)
            L = len(loop)
            current_twist_sum = np.zeros(6)
            for j in range(L):
                curr_node = loop[j]
                prev_node = loop[(j - 1 + L) % L]
                next_node = loop[(j + 1) % L]

                v_next = mode_vec[edge_to_col.get((curr_node, next_node), -1)] if (curr_node,
                                                                                   next_node) in edge_to_col else 0.0
                v_prev = mode_vec[edge_to_col.get((curr_node, prev_node), -1)] if (curr_node,
                                                                                   prev_node) in edge_to_col else 0.0

                d_theta = (v_next - v_prev) * dt
                screw = node_screw_map[curr_node]

                loop_drift += _lie_bracket(current_twist_sum, screw) * d_theta
                current_twist_sum += screw * d_theta
            loop_drifts_list.append(loop_drift)

        full_drift = np.concatenate(loop_drifts_list)
        projected = K_curr @ (K_pinv @ full_drift)
        if np.linalg.norm(full_drift) > 1e-12:
            if np.linalg.norm(full_drift - projected) / np.linalg.norm(full_drift) > 0.1:
                idof_vectors.append(mode_vec)

    return idof_vectors


def analyze_mobility_anchor(node_screw_map, topology_edges, nodes_info,
                            rigid_body_sets=None, base_node=None, ee_node=None,
                            manual_extended_path=None, dof_threshold=1e-4):
    if base_node is None or ee_node is None: return {"error": "Args missing"}
    if rigid_body_sets is None: rigid_body_sets = []

    G_raw = nx.Graph()
    for u, v in topology_edges: G_raw.add_edge(u, v)

    try:
        loops = nx.cycle_basis(G_raw)
    except:
        loops = []

    loop_nodes_set = set()
    loop_edges_set = set()
    source_edges = loops if len(loops) > 0 else topology_edges
    for item in source_edges:
        nodes = item if len(loops) > 0 else list(item)
        L = len(nodes)
        for i in range(L):
            if len(loops) > 0:
                u, v = nodes[i], nodes[(i + 1) % L]
            else:
                u, v = nodes[0], nodes[1]
                if i > 0: break

            loop_nodes_set.add(u);
            loop_nodes_set.add(v)
            loop_edges_set.add(tuple(sorted((u, v))))

    directed_edges = []
    for u, v in loop_edges_set:
        directed_edges.append((u, v));
        directed_edges.append((v, u))

    edge_to_col = {edge: i for i, edge in enumerate(directed_edges)}
    num_vars = len(directed_edges)
    num_loops = len(loops)
    num_nodes = len(loop_nodes_set)
    gauge_n = num_nodes

    def build_K_matrix(current_screw_map):
        if num_loops == 0: return np.zeros((6, num_vars))
        K_local = np.zeros((6 * num_loops, num_vars), dtype=np.float64)
        ortho_basis = np.eye(6, dtype=np.float64)
        for l_idx, loop_nodes in enumerate(loops):
            L = len(loop_nodes)
            row_start = l_idx * 6
            current_loop_set = set(loop_nodes)
            is_rigid = False
            for rb_set in rigid_body_sets:
                if current_loop_set == rb_set: is_rigid = True; break

            for i in range(L):
                curr = loop_nodes[i]
                next_node = loop_nodes[(i + 1) % L]
                prev_node = loop_nodes[(i - 1 + L) % L]
                screw = ortho_basis[i % 6] if is_rigid else current_screw_map[curr]
                if (curr, next_node) in edge_to_col:
                    K_local[row_start:row_start + 6, edge_to_col[(curr, next_node)]] += screw
                if (curr, prev_node) in edge_to_col:
                    K_local[row_start:row_start + 6, edge_to_col[(curr, prev_node)]] -= screw
        return K_local

    K_initial = build_K_matrix(node_screw_map)
    U, S_init, Vh = np.linalg.svd(K_initial)
    full_S = np.zeros(num_vars)
    full_S[:len(S_init)] = S_init
    spectrum = np.flip(full_S)

    potential_indices = [i for i in range(gauge_n, num_vars) if spectrum[i] < 0.1]
    idof_vectors = []
    if potential_indices:
        potential_basis = np.flip(Vh, axis=0)[potential_indices, :]
        idof_vectors = detect_instantaneous_modes(build_K_matrix, potential_basis, loops, edge_to_col, node_screw_map)

    K_final = augment_k_matrix_to_remove_modes(K_initial, idof_vectors) if idof_vectors else K_initial
    U_f, S_f, Vh_f = np.linalg.svd(K_final)

    S_padded = np.zeros(num_vars)
    S_padded[:min(K_final.shape)] = S_f
    final_spectrum = np.flip(S_padded)
    evecs = np.flip(Vh_f, axis=0).T

    valid_evals = final_spectrum[gauge_n:]
    physical_dof = 0
    max_gap = 0.0
    pot_idx = 0
    if len(valid_evals) > 0:
        for i in range(min(6, len(valid_evals) - 1)):
            v_curr = max(valid_evals[i], 1e-12)
            gap = valid_evals[i + 1] / v_curr
            if v_curr < dof_threshold and gap > 10.0:
                if gap > max_gap: max_gap = gap; pot_idx = i + 1
    physical_dof = pot_idx if max_gap > 10.0 else np.sum(valid_evals < dof_threshold)

    ee_rank = 0
    motion_desc = "Locked"
    ee_basis = []
    null_space = None
    if physical_dof > 0:
        idx = np.arange(gauge_n, gauge_n + int(physical_dof))
        if idx.max() < evecs.shape[1]: null_space = evecs[:, idx]

    if null_space is not None:
        path = manual_extended_path if manual_extended_path else []
        if not path and nx.has_path(G_raw, base_node, ee_node):
            raw = nx.shortest_path(G_raw, base_node, ee_node)
            path = _build_extended_path_nx(G_raw, raw)

        J_path = np.zeros((6, num_vars))
        if path and len(path) >= 3:
            for i in range(1, len(path) - 1):
                c, n, p = path[i], path[i + 1], path[i - 1]
                s = node_screw_map[c]
                if n is not None and (c, n) in edge_to_col: J_path[:, edge_to_col[(c, n)]] += s
                if p is not None and (c, p) in edge_to_col: J_path[:, edge_to_col[(c, p)]] -= s

        T_ee = J_path @ null_space
        try:
            Ue, Se, Vhe = np.linalg.svd(T_ee)
            max_s = Se[0] if len(Se) > 0 else 0
            ee_rank = np.sum(Se > max(1e-6, max_s * 1e-4))
            if ee_rank > 0:
                ee_basis = Ue[:, :ee_rank].T.tolist()
                if ee_rank == 1:
                    w = Ue[:3, 0]
                    if np.linalg.norm(w) < 1e-5:
                        motion_desc = "1P (Pure Translation)"
                    else:
                        h = np.dot(w, Ue[3:, 0]) / (np.linalg.norm(w) ** 2)
                        motion_desc = "1R (Pure Rotation)" if abs(h) < 1e-2 else f"1H (h={h:.2f})"
                else:
                    motion_desc = f"{ee_rank}-DOF Spatial"
        except:
            pass

    return {
        "dof": int(physical_dof),
        "idof_count": len(idof_vectors),
        "motion_type": motion_desc,
        "ee_rank": int(ee_rank),
        "ee_twist_basis": ee_basis,
        "spectrum": final_spectrum.tolist()
    }


# ==============================================================================
# PART 2: 适配器 (Adapter)
# ==============================================================================

class PoseKinematicsRunner:
    def __init__(self, json_data):
        self.pose_data = json_data['pose']
        self.topology = json_data['topology']
        # 读取 JSON 中的 settings 字段
        self.settings = json_data.get('settings', {})
        self.joints_info = json_data.get('joints', {})
        self.nodes = list(self.pose_data.keys())

    def _compute_screws(self):
        screws, nodes_info = {}, {}
        dists = []
        for u, v in self.topology:
            if u in self.pose_data and v in self.pose_data:
                p1 = np.array(self.pose_data[u]['p'])
                p2 = np.array(self.pose_data[v]['p'])
                dists.append(np.linalg.norm(p1 - p2))
        L_char = np.mean(dists) if dists else 1.0
        if L_char < 1e-6: L_char = 1.0
        self.L_char = L_char

        for jid, d in self.pose_data.items():
            if 'p' not in d or 'z' not in d: continue
            p = np.array(d['p'], float)
            z = np.array(d['z'], float)
            if np.linalg.norm(z) > 1e-9: z /= np.linalg.norm(z)

            jt = self.joints_info.get(jid, 'R')
            s = np.zeros(6)
            if jt == 'R':
                s[:3] = z
                s[3:] = np.cross(p, z) / L_char
            else:
                s[3:] = z
            screws[jid] = s
            nodes_info[jid] = {'screw': s, 'pos': p, 'axis': z}
        return screws, nodes_info

    def _construct_smart_path(self, base_link_str, ee_link_str):
        """
        [New] 智能路径规划算法
        自动尝试基座和末端杆件的所有方向组合，寻找一条合法的补余路径。
        """
        # 1. 准备原始图 (从 Topology 构建)
        G_full = nx.Graph()
        for u, v in self.topology:
            G_full.add_edge(u, v)

        try:
            b_u, b_v = base_link_str.split('_')
            e_u, e_v = ee_link_str.split('_')
        except ValueError:
            print(f"❌ Error: Link format must be 'Node1_Node2', got {base_link_str}, {ee_link_str}")
            return None, None, None

        # 2. 准备“切割图” (移除指定的两根杆件)
        G_cut = G_full.copy()
        if G_cut.has_edge(b_u, b_v): G_cut.remove_edge(b_u, b_v)
        if G_cut.has_edge(e_u, e_v): G_cut.remove_edge(e_u, e_v)

        # 3. 定义可能的起止点组合
        base_options = [
            {'head': b_u, 'start': b_v, 'desc': f"{b_u}(Ghost)->{b_v}(Start)"},
            {'head': b_v, 'start': b_u, 'desc': f"{b_v}(Ghost)->{b_u}(Start)"}
        ]
        ee_options = [
            {'end': e_u, 'tail': e_v, 'desc': f"{e_u}(End)->{e_v}(Ghost)"},
            {'end': e_v, 'tail': e_u, 'desc': f"{e_v}(End)->{e_u}(Ghost)"}
        ]

        print(f"🛣️  Smart Path Finding: {base_link_str} ... {ee_link_str}")
        valid_path_info = None

        # 4. 遍历所有组合，寻找通路
        for b_opt, e_opt in itertools.product(base_options, ee_options):
            start_node = b_opt['start']
            end_node = e_opt['end']
            try:
                path = nx.shortest_path(G_cut, source=start_node, target=end_node)
                valid_path_info = {
                    'path': path,
                    'head': b_opt['head'],
                    'tail': e_opt['tail'],
                    'desc': f"Plan [{b_opt['desc']} ... {e_opt['desc']}]"
                }
                print(f"   ✅ {valid_path_info['desc']} -> Connected!")
                break
            except nx.NetworkXNoPath:
                continue

        # 5. 构建结果
        if valid_path_info:
            full_chain = [valid_path_info['head']] + valid_path_info['path'] + [valid_path_info['tail']]
            calc_base = valid_path_info['path'][0]
            calc_ee = valid_path_info['path'][-1]
            return full_chain, calc_base, calc_ee
        else:
            print("❌ All combinations failed. Cannot bypass the specified links.")
            return None, None, None

    def run_analysis(self, base_arg=None, ee_arg=None):
        screws, nodes_info = self._compute_screws()

        # 1. 确定配置来源 (优先参数，其次 Settings)
        base_target = base_arg if base_arg else self.settings.get('base_link', self.settings.get('base_node'))
        ee_target = ee_arg if ee_arg else self.settings.get('ee_link', self.settings.get('ee_node'))

        extended_path = None
        calc_base = None
        calc_ee = None

        # 2. 判断是否为 Link 模式 ("_" 检测)
        is_link_mode = False
        if base_target and ee_target and ('_' in str(base_target) or '_' in str(ee_target)):
            is_link_mode = True
            extended_path, calc_base, calc_ee = self._construct_smart_path(str(base_target), str(ee_target))
            if extended_path is None:
                print("⚠️  Warning: Failed to construct path from links. Fallback to node auto-detect.")

        # 3. 回退策略 (Node 模式 或 Link 失败)
        if not extended_path:
            # 尝试把 Link 字符串降级为 Node (取第一个)
            calc_base = str(base_target).split('_')[0] if base_target else None
            calc_ee = str(ee_target).split('_')[0] if ee_target else None

            if not calc_base or not calc_ee:
                sorted_nodes = sorted(self.nodes)
                if not calc_base: calc_base = sorted_nodes[0]
                if not calc_ee: calc_ee = sorted_nodes[-1] if len(sorted_nodes) > 1 else calc_base

            try:
                G = nx.Graph()
                G.add_edges_from([tuple(e) for e in self.topology])
                if nx.has_path(G, calc_base, calc_ee):
                    raw_path = nx.shortest_path(G, calc_base, calc_ee)
                    extended_path = _build_extended_path_nx(G, raw_path)
            except:
                pass

        # 4. 执行计算
        result = analyze_mobility_anchor(
            screws, [tuple(e) for e in self.topology], nodes_info, [],
            calc_base, calc_ee, extended_path
        )

        # 5. 结果包装
        ghost_base = extended_path[0] if extended_path else None
        ghost_ee = extended_path[-1] if extended_path else None

        display_base = base_target if is_link_mode else f"{ghost_base}-{calc_base}" if ghost_base else str(calc_base)
        display_ee = ee_target if is_link_mode else f"{calc_ee}-{ghost_ee}" if ghost_ee else str(calc_ee)

        result["anchors"] = {
            "display_base": display_base,
            "display_ee": display_ee,
            "calc_base": calc_base,
            "calc_ee": calc_ee
        }
        result["L_char"] = self.L_char
        return result


# ==============================================================================
# PART 3: 接口与CLI
# ==============================================================================

def run_pose_to_kinematics(input_source, base=None, ee=None, output_path=None, verbose=True):
    if isinstance(input_source, str):
        with open(input_source, 'r', encoding='utf-8') as f:
            json_data = json.load(f)
        name = os.path.basename(input_source)
    else:
        json_data = input_source
        name = "memory_data"

    if verbose: print(f"📐 Analyzing Kinematics for: {name}")

    runner = PoseKinematicsRunner(json_data)
    result = runner.run_analysis(base, ee)

    if verbose: print_report(result)

    if output_path:
        d = os.path.dirname(os.path.abspath(output_path))
        if d and not os.path.exists(d): os.makedirs(d)
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, default=lambda x: float(x) if isinstance(x, np.float64) else x)
        if verbose: print(f"✅ Report saved to: {output_path}")
    return result


def print_report(result):
    anchors = result.get('anchors', {})
    print("\n" + "=" * 60)
    print(f"📊 Kinematics Report (L_char={result.get('L_char', 1.0):.2f})")
    print("=" * 60)
    print(f"⚓ Reference:          {anchors.get('display_base', '?')}  ->  {anchors.get('display_ee', '?')}")
    print(f"⚙️  DOF (Mobility):       {result['dof']}")
    print(f"⚠️  Instantaneous DOFs:   {result.get('idof_count', 0)}")
    print(f"🎯 End-Effector Rank:    {result['ee_rank']}")
    print(f"📝 Motion Type:          {result['motion_type']}")
    print("-" * 60)
    print("🌊 Twist Basis (Normalized):")
    if result['ee_twist_basis']:
        for i, twist in enumerate(result['ee_twist_basis']):
            fmt = ", ".join([f"{x:>7.4f}" for x in twist])
            print(f"   Mode {i + 1}: [{fmt}]")
    else:
        print("   (Locked or No Motion defined)")
    print("=" * 60)


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_input = os.path.join(base_dir, "data", "Bennett_pose.json")
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', default=default_input)
    parser.add_argument('-o', '--output', default=None)
    parser.add_argument('-b', '--base', default=None, help="Base Link (e.g. '1_4') or Node ('1')")
    parser.add_argument('-e', '--ee', default=None, help="EE Link (e.g. '3_4') or Node ('3')")
    parser.add_argument('-q', '--quiet', action='store_true')
    args = parser.parse_args()

    if args.input == default_input and not os.path.exists(default_input):
        fallback = os.path.join(base_dir, "data", "pose.json")
        if os.path.exists(fallback): args.input = fallback

    try:
        run_pose_to_kinematics(args.input, args.base, args.ee, args.output, not args.quiet)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()