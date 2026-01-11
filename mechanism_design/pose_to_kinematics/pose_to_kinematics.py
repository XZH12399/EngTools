import json
import argparse
import os
import sys
import numpy as np
import networkx as nx
import itertools
import random
from scipy.spatial.transform import Rotation as R
from typing import Union, Dict, Optional, List, Tuple


# ==============================================================================
# PART 1: Mathematical Core
# ==============================================================================

def _lie_bracket(twist1, twist2):
    """Computes the Lie Bracket (commutator) of two twists."""
    w1, v1 = twist1[:3], twist1[3:]
    w2, v2 = twist2[:3], twist2[3:]
    w_new = np.cross(w1, w2)
    v_new = np.cross(w1, v2) - np.cross(w2, v1)
    return np.concatenate([w_new, v_new])


def _build_extended_path_nx(G, raw_path):
    """Build extended path [ghost_prev, start, ..., end, ghost_next]"""
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
    """Adds penalty rows to the Jacobian to remove specific DOF modes."""
    # Use len() check to avoid NumPy boolean ambiguity error
    if len(bad_modes) == 0:
        return K

    rows_to_add = []
    for mode in bad_modes:
        mode_norm = mode / (np.linalg.norm(mode) + 1e-12)
        rows_to_add.append(mode_norm * weight)
    return np.vstack((K, np.array(rows_to_add)))


def calculate_loop_drift(mode_vec, loops, edge_to_col, node_screw_map, dt=1e-3):
    """
    Calculates the geometric drift (Lie Bracket accumulation)
    for a given motion mode vector.
    """
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

            # Accumulate Lie Bracket drift: [Cumulative, Current] * dt
            loop_drift += _lie_bracket(current_twist_sum, screw) * d_theta
            current_twist_sum += screw * d_theta
        loop_drifts_list.append(loop_drift)

    return np.concatenate(loop_drifts_list)


def detect_instantaneous_modes_numerical(K_curr, candidate_modes, loops, edge_to_col, node_screw_map):
    """
    Method 1: Numerical Drift Check (Original).
    Simulates a small step and checks if the drift can be projected back to the null space.
    """
    idof_vectors = []
    K_pinv = np.linalg.pinv(K_curr, rcond=1e-3)

    for mode_vec in candidate_modes:
        full_drift = calculate_loop_drift(mode_vec, loops, edge_to_col, node_screw_map, dt=1e-3)

        # Check if drift is in the range of K (correctable) or orthogonal to it (constraint violation)
        projected_correction = K_curr @ (K_pinv @ full_drift)
        drift_norm = np.linalg.norm(full_drift)

        if drift_norm > 1e-12:
            # Ratio of uncorrectable drift to total drift
            uncorrectable_ratio = np.linalg.norm(full_drift - projected_correction) / drift_norm
            if uncorrectable_ratio > 0.1:
                idof_vectors.append(mode_vec)  # Mark as "bad" (shaky/instantaneous only)

    return idof_vectors


def detect_modes_symmetric_svd(K_curr, candidate_modes, loops, edge_to_col, node_screw_map):
    """
    Method 2: Symmetric SVD / Eigenvalue Analysis (Based on Fernández de Bustos et al.)
    Constructs the quadratic constraint matrix [B] and checks its definiteness.
    """
    if len(candidate_modes) == 0:
        return []

    # 1. Identify the Constraint Space (Left Null Space of K)
    U, S, _ = np.linalg.svd(K_curr)
    # Threshold for "zero" singular value, indicating a constraint
    constraint_indices = np.where(S < 1e-4)[0]

    if len(constraint_indices) == 0:
        return []

    U_constraints = U[:, constraint_indices]

    # 2. Construct the Quadratic Matrix [B]
    num_modes = len(candidate_modes)
    B_matrix = np.zeros((num_modes, num_modes))

    basis_drifts = []
    for i in range(num_modes):
        d = calculate_loop_drift(candidate_modes[i], loops, edge_to_col, node_screw_map)
        basis_drifts.append(d)

    for i in range(num_modes):
        for j in range(i, num_modes):
            if i == j:
                drift_vec = basis_drifts[i]
            else:
                # Interaction term check: D(u+v) - D(u) - D(v)
                vec_sum = candidate_modes[i] + candidate_modes[j]
                drift_sum = calculate_loop_drift(vec_sum, loops, edge_to_col, node_screw_map)
                drift_vec = (drift_sum - basis_drifts[i] - basis_drifts[j]) * 0.5

            # Project drift onto the constraint space (U_constraints)
            # We take the component along the most restrictive constraint
            dominant_constraint = U_constraints[:, -1]
            val = np.dot(dominant_constraint, drift_vec)
            B_matrix[i, j] = val
            B_matrix[j, i] = val

    # 3. Eigenvalue Analysis (Definiteness Check)
    eigvals = np.linalg.eigvalsh(B_matrix)

    # Check for definiteness
    tol = 1e-9
    pos = np.sum(eigvals > tol)
    neg = np.sum(eigvals < -tol)

    is_definite = (pos == 0 and neg > 0) or (pos > 0 and neg == 0)

    if is_definite and (pos > 0 or neg > 0):
        # Return list to ensure compatibility with len() checks
        return list(candidate_modes)
    else:
        # Indefinite -> Mobile
        return []


def analyze_mobility_anchor(node_screw_map, topology_edges, nodes_info,
                            rigid_body_sets=None, base_node=None, ee_node=None,
                            manual_extended_path=None, dof_threshold=1e-4,
                            second_order_method='drift'):
    if base_node is None or ee_node is None: return {"error": "Args missing"}
    if rigid_body_sets is None: rigid_body_sets = []

    G_raw = nx.Graph()
    for u, v in topology_edges: G_raw.add_edge(u, v)

    try:
        loops = nx.cycle_basis(G_raw)
    except:
        loops = []

    # --- Setup Topology & Indices ---
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

            loop_nodes_set.add(u)
            loop_nodes_set.add(v)
            loop_edges_set.add(tuple(sorted((u, v))))

    directed_edges = []
    for u, v in loop_edges_set:
        directed_edges.append((u, v))
        directed_edges.append((v, u))

    edge_to_col = {edge: i for i, edge in enumerate(directed_edges)}
    num_vars = len(directed_edges)
    num_loops = len(loops)
    gauge_n = len(loop_nodes_set)

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
                if current_loop_set == rb_set:
                    is_rigid = True
                    break

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

    # --- 1. First Order Analysis (SVD) ---
    K_initial = build_K_matrix(node_screw_map)
    U, S_init, Vh = np.linalg.svd(K_initial)
    spectrum = np.flip(np.zeros(num_vars))
    spectrum[:len(S_init)] = S_init
    spectrum = np.flip(spectrum)

    # Detect potential Instantaneous Modes (Null Space)
    potential_indices = [i for i in range(gauge_n, num_vars) if spectrum[i] < 0.1]
    potential_basis = []
    if potential_indices:
        potential_basis = np.flip(Vh, axis=0)[potential_indices, :]

    # --- 2. Second Order Analysis (Switchable) ---
    bad_modes = []
    if len(potential_basis) > 0:
        if second_order_method == 'takagi':
            bad_modes = detect_modes_symmetric_svd(K_initial, potential_basis, loops, edge_to_col, node_screw_map)
        else:
            bad_modes = detect_instantaneous_modes_numerical(K_initial, potential_basis, loops, edge_to_col,
                                                             node_screw_map)

    # Filter out bad modes (shaky DOFs)
    K_final = augment_k_matrix_to_remove_modes(K_initial, bad_modes)

    # Re-evaluate SVD after augmentation
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

    # --- 3. End Effector Analysis ---
    ee_rank = 0;
    motion_desc = "Locked";
    ee_basis = [];
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

                # [RESTORED LOGIC] Detailed Motion Type Classification
                if ee_rank == 1:
                    w = Ue[:3, 0]
                    w_norm = np.linalg.norm(w)
                    if w_norm < 1e-5:
                        motion_desc = "1P (Pure Translation)"
                    else:
                        h = np.dot(w, Ue[3:, 0]) / (w_norm ** 2)
                        if abs(h) < 1e-2:
                            motion_desc = "1R (Pure Rotation)"
                        else:
                            motion_desc = f"1H (h={h:.2f})"
                else:
                    motion_desc = f"{ee_rank}-DOF Spatial"
            else:
                motion_desc = "Internal Motion Only"
        except:
            pass

    return {
        "dof": int(physical_dof),
        "idof_count": len(bad_modes),
        "motion_type": motion_desc,
        "ee_rank": int(ee_rank),
        "ee_twist_basis": ee_basis,
        "spectrum": final_spectrum.tolist(),
        "method_used": second_order_method
    }


# ==============================================================================
# PART 2: Adapter
# ==============================================================================

class PoseKinematicsRunner:
    def __init__(self, json_data):
        self.pose_data = json_data['pose']
        self.topology = json_data['topology']
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
            p, z = np.array(d['p'], float), np.array(d['z'], float)
            norm_z = np.linalg.norm(z)
            if norm_z > 1e-9:
                z /= norm_z
            else:
                z = np.array([0.0, 0.0, 1.0])

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

    def _create_perturbed_screw_map(self, original_screws, perturbation_scale=1e-4):
        perturbed = {}
        for node, screw in original_screws.items():
            w = screw[:3]
            rand_axis = np.random.randn(3)
            rand_axis /= np.linalg.norm(rand_axis) + 1e-9
            theta = perturbation_scale
            r = R.from_rotvec(rand_axis * theta).as_matrix()
            w_new = r @ w
            w_new /= np.linalg.norm(w_new) + 1e-9
            s_final = screw.copy()
            s_final[:3] = w_new
            s_final += np.random.randn(6) * perturbation_scale
            perturbed[node] = s_final
        return perturbed

    def _construct_smart_path(self, base_link_str, ee_link_str):
        """
        智能路径构建：支持 "杆件-杆件"、"杆件-节点"、"节点-节点" 的任意组合。
        如果输入是杆件 "u_v"，会自动切断 u-v 边以确定延伸方向。
        """
        G_full = nx.Graph()
        for u, v in self.topology: G_full.add_edge(u, v)

        # --- 内部辅助函数：解析节点或杆件选项 ---
        def parse_anchor_opts(anchor_str):
            s = str(anchor_str)
            if '_' in s:
                # 如果是杆件 "A_B"
                # 选项1: 从 A 出发 (Ghost是 B)
                # 选项2: 从 B 出发 (Ghost是 A)
                u, v = s.split('_')
                # 返回: (选项列表, 需要切断的边)
                return [
                    {'node': u, 'ghost': v}, 
                    {'node': v, 'ghost': u}
                ], (u, v)
            else:
                # 如果是单节点 "A"
                # 选项: 从 A 出发 (Ghost是 None，留给后续自动处理或留空)
                return [{'node': s, 'ghost': None}], None

        # 1. 解析基座和末端
        base_opts, base_edge_to_cut = parse_anchor_opts(base_link_str)
        ee_opts, ee_edge_to_cut = parse_anchor_opts(ee_link_str)

        # 2. 构建切割图 (G_cut)
        # 如果定义了杆件，必须切断杆件内部连接，强迫路径向外寻找
        G_cut = G_full.copy()
        
        if base_edge_to_cut and G_cut.has_edge(*base_edge_to_cut):
            G_cut.remove_edge(*base_edge_to_cut)
        
        if ee_edge_to_cut and G_cut.has_edge(*ee_edge_to_cut):
            G_cut.remove_edge(*ee_edge_to_cut)

        # 3. 组合寻找最短路径
        # 使用 itertools.product 会更优雅，这里用双重循环直观展示
        for b_opt in base_opts:
            for e_opt in ee_opts:
                try:
                    # 在切断了内部连接的图中寻找路径
                    path = nx.shortest_path(G_cut, source=b_opt['node'], target=e_opt['node'])
                    
                    # 路径构建成功！
                    # 组装完整路径: [BaseGhost, Start, ..., End, EEGhost]
                    # 注意：如果 Ghost 是 None，列表里就是 None，这是允许的
                    full_path = [b_opt['ghost']] + path + [e_opt['ghost']]
                    
                    return full_path, path[0], path[-1]
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    continue
        
        # 如果所有组合都通不通
        return None, None, None

    def run_analysis(self, base_arg=None, ee_arg=None, method='drift'):
        screws, nodes_info = self._compute_screws()
        base_target = base_arg if base_arg else self.settings.get('base_link', self.settings.get('base_node'))
        ee_target = ee_arg if ee_arg else self.settings.get('ee_link', self.settings.get('ee_node'))

        extended_path, calc_base, calc_ee = None, None, None
        if base_target and ee_target and ('_' in str(base_target) or '_' in str(ee_target)):
            extended_path, calc_base, calc_ee = self._construct_smart_path(str(base_target), str(ee_target))

        if not extended_path:
            calc_base = str(base_target).split('_')[0] if base_target else sorted(self.nodes)[0]
            calc_ee = str(ee_target).split('_')[0] if ee_target else (
                sorted(self.nodes)[-1] if len(self.nodes) > 1 else calc_base)
            try:
                G = nx.Graph()
                G.add_edges_from([tuple(e) for e in self.topology])
                if nx.has_path(G, calc_base, calc_ee):
                    extended_path = _build_extended_path_nx(G, nx.shortest_path(G, calc_base, calc_ee))
            except:
                pass

        # Standard Analysis
        result = analyze_mobility_anchor(
            screws, [tuple(e) for e in self.topology], nodes_info, [],
            calc_base, calc_ee, extended_path,
            second_order_method=method
        )

        # Perturbation Check
        if result['dof'] == 0 and result['idof_count'] > 0:
            print(f"⚠️  Potential Singularity ({method}). Retrying with Perturbation...")
            perturbed_screws = self._create_perturbed_screw_map(screws)
            perturbed_result = analyze_mobility_anchor(
                perturbed_screws, [tuple(e) for e in self.topology], nodes_info, [],
                calc_base, calc_ee, extended_path,
                second_order_method=method
            )
            if perturbed_result['dof'] > 0:
                result = perturbed_result
                result['is_perturbed'] = True

        ghost_base = extended_path[0] if extended_path else None
        ghost_ee = extended_path[-1] if extended_path else None

        result["anchors"] = {
            "display_base": base_target if base_target else f"{ghost_base}-{calc_base}",
            "display_ee": ee_target if ee_target else f"{calc_ee}-{ghost_ee}",
            "calc_base": calc_base,
            "calc_ee": calc_ee
        }
        result["L_char"] = self.L_char

        return result


# ==============================================================================
# PART 3: Interface
# ==============================================================================

def print_report(result):
    anchors = result.get('anchors', {})
    print("\n" + "=" * 60)
    print(f"📊 Kinematics Report (L_char={result.get('L_char', 1.0):.2f})")
    print(f"🔬 Method Used:       {result.get('method_used', 'unknown').upper()}")
    print("=" * 60)
    print(f"⚓ Reference:          {anchors.get('display_base', '?')}  ->  {anchors.get('display_ee', '?')}")
    print(f"⚙️  DOF (Mobility):       {result['dof']}")
    print(f"⚠️  Instantaneous DOFs:   {result.get('idof_count', 0)}")
    print(f"🎯 End-Effector Rank:    {result['ee_rank']}")
    print(f"📝 Motion Type:          {result['motion_type']}")
    if result.get('is_perturbed', False):
        print("💡 Note: Result derived via infinitesimal perturbation.")
    print("-" * 60)
    print("🌊 Twist Basis (Normalized):")
    if result['ee_twist_basis']:
        for i, twist in enumerate(result['ee_twist_basis']):
            fmt = ", ".join([f"{x:>7.4f}" for x in twist])
            print(f"   Mode {i + 1}: [{fmt}]")
    else:
        print("   (Locked or No Motion defined)")
    print("=" * 60)


def run_pose_to_kinematics(input_source, base=None, ee=None, output_path=None, verbose=True, method='drift'):
    if isinstance(input_source, str):
        with open(input_source, 'r', encoding='utf-8') as f:
            json_data = json.load(f)
    else:
        json_data = input_source

    if verbose: print(f"📐 Kinematics Analysis | Method: {method.upper()}")
    runner = PoseKinematicsRunner(json_data)
    result = runner.run_analysis(base, ee, method=method)

    if verbose:
        print_report(result)

    if output_path:
        with open(output_path, 'w') as f: json.dump(result, f, indent=2, default=float)
    return result


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_input = os.path.join(base_dir, "data", "Bennett_pose.json")

    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', default=default_input)
    parser.add_argument('-o', '--output', default=None)
    parser.add_argument('-b', '--base', default=None)
    parser.add_argument('-e', '--ee', default=None)
    parser.add_argument('-m', '--method', choices=['drift', 'takagi'], default='drift',
                        help="Second-order analysis method: 'drift' (numerical check) or 'takagi' (symmetric SVD)")
    parser.add_argument('-q', '--quiet', action='store_true')
    args = parser.parse_args()

    if args.input == default_input and not os.path.exists(default_input):
        fallback = os.path.join(base_dir, "data", "pose.json")
        if os.path.exists(fallback):
            args.input = fallback
        elif not args.quiet:
            print(f"⚠️  Default input not found: {default_input}")

    try:
        run_pose_to_kinematics(args.input, args.base, args.ee, args.output, not args.quiet, args.method)
    except Exception as e:
        print(f"❌ Error: {e}");
        sys.exit(1)


if __name__ == "__main__":
    main()
