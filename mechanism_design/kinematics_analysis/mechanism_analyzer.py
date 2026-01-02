import json
import numpy as np
import math
import os
import sys
import networkx as nx
import itertools

try:
    from dof_analysis import analyze_mobility_anchor
except ImportError:
    print("❌ 错误: 未找到 dof_analysis.py")
    sys.exit(1)


class KinematicEngine:
    def __init__(self, json_data):
        self.nodes = list(json_data['data']['joints'].keys())
        self.edges_data = json_data['data']['edges']
        self.joint_types = json_data['data']['joints']
        self.adj = {n: [] for n in self.nodes}
        self.edge_lookup = {}
        for key, params in self.edges_data.items():
            u, v = key.split('_')
            self.adj[u].append(v)
            self.adj[v].append(u)
            self.edge_lookup[tuple(sorted((u, v)))] = params

    def get_edge_params(self, u, v):
        key = tuple(sorted((u, v)))
        params = self.edge_lookup[key]
        is_forward = (key == (u, v))
        a = params['a']
        alpha = params['alpha']
        if is_forward:
            q = params['state_source']
            offset = params['offset_source']
        else:
            q = params['state_target']
            offset = params['offset_target']
        return a, alpha, offset, q

    def dh_matrix(self, a, alpha, d, theta):
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def compute_screws(self, base_node_id):
        neighbors = self.adj.get(base_node_id, [])
        if not neighbors: raise ValueError(f"Base Node {base_node_id} is isolated!")
        ref_node = neighbors[0]
        global_transforms = {}
        T_base = np.eye(4)
        global_transforms[base_node_id] = {'T': T_base, 'parent': ref_node}

        queue = [(base_node_id, -1, T_base)]
        visited = {base_node_id}
        PI = math.pi

        while queue:
            u, p, T_u = queue.pop(0)
            logical_parent = ref_node if p == -1 else p
            if p == -1:
                _, _, off_in, q_in = self.get_edge_params(u, ref_node)
            else:
                _, _, off_in, q_in = self.get_edge_params(u, p)

            for v in self.adj[u]:
                if v == logical_parent: continue
                a, alpha, off_out, q_out = self.get_edge_params(u, v)
                j_type = self.joint_types.get(u, 'R')
                if j_type == 'R':
                    theta = (q_out - q_in) - PI
                    d = (off_out - off_in)
                else:
                    theta = (off_out - off_in) - PI
                    d = (q_out - q_in)
                T_step = self.dh_matrix(a, alpha, d, theta)
                T_v = T_u @ T_step
                if v not in global_transforms: global_transforms[v] = {'T': T_v, 'parent': u}
                if v not in visited:
                    visited.add(v)
                    queue.append((v, u, T_v))

        screws = {}
        nodes_info = {}
        a_lengths = [p['a'] for p in self.edge_lookup.values()]
        L_char = np.mean(a_lengths) if a_lengths else 1.0
        for nid, data in global_transforms.items():
            T = data['T']
            z_axis = T[:3, 2]
            pos = T[:3, 3]
            j_type = self.joint_types.get(nid, 'R')
            screw = np.zeros(6)
            if j_type == 'R':
                screw[:3] = z_axis
                screw[3:] = np.cross(pos, z_axis) / L_char
            else:
                screw[3:] = z_axis
            screws[nid] = screw
            nodes_info[nid] = {'screw': screw, 'pos': pos, 'axis': z_axis}
        return screws, nodes_info, L_char


# =========================================================
# 🔍 核心逻辑改进：自动翻转寻找可行路径
# =========================================================
def construct_smart_path(json_data, base_link_str, ee_link_str):
    """
    自动尝试基座和末端杆件的所有方向组合，寻找一条合法的补余路径。
    """
    edges = json_data['data']['edges']

    # 1. 准备原始图
    G_full = nx.Graph()
    for key in edges.keys():
        u, v = key.split('_')
        G_full.add_edge(u, v)

    try:
        b_u, b_v = base_link_str.split('_')
        e_u, e_v = ee_link_str.split('_')
    except IndexError:
        print("❌ 错误: 杆件格式必须为 'Node1_Node2'")
        return None, None, None

    # 2. 准备“切割图” (移除指定的两根杆件)
    # 我们需要在不经过这两根杆的情况下找到路径
    G_cut = G_full.copy()
    if G_cut.has_edge(b_u, b_v): G_cut.remove_edge(b_u, b_v)
    if G_cut.has_edge(e_u, e_v): G_cut.remove_edge(e_u, e_v)

    # 3. 定义可能的起止点组合
    # Base 可能是: u->v (Head u, Start v) 或者 v->u (Head v, Start u)
    base_options = [
        {'head': b_u, 'start': b_v, 'desc': f"{b_u}(虚)->{b_v}(实)"},
        {'head': b_v, 'start': b_u, 'desc': f"{b_v}(虚)->{b_u}(实)"}
    ]

    # EE 可能是: u->v (End u, Tail v) 或者 v->u (End v, Tail u)
    # 注意: 这里 End 是路径终点，Tail 是末端虚拟点
    ee_options = [
        {'end': e_u, 'tail': e_v, 'desc': f"{e_u}(实)->{e_v}(虚)"},
        {'end': e_v, 'tail': e_u, 'desc': f"{e_v}(实)->{e_u}(虚)"}
    ]

    print(f"🛣️  智能路径规划: {base_link_str} ... {ee_link_str}")

    # 4. 遍历所有组合，寻找通路
    valid_path_info = None

    for b_opt, e_opt in itertools.product(base_options, ee_options):
        start_node = b_opt['start']
        end_node = e_opt['end']

        # 优化：如果是单环机构，且 Start == End (比如 4...4)，
        # 我们需要确保图中有至少一条回路，而不是仅仅返回单点。
        # 不过 nx.shortest_path 在 start==end 时会返回 [start]，
        # 这对于 dof_analysis 来说是可以接受的（表示相邻杆件，无中间关节）。

        try:
            # 在切割后的图中寻找路径
            path = nx.shortest_path(G_cut, source=start_node, target=end_node)

            # 找到路径！记录并跳出
            valid_path_info = {
                'path': path,
                'head': b_opt['head'],
                'tail': e_opt['tail'],
                'desc': f"方案 [{b_opt['desc']} ... {e_opt['desc']}]"
            }
            print(f"   ✅ {valid_path_info['desc']} -> 成功连通!")
            break
        except nx.NetworkXNoPath:
            # print(f"   ❌ 尝试 [{b_opt['desc']} ... {e_opt['desc']}] -> 不连通")
            continue

    # 5. 构建结果
    if valid_path_info:
        # 拼接完整链条: [Head] + Path + [Tail]
        full_chain = [valid_path_info['head']] + valid_path_info['path'] + [valid_path_info['tail']]

        # 实际计算用的 Start 和 End (即 Path 的首尾)
        calc_base = valid_path_info['path'][0]
        calc_ee = valid_path_info['path'][-1]

        return full_chain, calc_base, calc_ee
    else:
        print("❌ 所有组合均尝试失败，无法绕过指定杆件连通基座与末端。")
        # 最后的兜底：可能就是想直接连通？(但这违反了相对运动的初衷)
        return None, None, None


def run_full_analysis(json_path):
    if not os.path.exists(json_path):
        print(f"❌ 文件不存在: {json_path}")
        return

    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    settings = data.get('settings', {})
    base_link_str = settings.get('base_link', '1_4')
    ee_link_str = settings.get('ee_link', '3_4')

    # 1. 智能构建路径
    full_path, calc_base, calc_ee = construct_smart_path(data, base_link_str, ee_link_str)

    if full_path:
        path_str = " -> ".join(full_path)
        print(f"📍 最终分析路径: [{path_str}]")
    else:
        return

    # 2. 运动学计算
    engine = KinematicEngine(data)
    # 注意：必须以路径的 Start Node 为根计算 Screw
    screws, nodes_info, l_char = engine.compute_screws(base_node_id=calc_base)

    # 3. 准备拓扑
    topology_edges = []
    for key in data['data']['edges']:
        u, v = key.split('_')
        topology_edges.append((u, v))

    # 4. 分析
    print(f"🚀 开始分析 (Base: {calc_base} -> EE: {calc_ee})...")

    result = analyze_mobility_anchor(
        node_screw_map=screws,
        topology_edges=topology_edges,
        nodes_info=nodes_info,
        rigid_body_sets=[],
        base_node=calc_base,
        ee_node=calc_ee,
        manual_extended_path=full_path
    )

    print_report(result, l_char)


def print_report(result, l_char):
    print("\n" + "=" * 60)
    print(f"📊 分析报告 (特征长度 L={l_char:.2f})")
    print("=" * 60)

    print(f"⚙️  计算自由度 (DOF): {result['dof']}")
    if result.get('idof_count', 0) > 0:
        print(f"⚠️  检测到瞬时自由度: {result['idof_count']} (已剔除)")

    print(f"🎯 末端秩 (Rank):     {result['ee_rank']}")
    print(f"📝 运动性质:          {result['motion_type']}")

    print("-" * 60)
    print("🌊 相对运动螺旋 (Twist Basis):")
    if result['ee_twist_basis']:
        for i, twist in enumerate(result['ee_twist_basis']):
            fmt = ", ".join([f"{x:>7.4f}" for x in twist])
            print(f"   Mode {i + 1}: [{fmt}]")
    else:
        print("   (Locked / 无运动)")

    # if 'dof_details' in result and result['dof_details']:
    #     print("-" * 60)
    #     print("🔍 驱动关节 (Active Joints):")
    #     for detail in result['dof_details']:
    #         active_joints = [
    #             f"{item['edge'][0]}-{item['edge'][1]}"
    #             for item in detail['velocities'] if abs(item['vel']) > 1e-4
    #         ]
    #         print(f"   Mode {detail['mode_id']}: {', '.join(active_joints)}")

    print("=" * 60)