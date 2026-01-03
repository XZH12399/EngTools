import json
import argparse
import os
import sys
import numpy as np
from typing import Union, Dict, Optional
from build123d import *


# ==========================================
# 1. 核心生成类
# ==========================================

class PoseToModelGenerator:
    def __init__(self, json_data: Dict):
        if 'pose' not in json_data or 'topology' not in json_data:
            raise ValueError("Input JSON must contain 'pose' and 'topology'.")

        self.pose_data = json_data['pose']
        self.topology = json_data['topology']
        self.joints_info = json_data.get('joints', {})

        self.parsed_pose = {}
        self.all_points = []

        for jid, data in self.pose_data.items():
            if 'p' not in data or 'z' not in data:
                continue

            p = Vector(tuple(data['p']))
            z = Vector(tuple(data['z']))

            self.parsed_pose[jid] = {'p': p, 'z': z}
            self.all_points.append(data['p'])

        self.all_points = np.array(self.all_points)

    def _calculate_auto_dimensions(self):
        if len(self.all_points) < 2:
            return 2.0, 3.0, 10.0

        raw_max_dim = np.ptp(self.all_points, axis=0).max()
        max_dim = raw_max_dim if raw_max_dim > 1e-3 else 10.0

        link_radius = max_dim / 60.0
        axis_radius = link_radius * 1.5
        axis_length = max_dim / 5.0

        return max(link_radius, 0.5), max(axis_radius, 0.8), axis_length

    def build_model(self) -> Compound:
        if not self.parsed_pose:
            raise ValueError("No pose data to build model.")

        link_r, joint_r, joint_l = self._calculate_auto_dimensions()
        shapes = []

        # --- A. Links ---
        for edge in self.topology:
            u, v = edge[0], edge[1]

            if u not in self.parsed_pose or v not in self.parsed_pose:
                continue

            p1 = self.parsed_pose[u]['p']
            p2 = self.parsed_pose[v]['p']

            dist = (p2 - p1).length
            if dist < 1e-5: continue

            with BuildPart() as link_part:
                with BuildSketch(Plane(origin=p1, z_dir=p2 - p1)):
                    Circle(radius=link_r)
                extrude(amount=dist)
            shapes.append(link_part.part)

        # --- B. Joints ---
        for jid, info in self.parsed_pose.items():
            origin = info['p']
            z_axis = info['z']

            j_type = self.joints_info.get(jid, 'R')

            with BuildPart() as joint_part:
                plane = Plane(origin=origin, z_dir=z_axis).offset(-joint_l / 2)
                with BuildSketch(plane):
                    if j_type == 'P':
                        Rectangle(width=joint_r * 1.8, height=joint_r * 1.8)
                    else:
                        Circle(radius=joint_r)
                extrude(amount=joint_l)

            shapes.append(joint_part.part)

        return Compound(shapes)


# ==========================================
# 2. 封装函数
# ==========================================

def run_pose_to_model(
        input_source: Union[str, Dict],
        output_path: Optional[str] = None,
        verbose: bool = True
) -> bool:
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
        input_filename_base = "model_from_memory"

    if verbose:
        print(f"🔨 Generating model for: {input_filename_base}...")

    generator = PoseToModelGenerator(json_data)
    try:
        assembly = generator.build_model()
    except Exception as e:
        if verbose: print(f"❌ Build failed: {e}")
        return False

    if not output_path:
        base_dir = os.path.dirname(os.path.abspath(__file__))
        default_output_dir = os.path.join(base_dir, "output")
        clean_name = input_filename_base.replace("_pose", "")
        output_path = os.path.join(default_output_dir, f"{clean_name}.step")

    output_dir = os.path.dirname(os.path.abspath(output_path))
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    ext = os.path.splitext(output_path)[1].lower()
    if ext == '.step' or ext == '.stp':
        assembly.export_step(output_path)
    elif ext == '.stl':
        assembly.export_stl(output_path)
    else:
        assembly.export_step(output_path)

    if verbose:
        print(f"✅ [Pose->Model] Saved to: {output_path}")

    return True


# ==========================================
# 3. 命令行入口 (CLI)
# ==========================================

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # 【修改】默认使用同级目录 data/Bennett_pose.json 进行测试
    default_input = os.path.join(base_dir, "data", "Bennett_pose.json")

    parser = argparse.ArgumentParser(description="[Module] Spatial Pose to 3D Model")
    parser.add_argument('-i', '--input', default=default_input, help="Input JSON")
    parser.add_argument('-o', '--output', default=None, help="Output file path")
    parser.add_argument('-q', '--quiet', action='store_true', help="Suppress output")
    args = parser.parse_args()

    # 如果默认文件不存在，尝试找一下 pose.json (增加容错)
    if args.input == default_input and not os.path.exists(default_input):
        local_data = os.path.join(base_dir, "data/pose.json")
        if os.path.exists(local_data):
            args.input = local_data

    # 注意：如果连 data/Bennett_pose.json 都没有，run_pose_to_model 内部会抛出 FileNotFoundError，
    # 并在下方 except 块中被捕获并打印错误信息。

    try:
        run_pose_to_model(args.input, args.output, verbose=not args.quiet)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()