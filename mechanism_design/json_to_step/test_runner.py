import os
import json
from mechanism_builder import export_to_step


def main():
    # 1. 路径设置
    base_dir = os.path.dirname(os.path.abspath(__file__))
    json_name = "3RRR_planar.json"  # 这里可以修改为你想要读取的文件名
    json_path = os.path.join(base_dir, 'data', json_name)

    # 2. 自动生成输出路径: ./output/Bennett.step
    file_base_name = os.path.splitext(json_name)[0]
    output_dir = os.path.join(base_dir, 'output')
    output_step_path = os.path.join(output_dir, f"{file_base_name}.step")

    print(f"[Main] 读取数据: {json_path}")

    # 3. 执行逻辑
    if os.path.exists(json_path):
        with open(json_path, 'r', encoding='utf-8') as f:
            raw_data = json.load(f)
            # 提取 JSON 中的 data 部分传入封装函数
            data_part = raw_data.get('data', raw_data)

            # 调用封装好的函数，内部会自动处理文件夹创建、求解和导出
        export_to_step(data_part, output_step_path)
    else:
        print(f"[Error] 找不到输入文件: {json_path}")


if __name__ == "__main__":
    main()