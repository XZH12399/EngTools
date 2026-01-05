# Module: Spatial Pose to 3D Model

**功能**：这是机构设计与分析流水线的**第二块积木**。

它负责将抽象的关节空间数据（位置与朝向）实例化为可视化的三维模型。支持 `.step` (默认) 和 `.stl` 等格式导出。

## 📂 目录结构

```text
pose_to_model/
├── pose_to_model.py     # 核心脚本 (CLI + Library)
├── README.md            # 本文件
├── data/                # 测试数据目录
│   └── Bennett_pose.json  <-- 默认测试输入 (请手动放入)
└── output/              # 结果输出目录 (自动生成)
    └── Bennett.step
```

## 📥 输入格式 (Input)

支持 `dh_to_pose` 模块生成的简化版位姿数据。

**必需字段**：
* `pose`: 字典，包含每个关节的：
    * `p`: 位置向量 `[x, y, z]`
    * `z`: 轴向向量 `[dx, dy, dz]`
* `topology`: 连通关系列表 `[["1", "2"], ...]`

## 📤 输出格式 (Output)

* 默认为 `.step` 文件。
* 可通过指定输出文件名后缀（如 `.stl`）来支持其他格式（需 `build123d` 支持）。

## 🚀 使用方法

### 1. 命令行模式 (CLI)

```bash
# 默认读取 ./data/Bennett_pose.json，生成 ./output/Bennett.step
python pose_to_model.py

# 指定输入文件
python pose_to_model.py -i my_pose.json

# 指定输出格式 (例如 STL)
python pose_to_model.py -i my_pose.json -o output/model.stl
```

### 2. Python 库模式 (Library API)

```python
from pose_to_model import run_pose_to_model

# 假设 pose_data 是字典
run_pose_to_model(pose_data, output_path="mechanism.step")
```

## 📦 依赖

* `build123d`
* `numpy`

```bash
pip install build123d numpy
```