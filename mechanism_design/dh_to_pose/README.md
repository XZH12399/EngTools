# Module: DH Parameters to Spatial Pose

**功能**：这是机构设计与分析流水线的**第一块积木**。

它负责“**纯数学计算**”，将抽象的机构定义（包含 DH 参数或一般化连杆参数的 JSON）转换为具体的、相对于全局坐标系的三维空间关节位姿（Spatial Pose）。生成的中间数据将作为后续“模型生成”或“运动学分析”的标准输入。

## 📂 目录结构

```text
dh_to_pose/
├── dh_to_pose.py       # 核心计算脚本 (包含 CLI 和 Library API)
├── README.md           # 本说明文件
├── data/               # 输入数据示例
│   └── Bennett_dh.json # <--- 默认测试输入
└── output/             # 输出结果 (自动生成)
    └── Bennett_pose.json
```

## 📥 输入格式 (Input)

输入必须是一个 JSON 文件（或字典），且**必须包含 `data` 字段**。

**示例** (`data/Bennett_dh.json`):
```json
{
  "data": {
    "joints": { "1": "R", "2": "R" },
    "edges": {
      "1_2": { "a": 10.0, "alpha": 1.57, ... }
    }
  }
}
```

## 📤 输出格式 (Output)

输出为标准化的中间格式 JSON，不包含冗余的 Meta 信息。

**示例** (`output/Bennett_pose.json`):
```json
{
  "joints": { "1": "R", "2": "R" },
  "pose": {
    "1": { "p": [0,0,0], "z": [0,0,1] },
    "2": { "p": [10,0,0], "z": [0,1,0] }
  },
  "topology": [["1", "2"], ["2", "3"], ["3", "4"], ["1", "4"]] // 包含所有连杆
}
```

## 🚀 使用方法 (Usage)

### 1. 命令行模式 (CLI)

**最简用法**：
默认读取 `data/Bennett_dh.json`，并自动生成 `output/Bennett_pose.json`（自动将 `_dh` 替换为 `_pose`）。
```bash
python dh_to_pose.py
```

**指定输入文件（智能命名）：**
```bash
# 输入: custom.json -> 输出: output/custom_pose.json
# 输入: custom_dh.json -> 输出: output/custom_pose.json
python dh_to_pose.py -i data/custom_dh.json
```

**指定输入和输出：**
```bash
python dh_to_pose.py -i input.json -o result.json
```

### 2. Python 库模式 (Library API)

```python
from dh_to_pose import run_dh_to_pose

# 方式 A: 内存模式 (推荐)
pose_data = run_dh_to_pose("data/Bennett_dh.json", return_memory=True)

# 方式 B: 文件模式
run_dh_to_pose("data/Bennett_dh.json", output_path="out.json", return_memory=False)
```

## 📦 依赖 (Dependencies)

* `numpy`

```bash
pip install numpy
```