# Module: Spatial Pose to Kinematics

**功能**：这是机构设计流水线的**第三块积木**（也是最核心的“大脑”）。

它负责将静态的几何数据（Pose）转化为动态的运动学知识。它不依赖 DH 参数，而是直接基于**螺旋理论 (Screw Theory)** 和 **李括号 (Lie Bracket)** 算法，计算机构的自由度 (DOF)、瞬时自由度 (IDOF)、末端运动性质以及奇异性。

本模块现在支持**双重二阶分析引擎**，并内置了**微扰分析 (Perturbation Analysis)** 以自动处理奇异点分叉问题。

## 📂 目录结构

```text
pose_to_kinematics/
├── pose_to_kinematics.py   # [全功能] 核心脚本 (包含数学核、SVD/Takagi求解器、路径规划)
├── README.md               # 本文件
├── data/                   # 测试数据
│   └── test_pose.json      # 默认输入
└── output/                 # 结果输出
```

## 📥 输入格式 (Input)

支持 `dh_to_pose` 模块生成的位姿数据。推荐在 JSON 中直接配置分析目标。

**必需字段**：
* `pose`: 字典 `{"1": {"p": [x,y,z], "z": [ux,uy,uz]}, ...}`
* `topology`: 列表 `[["1", "2"], ...]`

**配置字段 (可选)**：
在 JSON 的 `settings` 中指定分析目标，无需每次输命令行参数。

```json
{
  "settings": {
    "base_link": "1_4",  // 基座杆件 (连接关节1和4的杆)
    "ee_link": "3_4"     // 末端杆件 (连接关节3和4的杆)
  },
  "pose": { ... },
  "topology": [ ... ]
}
```

## 📤 输出报告

程序会在终端打印详细报告，并可保存为 JSON。

* **DOF (Mobility)**: 机构的物理自由度。
* **Instantaneous DOFs**: 被算法剔除的虚假/瞬时自由度（"颤动"模态）。
* **Method Used**: 使用的二阶分析方法 (`DRIFT` 或 `TAKAGI`)。
* **Motion Type**: 详细运动性质：
    * `1R` (纯转动), `1P` (纯移动), `1H` (螺旋运动, 带螺距 h)
    * `X-DOF Spatial` (多自由度空间运动)
    * `Locked` (刚性/锁死)
* **Perturbation**: 如果显示 `via Perturbation`，说明机构处于奇异点，算法通过微扰自动解锁了分叉路径。

## 🚀 使用方法

### 1. 命令行模式 (CLI)

**基础运行** (使用默认的数值漂移法):
```bash
python pose_to_kinematics.py
```

**[新] 切换二阶分析方法**:
使用 `-m` 参数切换算法内核：
* `drift`: (默认) 数值李括号漂移法。速度快，适合 AI 训练和大规模筛选。
* `takagi`: 对称 SVD (Takagi 分解) 法。基于解析二阶约束，适合高精度验证和复杂分叉分析。

```bash
# 使用 Takagi 分解法进行严格分析
python pose_to_kinematics.py -m takagi
```

**指定杆件分析**:
```bash
python pose_to_kinematics.py -b 1_4 -e 2_3
```

### 2. Python 库模式 (Library API)

```python
from pose_to_kinematics import run_pose_to_kinematics

# 推荐：使用 Takagi 方法进行高精度校验
report = run_pose_to_kinematics(
    data_dict, 
    base="1_4", 
    ee="2_3", 
    method='takagi'  # 或 'drift'
)

print(f"自由度: {report['dof']}")
print(f"运动类型: {report['motion_type']}")
```

## 🧮 核心算法说明

本模块采用两级分析流程：

1.  **一阶分析 (First-Order Analysis)**:
    * 构建雅可比矩阵 $K$。
    * 使用 **SVD** 计算零空间，获取候选运动模式。

2.  **二阶分析 (Second-Order Analysis)** (由 `-m` 参数控制):
    * **Drift Method (默认)**: 计算沿候选模式运动微小步长后的几何闭环误差（李括号漂移），通过投影检查误差是否可被一阶自由度修正。适合 AI 训练（梯度平滑）。
    * **Takagi Method (Fernández de Bustos et al.)**: 构建二阶加速度约束的二次型矩阵 $B$，对其进行 **Takagi 分解 (Symmetric SVD)**。通过特征值的正定性严格判断机构是锁死（Shaky）还是可动（Bifurcation）。

3.  **微扰修复 (Perturbation Analysis)**:
    * 如果在标准位置判定为刚性 ($DOF=0$) 但存在瞬时自由度 ($IDOF>0$)，算法会自动对所有关节轴施加微小随机扰动。
    * 如果扰动后机构“解锁”，说明原位置为**运动学奇异点**，算法将返回分叉后的真实自由度。

4.  **智能图割 (Smart Graph Cut)**:
    * 自动“切断”用户指定的基座与末端杆件，在拓扑图中寻找开链路径，计算累积运动学效果。

## 📦 依赖

* `numpy`: 线性代数运算
* `networkx`: 图拓扑与路径搜索
* `scipy`: 空间变换与旋转处理

```bash
pip install numpy networkx scipy
```