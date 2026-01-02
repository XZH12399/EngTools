# EngTools: Engineering & Mechanism Design Toolbox

**EngTools** 是一个面向机械工程、机构学研究与自动化设计的 Python 工具集合。

本仓库旨在提供轻量、模块化且基于理论支撑的工程辅助工具，目前涵盖了**参数化几何生成**与**基于螺旋理论的运动学分析**两大核心板块。

## 📂 项目结构 (Project Structure)

```text
EngTools/
├── mechanism_design/           # 机构设计与分析核心模块
│   │
│   ├── json_to_step/           # [工具 1] 几何生成器
│   │   └── (将机构参数转换为通用 3D STEP 模型)
│   │
│   └── kinematics_analysis/    # [工具 2] 运动学分析器
│       └── (基于螺旋理论计算自由度、运动螺旋与瞬时运动)
│
├── .gitignore
└── README.md                   # 您正在阅读的文件
```

## 🛠️ 工具列表 (Tools Overview)

### 1. [Mechanism JSON to STEP](./mechanism_design/json_to_step/README.md)
> **功能**：从抽象的数学定义直接生成实体模型。

* **输入**：定义关节类型（R/P）和连杆参数（DH 参数或一般化几何参数）的 JSON 文件。
* **输出**：通用的 `.step` 格式 3D 模型，可直接导入 SolidWorks, Fusion 360, FreeCAD 等软件。
* **核心库**：`build123d`, `numpy`

### 2. [Kinematics Analysis (Screw Theory)](./mechanism_design/kinematics_analysis/README.md)
> **功能**：深入的运动学理论分析，不仅仅是简单的自由度公式。

* **输入**：与几何生成器通用的 JSON 结构，指定基座与末端。
* **核心能力**：
    * **全局自由度计算**：基于雅可比矩阵的秩分析。
    * **瞬时自由度剔除**：利用李括号 (Lie Bracket) 算法检测二阶几何漂移，精确识别奇异位形。
    * **运动螺旋输出**：计算末端执行器的可行运动基 (Twist Basis)。
* **核心库**：`networkx`, `numpy`

## 🚀 快速开始 (Quick Start)

### 1. 克隆仓库
```bash
git clone [https://github.com/YourUsername/EngTools.git](https://github.com/YourUsername/EngTools.git)
cd EngTools
```

### 2. 安装依赖
建议使用 Python 3.8+ 环境。各个工具可能有独立的依赖，但主要依赖如下：

```bash
pip install numpy build123d networkx
```

### 3. 使用示例

**生成几何模型：**
```bash
cd mechanism_design/json_to_step
# 修改 test_runner.py 指定输入文件
python test_runner.py
```

**进行运动学分析：**
```bash
cd mechanism_design/kinematics_analysis
# 修改 run_analysis.py 指定输入文件
python run_analysis.py
```

## 📄 数据格式 (Data Format)

本仓库下的工具共用一套 **JSON 数据标准**，实现了“一次定义，多维分析”。

```json
{
  "settings": {
    "base_link": "1_4",
    "ee_link": "4_3",
    "description": "Bennett 4R mechanism sample"
  },
  "data": {
    "joints": {
      "1": "R",
      "2": "R",
      "3": "R",
      "4": "R"
    },
    "edges": {
      "1_2": {"a": 10.0, "alpha": 2.6180, "offset_source": 0.0, "offset_target": 0.0, "state_source": -0.5236, "state_target": -0.5651},
      "2_3": {"a": 17.32, "alpha": 2.0944, "offset_source": 0.0, "offset_target": 0.0, "state_source": 0.5651, "state_target": -0.5236},
      "3_4": {"a": 10.0, "alpha": 2.6180, "offset_source": 0.0, "offset_target": 0.0, "state_source": 0.5236, "state_target": 0.5651},
      "1_4": {"a": 17.32, "alpha": 2.0944, "offset_source": 0.0, "offset_target": 0.0, "state_source": 0.5236, "state_target": -0.5651}
    }
  }
}
```

## 🤝 贡献 (Contributing)

欢迎提交 Issue 或 Pull Request 来增加新的工程计算工具（如动力学分析、拓扑优化等）。

## License

MIT License
