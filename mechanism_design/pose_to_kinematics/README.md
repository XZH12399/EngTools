\# Module: Spatial Pose to Kinematics



\*\*功能\*\*：这是机构设计流水线的\*\*第三块积木\*\*（也是最核心的“大脑”）。



它负责将静态的几何数据（Pose）转化为动态的运动学知识。它不依赖 DH 参数，而是直接基于\*\*螺旋理论 (Screw Theory)\*\* 和 \*\*李括号 (Lie Bracket)\*\* 算法，计算机构的自由度 (DOF)、瞬时自由度 (IDOF)、末端运动性质以及奇异性。



本模块内置了\*\*智能路径规划算法\*\*，支持直接指定“杆件”作为基座和末端（如 `1\_4` -> `3\_4`），程序会自动切断闭环并寻找合法的运动链。



\## 📂 目录结构



```text

pose\_to\_kinematics/

├── pose\_to\_kinematics.py   # \[全功能] 核心脚本 (包含数学核、路径规划、CLI/API)

├── README.md               # 本文件

├── data/                   # 测试数据

│   └── Bennett\_pose.json   # 默认输入 (由 dh\_to\_pose 生成)

└── output/                 # 结果输出

```



\## 📥 输入格式 (Input)



支持 `dh\_to\_pose` 模块生成的位姿数据。推荐在 JSON 中直接配置分析目标。



\*\*必需字段\*\*：

\* `pose`: 字典 `{"1": {"p": \[x,y,z], "z": \[ux,uy,uz]}, ...}`

\* `topology`: 列表 `\[\["1", "2"], ...]`



\*\*配置字段 (可选，但推荐)\*\*：

可以在 JSON 的 `settings` 中指定分析目标，这样无需每次输命令行参数。



```json

{

&nbsp; "settings": {

&nbsp;   "base\_link": "1\_4",  // 基座杆件 (连接关节1和4的杆)

&nbsp;   "ee\_link": "3\_4"     // 末端杆件 (连接关节3和4的杆)

&nbsp; },

&nbsp; "pose": { ... },

&nbsp; "topology": \[ ... ]

}

```



\## 📤 输出报告



程序会在终端打印详细报告，并可保存为 JSON。



\* \*\*DOF (Mobility)\*\*: 机构的整体活动度 (Chebychev–Grübler–Kutzbach 判据的螺旋理论修正版)。

\* \*\*End-Effector Rank\*\*: 末端相对于基座的可行运动维数。

\* \*\*Motion Type\*\*: 运动性质描述 (如 `1R` 纯转动, `1H` 螺旋运动, `2-DOF Spatial` 等)。

\* \*\*Twist Basis\*\*: 末端允许的运动旋量基底（归一化）。



\## 🚀 使用方法



\### 1. 命令行模式 (CLI)



\*\*最简运行\*\* (自动读取 JSON 中的 `settings` 或自动推断)：

```bash

python pose\_to\_kinematics.py

```



\*\*指定杆件进行分析\*\* (Link Mode，\*\*推荐\*\*):

计算 "杆件 2-3" 相对于 "杆件 1-4" 的运动：

```bash

python pose\_to\_kinematics.py -b 1\_4 -e 2\_3

```



\*\*指定关节进行分析\*\* (Node Mode):

计算 "关节 3" 相对于 "关节 1" 的运动：

```bash

python pose\_to\_kinematics.py -b 1 -e 3

```



\*\*指定输入与输出\*\*:

```bash

python pose\_to\_kinematics.py -i data/custom\_pose.json -o output/report.json

```



\### 2. Python 库模式 (Library API)



该模块完全自包含，适合集成。



```python

from pose\_to\_kinematics import run\_pose\_to\_kinematics



\# 方式 A: 使用 Link ID (推荐)

\# 算法会自动“切断”这两根杆，寻找一条开链路径进行分析

report = run\_pose\_to\_kinematics(data\_dict, base="1\_4", ee="2\_3")



print(f"自由度: {report\['dof']}")

print(f"末端运动类型: {report\['motion\_type']}")



\# 方式 B: 使用 Node ID

report = run\_pose\_to\_kinematics(data\_dict, base="1", ee="3")

```



\## 🧮 核心算法说明



1\.  \*\*特征长度归一化 ($L\_{char}$)\*\*: 

&nbsp;   自动计算所有连杆的平均长度。在构建运动螺旋时，将力矩项除以 $L\_{char}$，解决了转动量纲 (1) 与移动量纲 (L) 差异过大导致的数值秩判定失效问题。



2\.  \*\*智能图割路径规划 (Smart Graph Cut)\*\*:

&nbsp;   当用户指定杆件（如 `1\_4`）为基座时，算法会在拓扑图中暂时移除边 `(1,4)`，然后寻找从 `1` 到 `4` 的绕行路径。这确保了计算的是\*\*运动链\*\*的累积效果，而非直接通过基座杆件本身。



3\.  \*\*瞬时自由度剔除 (IDOF Rejection)\*\*:

&nbsp;   使用二阶李括号 (Lie Bracket) 这里的漂移测试，自动识别并剔除那些“瞬间能动但无法持续”的奇异位形自由度。



\## 📦 依赖



\* `numpy`: 矩阵运算

\* `networkx`: 图拓扑与路径搜索



```bash

pip install numpy networkx

```

