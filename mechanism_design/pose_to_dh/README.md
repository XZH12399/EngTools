\# Module: Spatial Pose to DH Parameters



\*\*功能\*\*：这是机构设计流水线的\*\*第四块积木\*\*（逆向工程引擎）。



它负责将三维空间中的\*\*关节位姿（Pose）\*\*逆向还原为标准的\*\*DH 参数（Link Parameters）\*\*。这使得从运动捕捉、3D 扫描或 CAD 软件中导出的几何数据能够被转化为可计算、可优化的机构学参数。



本模块内置了\*\*数值清洗\*\*和\*\*智能参数归一化\*\*算法，能够处理浮点误差，并自动识别转动副（R）与移动副（P）的参数映射。



\## 📂 目录结构



```text

pose\_to\_dh/

├── pose\_to\_dh.py       # 核心脚本 (包含几何算法、CLI 和 Library API)

├── README.md           # 本文件

├── data/               # 测试数据

│   └── Bennett\_pose.json

└── output/             # 结果输出

&nbsp;   └── Bennett\_dh.json

```



\## 📥 输入格式 (Input)



支持由 `dh\_to\_pose` 生成的位姿数据，或任何包含 `pose` 和 `topology` 的 JSON。



\*\*必需字段\*\*：

\* `pose`: 字典 `{"1": {"p": \[x,y,z], "z": \[ux,uy,uz]}, ...}`

\* `topology`: 列表 `\[\["1", "2"], ...]`

\* `joints` (可选): 字典 `{"1": "R", "2": "P"}`。默认为 "R"。用于决定参数映射逻辑。



\## 📤 输出格式 (Output)



生成与 \*\*Block 1 (dh\_to\_pose)\*\* 输入格式完全兼容的 JSON 文件。



\*\*结构示例\*\*：

```json

{

&nbsp; "data": {

&nbsp;   "joints": { "1": "R", ... },

&nbsp;   "edges": {

&nbsp;     "1\_2": {

&nbsp;       "a": 10.0,

&nbsp;       "alpha": 1.57,

&nbsp;       "offset\_source": 0.0,      // 对于 R 副，这是 d

&nbsp;       "state\_source": 0.0,       // 对于 R 副，这是 theta (中心化后)

&nbsp;       "offset\_target": 5.0,

&nbsp;       "state\_target": 1.57

&nbsp;     }

&nbsp;   }

&nbsp; }

}

```



\## 🚀 使用方法



\### 1. 命令行模式 (CLI)



\*\*最简运行\*\* (默认读取 `data/Bennett\_pose.json`，使用默认清洗阈值 1e-4)：

```bash

python pose\_to\_dh.py

```



\*\*指定阈值清洗噪声\*\* (Tolerance Cleaning):

如果发现输出包含微小的非零值（如 `0.000012`），可以调大阈值强制归零：

```bash

\# 将绝对值小于 0.001 的参数强制置为 0

python pose\_to\_dh.py -t 0.001

```



\*\*指定输入与输出\*\*:

```bash

python pose\_to\_dh.py -i my\_scan.json -o result\_dh.json

```



\### 2. Python 库模式 (Library API)



```python

from pose\_to\_dh import run\_pose\_to\_dh



\# 内存模式：直接获取字典数据

\# tolerance 控制数值清洗力度

dh\_data = run\_pose\_to\_dh(pose\_dict, return\_memory=True, tolerance=1e-5)



\# 打印连杆扭角

print(f"Twist Angle: {dh\_data\['data']\['edges']\['1\_2']\['alpha']}")

```



\## 🧮 核心算法特性



1\.  \*\*公垂线几何重建 (Common Normal Reconstruction)\*\*:

&nbsp;   利用异面直线算法精确计算任意两个关节轴线之间的距离 ($a$) 和扭角 ($\\alpha$)。针对平行轴或共线轴设计了特殊的数值保护逻辑。



2\.  \*\*R/P 副自动映射 (Joint Type Mapping)\*\*:

&nbsp;   \* \*\*转动副 (R)\*\*: 输出 `Offset` = $d$, `State` = $\\theta$。

&nbsp;   \* \*\*移动副 (P)\*\*: 输出 `Offset` = $\\theta$, `State` = $d$。

&nbsp;   \* 算法会自动读取 `joints` 字段进行判断，无需人工干预。



3\.  \*\*状态变量中心化 (State Centering)\*\*:

&nbsp;   对于同一节点连接的多条连杆，算法会自动计算角度均值并扣除。这确保了生成的 `state` 变量在数值上围绕 0 对称分布（例如 `\[30, -30]` 而不是 `\[0, -60]`），更符合机构设计的直觉。



4\.  \*\*数值清洗 (Numerical Snapping)\*\*:

&nbsp;   内置 `\_clean\_float` 函数，自动将小于阈值 (`tolerance`) 的浮点噪声（由矩阵运算累积产生）清洗为完美的 `0.0`。



\## 📦 依赖



\* `numpy`



```bash

pip install numpy

```

