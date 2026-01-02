\# Mechanism Kinematics Analysis (Based on Screw Theory)



这是一个基于\*\*螺旋理论 (Screw Theory)\*\* 和\*\*图论 (Graph Theory)\*\* 的机构运动学分析工具。



它不仅仅是一个简单的自由度计算器，还能深入分析机构的\*\*瞬时运动特性\*\*。工具能够自动解析闭环机构的拓扑结构，构建全局雅可比矩阵，检测并剔除瞬时自由度 (Instantaneous DOF/Singularity)，并输出末端执行器相对于基座的运动螺旋系 (Twist System)。



\## ✨ 核心功能



1\.  \*\*全局自由度分析 (Global Mobility)\*\*

&nbsp;   \* 基于螺旋系互易积 (Reciprocal Screw) 和雅可比矩阵秩的分析。

&nbsp;   \* \*\*高级特性\*\*：通过李括号 (Lie Bracket) 漂移检测算法，自动识别并剔除\*\*瞬时自由度 (IDOF)\*\*，防止将死点误判为有效自由度。

2\.  \*\*运动特征分析\*\*

&nbsp;   \* 计算末端执行器的秩 (End-Effector Rank)。

&nbsp;   \* 识别运动类型：纯转动 (1R)、纯移动 (1P)、螺旋运动 (1H) 或空间一般运动。

3\.  \*\*智能路径规划\*\*

&nbsp;   \* 只需指定基座连杆 (Base) 和末端连杆 (EE)，算法会自动在拓扑图中寻找一条合法的运动链（自动处理闭环切割）。

4\.  \*\*详细报告\*\*

&nbsp;   \* 输出每个自由度模式下的驱动关节分布 (Active Joints)。

&nbsp;   \* 输出末端的可行运动螺旋基 (Twist Basis)。



\## 🛠️ 依赖库



请确保安装了以下 Python 库：



```bash

pip install numpy networkx

```



\## 🚀 使用方法



\### 1. 准备数据

在 `data/` 目录下创建一个 JSON 文件（或使用现有的 `data/Bennett.json`）。数据格式与 `json\_to\_step` 工具共用。



\*\*关键配置\*\*：在 JSON 的 `settings` 字段中指定基座和末端。



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



\### 2. 运行分析

在终端中运行 `run\_analysis.py`：



```bash

cd kinematics\_analysis

python run\_analysis.py

```



\### 3. 解读输出报告



运行后，控制台将打印如下详细报告：



```text

============================================================

📊 分析报告 (特征长度 L=10.00)

============================================================

⚙️  计算自由度 (DOF): 1

⚠️  检测到瞬时自由度: 0 (已剔除)

🎯 末端秩 (Rank):     1

📝 运动性质:          1H (Screw, h=10.00)

------------------------------------------------------------

🌊 相对运动螺旋 (Twist Basis):

&nbsp;  Mode 1: \[ 0.0000,  0.0000,  1.0000,  0.2500, -0.4000,  0.0000]

------------------------------------------------------------

🔍 驱动关节 (Active Joints):

&nbsp;  Mode 1: 1-2, 2-3, 3-4, 1-4

============================================================

```



\* \*\*Twist Basis\*\*: 输出向量为 \[ ωx, ωy, ωz, vx, vy, vz ]。

\* \*\*Active Joints\*\*: 指示在当前自由度模式下，哪些关节在运动。



\## 📂 文件结构说明



\* `run\_analysis.py`: \*\*用户入口\*\*。设置输入文件路径并启动分析。

\* `mechanism\_analyzer.py`: \*\*解析核心\*\*。负责读取 JSON，构建运动学树，计算每个关节在全局坐标系下的初始螺旋 (Initial Screw)。

\* `dof\_analysis.py`: \*\*数学核心\*\*。

&nbsp;   \* 构建闭环约束雅可比矩阵 (Constraint Jacobian)。

&nbsp;   \* 执行 SVD 分解分析自由度。

&nbsp;   \* 包含 `detect\_instantaneous\_modes` 函数，通过二阶李括号检测瞬时活动度。



\## 📐 理论背景



本工具基于螺旋理论，将每个关节表示为一个运动螺旋 $\_i$。对于一个单闭环机构，其闭环约束方程为：



$$ \\sum \\theta\_i \\cdot \\$\_i = 0 $$



通过构建系统整体的雅可比矩阵 $\\mathbf{K}$ 并对其进行奇异值分解 (SVD)，可以求得机构的零空间 (Null Space)，从而确定机构的自由度数及运动模式。



对于特殊的瞬时机构（如某些处于死点位置的连杆），一阶雅可比矩阵可能显示其“可动”。本工具引入了基于李代数 (Lie Algebra) 的二阶分析：

$$ \[\\mathbf{v}, \\mathbf{w}] \\quad (\\text{Lie Bracket}) $$

通过检查一阶运动产生的几何漂移是否能被机构结构吸收，从而精确剔除瞬时自由度。



\## License



MIT License

