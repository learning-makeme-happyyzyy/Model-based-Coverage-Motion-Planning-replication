## Goal

开始执行 Paper 05 Traditional Inspection Planning reproduction 的 **M1 Surface Discretization**。

不要重新制定复现方案。

---

## Input

请先读取并遵循：

1. `paper05_replication.md`
2. `pi05_industrial_inspection_research.md`
3. Paper 05 原文 PDF（如果项目中存在）

`paper05_replication.md` 已经是本项目的复现指南，不需要重新创建 `reproduction_spec.md`。

如果实现过程中发现：

* 论文与复现指南冲突
* 论文参数无法确认
* 复现指南存在明显错误

请记录下来并告诉我，但不要擅自修改复现目标。

---

## Environment

环境已经搭建完成：

* Python 3.11
* uv
* `pyproject.toml`
* `uv.lock`

严格使用：

```bash
uv run ...
```

不要使用 pip。

---

## Task

只实现：

```text
Mesh
 ↓
M1 Surface Discretization
 ↓
Surface Patches
```

每个 patch 至少包含：

```text
patch_id
vertices
center
normal
area
```

按照 `paper05_replication.md` 中已经确定的 reproduction strategy 实现。

---

## Requirements

实现：

1. mesh loading
2. surface discretization
3. patch center
4. patch normal
5. patch area
6. bounding box
7. visualization
8. serialization

添加必要的 unit tests。

不要实现：

* M2
* M3
* M4
* M5
* Baselines
* Phase 2
* Pi0.5

---

## Validation

运行：

```bash
uv run pytest
```

并运行一个最小 workpiece example。

输出：

```text
Mesh:
Vertices:
Faces:

Patch count:
Bounding box:

Normal statistics:
Area statistics:
```

生成一个 surface-patch visualization。

---

## Final Report

最后只报告：

```text
M1 status: PASS / FAIL

Files created:
...

Tests:
...

Numerical result:
...

Visualization:
...

Known issues:
...

Paper/reproduction ambiguities:
...
```

**完成 M1 后停止，不要自动进入 M2。**


## Goal

请直接开始执行 `Model-based Coverage Motion Planning for Industrial 3D Shape Inspection Applications`（IEEE CASE 2017, Paper 05）的 Phase 1 Traditional Inspection Planning reproduction。

目标不是只给我代码框架，而是最终得到一个可以实际运行的 Traditional Inspection Planning baseline：

```text
CAD / Mesh
  ↓
M1 Surface Discretization
  ↓
M2 Viewpoint Sampling
  ↓
M3 Visibility Matrix
  ↓
M4 Travel Cost
  ↓
M5 RKGA-SCTSP
  ↓
Ordered Viewpoints
  ↓
Coverage / #Viewpoints / Travel Cost / Cycle Time
```

当前阶段只执行 Phase 1。

暂时不要实现：

* Pi0.5
* LoRA
* rollout dataset
* Blender rendering
* pose perturbation
* Phase 2 demonstration generation

---

## Input

请先读取并理解以下资料：

1. `paper05_replication.md`
2. `pi05_industrial_inspection_research.md`
3. Model-based Coverage Motion Planning for Industrial 3D Shape
Inspection Applications 原文 PDF（如果项目中存在）

如果这些文件存在，请以它们作为主要依据。

特别注意：

* 不要把 reproduction plan 中“我自己选择的参数”误认为论文原始参数。
* 不要把 reproduction simplification 误认为论文实现。
* 对论文没有明确给出的参数，必须标记为：
  `REPRODUCTION CHOICE`
* 对论文描述不清楚或存在歧义的地方，必须标记为：
  `PAPER UNCERTAINTY`

---

## Output

最终得到一个可以运行的 Python reproduction project。

至少能够运行：

```bash
python run.py --part bracket --n_candidates 200
```

并输出：

```text
Coverage:
Selected viewpoints:
Travel cost:
Inspection cost:
Total cycle time:
Viewpoint sequence:
```

同时保存：

```text
outputs/
├── ...
├── visibility_matrix
├── selected_viewpoints
├── viewpoint_sequence
├── metrics
└── plots
```

最终至少支持：

```text
candidate number = 200 / 300 / 400
```

并且能够运行：

1. GA-SCTSP
2. Greedy-SCP + GA-TSP
3. GA-SCP + GA-TSP

---

## Steps

### Step 0 — Repository inspection

先扫描当前项目：

```text
目录结构
已有代码
Python 环境
已有依赖
已有 mesh
已有配置
```

不要立即开始写大量代码。

先告诉我：

```text
Current repository:
...

Existing reusable components:
...

Missing components:
...

Potential conflicts:
...
```

然后再开始实现。

---

### Step 1 — Paper reproduction specification

在正式编码前建立：

```text
docs/reproduction_spec.md
```

把整个 pipeline 写成：

```text
M1
Input:
Output:
Algorithm:
Paper parameters:
Reproduction parameters:
Simplifications:

M2
...

M3
...

M4
...

M5
...
```

特别把：

```text
PAPER PARAMETER
REPRODUCTION CHOICE
SIMPLIFICATION
```

三者严格区分。

---

### Step 2 — M1 Surface Discretization

实现：

```text
mesh
 ↓
surface patches
```

输出至少包含：

```text
patch_id
vertices
center
normal
area
```

例如：

```python
patch = {
    "id": 347,
    "center": [...],
    "normal": [...],
    "vertices": [...],
}
```

实现基本检查：

* patch 数量
* 法向方向
* 面片面积
* mesh bounding box
* 可视化

---

### Step 3 — M2 Viewpoint Sampling

实现候选 viewpoint：

```text
surface patches
+
sensor parameters
 ↓
candidate viewpoints
```

每个 viewpoint 至少包含：

```text
position
orientation / viewing direction
```

支持：

```text
N = 200
N = 300
N = 400
```

实现论文中的 potential-field viewing-direction 方法。

但是：

**先重新核对 Paper 05 Eq.10 的符号和坐标定义，再编码。**

不要仅仅根据 reproduction plan 中的解释决定正负号。

输出：

```text
candidate positions visualization
viewing directions visualization
```

---

### Step 4 — M3 Visibility Matrix

实现：

```text
V × S
 ↓
A ∈ {0,1}^{N×M}
```

其中：

```text
A[i,j] = 1
```

表示：

> viewpoint i 理论上可以有效观察 surface patch j。

按照论文 / reproduction specification 实现：

1. FOV
2. FOD
3. Viewing Angle
4. Occlusion

特别注意 ray casting：

不要简单判断“有没有 ray intersection”。

必须判断：

> 是否存在一个遮挡物在 camera 与目标 patch 之间。

同时处理：

* self intersection
* floating point tolerance
* target triangle hit
* behind-target intersection

实现：

```text
tests/test_visibility.py
```

至少覆盖：

1. 正常可见
2. 距离过近
3. 相机方向错误
4. viewing angle 超限
5. 前后遮挡
6. FOV 边界

这是整个 reproduction 最重要的模块之一。

---

### Step 5 — M4 Travel Cost

实现 viewpoint-to-viewpoint travel cost：

```text
e(i,j)
```

当前 reproduction 使用论文真实机器人规划的 proxy：

```text
e(i,j)
=
α ||p_i-p_j||
+
β Δθ
```

其中：

```text
Δθ = arccos(v_i · v_j)
```

但是：

**不要为了让结果落入论文的 15–30s 范围而反向调整 α / β。**

α、β 必须：

1. 明确记录
2. 全部实验固定
3. 所有 baseline 使用相同参数

并在 documentation 中明确：

> This is a reproduction proxy rather than the original MoveIt/OMPL/IKFast/FCL travel-time computation.

---

### Step 6 — M5 RKGA-SCTSP

实现论文中的：

```text
Single-Solve Coverage Traveling Salesman Problem
```

染色体：

```text
x ∈ [0,1]^N
```

通过：

```python
order = argsort(x)
```

得到 viewpoint ordering。

Fitness 必须同时考虑：

```text
number of viewpoints
+
travel cost
+
coverage constraint
```

并实现：

```text
coverage >= δ
```

其中 δ 默认使用：

```text
0.995
```

但是如果论文原文实际采用严格 `>`，请在代码和文档中说明差异。

先单独测试：

```text
TSP
↓
SCP
↓
SCTSP
```

再组合。

---

### Step 7 — Baselines

所有 baseline：

**必须使用完全相同的 candidate viewpoints 和 visibility matrix。**

实现：

```text
Baseline A:
Greedy-SCP + GA-TSP

Baseline B:
GA-SCP + GA-TSP

Proposed reproduction:
GA-SCTSP
```

输出：

```text
coverage
number of viewpoints
travel cost
cycle time
```

---

### Step 8 — Evaluation

实现统一 evaluation harness。

至少运行：

```text
3 workpieces
×
3 candidate counts
×
10 trials
×
3 methods
```

如果当前阶段计算量过大：

先完成：

```text
1 workpiece
×
200 candidates
×
3 methods
```

确认 pipeline 正确后再扩展。

输出：

```text
results.csv
```

以及：

```text
method
candidate_count
coverage
num_viewpoints
travel_cost
cycle_time
mean
std
```

---

### Step 9 — Visualization

至少生成：

1. mesh + surface patches
2. candidate viewpoints
3. selected viewpoints
4. viewpoint sequence
5. visibility matrix
6. coverage progression
7. RKGA convergence curve

---

### Step 10 — Validation

不要只告诉我“程序运行成功”。

必须检查以下现象：

```text
A:
coverage >= target

B:
selected viewpoints actually cover target percentage

C:
GA-SCTSP produces a feasible solution

D:
Greedy-SCP + GA-TSP produces a feasible solution

E:
GA-SCP + GA-TSP produces a feasible solution

F:
same candidate set is used across methods

G:
RKGA convergence is observable

H:
results are reproducible with fixed random seed
```

关于：

```text
GA-SCTSP 比 GA-SCP-GA-TSP 快 5–25%
```

不要把它作为硬性正确性条件。

它只能作为 paper-level expected pattern。

如果没有出现：

```text
先诊断
```

而不是修改参数强行让结果符合论文。

---

## Important Research Rules

### Rule 1

不要为了“复现出论文结果”而调参数。

### Rule 2

任何论文没有明确给出的参数，都标记：

```text
REPRODUCTION CHOICE
```

### Rule 3

任何我们主动简化的模块，都标记：

```text
SIMPLIFICATION
```

### Rule 4

Paper 05 的真实机器人规划：

```text
IKFast
FCL
MoveIt
OMPL
RRT-Connect
cubic spline
```

当前 reproduction 的：

```text
geometric travel proxy
```

不能混为一谈。

### Rule 5

Visibility Matrix：

```text
A[i,j] = theoretical visibility
```

不要把它称为：

```text
actual observed coverage
```

后续 Phase 2 才建立：

```text
actual coverage state c_t
```

### Rule 6

如果发现论文描述、review notes、reproduction plan 三者存在冲突：

不要自行隐藏冲突。

记录：

```text
Conflict
Evidence
Chosen implementation
Reason
```

---

## Final Report

完成后生成：

```text
docs/reproduction_report.md
```

包含：

1. What was reproduced
2. What was simplified
3. Paper parameters
4. Reproduction parameters
5. Implementation details
6. Unit-test results
7. Numerical results
8. Baseline comparison
9. Failure cases
10. Differences from original paper
11. What should be done next

最后给我一个非常简洁的 summary：

```text
Implementation status:
M1: PASS/FAIL
M2: PASS/FAIL
M3: PASS/FAIL
M4: PASS/FAIL
M5: PASS/FAIL

Experiments:
...

Main result:
...

Known deviations:
...

Next step:
...
```

### Execution rule

**现在直接开始执行。**

不要先给我一份新的长篇方案。

先扫描 repository 和相关资料，然后从 Step 0 开始。

如果某个关键论文细节无法确认：

```text
STOP THAT SUBMODULE
↓
report uncertainty
↓
continue with other independent modules
```

不要因为一个局部不确定点把整个工程停住。

同时保持代码模块化，使后续能够替换：

```text
M4 travel model
M5 optimizer
viewpoint sampler
visibility implementation
```

而不需要重写整个 pipeline。

搭建环境采用uv：Environment:
- Python 3.11
- uv for environment and dependency management
- Do not use pip directly
- Create and maintain pyproject.toml and uv.lock
- Run Python commands with `uv run`
环境部分已经搭建好，参考终端
