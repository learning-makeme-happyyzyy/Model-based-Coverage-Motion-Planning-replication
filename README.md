# trad-replication

paper_05 (SCTSP + RKGA industrial inspection planner) 传统规划复现 —— Phase 1 baseline。
复现策略与任务拆分见 `paper05_replication.md`；论文细节见 `paper_05_review.md`。

## 状态

- [x] M1 表面离散化（`src/m1_discretize.py`）—— mesh → 均匀三角面片集 S
- [x] M2 候选视点生成（`src/m2_sample.py`）—— S → V = {位置 + 视线方向}（Eq.10 势场）
- [x] M3 可见性矩阵（`src/m3_visibility.py`）—— S × V × mesh → A ∈ {0,1}^{N×M}（FOV/FOD/视角/遮挡）
- [x] M4 行走代价（`src/m4_travel_cost.py`）—— V → E ∈ R^{N×N} + home 往返（§8 代理公式）

## M1 用法

```bash
# 最小工件示例（L 型支架，m=1000 面片，输出报告 + S.npz + 可视化）
uv run python -m examples.run_m1_example

# 任意网格（可选 --config configs/base.yaml 读取 mesh.target_patches）
uv run python -m src.m1_discretize --mesh data/meshes/bracket.obj --patches 1000
```

## M2 用法

```bash
# 最小工件示例（读取 M1 的 S_bracket_1000.npz，生成 n=400 主集 + 200/300 前缀子集）
uv run python -m examples.run_m2_example

# CLI（--tune-dmax 按指南 §6.4 诊断自动调 d_max）
uv run python -m src.m2_sample --surface data/outputs/S_bracket_1000.npz \
    --config configs/base.yaml --n 400 --tune-dmax

# 单元测试
uv run pytest
```

## M2：Eq. (10) 的符号歧义与采用的解释（记录在案）

论文 Eq. (10) 字面形式：

    v = ( Σ_i K d_i/‖d_i‖³ ) / ‖ Σ_i K d_i/‖d_i‖³ ‖,   d_i = p_vp − p_patch_i

按字面 d_i = p_vp − p_patch_i，求和方向为 **面片 → 相机**。但论文同时给出两条
物理约束：(a) 势场法"确保每个视点朝向能**采集到**近邻面片加权平均的方向"；
(b) 论文对 IK/碰撞失败的处理是"绕**光轴**旋转视点，因为绕光轴旋转不显著改变
传感器采集几何数据的能力"。二者都作用于**相机光轴（相机 → 场景）**。

因此本复现（与复现指南 §6.2 一致）存储的视线方向为

    v̂ = normalize( Σ_{距离<d_max} (p_patch_i − p_vp)/‖p_patch_i − p_vp‖³ )

即论文字面求和取负（K 在归一化中约掉）。这是对论文符号的**解读**，不是论文
明文；论文若把 v 定义为"表面 → 相机的视线方向"，两解读在光轴这一物理量上
完全一致。roll（绕光轴转角）论文未定，本复现按指南 §7.1 约定（M3 FOV 用）。

已知的指南内部矛盾（已按指南自己的调参指令处理）：指南 §6.1 采样壳的最小
视点-表面距离 = FOD_min + margin = 0.45 m，而指南 §6 的 d_max 初值 0.30 m
小于它——任何工件上邻域都为空。按 §6.4 诊断调 d_max（本工件 0.746 m）；空
邻域时使用势场的 d_max→0+ 极限（指向最近面片），并在元数据中如实记录。

## M3 用法

```bash
# 最小工件示例（S_bracket_1000 × V_bracket_{200,300,400} → A，含四面板调试图）
uv run python -m examples.run_m3_example

# CLI
uv run python -m src.m3_visibility --surface data/outputs/S_bracket_1000.npz \
    --viewpoints data/outputs/V_bracket_400.npz --config configs/base.yaml
```

## M3：viewing angle 的歧义与采用的解释（记录在案）

论文只给出 "The viewing angle must be within the specified range" + Table I 的
75°，**没有定义这个角是哪个夹角**。复现指南 §7.5 的手算例钉死了意图（正对
= 0° 且通过；相机 (0.6,0,0.1) 看 +z 面片 ≈ 80° 且不通过），故本复现采用：

    θ = arccos( clip( dot(n_j, p_c − c_j) / ‖p_c − c_j‖, −1, 1) ) ≤ 75°

即（视点 → 面片中心）与外法向的夹角，背向面片（θ>90°）自动不可见。
**注意**：指南 §7.1 的代码片段（`los = c_j − p_c; arccos(los·n_j)`）有符号
笔误——外向法向下正对视角会算出 180°，与其自身 §7.5 的"角度 0°"矛盾；
本实现按其测试例的正确符号实现。其余判据：FOV 30°(水平)×21°(垂直) 按顶点
在相机帧判定（边界含端点，论文未规定边界约定）、FOD 0.40–1.00 m 按顶点距离
（Table I）、遮挡 = 视点到各顶点的射线对离散化工件网格求交（自命中容差
ε=1e-3 m，指南 §7.1）；聚合规则 = 论文原文"全部顶点满足全部条件"。

已知的结构性结果（诊断结论，非 bug）：工件底面（法向 −z）在 M2 的桌面约束
（视点 z ≥ 桌面+0.1）下**永远不可见**——视点恒在底面上方，视角 θ>90° 必失败。
bracket 上底面 164/976 面片（16.8%）因此恒零列，可见表面 83.1% 全覆盖
（n=400）。论文工件同样"放在台面上"，该类不可达表面是复现指南 §0 把验收线
放宽到 95% 的原因之一；后续阶段需决定底面是否计入覆盖率分母（不要改 M2）。

## M4 用法

```bash
# 最小工件示例（V_bracket_{200,300,400} → E + home 往返，三面板图）
uv run python -m examples.run_m4_example

# CLI
uv run python -m src.m4_travel_cost --viewpoints data/outputs/V_bracket_400.npz \
    --config configs/base.yaml
```

## M4：travel-cost 模型与参数来源（记录在案）

论文的行走代价 = MoveIt/OMPL（RRT-Connect）+ FCL 无碰撞路径 + cubic spline
参数化的**关节空间时间**，存于无向图 G(V,E) 的边；论文没有给出任何数值系数。
本复现按指南 §8（简化偏差 #1）使用代理：

    e(v_i, v_j) = α·‖p_i − p_j‖ + β·arccos(v̂_i · v̂_j)   （单位：秒）

- **α = 1.2 s/m、β = 0.4 s/rad**：reproduction 自选（理由：臂末端有效速度
  0.8–1.5 m/s + 姿态调整开销）；两量纲分别为 s/m·m 与 s/rad·rad，同为秒故可加。
- **θ = 光轴夹角**（指南公式只含 v̂）：roll 不入模型（论文 roll 自由），记为限制。
- **home pose**（指南 §8，论文仅 Fig.2 图像无数值）：p_h = 工件中心 +
  (0.9, 0, 0.6) m，视线指向工件中心；roll 未定义且不需要（最小假设）。
- 对称性：E = Eᵀ（论文图为无向）；C_ii ≡ 0 按定义显式置零（输入方向归一化
  的 fp 误差否则会留下 ~1e-8 s 残差）。
- 已知限制：欧氏距离 ≠ 真实机械臂路径长度——绝对 cycle time 不可与论文直接
  比较，相对模式（联合 vs 分开求解）可比；指南 §8 标定技巧（M5 后调 α 使
  cycle time 落 15–30 s）留给 M5 阶段。
- **索引约定（M5 直接消费）**：E[i,j] = v_i → v_j（输入顺序）；
  home_to_viewpoint[i] = home → v_i；viewpoint_to_home[i] = v_i → home。
- M4 不删除/重排任何候选视点，不使用 M3 可见性（纯行走代理）。

## 目录

```
configs/base.yaml     # 全部参数（paper05_replication.md §14）
data/meshes/          # 工件网格
data/outputs/         # S_*.npz / V_*.npz / A_*.npz / E_*.npz（+ .json meta / .png 可视化）
src/m1_discretize.py  # M1：load → prepare → discretize → check → save → visualize
src/m2_sample.py      # M2：位置采样壳 + 可达壳 + Eq.10 势场视线 + 校验/可视化
src/m3_visibility.py  # M3：FOV/FOD/视角/遮挡四判据 → A 矩阵 + 诊断/可视化
src/m4_travel_cost.py # M4：§8 代理公式 → E 矩阵 + home 往返 + 校验/可视化
tests/                # M1/M2/M3/M4 单元测试（含手算用例与自检）
examples/             # 可运行的示例
```
