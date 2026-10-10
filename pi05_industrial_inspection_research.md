# Pi0.5 VLA 迁移至工业自主质检：RA 阶段研究方案

> 本方案基于 9 篇工业检测/视点规划文献综述（见 [notes/](notes/README.md)）与项目内两份 Pi0.5 技术文档（[pi05.md](pi05/pi05.md)、[Pi_franka.md](pi05/Pi_franka.md)）撰写。
> 所有定量数字均来自上述文档，未标注来源处为方案自身的设计判断；不确定处显式写"需进一步验证"。
> 注意：paper_01 的 Table 1/2 与 paper_03 的 Table I–V 在原始 PDF 中无法提取（详见 [README](notes/README.md)），本方案中对应数字引自论文正文叙述，引用时建议复核原文。

---

## 1. Literature Summary

### 1.1 演进主线：2012 → 2025

**第一阶段（2012）：离线 CAD 驱动的任务化视点规划。** [paper_06_review.md](notes/paper_06_review.md) 是把"检测规划"从"最少视图覆盖全表面"这一通用目标上拆下来的关键一步：它把机械检测任务显式分成 5 类（GD&T 公差验证、钣金切割轮廓、大曲面外观、全局形变、特定点定位），用 NURBS B-Rep + GD&T 属性驱动离线视点生成，用 Visibility Map / Combined Visibility Map（单位球 5° az/el 采样、GPU OpenGL 实现）判可见性，用 Dijkstra 在位姿图上排序。它还首次把"可配准性（alignability）"当作一等约束写进视点生成（光学扫描仪偏好 30–50° 倾斜位姿，以保留相邻面信息支持 ICP）。其代价是覆盖率不完整成为常态：后车门 32 views / 91.6% 控制点 / 93.7% 可对齐，引擎盖 19 / 75% / 79%，前车门 17 / 62.7% / 82.3%，作者把这归为物理限制（标记框架遮挡）而非算法问题。

**第二阶段（2017）：组合优化接管"选哪些视点 + 按什么顺序"。** [paper_05_review.md](notes/paper_05_review.md) 与 [paper_07_review.md](notes/paper_07_review.md) 出自同一技术脉络，把 View Planning 形式化为 SCP、Path Planning 形式化为 TSP，并进一步合并为 **SCTSP**（paper_05）与 **sequencing SC-GTSP**（paper_07），用 Random-Key Genetic Algorithm 单次求解。这阶段最有价值的两个结论是：

- **"所需视点数最少 ≠ cycle time 最短"**：paper_05 的 Table II/III 中 GA-SCP-GA-TSP 的 $N_{vp}$ 反而最少，但 cycle time 最差；分离求解时 SCP solver 不知道 traveling cost，必然次优。paper_05 在 Target Object 1 上改善 10.8% / 19.6%，Target Object 2 上 18.6% / 23.3%，最好情形 28.4%；paper_07 在 4 个物体上相对 TSP / GTSP 基线分别改善 14.5%/17.2%、22.7%/21.9%、18.7%/20.5%、17.6%/23.3%，最大降幅 28.1%（Target Object 4）。
- **"约束可以搬进解码器"**：paper_07 的 sequencing 表述一旦把访问顺序交给排列隐式决定，TSP 的入度/出度/子回路约束（Eq.3–5）全部消失，覆盖率约束被吸收进适应度函数（累加到 $\delta' > \delta$ 即 break），于是约束优化变成无约束的 random-key 优化。这是"先算后查"工程哲学的顶点：可见性矩阵 $A\in\{0,1\}^{n\times m}$ 与行进代价 $E$ 全部预计算，优化阶段只查表。

paper_07 还填了另一块空白——**冗余系统的联合优化**：6-DOF 臂 + 1-DOF 转台 = 7-DOF，每个视点保留 $l=5$ 个 IK 解构成 cluster，GTSP 的 Eq.(7) 规定"解中任意两个位姿不能来自同一 cluster"。同期的 paper_05 则停留在非冗余 6-DOF，且在 Table II/III 中暴露了另一个事实：400 samples 下 RKGA 需要约 3 小时（i7-5500U + 12G RAM），其中位姿间轨迹与 traveling cost 计算占绝大部分——**离线一次性规划的算力代价是这条路线的天花板**。

**第三阶段（2018）：把 CPP 写成 MDP，在线闭环对抗位姿扰动。** [paper_01_review.md](notes/paper_01_review.md) 是这一阶段唯一的在线方法。它的核心动机是产线的 placement/localization error 导致 pose variation，使离线 CAD-based open-loop 方法失效；既有 NBV 只能单步局部最优。它的解法是状态增广 $s=[p^s, m^s_{surf}]$——把"机器人位姿 + 全部历史覆盖信息"编码进当前状态（从而满足 Markov 性），用 ε-greedy Forward Tree Search（MCTS 变体，Selection/Expansion/Simulation/Back-propagation）在线建搜索树，Simulation 阶段的 default policy 是 ForwardGreedySearch，取"单位代价带来的增量覆盖"最大的动作；BestChild 用 $(1-\alpha)q_{v'} + \alpha\, c_{v'}/N_{v'}$ 折衷最优代价与平均代价。

结果：算例只需 4 次测量即可完成（2252 个三角面片、bounding box 187mm × 374mm × 237mm、500 个候选视点、覆盖率要求 96%/98%、检测时间常数 1.0 s）；8 个扩展工件上 ε-greedy FTS **无例外地**击败 NBV-greedy，cycle time 下降区间 9.8%–43.5%，平均下降 21.4%（96% coverage）与 26.6%（98% coverage），结论段给出的总体数字是最大 43.5%、平均 24.0%；coverage 要求越高改善越明显。它同时暴露了这条路线的结构缺口：可见性矩阵 $M$ 是**离线预计算并固化**的，在线阶段只靠重查同一张表更新状态，没有真实视觉反馈闭环；且全部是计算实验，没有真机验证。

**第四阶段（2021）：回头看"候选视点是怎么生成的"。** [paper_02_review.md](notes/paper_02_review.md) 与 [paper_09_review.md](notes/paper_09_review.md) 是同一组作者的方法论修正。paper_02 的立场是：既有工作大多"先随便撒一个足够密的候选集，再去做优化"（Tarbox & Gottschlich 的 15,000 候选、Englot 的约 2500 候选、Scott 的 72 候选），候选不基于几何特征，inherent 地给下游 NP-hard 的 set covering 设了 handicap。它的解法是 B-spline 曲面网络 + 六个几何泛函（curvature / thin plate / normal deviation / incidence angle / area / ROI）驱动的参数域递归细分，每个 segment 放一个候选（pivot = 参数域中心，相机沿法向退 $d$），候选数与 mesh 分辨率解耦、由阈值或目标数 $t_n$ 直接控制（实测 42–1571）。thin plate 泛函的细分耗时在含 452 张曲面的 tangle cube 上仅 0.008 s（$M$ 矩阵可预计算跨模型复用），比曲率泛函快 3–4 个数量级。

paper_09 则把"生成"与"优化"显式解耦，用 292 个候选集（Gear 98 / Hirth 107 / Crankshaft 87）做公平对比，并首次把自动规划与专家手工规划直接比较。它的三条结论对本方案是决定性的：

1. **Space Sampling 在工业件上不可接受**：平均 61.87% 候选视点不可接受（Vertex 39.51%、Patch 15.86%，Crankshaft 的 patch 方法低至 0.3%，因为该件无孔洞空腔）；Crankshaft 用 Avg bbox 半径时覆盖率只有 29.14%（34 个视点），Max bbox 仅 30.38%。造成不可接受的主因是**拓扑复杂度（孔洞/空腔）而非形状复杂度**。
2. **generate-and-use 这样的纯反应式规则是稳定的**：不需要候选集、不需要参数，随机取一个未覆盖 primitive → 沿其法向退 focusing distance → 检查光轴处约束 → 满足则纳入、否则丢弃该 primitive，可稳定达到 ≥90% 覆盖。论文原话评价它"will not provide a minimal set of viewpoints"，但形态上与 $o_t \to a_t$ 的同构性最强。
3. **"什么是好的检测计划"没有答案**：专家计划明显不同于自动计划——Gear 上 44/44 个视点（两位专家）达到 100% 覆盖，而最佳自动结果只要 23（exact curvature）；专家的视点更多但**呈现空间模式**（集中在侧面、由两个视点的旋转复制得到，等价于"两个固定相机位 + 转台"），而自动生成"exhibit no particular pattern in spatial distribution"。论文据此提出一个尖锐问题：**"更少的视点不等于更好的计划"**（视点数少的代价是路径规划与节拍问题），并把"数字化人工检测规划可能与自动化同等重要"作为结论。

**第五阶段（2024）：分层与实测点云。** [paper_03_review.md](notes/paper_03_review.md) 的 MG-HVP 用 PBPS 点云分割网络（输出点级实例 $I$、几何类型 $T$、边界概率 $B$）把表面分成 4 类规则 primitive（plane/cylinder/sphere/cone）+ 1 类不规则面：规则面走 bottom-up 闭式网格化（Eq.8–13 直接解出等距划分间隔，完全可并行、无冗余），不规则面走 top-down（HVCR 混合置信度 $V^t_i = C^t_i Q^t_i(\varepsilon + b_i\kappa_i)$ 三项乘积 + 自适应半径 DNS + FOV-Shift 用覆盖点集 PCA 主方向确定绕视轴旋转）；路径规划用边权 $l_{ik}=\|p^v_i-p^v_k\|_2(1-\mathrm{sim}(e_i,e_k))$ 的图遍历。

数字上：Synthesis 数据集上仅用 64.4 个视点实现完整覆盖（常用贪心方法需要两倍以上）；ABCParts 上 $C_{rate}$ 与 $S_{rate}$（定义为 $C_{rate}>95\%$）均为 100%；TLess 上 $V_{num}$ 约为其他贪心方法的 50%，求解时间仅比最快的 K-Greedy 多 0.4 s；消融中 DNS 使 $P_{time}$ 降低近 50%，HVCR 使 $P_{leng}$ 减少近 10，点云密度实验里 $V_{num}$ 快速稳定在 52 左右而贪心方法持续上升且耗时指数增长。真机平台是 UR5e（航空叶片）与 5-DoF 检测台（手机中框），扫描仪为 Mech-Eye Nano。论文的诚实声明值得记录：**遇到小孔洞或褶皱物体时直接采用 MG-HVP 生成的视点可能有问题**，作者把视点角度调整当作后处理集成进来。

[paper_04_review.md](notes/paper_04_review.md) 把"不依赖 CAD"推到实用化：输入是 3D 相机扫描得到的实测点云（5000~10000 点），先做 KNN 距离权重 + 库仑力法向修正（复杂物体角偏差从 Open3D 的 1.31 rad 降到 0.09 rad，角度偏移率 74.40% vs 49.80%），再做概率边界引导的图元分割，层次化视点用遗传算法最小化视点数，PR3Net（单张标注样本 + 半监督一致性损失）回归物体平面 3-DoF 位姿 $(u,v,\theta)$ 在线补偿随机摆放，最后把 1-DOF 转台建模为 DH 链第 1 关节，用梯度下降 IK 求 7 个关节角顺序执行。实测：$AC_{rate}$ 96.45%、$V_{num}$ 25.00、$E_{time}$ 82.75 s（对比 K-Greedy 39.00/127.75/92.80、P-Greedy 35.00/114.25/91.15、TCluster 29.00/97.75/83.95）；五个实物 $C_{rate}$ 全部 100.00%，$RepeatC_{rate}$ 均 < 50%；真机对比示教在固定位姿下 87.73 s / 99.47% vs 371.42 s / 97.86%，非固定位姿下 91.25 s / 96.31% vs 437.16 s / 83.26%。但它在形式化上明显退步：**没有可见性矩阵、没有 NBV 闭环、没有访问顺序优化、方法部分没有任何避障算法描述**——$E_{time}$ 的收益主要来自"视点数少"而非"路径短"，这正是 paper_05/07 明确指出的次优来源。

**第六阶段（paper_08，预印本，年份未标注）：主动 Real-to-Twin。** 前五阶段全部止步于"几何可见 + 成像清晰"，没有任何一篇给出缺陷检出率指标（paper_03 自己都不把缺陷检出率作为指标）。AVATAR 换了思路：不改进 2D 判决边界，而是**消除视点这个自由度**——机器人主动把相机搬到 CAD 视点（等面积多面体采样 + 仰角 $20^\circ \le \theta \le 80^\circ$ 过滤），Blender 在完全相同配置 $(C, T^{base}_{obj}, T^{base}_{cam,k}, K)$ 下渲染 Digital Twin，冻结 DINOv2 提 dense patch token、双 projector 映射到共享语义空间、用亲和矩阵 $M=\text{Softmax}(Q(Z_r)E(Z_s)^T/\sqrt{d})$ 把数字语义重排到物理坐标系，最后用前景加权 CosSim 残差 $A=(1-\text{CosSim}(Z_r,\hat{Z}_s))\odot W$ 定位异常。训练只用 216 对正常（无缺陷）配对，却拿到 P-AP 23.75 / P-F1 33.07，分别比第二名 UniAD 高 8.62 / 10.50 个点。它的两个"洞"也很清楚：**"Active"是开环的**（视点是预先算好的确定性轨迹，不是 NBV），以及 **robotic jitter 与标定误差被作者自己列为必须压制的干扰**，而对策只是"在训练中学会吸收它"（统计性、非保证性）。

### 1.2 时间线总表

| 年份 | 论文 | 一句话方法 | 覆盖/规模 | 关键数字 | 遗留缺口 |
|---|---|---|---|---|---|
| 2012 | [paper_06](notes/paper_06_review.md) | GD&T 驱动的离线视点 + VM/CVM + Dijkstra | 3 个钣金件 5,683–6,111 faces；1 个压铸件 | 控制点采集 91.6% / 75% / 62.7%；可对齐 93.7% / 79% / 82.3% | 需手工加 2 个 pose 桥接；覆盖率不完整是常态 |
| 2017 | [paper_05](notes/paper_05_review.md) | SCTSP + RKGA 单次联合求解 | 2 个仿真工件 + 1 组真机；200/300/400 候选 | 改善 10.8%–19.6% 与 18.6%–23.3%，最好 28.4%；真机 19.2 s vs 23.8 / 25.7 s | 400 samples 约 3 小时；offline 开环；多 IK 解未处理 |
| 2017 | [paper_07](notes/paper_07_review.md) | sequencing SC-GTSP + RKGA + 冗余 cluster | 4 个物体；6-DOF 臂 + 1-DOF 转台 | 改善 14.5%/17.2%、22.7%/21.9%、18.7%/20.5%、17.6%/23.3%，最大 28.1% | 需 CAD mesh；无 NBV；不建模反射/高光 |
| 2018 | [paper_01](notes/paper_01_review.md) | CPP 建模为 MDP + ε-greedy FTS 在线规划 | 8 个工件；2252 面片 / 500 视点 | 平均下降 24.0%（96% 时 21.4%、98% 时 26.6%），最高 43.5%；4 次测量 | $M$ 离线固化；无真实视觉反馈闭环；无真机 |
| 2021 | [paper_02](notes/paper_02_review.md) | B-spline 六泛函驱动的候选生成 + set cover + NBV | 3 个模型（21,800 / 51,800 / 82,672 faces） | 候选 42–1571；thin plate 细分 0.001–0.008 s；tangle cube RT 1771.99–3103.06 s | 不显式保证覆盖率；不处理遮挡与碰撞；不做路径 |
| 2021 | [paper_09](notes/paper_09_review.md) | 三种 sampling 范式系统对比 + 自动 vs 专家 | 292 个候选集；3 个物体（74,018–99,998 faces） | 不可接受视点率 Space 61.87% / Vertex 39.51% / Patch 15.86%；最佳 99.09%@34、99.65%@13、99.67%@24；专家 44/44、21/13、34/32；ray tracing 3.81 s/视点 | 无 travel cost、无运动规划；只考虑几何不考虑材质光照 |
| 2024 | [paper_03](notes/paper_03_review.md) | PBPS 分割 + 规则闭式网格 / 不规则自适应 NBV | 20+40+10 个点云；UR5e 与 5-DoF 真机 | Synthesis 64.4 视点；ABCParts $C_{rate}$/$S_{rate}$ 100%；TLess $V_{num}$ 约 50%；DNS 使 $P_{time}$ 降近 50% | 规则 primitive 只有 4 类；依赖完整先验点云；路径只到图遍历层 |
| 2025 | [paper_04](notes/paper_04_review.md) | 实测点云 + 库仑力法向 + 图元分割 + 遗传算法视点 + 梯度下降 IK | 40 个 ABC 点云 + 5 个实物；UR5e + TBR100 转台 | $AC_{rate}$ 96.45%、$V_{num}$ 25.00、$E_{time}$ 82.75 s；实物 $C_{rate}$ 100%；87.73 s vs 371.42 s | 无可见性矩阵、无 NBV、无顺序优化、无避障描述 |
| 未标注 | [paper_08](notes/paper_08_review.md) | 主动对齐 CAD 视点 + Blender Twin + 残差打分 | 216 训练对 / 214 测试对；UR 臂 + D435 | P-AP 23.75 / P-F1 33.07（+8.62 / +10.50 超 UniAD）；I-AUROC 71.15 | "Active" 是开环；robotic jitter 只能统计补偿；只验证 2 个类别 |

### 1.3 当前方法已经解决了什么

| 已解决 | 代表 | 证据 |
|---|---|---|
| **覆盖保证与可验证性** | paper_02 / paper_05 / paper_07 / paper_09 | 可见性矩阵 $A\in\{0,1\}^{n\times m}$ + set cover 给出"$n_{opt}$ 个视点覆盖全部面片"的可验证结论；$\delta=99.5\%$ 是任务级终止条件 |
| **视点数优化与候选集质量控制** | paper_02 / paper_03 / paper_09 | 候选数可控（42–1571）、由几何特征驱动、与 mesh 分辨率解耦；规则面用闭式网格化避免冗余（Synthesis 上 64.4 视点） |
| **排序优化与子问题耦合** | paper_05 / paper_07 | SCTSP / SC-GTSP 单次求解，最坏情形省 28.1%–28.4%；"点少 ≠ 时间短"被实验证实 |
| **冗余自由度利用** | paper_04 / paper_07 | 转台 + 臂 = 7 DOF；cluster 保留 $l=5$ 个 IK 解供 GTSP 选择；paper_04 的 $RepeatC_{rate}$ 全部 < 50% |
| **无 CAD 采集** | paper_04 | 实测点云 → 图元分割，五个实物 $C_{rate}$ 100%，真机 87.73 s vs 示教 371.42 s |
| **评估基准与公平对比** | paper_09 | R1–R4 四条可检验要求；292 个候选集；首次自动 vs 专家对比；公开模型与代码 |
| **零缺陷标注的判异** | paper_08 | 训练只用 216 对正常配对，P-AP 23.75（+8.62） |

### 1.4 仍然存在的问题

1. **CAD 依赖（model-based 的边界）**：paper_01/02/03/05/06/07 全部要求目标几何已知；论文们自己也承认"对未知物体不适用"。paper_04 虽然去掉了 CAD，但退化成**必须先扫描一轮**才能规划，不能边看边规划。
2. **位姿扰动脆弱**：离线一次性规划的 $A$ 与 $G(V,E)$ 是基于名义几何计算的，工件摆放一变即失效（paper_05 把非固定位姿列为 future work）；只有 paper_01 用在线 MDP 应对，代价是放弃真机验证。
3. **专家驱动成本高**：paper_02 明确说传统配置流程"can take days or even weeks"；paper_06 的五大批评第一条就是"the choice of many parameters"；paper_09 指出专家规划时间**未被测量**，而这恰是自动化的主要卖点。
4. **"好计划"无判据**：paper_09 结论直陈"evaluation of an inspection plan as adequate is still not possible"——纯覆盖率无法表达专家隐式偏好（fixed camera + turntable 复制、floating viewpoint、留出配准余量）。
5. **被动感知**：九篇文献里没有任何一篇把"当前采集到的图像质量"反馈回规划——paper_03 的"图像清晰到足以检测缺陷"是**被断言而非被实验证明**，paper_02/05/07 的可见性判据是理想化几何判定，不建模反射/高光/景深模糊。
6. **与学习系统割裂**：全部九篇都是手工设计的目标函数 + 组合优化或贪心搜索，没有一篇涉及策略学习；paper_01 只在 future work 里提到"用神经网络做 value function approximation"。

---

## 2. Pi0.5 Capability Analysis

依据 [pi05.md](pi05/pi05.md)，逐项拆解 Pi0.5 在工业检测语境下**能提供什么**与**不能提供什么**。

### 2.1 Pi0.5 可以提供的四项能力

| 能力 | 架构组件（出处：pi05.md） | 工业检测语义 |
|---|---|---|
| **Visual understanding** | SigLIP So400m ViT：输入 224×224×3、patch=14 → 每相机 **256 个 patch token**、宽度 1152、逐 token 投影到 2048；3 路相机拼成 **768 token**；`pool_type="none"` 保留全部 patch token | "这张图里有什么"——本零件/本区域是否出现在视野内、图像是否异常、表面是否有可见瑕疵。**是语义级的，不是几何级的** |
| **Language conditioning** | PaliGemma sentencepiece（$V=257152$）；prompt 上限 **200 token**；π₀.₅ 特色是把 8 维状态按 256 bins 离散成 0~255 整数拼进 `"Task: {指令}, State: 32 128 5 ...;\nAction: "` | 任务类别（paper_06 的 5 类）、ROI 关键词（paper_02 的 ROI 泛函语义）、约束描述（"倾斜 30–50 度"）都能走这条通道，**零架构成本** |
| **Robot state fusion** | 归一化 state（本项目为 z-score）→ `np.digitize(state, bins=linspace(-1,1,257)[:-1]) - 1` → 0~255 → 数字文本进 prompt | 这是把 $(u,v,\theta)$（paper_04）或覆盖状态摘要接入模型的**最低成本通道**；也是唯一能让 policy 知道"当前相机在哪"的入口 |
| **Action prediction** | 流匹配（rectified flow）：训练目标 $u_t=\varepsilon-a$，loss 只有 $\mathrm{MSE}(v_t,u_t)$；推理 10 步 Euler（$t:1\to0$, $dt=-0.1$）+ 前缀 KV cache 一次；动作块 32 维内部、**8 维真实**（7 关节速度 + 1 夹爪，或 EE delta pose），其余补零；`action_horizon` 默认 50 | 视点间的连续运动生成。**与 paper_04/paper_07 的 7-DOF 系统（6-DOF 臂 + 1-DOF 转台）维度一一对应**，夹爪那一维可重定义为转台通道 |

一次完整前向约 $968 + 10 = 978$ 个 token（前缀 968 = 768 图像 + 200 文本；后缀每次 10 个动作 token，采样 10 次），这是实时控制的延迟来源。

### 2.2 Pi0.5 缺失的三类能力（附论证）

**（1）Explicit geometry——结构性缺失，非数据问题。**
SigLIP 的输入是固定的 224×224×3 RGB，**没有深度通道、没有点云输入通路、没有 CAD 通道、没有相机内参通道**。它的输出是 256 个语义 patch 嵌入，没有任何一个 token 显式表示"这里曲率是多少"（paper_02 的 $E_\kappa$）、"法向与视线夹角多少"（paper_02 的 $E_{IA}$）、"该面片是否被遮挡"（paper_05/07 的 $A_{ij}$）。paper_06 的 VM 是 $M\in\{0,1\}^{72\times72}$ 的精确布尔结构、paper_09 的判定建立在 2448×2050 px 物理相机的射线求交上——SigLIP 把输入降到 224×224，**降采样约 10 倍**，patch=14 意味着一个 token 覆盖 14×14 像素，亚 patch 级的孔洞、螺纹、窄槽在层次上就会丢失。**几何量必须留在模型之外。**

**（2）Coverage representation——表示能力缺失。**
覆盖状态 $m^s_{surf}$（paper_01）本质是一个"面片是否已被看到"的集合，paper_05/07 把它写成硬约束 $\sum_i b_i A_{ij}\ge 1$。Pi0.5 的注意力是 **连续 softmax**（$A_{s\to p}\propto Q_aK_p^{\mathrm T}$），**没有任何机制表达"$\ge 1$ 覆盖"这种集合运算**；KV cache 只缓存前缀的 K/V，**不带任何覆盖状态**，模型也没有跨 episode 的显式状态持久化。paper_07 的 review 把这一点讲得最直接："让 VLA 记住还剩哪些面片没覆盖，在架构上没有对应的记忆结构"。

**（3）Inspection completeness guarantee——保证能力缺失。**
三层原因叠加：
- **Reactive policy**：单次前向 + 10 步 Euler 直接吐出 action chunk，没有候选集评估、没有 rollout、没有组合搜索、没有可验证的最优性；
- **开环 chunk 执行**：[Pi_franka.md](pi05/Pi_franka.md) 5.1.6 明确记录 `ActionChunkBroker` 是**纯开环**——10 步 chunk 全部执行完才重新推理；`examples/droid/main.py` 的 15 Hz 循环里每 8 步（`open_loop_horizon`）才向 WebSocket 远端请求一次新 chunk；
- **纯 MSE loss**：训练损失只有流匹配 MSE（$v_t$ vs $u_t=\varepsilon-a$），**没有任何约束满足机制**；prompt 里写"覆盖率需达 96%"只能提供软性偏置。

paper_03 定义的 $C_{rate}>95\%$ 是**证书型约束**，paper_02 能给出"$n_{opt}$ 个视点覆盖全部 $n_F$ 面片"的可验证结论；Pi0.5 只能给出"我倾向去哪里"，**无法自证"我已经看全了"**。

### 2.3 本仓库 config 变体对上述能力的实际影响

本仓库 `pi05_lora_finetune_*` 系列 config 与标准 π₀.₅ 有 5 处差异（pi05.md 第 5 节），其中 4 处直接影响能力：

| 变体项 | 标准 π₀.₅ | 本项目 | 对工业检测能力的实际影响 |
|---|---|---|---|
| `discrete_state_input` | True（状态进 prompt） | **False** → prompt 无状态文本，suffix 也无状态 token | **最严重的一条**。paper_02/03/06/09 的 review 都独立指出同一结论：模型不知道相机当前在哪、离表面多远、已看过哪些区域。paper_09 的措辞是"这在数学上是不可解的"。**要注入覆盖状态或视点位姿，重开本通道是最直接的路径**；另一条零架构改动的路径是客户端逐次构造 prompt 文本注入（`examples/droid/main.py` 的 request_data 中 `prompt` 由客户端发送，`InjectDefaultPrompt` 只在缺省时注入默认值），两条路径都受 200 token 预算约束 |
| 归一化 | 分位数 q01/q99 → [-1,1] | **z-score** | 对离群值更不鲁棒；256 bins 的离散化区间与 z-score 的尾部不匹配，需在数据侧做 clip 或统计校正 |
| `action_horizon` | 50 | **10** | 单次 chunk 覆盖的时域缩短 5 倍。按 15 Hz 控制循环、每 8 步重规划计，一次 chunk 实际只执行 8 步 ≈ 0.53 s。对"从视点 A 走到视点 B"这种可能持续数秒的运动，需要多次重规划拼接 |
| 微调方式 | — | **异构 LoRA：PaliGemma rank 16 / 动作专家 rank 32**，冻结主干 | 训练代价低，但**表达力受限**：若要把几何先验蒸馏进策略，低秩旁路是否有足够容量是**需进一步验证**的开放问题 |
| 输入相机 | 3 路（第 3 路可 mask） | **2 路真实（base + wrist），第 3 路填零 + mask** | 第 3 路是**天然空闲插槽**。paper_02/03/06/07/08 的 review 都指向同一用法：把几何伪彩图或 CAD 渲染 twin 填入该槽位（机制同构：缺失相机填零 + `image_mask=False`，mask 掉的 256 个 token 整行整列置 `big_neg=-2.38e38`） |

---

## 3. Traditional Pipeline vs Pi0.5 Pipeline

### 3.1 两条数据流并排

```
┌──────────────── TRADITIONAL (paper_02/05/06/07 主线) ────────────────┐
│                                                                     │
│  CAD / NURBS B-Rep / mesh                                           │
│      │ 表面离散化（Bubble Mesh 均匀重采样 / cordal tolerance 三角化）│
│      ▼                                                              │
│  S = {s_1..s_m}  面片集合（m = 数百~数万）                          │
│      │ 视点候选生成（椭球半球随机采样 / 特征泛函细分 / 关键点检测）  │
│      ▼                                                              │
│  V = {v_1..v_n}  候选视点（n = 42 ~ 2002）                          │
│      │ IK + FCL 过滤（IKFast / 梯度下降 IK）                        │
│      ▼                                                              │
│  A ∈ {0,1}^{n×m}  可见性矩阵（FOV ∧ 视距 ∧ 视角 ∧ 无遮挡）          │
│      │ + G(V,E) 运动图（RRT-Connect + cubic spline → traveling time）│
│      ▼                                                              │
│  SCP / SCTSP / SC-GTSP 组合优化（贪心 NBV / RKGA 单次求解）          │
│      ▼                                                              │
│  O ⊆ V  有序视点序列 + 无碰撞轨迹（ROS trajectory / ROS-Industrial） │
│      │ 执行（ABB IRB-4600 / UR5e / 转台）                            │
│      ▼                                                              │
│  I = {I_1..I_k}  每个视点采集的图像                                  │
│      │ 下游缺陷检测（YOLO / 点云-CAD 比对 / 逆向工程软件）           │
│      ▼                                                              │
│  ┌─ VERIFICATION ────────────────────────────────────────────────┐  │
│  │ · coverage ratio = 已采集面积 / 总面积（按 feature/face/datum）│  │
│  │ · C_rate = |∪C(v_i)| / N；C_rate > 95% 为一次成功规划（p03）  │  │
│  │ · 控制点采集率 / 可对齐视图率（p06）；alignability 判据        │  │
│  │ · A_Crate 关键部位清晰覆盖率；ape/AAE 位姿精度（p04）          │  │
│  │ · 未覆盖面片 → 重新聚类 → 新增 pose（p06 的迭代加位姿闭环）    │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘

┌──────────────────────── Pi0.5 (openpi 栈) ──────────────────────────┐
│                                                                     │
│  RGB 图像 × 2（base + wrist，224×224 resize 后经 msgpack 上传）      │
│  + 语言 prompt（≤200 token，本项目无状态文本）                       │
│      │ SigLIP So400m：224/14 = 16×16 → 256 token/相机 ×3 = 768      │
│      ▼  逐 token 投影 1152→2048                                     │
│  Z_prefix = [B, 968, 2048]   （第 3 路填零 + image_mask）            │
│      │ 18 层双专家 Transformer（共享注意力，独立 FFN）               │
│      │   PaliGemma 专家 D=2048 双向 / 动作专家 D=1024 因果           │
│      │   前缀 KV cache 只算一次                                      │
│      ▼                                                              │
│  10 步 Euler 流匹配采样（t: 1→0, dt=-0.1），每步只算 suffix          │
│      ▼                                                              │
│  x_0 ∈ [B, 10, 32]  → 取前 8 维（7 关节速度 + 1 夹爪 或 EE delta）   │
│      │ msgpack 回传，ActionChunkBroker 纯开环执行                    │
│      ▼                                                              │
│  机械臂执行（15 Hz 循环，每 8 步重新请求 chunk）                     │
│      ▼                                                              │
│  ┌─ VERIFICATION ────────────────────────────────────────────────┐  │
│  │ 【架构内不存在】                                              │  │
│  │ · 无 coverage 概念、无 $m^s_{surf}$ 状态、无 A 的查询接口       │  │
│  │ · 无 dense 输出头（唯一输出头是 action_out_proj 1024→32）      │  │
│  │ · loss 只有流匹配 MSE，无约束项                                │  │
│  │ · 无评估回环：chunk 执行完只会"再推理一次"，不会"检查是否看全" │  │
│  │  → 必须由外部几何 verifier 补齐（本方案第 4/6 节）             │  │
│  └───────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.2 逐环节对比

| 环节 | Traditional Method | Pi0.5 | 差距与可替代性 |
|---|---|---|---|
| **Observation** | CAD/mesh（离线先验）+ 传感器规格（FOV/viewing range/viewing angle）；paper_04 用实测点云替代 CAD | 2 路 RGB（224×224×3）+ 语言 prompt；**无深度、无点云、无 CAD、无内参** | **不可替代**。模态级差距：$\vec n_i$、$\kappa_i$、$v=[p,d,u]$ 在 token 里不存在。若要用 CAD，只能渲染成图喂第 3 路相机 |
| **Representation** | 面片集合 $S$、候选视点集 $V$、二值矩阵 $A\in\{0,1\}^{n\times m}$、运动图 $G(V,E)$ | 768 图像 token + ≤200 文本 token；**无集合结构** | 可部分桥接：$A$ 的低维摘要（覆盖率、未覆盖方位）+ 视点坐标经 **256 bins 离散化文本**进 prompt（需重开 `discrete_state_input`）；paper_07 的 review 量化过风险——256 bins 的分辨率 = 归一化量程/256：按 ±0.5 m 量程约 **3.9 mm**/bin，若按 paper_04 的 215±30 mm 工作距离把量程收紧到 ±30 mm 则约 **0.23 mm**/bin——**分辨率完全由归一化区间的选择决定**，检测场景下并非不可用 |
| **Viewpoint Selection** | 显式：椭球半球随机采样（p01）、特征泛函细分（p02/p09）、规则闭式网格 + HVCR/DNS（p03）、遗传算法（p04）、关键点检测（p04） | 隐式：无候选集、无排序、无打分函数 | **整体不可替代，局部可学习**。paper_02 的候选生成是**开环全局**的（pivot 固定、距离 $d$ 固定、方向固定为法向反向、不做遮挡/碰撞检查），论文自己在 7.4 承认并说"Additional post-processing steps can be applied to correct the positioning of invalid viewpoint candidates"——**这个 post-processing 就是 VLA 的位置** |
| **Coverage** | 硬约束 $\sum_i b_i A_{ij}\ge 1$（p05/07）；$\delta=99.5\%$；$C_{rate}>95\%$ 判定成功（p03）；迭代加位姿直到无法再加 pose（p06） | **无** | **完全不可替代**。这是 VLA 唯一给出的答案是"我倾向去哪"，而非"我是否看全"。必须保留为 external verifier（p02/05/07/09 的 review 四方一致） |
| **Sequencing** | SCTSP / SC-GTSP 联合求解（p05/07），边权含姿态差惩罚 $l_{ik}=\|p^v_i-p^v_k\|_2(1-\mathrm{sim}(e_i,e_k))$（p03）；Dijkstra（p06） | 无（chunk 是局部短程运动，不含全局访问顺序） | **不可替代**。paper_09 的 NBV 优化在 1000 视点下耗时不足 10 秒，而 RKGA 在 paper_05 的 400 samples 下要 **约 3 小时**——离线序列器与在线执行器是两个时间尺度 |
| **Execution** | IKFast 解析逆解 + FCL 碰撞 + RRT-Connect 概率完备路径 + cubic spline 时间参数化 + ROS-Industrial 下发 | 流匹配 10 步 Euler 出 8 维连续 chunk，15 Hz 开环执行 8 步 | **可部分替代**。维度上高度契合：paper_04/07 的 **7 DOF（6 臂 + 1 转台）↔ 7 关节速度维**，夹爪维改转台通道；paper_06 的 $v=[p,d,u]$ 是 9 维，**Pi0.5 单步仅 8 维，一步表达不了完整 viewpoint**，须靠 chunk 时序展开或改动作头 |
| **Verification** | 覆盖率证书 + 可对齐性判据 + 位姿精度（APE 2.15 / AAE 3.01，p04）+ 接触仿真 | **架构内不存在** | **必须外置**。AVATAR 的残差打分（p08）是九篇里唯一提供"哪里有问题"空间定位的模块，且它需要 dense 输出接口——Pi0.5 的输出头是 `action_out_proj: 1024→32`，**残差打分层无法嫁接到输出端** |
| **Replanning** | paper_01 的在线 MDP（观测 → 更新 $m^s_{surf}$ → 再规划）；paper_06 的迭代加位姿；其余论文**无**（离线开环） | 有，但是**同质的**：chunk 执行完再推理一次，没有跨 episode 的显式状态 | **形态不同**。VLA 的"再推理"不携带覆盖记忆，等价于每次从零开始决策；传统在线方法的"再规划"依赖 $m^s_{surf}$ 的显式更新。若要 VLA 具备真正的重规划，必须补状态通道 |

### 3.3 本质差异

| 维度 | Traditional | Pi0.5 |
|---|---|---|
| 范式 | **几何显式规划**：先建几何模型，再在离散/组合空间里搜索 | **语义隐式策略**：$p(a_{t:t+H}\mid o_t,\ell)$，一次前向 + 10 步采样 |
| 保证性 | **可证书**：$n_{opt}$ 个视点覆盖 $n_F$ 个面片是可验证的结论；碰撞安全是概率完备的 | **概率性**：输出是"无约束动作分布均值附近的一个样本"，安全性来自训练分布的统计偏好 |
| 时间尺度 | 离线/准离线：paper_05 的 400 samples 约 3 小时；paper_02 的 tangle cube ray tracing 1771.99–3103.06 s | 在线：一次完整前向 978 token，15 Hz 控制循环下每 8 步重规划 |
| 泛化机制 | 换零件需重跑全流程（paper_06 的"manual intervention"、paper_05 的 3 小时） | 预训练自大规模跨具身数据（DROID 76k 轨迹），具备跨物体/跨位姿的隐式泛化 |
| 可复现性 | cubic spline 给出确定性带时间戳轨迹，可做节拍标定与安全认证 | 采样随机（$t$ 从 1→0，10 步 Euler），**轨迹不可复现**，$T_{std}$ 意义上的稳定性无保证 |

**一句话**：传统方法把"看清楚"这件事**证明**出来，Pi0.5 把"怎么动"这件事**学会**。二者不是替代关系，而是两个时间尺度上互补。

---

## 4. Migration Possibilities

### Option A：Pure End-to-End VLA Inspection

**核心主张**：完全去掉几何规划器，把检测任务当作一个长时域 manipulation 任务，直接让 LoRA 微调后的 Pi0.5 从 RGB + 语言 prompt 学出"看哪 → 走过去 → 换点"的完整行为。

```
┌─ DATA / TEACHER ────────────────────────────────────────────────────┐
│  ① paper_01 的 ε-greedy FTS：rollout 生成 (观测图像, m^s_surf 覆盖向量,│
│     视点序列) 三元组（它在 8 个工件上无例外优于 NBV-greedy，平均 24.0%）│
│  ② paper_09 的 generate-and-use：无参数反应式规则，稳定 ≥90% 覆盖，   │
│     形态与 o_t→a_t 同构                                               │
│  ③ 专家示教（paper_09 的交互界面精确记录 44/44、21/13、34/32 个视点） │
└──────────────────────────────┬──────────────────────────────────────┘
                               ▼
┌─ TRAINING ──────────────────────────────────────────────────────────┐
│  pi05_lora_finetune config（异构 LoRA：PaliGemma rank16 / 动作专家    │
│  rank32，冻结主干）+ scripts/train.py（JAX + FSDP + EMA 0.99）       │
│  输入：RGB(base+wrist) + prompt "Task: inspect <part> fully;\nAction:"│
│  输出：8 维 action chunk（7 关节速度 + 1 夹爪）                       │
└──────────────────────────────┬──────────────────────────────────────┘
                               ▼
┌─ INFERENCE ─────────────────────────────────────────────────────────┐
│  Policy.infer(obs) → (10, 8) chunk → ActionChunkBroker 开环执行 8 步 │
│  → 15 Hz 循环 → 再推理 … 直到人工/外部信号停止                        │
└──────────────────────────────┬──────────────────────────────────────┘
                               ▼
┌─ VERIFICATION（外部，非模型内）──────────────────────────────────────┐
│  离线用 A 检查哪些面片未被覆盖（Ax ≥ b(F) 是否满足）→ 作为下一轮 ROI │
└─────────────────────────────────────────────────────────────────────┘
```

- **Advantage**：架构零改动（除重开状态通道）；实现成本最低；能验证"VLA 能否学会检测行为"这一基本问题；paper_09 的 generate-and-use 给出了一个明确可模仿的目标行为。
- **Limitation**：**覆盖保证完全丧失**。缓解手段有三条，但都只是统计性的：(i) 训练分布覆盖足够多样的初始位姿 → 依赖数据量与多样性；(ii) 由外部 verifier 决定 episode 终止（而非模型自己判断）→ 但这就引入了 Option B/C 的组件；(iii) prompt 里写任务目标 → paper_01/02/05/07 的 review 一致指出 prompt 只能提供软性偏置。此外 paper_09 的观察在此直接适用：自动/学习得到的视点"exhibit no particular pattern"，**视点数少 ≠ 计划好**。
- **Difficulty**：**中**。训练与推理栈现成（LoRA config 已有大量 `pi05_lora_finetune_task*_ep*`），但需要先造数据（Phase 1），且缺少 coverage 判据就无法判断"学到没有"。

### Option B：Planning-guided VLA

**核心主张**：传统 planner 继续做"去哪"（which），Pi0.5 只做"怎么到那"（how）。paper_03 的 review 把这条分工总结得最清楚：**从"视点序列"到"机器人可执行运动"这一段是空白，恰好是 Pi0.5 可以填的位置**。

```
┌─ GEOMETRIC PLANNER（离线/准在线，不改动）───────────────────────────┐
│  CAD/点云 → 候选生成（paper_02 特征泛函细分 / paper_03 规则闭式网格）│
│         → A ∈ {0,1}^{n×m}（FOV∧视距∧视角∧无遮挡）                   │
│         → G(V,E)，边权 l_ik = ‖p_i-p_k‖₂(1-sim(e_i,e_k))  【p03】   │
│         → SCP/TSP 联合求解 → 有序视点序列 X = (x_1..x_{n_x})         │
│  输出接口（三种可选，可叠加）：                                       │
│   (a) 覆盖状态摘要 c ∈ {0,1}^m 的低维压缩 → 离散化文本               │
│   (b) 渲染 twin 图 I^s（paper_08 路线）→ 第 3 相机槽位               │
│   (c) 语言指令携带 next viewpoint 描述 "next: pivot(p)=..., n=..."   │
└──────────────────┬──────────────────────────────┬───────────────────┘
                   │ (a) 状态文本                  │ (b) twin 图
                   ▼                              ▼
┌─ Pi0.5 ─────────────────────────────────────────────────────────────┐
│  prompt（≤200 token）= "Task: <指令>, State: <256-bin 覆盖/位姿离散>,│
│                          Next: <目标视点>, ;\nAction: "              │
│  相机槽位：base(实拍) | wrist(实拍) | right_wrist(**渲染 twin**)      │
│  → 768 图像 token + ≤200 文本 token → 双专家 → 流匹配 → chunk(10,8)  │
└──────────────────────────────┬──────────────────────────────────────┘
                               ▼
┌─ EXECUTION + GUARD ─────────────────────────────────────────────────┐
│  ActionChunkBroker 执行 → 约束投影层（paper_03 Eq.2/3/5）：           │
│   p'_ix ≤ W, p'_iy ≤ H, D_min ≤ p'_iz ≤ D_max                        │
│   n_i·n^v_i < τ_n（τ_n = -0.3）                                      │
│   2√(p'^2_ix + p'^2_iy) < min{W,H}（绕视轴旋转的 worst-case 判据）    │
│  → 越界动作拒绝/吸附 → FCL 兜底碰撞检查 → 执行                        │
└──────────────────────────────┬──────────────────────────────────────┘
                               ▼
┌─ VERIFIER（沿用传统层的证书）────────────────────────────────────────┐
│  查 A 更新 m^s_surf → 未覆盖面片 → 触发下一轮（paper_06 的迭代加位姿）│
└─────────────────────────────────────────────────────────────────────┘
```

三种辅助输入机制的具体说明：

- **(a) 离散化覆盖状态进 prompt**：有两条实现路径——(i) 重新启用 `discrete_state_input`（本仓库 config 为 False，与现有 LoRA checkpoint 的兼容性需在代码层验证）；(ii) **零架构改动路径（推荐先试）**：客户端逐次构造 prompt 文本（`examples/droid/main.py` 的 request_data 中 `prompt` 由客户端发送，`InjectDefaultPrompt` 只在缺省时注入默认值），由外部 verifier 把 $c\in\{0,1\}^m$ 压缩成低维摘要（覆盖率百分比 + 未覆盖区域的球面方位分箱）后按 256 bins 转成 0~255 整数文本，逐次注入。风险：256 bins 分辨率 = 量程/256（±0.5 m 量程约 3.9 mm/bin，收紧到 paper_04 的 215±30 mm 工作距离量程则约 0.23 mm/bin）；覆盖摘要占用 200 token 预算，须做优先级取舍。
- **(b) paper_08 的渲染 twin 占第 3 相机槽位**：机制同构（缺失相机填零 + `image_mask=False`，被 mask 的 256 token 整列置 `big_neg`）。**但必须清醒**：这只是把 CAD 上下文提供给动作专家（D=1024，经共享注意力读 2048 维前缀），**不能产生异常分数图**——paper_08 的残差机制需要 dense 输出头，Pi0.5 没有。同时 paper_08 的 review 提醒了反向风险：SigLIP 是 224×224，而 AVATAR 用 DINOv2 在 504×504 上取 dense token，**分辨率差 2.25 倍**。
- **(c) 语言指令携带 next viewpoint 描述**：paper_06 的 review 建议把目标表达成 "surface point + normal + distance" 而不是绝对 6-DoF 位姿——让 VLA 只需做"选点 + 朝向"，绝对位姿由几何层解析算出（paper_03 的 coupled point 映射 $p^v_i = p_i + \frac{D_{min}+D_{max}}{2}n_i$），**大幅降低回归难度与误差累积**。

- **Advantage**：保留完整覆盖保证与碰撞安全（都由几何层承担）；VLA 只需学短程运动，训练数据需求小；接口清晰、可增量实施（先做 (c)，再试 (a)/(b)）。
- **Limitation**：VLA 的增量价值需要被证明——如果几何层已经给出目标位姿，传统 IK + 运动规划也能执行，**必须论证 VLA 在执行段确实更好**（候选理由：对位姿扰动/遮挡的视觉反应式修正、免 IK 建模、对多物体场景的泛化）。此外 15 Hz + 开环 chunk（Pi_franka.md 5.1.6）+ config 不吃 proprioception 三项叠加，会让"伺服到目标位姿"退化成纯视觉反应式行为，闭环精度无保障。
- **Difficulty**：**低–中**。几何 planner 用现成工具（Phase 1 就要实现简化版）；主要工作在 prompt 工程与状态通道打通；不需要新数据采集（planner rollout 即可）。

### Option C：Coverage-aware VLA

**核心主张**：把 Visibility Matrix / Coverage Score 当作**监督信号**而不是 prompt 输入，让几何知识蒸馏进 reactive policy 本身。

```
┌─ (a) RL 微调路线 ───────────────────────────────────────────────────┐
│  paper_01 的 MDP：s = [p^s, m^s_surf]，r 来自 Δcov(s,a)              │
│  reward = w_1·Δcov(s,a) − w_2·T_travel   （含 home pose 往返，p05）  │
│  → 对 LoRA 旁路做 policy gradient / 提示微调                          │
└─────────────────────────────────────────────────────────────────────┘
┌─ (b) 行为克隆 + 覆盖向量条件化路线 ─────────────────────────────────┐
│  planner rollout → {(o_t, c_t ∈ {0,1}^m, a_t)} 数据集                │
│  c_t 经压缩后做条件进 prompt（依赖重开 discrete_state_input）         │
│  → 异构 LoRA 微调；消融"有/无 c_t 条件化"的贡献                       │
└─────────────────────────────────────────────────────────────────────┘
┌─ (c) 辅助回归头路线（困难）─────────────────────────────────────────┐
│  目标：让 policy 预测 "当前视角下尚未覆盖的区域" / "可见面片集合"      │
│  障碍：Pi0.5 唯一的输出头是 action_out_proj: 1024→32，loss 只有      │
│        流匹配 MSE；要加辅助头必须改架构（新增 dense 头 + 新损失项）    │
│  → RA 阶段判定：**不做**，仅作为论文中的讨论项                        │
└─────────────────────────────────────────────────────────────────────┘
```

- **Advantage**：这是唯一可能让 VLA 自身具备"覆盖意识"的路线；paper_01 已证明"Δcov 作分子、travel cost 作分母"的奖励结构有效（平均 cycle time 降 24.0%，最高 43.5%），**reward 无需重新设计**；paper_02/03/05/07/09 的 review 都把 $\Delta cov$ / $A$ / $C_{rate}$ 列为天然的监督源或奖励项。
- **Limitation**：(a) 需要 RL 基础设施（本项目只有监督训练栈，**RL 代码不存在，需进一步验证工程可行性**）；(b) 的覆盖向量条件化受 200 token 与 256 bins 约束，$c\in\{0,1\}^m$ 必须强压缩，信息损失是否可接受**需实验验证**；(c) 明确超出 RA 范围。此外 paper_09 给出的一条反例必须正视：即便把覆盖率优化到最高，也**不构成"好计划"的判据**——纯覆盖率无法表达专家隐式偏好。
- **Difficulty**：**高**。核心难点是"几何监督如何蒸馏进 reactive policy"——Pi0.5 的 loss 只有 MSE，没有语义/覆盖项，任何覆盖监督都必须走 RL 或 prompt-conditioned 微调（paper_07 的 review 原话）。

### 4.1 三方案对比

| 维度 | Option A 纯端到端 | Option B 规划引导 | Option C 覆盖感知 |
|---|---|---|---|
| **保证性** | 无（覆盖/碰撞全丢） | **高**（覆盖与碰撞都由几何层保留） | 中（覆盖成为软目标，有 verifier 兜底时提升） |
| **泛化性** | 高（隐式泛化来自预训练 + 数据分布） | 中（VLA 只泛化执行段，规划段仍依赖 CAD） | 中高（覆盖条件化理论上可跨零件迁移） |
| **数据需求** | 大（需覆盖多样初始位姿的长时域轨迹） | **小**（planner rollout + 少量示教） | 中（rollout 数据 + 覆盖标注） |
| **工程难度** | 中（无架构改动，但要数据与评测） | **低–中**（prompt + 状态通道 + 约束投影层） | 高（RL 栈不存在 / 需强压缩 / 辅助头要改架构） |
| **RA 可行性** | 中（数据是瓶颈） | **高**（Phase 1 的 planner 直接复用） | 低（建议只做 (b) 的消融部分） |
| **对 9 篇文献资产的利用率** | 低（只用了 teacher） | **高**（planner + A + 约束 + twin 全用上） | 中高（A 与奖励结构全用上） |
| **论文主张强度** | 中（"VLA 能学会检测"是一个弱结论） | **强**（"分层分工 + 位姿扰动下的对比"是清晰贡献） | 中（消融发现本身有价值） |

### 4.2 交叉使能者：Real-to-Twin 数据引擎（paper_08）

paper_08 的 Real-to-Twin 流水线**同时支撑 A 与 C 的数据需求**，且是九篇里唯一"可规模化、免标注"的数据生成链路：位姿估计（模板匹配 + 可微渲染精化 $T^{base}_{obj} = T^{base}_{ee}T^{ee}_{cam}T^{cam}_{obj}$）→ 等面积多面体视点采样（仰角 20°–80° 过滤）→ 机器人执行 → 渲染引擎同步产出 → **严格位姿对齐的成对图像**。它能同时产出 (i) 判异模块用的对齐对，(ii) **VLA 微调所需的"多视角工业场景 + 位姿真值"轨迹数据**。

需要注意两个限界：(i) 训练集只有 216 对、测试 214 对、**2 个类别**（flange-bearing seats 与 J6 关节系统），论文自认需"extend to broader objects, materials, defects"；(ii) 渲染 mask $W$ 若要随 twin 图一起进 Pi0.5，应转成 token 级掩码（与 `image_mask` 同构），而不是让模型自己学。

---

## 5. Key Technical Challenges

### 5.1 Data

**工业检测 trajectory 数据如何获得？四条路线：**

| 路线 | 具体做法 | 单条轨迹成本 | 覆盖多样性 | 风险 |
|---|---|---|---|---|
| ① 专家示教 | 用 paper_09 的交互式 3D 界面精确记录专家视点（该论文是唯一能精确记录专家视点的工具，得到 44/44、21/13、34/32 个视点的计划） | 高（人工）；且 paper_09 **未测量**专家耗时 | 低（2 名专家 × 2 份/物体） | 专家计划有隐式模式（fixed camera + turntable 复制），直接模仿会把硬件偏好也学进去 |
| ② planner rollout 蒸馏 | paper_01 的 FTS（平均降 24.0%）或 paper_05/07 的 RKGA 对多工件反复求解，产出 (观测, 覆盖状态, 视点序列) | 低（纯计算，但 paper_05 的 400 samples 约 3 小时） | 高（可任意换初始位姿与工件） | 教师本身是次优的（贪心 or 启发式）；分布外行为无法覆盖 |
| ③ Real-to-Twin 合成 | paper_08 流水线自动生成位姿严格对齐的配对数据 | 低（自动化） | 高（等面积多面体视点可任意加密） | 渲染与实拍的 Sim2Real 域差；paper_08 只验证了 2 个类别 |
| ④ 真机采集转 LeRobot | 用 `examples/droid/main.py` 的录像 + 人工评分 CSV 采轨迹，再用 `convert_*_to_lerobot.py` 转 LeRobot 格式（微调第一步） | 中（真机时间） | 中 | 采集端图像在机器人侧已 resize 到 224 再上传（省带宽），**分辨率损失不可逆** |

**sim-to-real gap**：本项目已有 LIBERO Docker 仿真评测（EGL headless，`MUJOCO_GL=egl`），但 LIBERO 是桌面 manipulation 基准，**没有工业零件与相机视点几何**。若要 sim 出检测任务，需自建仿真环境——工作量**需进一步验证**。

**缺陷样本稀少**：paper_08 的启示是：**不要学"缺陷长什么样"，而是学"与设计蓝图的不可对齐偏差"**（异常分数量只用 216 对正常配对训练）。这条路线绕开了缺陷标注与类别枚举两个死结，但要求严格位姿对齐——而 Pi0.5 的采样是随机的、轨迹不可复现，与"严格对齐"天然冲突（paper_08 的 review 明确指出"一个不可复现的执行器会给对齐质量带来新的不确定性"）。

### 5.2 Representation

**Geometry representation 如何转 token？选项对比：**

| 方案 | 实现路径 | Token 成本 | 表达力 | 风险 |
|---|---|---|---|---|
| (i) 覆盖状态 → 离散化文本 | $c\in\{0,1\}^m$ 压缩为低维摘要 → 256 bins → 0~255 整数 → prompt | ≤200 token 预算内 | 低维、有损 | 需**重开 `discrete_state_input`**；paper_07 量化：±0.5 m 量程下 256 bins ≈ **3.9 mm** 分辨率 |
| (ii) 渲染 twin 占第 3 相机槽 | paper_08 的 Blender 渲染图 → `right_wrist_0_rgb` 槽位 | **256 token**（槽位本已分配，等于零边际成本） | 高（完整几何上下文） | 破坏预训练输入分布，需 LoRA 适配；SigLIP 224×224 vs AVATAR 504×504（差 2.25 倍） |
| (iii) depth 附加通道 | 需改 SigLIP 编码器输入通道（3→4）或另开一个编码分支 | 新增 | 高 | **架构改动**，超出 RA 范围；SigLIP 是冻结的预训练权重，改输入通道会破坏其初始化 |
| (iv) viewpoint 作为语言目标 | 序列化 `"next: pivot(p)=[x,y,z], n=[..], d=140mm"` | 数十 token | 中 | 模型能否把文本坐标与视觉对齐**需验证**；paper_06 的 review 建议用 "surface point + normal + distance" 表示以降低回归难度 |
| (v) viewpoint 作为连续 EE 目标 | 直接回归 6-DoF delta | 0 token | 高 | 需绝对位姿显式监督；delta 多步积分累积漂移，且本 config 无 proprioception 无法校正 |

**Token 预算约束**：前缀总长 968（768 图像 + 200 文本）。图像侧 3 个槽位固定分配（每槽 256），**无法压缩**；文本侧 200 token 要同时容纳任务指令、ROI 描述、覆盖状态、可能的目标视点。paper_06 的 review 给出的样例 prompt 已接近这个预算形态：`"Task: GD&T inspection of datum A (priority planar face), cover fully, tilt 30-50 deg, keep adjacent face in view;\nAction: "`。**结论：必须做优先级排序，200 token 装不下全部几何信息。**

### 5.3 Action

- **精度极限**：Pi0.5 的动作是**相对量**（EE delta pose 或关节速度），没有绝对位姿的显式监督，长时间执行会累积漂移（paper_04 的 review 指出工业采集要求 mm 与角度级可验证精度，其基准是 PR3Net 的 APE 2.15 px / AAE 3.01°）。而工业成像约束是硬的：paper_03 的 $D_{min}\le p'_{iz}\le D_{max}$（实验取 DOF 0.05，在包围盒长边归一化到 1 的尺度下）、$p'_{ix}\le W, p'_{iy}\le H$（FOV 取 (0.2, 0.3)）、观测角 $n_i\cdot n^v_i < \tau_n$（$\tau_n=-0.3$）；paper_04 的 CCD32-1 是工作距离 215 mm ± 30 mm、景深 100 mm × 80 mm。**这些要求 cm 甚至 mm 级的位姿精度。**
- **开环 chunk 无闭环修正**：Pi_franka.md 5.1.6 明确记录：`ActionChunkBroker` 是纯开环，10 步 chunk 全部执行完才重新推理。paper_06 的 review 指出这与工业需求直接冲突——"在固定位姿采集数据"的位姿本身要求闭环，而模型甚至无法感知自身是否到位（因为 `discrete_state_input=False`）。
- **约束投影层思路**：paper_03 的三条成像判据（Eq.2/3/5）应作为**双重使用**的硬约束层：(a) 视点合法性过滤器（生成候选时）；(b) Pi0.5 输出动作的约束检查/投影（执行时）。其中 **Eq.5 的 worst-case 判据 $2\sqrt{p'^2_{ix}+p'^2_{iy}} < \min\{W,H\}$** 尤其值得复用——它把"绕视轴旋转不确定"这个自由度显式做了保守处理。paper_02 的 $E_{IA}$（最大入射角泛函）同样可构造约束投影/拒绝判据。
- **up-vector 缺失**：paper_06 与 paper_09 的 viewpoint 都是 **9 维 $v=[p,d,u]$**，而 Pi0.5 单步真实动作仅 **8 维**（其中 6 维可承载 SE(3)）——**一步表达不了完整 viewpoint**。更麻烦的是 $u$（绕光轴的 roll）在 Pi0.5 的训练数据里**没有对应语义**，而工业检测对它有明确要求（图像方向一致性、转台复制旋转、下游 2D 边缘检测算法）。这是一个需要额外标定/监督的语义缺口。
- **缓解方向（来自 paper_09 的观察）**：专家的计划天生是"**两个固定相机位置 + 转台旋转复制**"的形式。把控制问题从"自由 6-DoF 视点放置"降维成**离散运动基元选择**（哪个固定位、转台转多少度），既降低了动作空间的表达力要求（"rotate turntable by 45°" 是 prompt + action 都能自然表达的形式），也降低了碰撞风险与硬件成本。

### 5.4 Evaluation

从 9 篇文献汇总的指标，按"sim 可测 / 真机需测"区分：

| 指标 | 定义 | 来源 | sim 可测 | 真机需测 |
|---|---|---|---|---|
| **Coverage rate** | $C_{rate} = \|\cup C(v_i)\|/N$；或已采集面积/总面积 | p03, p04, p06 | ✅（需渲染验证 ground truth） | ✅ |
| **清晰覆盖率** | $AC_{rate} = A_c/A_{total}$（关键表面面积比） | p04 | ✅ | ✅ |
| **视点数** | $V_{num}$ / $n_{opt}$ / $N_{vp}$ | p02, p03, p04, p05, p09 | ✅ | ✅ |
| **Cycle time** | $T_{avg}$（含 inspection + traveling），10 trials | p01, p05, p07 | ✅（但 paper_07 提醒：$e(p_i,p_j)$ 来自 RRT + spline 的时间估计，未考虑加速度约束与控制器插补） | ✅ |
| **Path length** | $P_{leng}$ | p03, p09 | ✅ | ✅ |
| **规划求解时间** | $P_{time}$ / $E_{time}$ | p03, p04 | ✅ | ✅ |
| **方向一致性** | $N_{vc} = \frac{1}{N}\sum\max(-n_i\cdot n^v)$ | p03 | ✅ | — |
| **不可接受视点率** | discarded viewpoints %（R3） | p09 | ✅ | — |
| **位姿精度** | APE / AAE | p04 | ✅（真值已知） | ✅（需外部标定） |
| **R1–R4** | 全覆盖 / 自适应冗余 / 最小化不可接受视点 / 冗余可缩放 | p09 | ✅（作为验收准则） | — |
| **P-AP / P-F1 / I-AUROC** | 像素级与图像级异常检测 | p08 | ✅ | ✅ |
| **重复覆盖率** | $RepeatC_{rate}$（冗余视点比例） | p04（实测均 < 50%） | ✅ | ✅ |
| **对齐成功率** | alignable views % / SR | p04, p06 | — | ✅ |

**sim 中 coverage ground truth 怎么算**：可行做法是**渲染验证**——在仿真中对每个视点渲染深度缓冲，判定每个面片是否落在 FOV ∩ 视距 ∩ 视角内且无遮挡（这正是 paper_06 的 VM 与 paper_02/05/07 的 ray tracing，paper_09 实测每个视点平均 **3.81 s**）。注意这个开销在设计实验规模时必须预先估算。

**与专家计划对比（paper_09 范式）**：应报告"视点数—覆盖率"的**收敛曲线包络**而不是单点覆盖率。paper_09 的判据是："Ideally, all the runs should be coherent and have similar convergence, with no outliers causing the envelope to spread"——**包络宽度衡量稳定性**，而稳定性比单点最优更重要。对比基线可直接采用 paper_09 的 292 个候选集/公开数据。

### 5.5 Safety

- **Franka 实机安全**：Franka 7-DoF 自带力矩限制，但检测场景是"相机贴近零件表面（paper_04 的 215 mm ± 30 mm 工作距离量级）做倾斜观察"，碰撞风险高。必须配置：关节力矩/速度限制、虚拟墙（workspace 包围盒）、速度缩放（speed scaling）、以及**人类监督急停**。
- **VLA 无碰撞保证 vs 传统 planner**：paper_05/07 的 RRT-Connect + FCL 提供的是**概率完备**的碰撞安全；Pi0.5 的动作头是无约束的连续输出，**没有碰撞模型、没有关节限位、没有奇异位形避让**。"安全"只来自训练分布的统计偏好，不构成保证。九篇文献的 review 在这方面结论完全一致：**碰撞安全不能交给 VLA**。
- **约束投影层 + 动作掩码**：
  - paper_03 的三条成像硬约束 → 动作投影/拒绝判据；
  - paper_09 的 R3（最小化不可接受视点）→ 做成**动作空间的硬掩码或即时否决**（该文的实测是 Space Sampling 61.87% 候选不可接受，直接浪费算力）；
  - paper_07 的 cluster 思想 → 对同一指令采样 $K$ 个 chunk，用几何可行性（碰撞/可达/视角）筛选后执行（等价于把 GTSP 的"从 cluster 选一"搬到 VLA 采样阶段）。
- **sim-first 验证流程**：所有策略先在仿真中跑通（覆盖率达标 + 无碰撞），再上真机；真机首测使用低速 + 小工作空间 + 人工监督。

---

## 6. RA Feasible Research Plan（3–6 个月）

### 总体目标（一句话）

**在仿真中把"传统几何检测规划器"与"LoRA 微调后的 Pi0.5"做成分层对照系统，量化回答"VLA 在工业检测管线中能替代哪一段、以及在被检测对象位姿扰动时 VLA 相对几何重规划的优势与代价"。**

**项目已有资产**：Franka 7-DoF 实机 + 外置相机 + 腕部相机；openpi 训练栈（`scripts/train.py`，JAX + FSDP + EMA 0.99）；大量 `pi05_lora_finetune_task*_ep*` config（异构 LoRA rank16/32）；LIBERO Docker 仿真评测（EGL headless）；LeRobot/RLDS 双数据链路 + `convert_*_to_lerobot.py`；`examples/droid/main.py` 的 15 Hz 实机控制循环。

---

### Phase 1（第 1 个月）：Traditional Baseline

**目标**：在仿真中搭出一条简化但自洽的几何检测管线，建立评测 harness，并让它产出可供模仿学习的演示数据。

**任务清单**
1. 选 1–3 个有 CAD 的机械零件（可用 paper_02 的 stretched cylinder / spring 类几何，或 paper_07 的 Target Object 1/2 尺寸 0.24×0.47×0.30 m 与 0.20×0.30×0.30 m 量级），导入为三角网格。
2. 实现简化几何检测管线（参考 paper_02/06/07 的简化版）：
   - 表面离散化（均匀重采样，规模控制在 500–2000 面片，对应 paper_01 的 2252 面片量级）；
   - 候选视点生成（在包围球/椭球半球内随机采样 + 视线指向工件中心 + 高斯扰动，paper_01 的 Eq.2 思路；规模 200–500，对应 paper_05/07 的 200/300/400 消融）；
   - 可见性矩阵 $A\in\{0,1\}^{n\times m}$（四条判据：FOV ∧ 视距 ∧ 视角 ∧ 无遮挡）；
   - 贪心 set cover / NBV（paper_02 的 Algorithm 1 或者 paper_07 的 Greedy-SCP）；
   - TSP 排序（可先用贪心，参考 paper_03 的边权 $l_{ik}=\|p^v_i-p^v_k\|_2(1-\mathrm{sim}(e_i,e_k))$）。
3. 建立评测 harness：**coverage %、视点数、cycle time**（含常数检测时间 1.0 s，沿用 paper_01/05 的设定）、不可接受视点率；coverage 用渲染验证（深度缓冲）计算 ground truth。
4. 让该管线在不同工件位姿下 rollout，产出 (观测图像, 覆盖状态 $c_t$, 视点序列/动作) 演示数据。

**产出**：(i) 可运行管线代码 + 配置；(ii) 演示数据集（含位姿扰动版本）；(iii) 评测 harness 与 baseline 数值表。

**依赖与风险**：仿真环境采用**渲染优先、物理从简**的方案——可见性矩阵与覆盖真值只需 ray casting + 深度渲染（paper_08 已证明 Blender 的 Python API 可承担此角色，无需物理引擎）；机器人运动只影响 cycle time，可用 travel-cost 代理（paper_03 的边权 $l_{ik}=\|p^v_i-p^v_k\|_2(1-\mathrm{sim}(e_i,e_k))$）估算，**Phase 1 不需要 MuJoCo 手臂仿真**（真机闭环验证推迟到 Phase 3 可选任务）。风险：ray tracing 构建 $A$ 的耗时（paper_09 实测 3.81 s/视点，paper_02 的 tangle cube 要 1771.99–3103.06 s），**必须在候选数与面片数上做取舍**。

**验收标准**：管线在 3 个工件上稳定达到 ≥ 95% 覆盖（对应 paper_03 把 $C_{rate}>95\%$ 定义为成功规划），且能对单个工件输出完整的（观测、覆盖向量、动作）轨迹 ≥ 50 条。

---

### Phase 2（第 2–3 个月）：Small-scale Pi0.5 Adaptation

**目标**：用现有 LoRA config 做小规模微调，验证"Pi0.5 能否学会执行视点间运动"，并对比不同条件化方式。

**任务清单**
1. 数据：Phase 1 的 planner rollout 为主（+ 少量人工示教作为分布补充）；经 `convert_*_to_lerobot.py` 转 LeRobot 格式。
2. 训练：用 `pi05_lora_finetune_*` config 做异构 LoRA（PaliGemma rank 16 / 动作专家 rank 32，冻结主干），`scripts/train.py`（JAX + FSDP + EMA 0.99）。
3. 实验 prompt 变体：
   - **V0**：纯任务指令 `"Task: inspect the part fully;\nAction: "`；
   - **V1**：任务指令 + 离散覆盖状态（两条路径二选一：**客户端构造 prompt 文本注入【零改动，推荐先试】**，或重新启用 `discrete_state_input`【需验证与现有 LoRA checkpoint 的兼容性】；$c_t$ 压缩成覆盖率 + 未覆盖方位分箱 → 256 bins → 0~255 文本）；
   - **V2**（可选）：第 3 相机槽位填渲染 twin（paper_08 路线，需确认渲染管线可得性）。
4. 全部训练在仿真数据上进行；推理侧接 `Policy.infer` → chunk 执行闭环（可先离线回放验证，再进仿真闭环）。
5. 消融记录：各变体的训练 loss 曲线、动作误差、以及仿真闭环中的覆盖率与视点数。

**产出**：(i) 微调后的 checkpoint 与训练配置；(ii) 变体对比表；(iii) 失败模式清单（哪些位姿/哪类工件学不会）。

**依赖与风险**：依赖 GPU 与 openpi 训练栈。风险：(i) `discrete_state_input=True` 与现有 config 的兼容性**需进一步验证**（需确认 `pi05=True` 时 suffix 是否也会加状态 token）；**若 V1 走客户端 prompt 注入路径则无此风险**；(ii) LoRA 低秩旁路的容量是否足以吸收覆盖条件化信号**未知**；(iii) 渲染 twin 的获取成本。

**验收标准**：仿真中 V0/V1 至少一个变体在 3 个测试工件上达到 **baseline 覆盖率的 90%**（即若 baseline 为 96%，则 ≥ 86.4%），且**推理过程中不需要重规划**（模型能从任意初始位姿自主走完全程）。

---

### Phase 3（第 4–6 个月）：Comparison Experiment

**目标**：完成主对照实验，产出可投稿的结果。

**任务清单**
1. 仿真对比实验矩阵：

| 行 | 方法 | 说明 |
|---|---|---|
| R1 | Geometric baseline | Phase 1 的简化管线（全量重规划） |
| R2 | Option A（纯端到端 VLA） | 无几何层 |
| R3 | Option B-a（覆盖状态进 prompt） | V1 变体 |
| R4 | Option B-b/c（twin 占槽位 / 语言目标） | V2 变体（若可得） |
| R5 | Option C-b（覆盖向量条件化的行为克隆） | 与 R3 的差别在于监督信号组织方式 |

2. 指标：coverage %、视点数、cycle time、**位姿扰动鲁棒性**——把工件位姿在训练分布内/外各偏移若干档（参考 paper_01 的 15 mm in x,y + 5° z-axis 量级），观察 geometric baseline 是否必须重规划、VLA 是否零样本适应。
3. 消融：覆盖状态条件化（V1 vs V0）的贡献；第 3 相机槽位（V2 vs V1）的贡献；LoRA rank 的敏感性（rank16/32 vs 更大）。
4. 可选真机验证：选 1 个零件，用 Franka + 2 相机做低速 demo（先离线回放，再真机执行），验证 sim 结论是否迁移。
5. 撰写论文。

**产出**：(i) 主结果表 + 收敛曲线包络（paper_09 范式）；(ii) 消融表；(iii) 论文初稿；(iv) （可选）真机 demo 视频。

**依赖与风险**：依赖 Phase 1/2 的产物。主要风险：(i) VLA 在仿真中的表现可能**显著弱于** geometric baseline，导致"无正结果"——此时应把论文定位改为**否定性/边界性结论**（"VLA 适合执行段而非规划段"），这本身对社区有价值；(ii) 位姿扰动的幅度设定不当会同时打击双方。

**验收标准**：完成 ≥ 2 个工件 × 5 种方法 × ≥ 10 次重复在 ≥ 3 档位姿扰动下的对比，所有指标带均值和标准差；R2 或 R3 在**至少一档扰动**下 coverage 或 cycle time 优于 R1。

---

## 7. Final Research Proposal Direction

### 7.1 核心研究问题

> **RQ：在工业表面检测任务中，把几何规划器（coupled-point 目标表示 + 可见性矩阵）作为外层、把 LoRA 微调后的 Pi0.5 作为内层执行器，能否在保留覆盖率证书的同时，获得对工件位姿扰动的鲁棒性——即"零样本免重规划"——以及这一收益的代价是什么？**

### 7.2 子问题分解

- **SQ1（可执行性）**：Pi0.5 的 8 维动作能否在工业成像约束（paper_03 的 FOV/景深/观测角三条硬判据）下把相机伺服到目标视点？在外层约束投影层的保护下，动作级可行性如何量化？
- **SQ2（条件化机制的贡献）**：把覆盖状态经 256 bins 离散化注入 prompt（重开 `discrete_state_input`），相比纯语言指令，能提升多少覆盖率/减少多少冗余视点？
- **SQ3（鲁棒性对比）**：工件位姿扰动时，几何 baseline 需要重规划（额外数小时，参考 paper_05 的约 3 小时；或 paper_01 的在线 FTS），VLA 能否零样本适应？在多大扰动幅度内成立？
- **SQ4（代价侧）**：VLA 带来的代价是什么？——节拍不可复现（采样随机）、无碰撞保证、无覆盖率自证、up-vector 语义缺口。

### 7.3 可验证性

| 子问题 | 核心实验 | 判定标准 |
|---|---|---|
| SQ1 | 在仿真中让 VLA 从随机初始位姿执行到目标视点，用外部几何模块测量终点位姿误差 | 终点位姿误差 < paper_03 的 FOV/景深约束容许范围（在归一化尺度下 FOV (0.2, 0.3)、DOF 0.05） |
| SQ2 | V1（覆盖状态条件化）vs V0（纯指令）的受控消融，同数据同超参 | V1 的覆盖率显著高于 V0，或冗余视点数（$RepeatC_{rate}$ 类指标）显著更低 |
| SQ3 | 工件位姿在 ≥ 3 档偏移下对比 R1–R5；baseline 按需重规划 | VLA 变体在偏移 ≥ 训练分布边缘时 coverage 下降幅度显著小于 baseline 需重规划的代价 |
| SQ4 | 报告 cycle time 的均值 + 标准差；报告动作可复现性（固定噪声注入前后差异，`Policy.infer` 支持 `noise` 参数用于确定性评测） | 定性结论 + 量化 $T_{std}$ |

### 7.4 可实验性（依赖现有资产即可完成）

- **硬件**：Franka 7-DoF + 外置/腕部相机（真机 demo 用，非必需）；
- **训练**：openpi `scripts/train.py` + 现成 `pi05_lora_finetune_*` config（异构 LoRA rank16/32，冻结主干）；
- **数据**：Phase 1 的 planner rollout（自产）+ `convert_*_to_lerobot.py`；
- **评测**：LIBERO Docker（EGL headless）可复用容器化评测范式；
- **无需**：新架构、大规模数据采集、理论保证证明。

### 7.5 可发表性与预期贡献

**贡献候选（按强度排序）**：

1. **planner → VLA 蒸馏的工业检测数据集与评测协议**：把 paper_01/02/05/07 的几何规划器当作 teacher，生成带覆盖真值标注的视点轨迹数据，并给出以 paper_09 的 R1–R4 与收敛曲线包络为基础的评测协议。这是本领域**当前不存在**的公开资产。
2. **覆盖状态条件化的消融发现**：验证"把 $c\in\{0,1\}^m$ 的压缩摘要经 256 bins 注入 prompt"是否有效，并量化 256 bins 的精度上限（paper_07 的 review 给出 ±0.5 m 量程下约 3.9 mm/bin 的估计；分辨率随归一化区间线性缩放，检测场景量程下可达亚毫米级）。
3. **位姿扰动下 VLA vs 几何重规划的比较研究**：把 paper_01 的核心动机（pose variation 使离线 CAD 方法失效）搬到一个学习式系统上重新检验，回答"学习式策略的鲁棒性是否真的优于在线重规划，代价是什么"。
4. **分层分工的工程结论**：明确"几何层负责保证，VLA 层负责执行"的接口规范（coupled point 目标表示、约束投影层、mask 机制），以及开环 chunk 在检测任务上的适用边界。

**投稿方向**：ICRA/IROS workshop（如工业检测、VLA 应用方向）或 RA 阶段友好的期刊短文（方案已在 Phase 3 给出完整实验矩阵）。以"数据集 + 消融 + 对比研究"三个贡献定位，不追求 SOTA 数值。

### 7.6 明确不做的事（避免博士级课题）

- **不做新架构**：不改 SigLIP 输入通道、不加 dense 输出头、不实现 Option C 的辅助回归头、不做 RL 基础设施（Option C 的 (a) 路线不在 RA 范围内）；
- **不做大规模数据采集**：不追求实验室级数据集规模；数据以 planner rollout 为主，真机只做 1 个零件的 demo 验证；
- **不做理论保证证明**：不尝试证明覆盖率下界或收敛性；覆盖保证由几何层承担并直接复用 paper_02/05/07 的既有结论；
- **不做多传感器/多物体/强反光场景**：仅验证单零件、静态摆放、无遮挡的受控设定，其余列为 future work。

### 7.7 方向总结

本方案主张**不在"VLA 能否替代几何规划"这一非此即彼的问题上下注**，而是把九篇文献共同暴露的空白——"从视点序列到机器人可执行运动之间没有任何一层"（paper_03/06/07/09 的 review 四方一致）——作为切入点：保留几何层作为保证提供者与目标生成者，让 Pi0.5 只承担随位姿扰动而变化的执行段。可行性来自现有资产（Franka + openpi LoRA 栈 + LeRobot 工具链）与现成的几何算法（可见性矩阵 + set cover + 约束投影），而论文的增量在于回答一个当前无人验过的问题：**当工件位姿偏离名义值时，一个从几何规划器蒸馏出来的反应式策略，能否在覆盖率上追平需要数小时重规划的传统管线**。相应地，本方案对 Pi0.5 的真实短板保持坦诚——无几何输入、无覆盖表示、无保证能力、无法复现的随机采样、up-vector 语义缺失——并把它们要么交给外部几何层，要么显式列为方案的能力边界。

---

*引用索引：[notes/README.md](notes/README.md) · [paper_01_review.md](notes/paper_01_review.md) · [paper_02_review.md](notes/paper_02_review.md) · [paper_03_review.md](notes/paper_03_review.md) · [paper_04_review.md](notes/paper_04_review.md) · [paper_05_review.md](notes/paper_05_review.md) · [paper_06_review.md](notes/paper_06_review.md) · [paper_07_review.md](notes/paper_07_review.md) · [paper_08_review.md](notes/paper_08_review.md) · [paper_09_review.md](notes/paper_09_review.md) · [pi05.md](pi05/pi05.md) · [Pi_franka.md](pi05/Pi_franka.md)*


0. Option B 的一句话手术方案
M1–M4（几何层）原封不动，M5 从中间劈成两半：全局排序留给几何层，局部运动交给 Pi0.5；M6 换执行器；M7 从流水线末端挪到环上；另加两个新黑盒（翻译层 + 约束投影层）粘合两边。

1. M1–M7 逐模块变化（黑盒 I/O 对照）
M1 表面离散化：不改

输入:  W 的 CAD 网格 → 输出: 1000 个面片（带中心+法向）
例:    s_347 = (中心 (87.3, 45.1, 60.0) mm，法向 (0.12, 0.85, 0.51))
变化:  无。几何层照旧，Pi0.5 永远看不到这个模块——它不需要看到
M2 视点候选生成：不改

输入:  面片集合 + 传感器参数 → 输出: 500 个候选视点
例:    v_237 = (位置 (180, -120, 240) mm，方向 (-0.6, 0.4, -0.7))
变化:  无。但注意它产出的东西第一次有了"第二消费者"：以前只喂给 M3，
       现在还会被翻译层转成 Pi0.5 的导航目标
M3 可见性判定：不改，但多了一条读出路

输入:  500 视点 × 1000 面片 → 输出: A ∈ {0,1}^{500×1000}
例:    A[237][347] = 1（v_237 能看到面片 347）
变化:  矩阵本身一字不改。改的是"谁在查这张表"：
       以前只有 M4 查（优化器内部用）；
       现在 M7 验证时也要查（判断"还有哪些面片没被看到"），
       并把查询结果经翻译层转成 Pi0.5 的输入——A 成了几何层与 VLA 之间的"契约"
M4 覆盖优化：不改，但身份从"终点站"降为"目标生成器"

输入:  A + 覆盖率要求 96% → 输出: 被选视点序列 {v_87 → v_3 → v_237 → ...}
例:    12 个视点合并覆盖 960 个面片 → 96.0%
变化:   传统方案里 M4 的输出是"最终答案"（一次算完，离线固化）；
       Option B 里它变成"导航目标序列"——Pi0.5 每个时刻只关心序列里的
       下一个目标，而不是整条序列。
       连带变化：它从"离线一次性"变成"准在线"——M7 发现位姿扰动导致某视点
       失效时，可以只局部补算（重查 A 的几列），不用全量重跑
M5 排序/运动规划：被劈成两半——这是 Option B 的核心手术
传统 M5 黑盒内部其实装了三件事：全局排序（TSP/GTSP）、机器人位姿求解（IK）、轨迹生成（RRT + spline + 碰撞检查）。Option B 按"全局 vs 局部"把它们拆开：


┌─ M5a 全局排序（留在几何层，不改）──────────────────────┐
│ 输入:  被选视点集（12 个）+ 行走代价矩阵 E             │
│ 输出:  有序序列 + 每个视点的目标位姿                    │
│ 例:    v_87 → v_3 → v_237 → ...（顺序是全局最优的）   │
└──────────────────────────────────────────┬─────────────┘
                                           ▼
┌─ M5b 局部运动"走到下一视点"（Pi0.5 接管）──────────────┐
│ 输入:  RGB 图 ×2 + 语言 prompt（含"下一目标视点"描述）  │
│ 输出:  动作 chunk（10 步 × 8 维关节速度/EE 增量）      │
│ 例:    prompt = "Task: inspect bracket; Next: surface   │
│        point (87.3, 45.1, 60.0), normal (0.12,0.85,0.51),│
│        standoff 140mm;\nAction: "                       │
│        → 输出 10 步动作，机械臂朝该面片正前方走         │
└─────────────────────────────────────────────────────────┘
为什么这样劈？因为 M5a 是全局、离线、可证书的（一次 3 小时没关系），而 M5b 是局部、在线、对位姿扰动敏感的——正是 Pi0.5 反应式策略的长处。传统 M5 里原来的 IK/碰撞检查没有消失，而是变成了下面的新黑盒（约束投影层）。

M6 执行：换执行器

传统:  输入 ROS 轨迹（带时间戳）→ 控制器插补执行 → 每个视点拍摄
Option B: 输入动作 chunk → 15 Hz 开环执行（每 8 步重新请求）→ 到视点拍摄
例:    chunk 第 3 步 = 关节速度 (0.2, -0.1, 0, 0.15, 0, 0, 0, 夹爪 0.9)
变化:   轨迹不再由几何层预先算死，而是"边走边生成"
       （代价：开环 chunk 无闭环修正——见下方约束投影层）
M7 验证：从"流水线末端"变成"环上的一站"

传统:  图像集 ──▶ M7 ──▶ 覆盖率报告（流水线结束，报告给人看）
Option B: 图像 ──▶ M7 ──▶ (1) 更新覆盖向量 m^s_surf（查 A 的列）
                       └─▶ (2) 发现有面片没看到/视点失效 → 反馈给 M4 局部补算
                       └─▶ (3) 把覆盖摘要交给翻译层 → 写进 Pi0.5 的下一次 prompt
例:    拍完 v_87 后查表发现面片 803（窄缝内部）仍不可见 → 覆盖 96% 不达标
       → 不终止！而是把 "Coverage: 730/1000, uncovered: 窄缝区域" 写进下一次
       prompt，Pi0.5 带着"还有窄缝没看"的意识继续走
这是 Option B 相对传统管线最本质的一处改动：M7 的输出去向变了——从"给人看的报告"变成"给机器看的下一轮输入"。

2. 新增的两个黑盒（M 骨架里原本没有）
新模块①：翻译层（几何量 → token）
几何层与 Pi0.5 的接口只有三种形式，翻译层就是干这个的：


输入:  几何层的任意中间量
输出:  Pi0.5 能消费的三种 token 形式之一
例1:  覆盖向量 c（1000 维 0/1）→ 压缩成摘要 → 256 bins 离散 → "State: 187 45 ..."
      含义: "73% 已覆盖，未覆盖集中在右后上方"变成一串 0~255 整数进 prompt
例2:  CAD 渲染 twin 图 → 填进第 3 相机槽位（Pi0.5 预留 3 路，本仓库第 3 路空闲）
      含义: "零件应该长这样"以图像形式给 Pi0.5 看
例3:  下一目标视点 → 语言描述 "Next: surface point (...), normal (...)"
      含义: 只描述"去哪"（coupled point），不指定绝对 6-DoF 位姿——
            绝对位姿由几何层算好，Pi0.5 只负责走过去
新模块②：约束投影层（安全护栏，包在 Pi0.5 输出外面）

输入:  Pi0.5 的每个动作 chunk
输出:  通过/拒绝/投影后的动作
例:    chunk 第 5 步预测相机视距 172 mm < 下限 185 mm（paper_04 的 215±30 mm）
       → 拒绝该步或把该维 clamp 回来 → 重新请求
另外:  越界动作走 paper_03 的三条成像判据（FOV/景深/观测角）+ FCL 碰撞兜底
含义:  几何层丢掉的那部分保证（原来由 RRT+碰撞检查承担）在这里补回来——
       这就是"碰撞安全不交给 VLA"在架构上的落点
3. 连成完整流程（Option B 黑盒总图）

        ┌────────────── 几何层（离线/准在线，保证的提供者）──────────────┐
        │ M1 离散化 → M2 候选生成 → M3 可见性矩阵 A → M4 覆盖优化        │
        │   → M5a 全局排序 → 目标视点序列 X = (x_1, x_2, ...)            │
        └───────┬──────────────────────────────┬────────────────────────┘
                │ 目标视点                      │ 覆盖向量 c / A 查询结果
                ▼                              ▼
        ┌──────────── 翻译层①（几何量 → token）─────────────────────────┐
        │  (a) c 压缩 → 256 bins 文本   (b) 渲染 twin → 第 3 相机槽      │
        │  (c) 目标视点 → "Next: surface point..." 语言描述              │
        └───────┬────────────────────────────────────────────────────────┘
                ▼  prompt（≤200 token）
        ┌──────────── Pi0.5（M5b 局部运动 + M6 执行）─────────────────────┐
        │  2 路 RGB + prompt → SigLIP → 双专家 → 流匹配 → chunk(10,8)    │
        │  → 15 Hz 开环执行                                              │
        └───────┬────────────────────────────────────────────────────────┘
                ▼  每个 chunk
        ┌──────────── 约束投影层②（护栏）────────────────────────────────┐
        │  成像约束（FOV/景深/观测角）检查 → 拒绝/投影 → FCL 碰撞兜底     │
        └───────┬────────────────────────────────────────────────────────┘
                ▼  执行到视点，拍摄
        ┌──────────── M7 验证（环上）────────────────────────────────────┐
        │  查 A 更新覆盖向量 → 覆盖达标? ──否──▶ 反馈给 M4 局部补算       │
        │  也把覆盖摘要交回翻译层 → 写进下一次 prompt ──────────┘        │
        └────────────────────────────────────────────────────────────────┘
4. 走查一个具体场景：工件被挪了 15 mm + 5°
假设执行到序列中段，工件 W 在转台上被碰了一下（15 mm 平移 + 5° 旋转，paper_01 的扰动量级）。传统管线此时序列全部失效，要全量重跑 M1–M5（paper_05 量级：约 3 小时）。Option B 里各模块依次这样动：

步骤	模块	发生什么	耗时量级
1	M7	拍到当前视点，发现图像与预期区域不符（或外部位姿估计报警）	实时
2	M3/M4	只局部补算：重查 A 中与受影响面片相关的列，判定哪些目标视点失效，补 1 个新目标	数秒
3	翻译层	把"下一个有效目标 + 当前覆盖摘要"写进 prompt（客户端逐次构造，零架构改动）	毫秒
4	Pi0.5	从当前实际位姿出发，靠 RGB + 目标描述反应式走过去——它不需要知道工件怎么挪的	实时（15 Hz）
5	约束投影层	每个 chunk 过成像约束检查，越界动作被拒绝	实时
6	M7	更新覆盖向量，判断是否达标，回到步骤 1 或终止	实时
一句话对比：传统管线对位姿扰动的响应是"重算 3 小时"，Option B 的响应是"局部补算数秒 + 反应式走过去"。

一个诚实的边界条件：步骤 1 假设存在一个"发现位姿变了"的机制（外部位姿估计或 M7 的图像差异判断）——九篇文献都没有真正闭环的"当前图像质量反馈"，这一步本身也是研究点；Pi0.5 的视觉反应可以在一定程度上兜底（它不看绝对位姿，天然抗扰），但这不构成保证。

5. 总结：M1–M7 改动一览表
模块	传统方案	Option B	谁执行	I/O 变化	例子
M1 离散化	同左	不改	几何层	无	s_347 不变
M2 候选生成	同左	不改	几何层	输出多了一个消费者（翻译层）	v_237 不变
M3 可见性矩阵	同左	不改	几何层	多一条读出路：M7 查询结果喂翻译层	A[237][347]=1 不变
M4 覆盖优化	离线一次性	不改，但准在线化	几何层	输出从"最终答案"变"目标序列"；支持局部补算	12 视点→96% 不变
M5 排序/运动规划	一体化	劈成 M5a（几何）+ M5b（Pi0.5）	混合	局部运动从"预计算轨迹"变"实时动作 chunk"	prompt 带 Next 目标 → chunk(10,8)
M6 执行	轨迹执行	换执行器	Pi0.5	输入从轨迹变 chunk；15 Hz 开环	关节速度步进
M7 验证	末端报告	挪到环上	几何层	输出从"给人看"变"给下一轮用"	覆盖摘要进下一次 prompt
（新）翻译层	不存在	新增	几何→token 接口	—	c → "State: 187 45..."
（新）约束投影层	不存在	新增	护栏	—	视距 172<185 → 拒绝
核心思想用模块化的话说就是：Option B 没有发明任何新算法模块，它只是重新划定了 M1–M7 的边界和连线——把"对位姿扰动敏感"的那一段（M5b/M6）划给擅长反应式输出的 Pi0.5，把"需要可证书保证"的模块（M1–M4、M5a、M7）留在几何层，并用两个新黑盒（翻译层、约束投影层）把两边粘起来。

1. Option A：人类专家是"行为定义者"（主数据源）
为什么需要专家：Option A 没有几何层，Pi0.5 要从头学完整行为"看哪 → 走过去 → 换点"。能定义这个完整行为的主体只有两类——人类专家，或自动化伪专家（FTS / generate-and-use）。专家数据是质量锚点，因为 paper_09 已经证明专家计划有自动方法没有的好性质（空间规律性、可验证性）。

人类专家演示的三个具体来源：


① paper_09 的交互式 3D 规划界面（九篇文献中唯一能精确记录专家视点的工具）
   专家在 3D 模型上点选视点 → 记录为视点序列
   例: Gear 工件上，专家甲的记录 = 44 个视点（达到 100% 覆盖）
   ↓ 转化（不是直接可用，需一步补全）
   视点序列 ≠ 轨迹。要在仿真中用 IK+插值补成连续轨迹，渲染成 RGB，
   再切成 (o_t, a_t) 对、转 LeRobot 格式，才是可训练的 SFT 数据

② 真机遥操作（Franka 手把手演示完整检测流程）
   机器人执行"走到位 → 停 → 拍摄 → 换点"全过程，录像采集

③ examples/droid/main.py 的录像 + 人工评分 CSV（工程上现成的采集通道）
   convert_*_to_lerobot.py 转格式
风险（文档里已列明）：成本高（人工，且 paper_09 自己都没测量专家耗时）；多样性低（2 名专家 × 每物体 2 份）；会把专家的硬件偏好也学进去（专家的计划天生是"固定相机位 + 转台旋转复制"形态——如果训练机没有转台，这个隐式模式就是噪声）。

辅助教师（自动化，不是人类）：paper_01 的 FTS rollout（伪专家，平均降 24.0%）、paper_09 的 generate-and-use 规则 rollout（≥90% 稳定，形态与 o_t→a_t 同构）、paper_08 的 Real-to-Twin 合成流水线（交叉使能者，免标注）。

2. Option B：人类专家退居"少量分布补充"，主数据源是 planner rollout
为什么几乎不需要专家：Option B 里 Pi0.5 只学 M5b（从当前位姿走到几何层指定的下一个视点）——目标由几何层给出，行为简单，执行标签可以在仿真里自动生成，不需要人类定义行为。文档原话："不需要新数据采集（planner rollout 即可）"。

planner rollout 数据的生成方式：


Phase 1 的几何管线（M1–M4 + M5a）输出视点序列
   ↓ 在仿真中用脚本化 IK + 插值控制器沿序列执行
自动产出 (o_t, prompt_t, a_t) 对：
   例: 时刻 t 的样本 = (RGB 当前视角, "Task: ...; Next: surface point
       (87.3, 45.1, 60.0), normal (0.12,0.85,0.51);\nAction: ",
       朝该目标运动的 10×8 动作块)
   覆盖状态 c_t 由 M7 查 A 得到，逐时刻写入 prompt 作条件（V1 变体）
   —— 教师是几何规划器，不是人
人类专家的角色：少量示教，只做分布补充——补的是 planner rollout 里没有的东西：真实执行段的动作细节（接近零件时的减速习惯、避障微调、平滑性）。来源与 Option A 相同（真机遥操作 / 录像），但需求量小一个量级，且 Option B 在原理上可以完全不用专家数据跑通（Phase 2 验收不依赖它）。

3. 对照表
Option A	Option B
专家演示的角色	主数据源 / 行为定义者	少量分布补充（非必需）
专家数据来源	paper_09 交互式 3D 界面记录 + 真机遥操作 + 录像	真机遥操作 / 录像（量少）
视点序列→训练数据的转化	需 IK+插值补全轨迹（仿真或遥操作）	同左（若用专家数据）
自动教师（非人类）	FTS、generate-and-use、Real-to-Twin（辅助）	planner rollout（主）
为什么是这个比例	无几何教师，完整行为只能从"会做的主体"来	目标由几何层给，执行标签可仿真自动生成
一句话：Option A 的 Pi0.5 学"怎么做检测"，教师主要是人；Option B 的 Pi0.5 只学"怎么走到下一个视点"，教师是规划器，人只负责补一点执行细节。

最后提醒一个共性的工程限制：真机采集的图像在机器人侧就已 resize 到 224×224 再上传（省带宽），分辨率损失不可逆——所以专家演示数据里"缺陷细节"这个尺度在训练时本来就是模糊的，这也是为什么判异任务不能指望 VLA 的视觉通道直接完成。

纯传统规划复现：

复现 paper_05（SCTSP + RKGA），借用 paper_09 的评测协议，Phase 3 再加 paper_01 的 FTS 做在线对照
三者分工：


Phase 1 baseline 本体   → paper_05（离线传统规划器，全管线可复现）
评测协议/验收准则       → paper_09（R1–R4 + 收敛曲线包络 + 公开代码）
Phase 3 位姿扰动对照    → paper_01（FTS 在线重规划，最强者充当"传统阵营上限"）
为什么首选 paper_05：它是唯一满足 Phase 1 全部硬性条件的候选
回忆 Phase 1 的验收标准：1 个月内，在 3 个工件上稳定 ≥95% 覆盖，产出 ≥50 条完整演示轨迹。对照 paper_05：

管线自洽完整——M1 离散化 → M2 视点采样 → M3 可见性矩阵 → M4+M5 联合求解 → 输出有序序列+cycle time，一个都不缺。多数论文只覆盖其中 2–3 个模块，复现完还得自己补其余的，反而更慢。
零学习组件——纯组合优化（RKGA），不需要像 paper_03（PBPS 网络）或 paper_04（PR3Net）那样先复现/训练一个网络才能跑。
参数完备到可以直接抄——Artec Eva 传感器（FOV 30°×21°、536×371 px、视角 75°、视距 0.4–1.0 m）、覆盖率要求 δ=99.5%、工件尺寸（0.24×0.47×0.30 m）、候选规模 200/300/400 消融——论文的 Table I–III 数据完整（九篇里提取质量最好），复现完有明确的对表数字可自检。
输出形态与 Phase 3 指标直接对齐——它的输出就是 cycle time（检测 1.0 s/视点 + 行走时间），正是我们要比的核心指标；而且它自带"点少≠时间短"的结论，等于提前告诉你在评测 harness 里该把哪些数字分开报。
天然是数据教师——RKGA 对不同工件位姿反复求解即产出 rollout 轨迹（Phase 1 任务 4 就是这个），它的最优解也是 Option C 的监督信号来源。
代价只有一条：400 samples 要跑约 3 小时——但我们 Phase 1 的规模（500–2000 面片、200–500 候选）比论文小得多，且行走代价用 travel-cost 代理（paper_03 边权公式）代替 RRT+spline，单次求解控制在分钟级即可。

逐篇排除表（为什么不是其他八篇）
论文	能否当 baseline	一句话理由
p01 FTS	✅ 二期加（在线对照）	最强传统在线方法、天然处理位姿扰动，但树搜索实现量大，且它本来就是要被 VLA 挑战的对象——先做离线 baseline 更稳
p02 特征细分	❌	B-spline 曲面网络拟合工作量大，且只覆盖 M2 一个模块，不构成完整管线
p03 MG-HVP	❌	PBPS 是训练出来的分割网络，不是"纯传统"；复现要先解决网络权重
p04 采集系统	❌	PR3Net 同样是学习组件；且要 3D 相机实测点云，仿真里要先造传感器模型
p06 离线视点	❌	依赖 NURBS B-Rep + GD&T 属性，需要 CAD 内核；5 类任务规则是钣金专用，难迁移到通用 mesh
p07 SC-GTSP	⚠️ 与 p05 二选一	同源方法多了 IK 冗余簇（每视点 5 个解 + 转台 7-DOF），表达力更强但复杂度更高；建议一期先用 p05，若 Phase 3 想讨论冗余自由度再加 p07
p08 AVATAR	❌ 当规划 baseline 太弱	它的视点规划只有"等面积多面体采样+仰角过滤"（没有覆盖优化）；但它贡献 Blender 渲染管线——Phase 1 的仿真环境直接抄它的工程
p09 对比研究	✅ 借协议不借方法	它本身是评测论文不是方法论文；借用它的 R1–R4 验收准则、收敛曲线包络图、292 候选集/公开代码，以及它提供的"专家计划"作为另一条 baseline（在 Phase 3 讨论"视点少≠计划好"时用）
复现 paper_05 的模块级工作量拆解（约 2–3 周，含余量）

M1 表面离散化（1–2 天）
   工具: trimesh 重采样（不必实现 bubble mesh，均匀重采样即可）
   自检: 输出 500–2000 面片，每片带中心+法向

M2 视点候选生成（1 天）
   照 paper_01 的采样: 包围椭球半球随机撒点，视线指向工件中心 + 扰动
   自检: 200–500 候选，数量可调

M3 可见性矩阵（3–5 天，工作量主体）
   四条判据（FOV ∧ 视距 ∧ 视角 ≤75° ∧ 无遮挡）用 trimesh ray-mesh 求交
   （或照 paper_08 的工程用 Blender 深度渲染——p06 的 VM 也是这个思路）
   自检: A 的密度与 paper_05 的 δ=99.5% 设定下"约几十个视点覆盖"的数量级一致

M4+M5 RKGA 联合求解（3–5 天）
   random-key 编码（小数排序 = 访问顺序）+ 适应度 = 覆盖率罚项 + 总时间
   自检: 在相同候选规模下，联合求解 vs 分开求解（先贪心 SCP 再 TSP）
   的 cycle time 差异应能重现"点少≠时间短"现象

评测 harness（2–3 天）
   覆盖真值用深度缓冲渲染验证（paper_09 的做法）；指标 = coverage%、
   视点数、cycle time；验收 = 3 个工件 ≥95% 覆盖 + 50 条 rollout 轨迹
自检阈值建议：先用论文 Target Object 1 的量级（0.24×0.47×0.30 m）做一次完整复现，如果联合求解相对分开求解的 cycle time 改善落在 5–25% 区间，就说明复现的几何与求解器都对了（论文报 10.8%–28.4%）。

一个 Phase 3 的提醒
位姿扰动实验需要"传统阵营"的在线对手：paper_01 的 FTS 是唯一选择（它的全部动机就是对抗 15 mm + 5° 的摆放误差，平均降 24.0%）。如果 Phase 3 时间紧张，退而求其次的弱化版是"paper_09 的 NBV 贪心 + 每次扰动后全量重规划"——实现一天就能完成，但结论的强度会弱一档（FTS 才代表传统方法的上限）。研究文档 Phase 3 的 R1 行目前写的是"全量重规划"，建议在实施时把 R1 拆成 R1a（NBV+重规划，弱 baseline）和 R1b（FTS 在线，强 baseline），这样无论 VLA 结果好坏，结论都站得住。