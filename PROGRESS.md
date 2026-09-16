# 进度追踪

按 `PROJECT_PLAN.md` 的 Phase 顺序记录。每完成一个里程碑就在这里加一条，commit 时带上这次改动对应的记录。

## 使用方式（维护约定）

- 每次开始工作前，先读这份文件的"当前状态"和最近几条记录，再看 `git log --oneline -20` 确认实际代码进度和这里记的一致。
- 每完成一个可验证的里程碑（环境跑通/控制器能站稳/一组实验跑完/遇到卡点需要下次继续），追加一条记录，然后 commit + push。
- 遇到需要用户决策的问题（比如某个开源仓库能不能用、是否要换机器人模型），记录在"待决策"里，不要自己瞎猜决定重大方向。

---

## 当前状态

`Phase 2 - 进行中（①②③ 完成；④ 集成回路的失稳根因已精确定位到力矩饱和机制，具体修复方向待定）`

- Phase 1 ✅ 完成：MuJoCo + Go2 环境跑通，`src/sim_env.py`。
- Phase 2 选型 ✅ 方案 A（参考架构自实现，MuJoCo 原生，两层 OSQP）。
- Phase 2 ① ✅ `src/gait.py`；② ✅ `src/mpc.py`；③ ✅ `src/wbc.py`——三层各自单元测试全 PASS。
- Phase 2 ④ 🚧 `scripts/trot_demo.py`：站立（stand 步态）长期稳定；原地踏步（walk 步态）约 3.8s / 5 个周期后翻倒。
  尝试过分层 QP、显式松弛变量、接口时序平滑三条修复路径，均未跑赢单层加权 QP 基线（详见记录，负结果已归档到
  `wbc-hierarchical-qp` 分支）。转向精细化诊断后，**已经把翻倒的物理机制精确定位到**：单次对角腿切换瞬间
  （一腿落地+另一腿抬起同拍发生），刚落地那条腿的关节力矩里"姿态动态修正分量"在约 30ms 内从 0 飙升到饱和，
  而"静态扛重分量"全程只有几牛米、远未触顶——不是承重不够，是姿态纠偏的动态力矩需求把执行器顶爆的。
  但"调弱姿态增益"这个直觉修法被证伪：权重减半无效、PD 增益减半反而更差。真正的杠杆还没定位到，
  候选方向见下方记录。诊断图 `results/transition_diagnostic.png`，两段真实视频 `results/stand_demo.mp4` /
  `results/walk_demo.mp4`。

### 重开 Pod / 新会话恢复工作的步骤（重要）

`/workspace` 持久化，但 apt/pip 依赖在 pod 重启后通常会丢失。新会话先做：

```bash
cd /workspace/quadruped-wbc-mpc
bash scripts/setup_env.sh          # 装系统 GL + pip 依赖，并自检模型加载
python scripts/stand_test.py       # 应打印 RESULT: PASS，确认环境 OK
git log --oneline -10              # 核对代码进度与本文件一致
```

然后读本文件"当前状态"+"Phase 2 实现计划"，从上次断点接着做。

## 待决策（已解决）

- **[已决策 - Phase 2④ trot 镇定方向] 采用方案 (a)：MPC 做预测式主镇定 + WBC 忠实跟踪 MPC 反力。**
  执行策略：先用默认 `WBCGains()` 把"能走"（先 walk 后 trot）跑出来，再逐步把 MPC 力权重接进来。
  进展：走 (a) 的路线过程中修了两个真实 bug（摆动轨迹加速度不连续、摆动任务劫持基座加速度），
  walk 从 <1.5s 撑到 3.82s，但继续调权重数字收益递减。**下一步方向已经明确、不需要再拍板**：
  把 WBC 从单一加权 QP 改成任务优先级/分层 QP（即原候选 (b)），而不是继续调权重。详见 Phase 2④ 记录。


- **[已决策 2026-09-15] Convex MPC + WBC 参考实现选型 → 采用方案 A（参考架构自实现，MuJoCo 原生）。**

  目标架构（MIT Cheetah 一脉，见 PLAN）：Convex MPC（SRBD 13 维线性模型 → 足端反力）+ WBC（关节力矩 QP，含接触/摩擦锥/力矩约束），MuJoCo + Go2，OSQP 求解，尽量避免重依赖。

  **候选评估：**

  | 仓库 | 覆盖层 | 依赖 | License | 维护/热度 | 契合度 |
  |---|---|---|---|---|---|
  | [elijah-waichong-chan/go2-convex-mpc](https://github.com/elijah-waichong-chan/go2-convex-mpc) | Convex MPC + 摆动/支撑阻抗控制（**无独立 WBC QP**） | CasADi+OSQP+**Pinocchio** | **MIT** | **127★，2026-02 仍在更新** | MPC 层最佳参考；Go2+MuJoCo；但缺 WBC，依赖偏重 |
  | [benaziel/quadruped_wbc](https://github.com/benaziel/quadruped_wbc) | **WBC QP**（42 维：18 加速度+12 力+12 力矩，EoM 等式约束+摩擦锥+力矩限幅）；**无 MPC** | MuJoCo+**OSQP**（纯 Python，无 Pinocchio） | **无 License(!)** | 0★，21 commits | WBC 层最佳参考，依赖最干净、正好用 osqp；但无 license 只能参考不能照抄 |
  | [kairoi-k/go2-mujoco-control](https://github.com/kairoi-k/go2-mujoco-control) | **SRBD MPC + 18-DoF ID-WBC 都有** | C++/CMake + Unitree SDK2 **DDS**，dense QP | BSD-3 | 0★ | 架构最全，但 **C++ + DDS 多进程**，移植到我们 Python sim_env 代价大 |
  | fast_and_efficient (yxyang) | 卷积 MPC + Raibert 摆动 | Python, **PyBullet**, A1 | — | 成熟 | 非 MuJoCo、非 Go2、无 WBC QP |
  | rl-mpc-locomotion / PyDog / johnzhang3/mujoco_mpc_deploy | RL+MPC / 教学 MPC+WBIC / 全身 iLQR-MPC | Isaac Gym / 自带 sim / MuJoCo | — | — | 方法或平台不对口（iLQR 全身 MPC ≠ 计划的 SRBD 凸 MPC+WBC 分层）|

  **✅ 采用方案 A（参考架构自实现，MuJoCo 原生）**：以 elijah 仓库作 **MPC 层**参考、benaziel 仓库作 **WBC 层**参考，两层都用 OSQP 在我们的 `sim_env` 上自己写 `src/mpc.py` / `src/wbc.py`；动力学量（质量矩阵/雅可比/偏置力）直接用 MuJoCo 的 `mj_fullM`/`mj_jac`，**免掉 Pinocchio/CasADi/DDS** 重依赖。
  - 理由：①命中 WBC + MPC + 约束 QP 三个 JD 关键词且两层齐全；②依赖最轻、正好用计划里定的 osqp；③简历项目需要能讲清推导，自己实现比跑别人二进制更有区分度；④站在两个可读的 Python 参考上，不是从零盲写。
  - License：elijah(MIT)/kairoi(BSD-3) 可参考并署名；benaziel 无 license，仅作**架构参考、不照抄代码**。
  - （未采纳）备选 B：移植 elijah 全仓再补 WBC —— 引入 CasADi+Pinocchio、stance 非 WBC QP，关键词偏弱。
    备选 C：移植 kairoi —— C++/DDS 移植成本高。

## Phase 2 实现计划（方案 A 的分解，按此顺序推进）

目标交付：`src/mpc.py`、`src/wbc.py`，以及平地稳定 trot 的 demo（`scripts/trot_demo.py` + `results/` 视频）。

- **① 步态调度 `src/gait.py`**：trot（对角腿同相）时钟/相位调度，输出每条腿 stance/swing 状态与相位；
  Raibert 落脚点启发式 + 摆动腿轨迹（摆线/五次多项式，抬腿高度可调）。参考 elijah `gait.py` / benaziel 的 contact-gated clock。
- **② Convex MPC `src/mpc.py`**：SRBD 13 维状态 [θ(3), p(3), ω(3), v(3), g(1)]，绕当前姿态线性化（标准 MIT Cheetah 公式），
  预测时域 N≈10、dt_mpc≈0.02–0.03s，决策变量为各接触腿 3 维反力（时变接触序列由 ① 提供），
  代价：base 位姿/速度跟踪 + 力平滑；约束：摩擦锥 + 单边法向力 + 摆动腿力=0。QP 用 **OSQP**（稀疏、warm-start）。
  动力学参数（质量、转动惯量）从 MuJoCo 模型读。
- **③ WBC `src/wbc.py`**：决策变量 [qddot(18), f_contact(12), tau(12)]，用 MuJoCo 的 `mj_fullM`（质量矩阵）、
  `mj_jac`（接触/摆动足雅可比）、`data.qfrc_bias`（科氏+重力）。等式约束：浮动基 EoM；不等式：摩擦锥、力矩限幅；
  任务代价：跟踪 MPC 反力 + base 姿态/高度 + 摆动足加速度。输出 12 关节力矩下发给 `sim_env.step`。参考 benaziel `wbc.py` 架构（不照抄）。
- **④ 集成回路 `scripts/trot_demo.py`**：MPC 低频（~40–50Hz）+ WBC/仿真高频（500Hz）+ 摆动腿；
  平地调参跑通 trot，出视频到 `results/`，记录一条 PROGRESS。之后才进 Phase 3 定量实验。

## 记录

<!-- 格式：### Phase X - 一句话摘要 \n 具体做了什么、结果如何、下一步是什么（尽量不写日期，git 已有时间戳） -->

### Phase 2④ - 精细化诊断：翻倒根因定位到"落地腿姿态动态修正力矩饱和"，但简单调弱增益的修法被证伪

放弃了继续在 WBC 架构/时序上盲试（分层 QP、松弛变量、接口平滑都试过、都没跑赢基线，见下面两条记录），
改用"隔离单个物理事件 + 定量拆解"的方式一步步逼近根因。整个链条如下。

**第一步：静态站立诊断，排除模型标定问题**

跑 8 秒纯静态四足站立（`gait_name="stand"`，MPC+WBC 全程闭环，不涉及任何步态切换）：
- 左右完全对称：FL=FR=35.89N，RL=RR=38.70N，左右合力差 = 0.000N，roll 全程 = 0.0000°（线性拟合斜率
  ≈0），8 秒零漂移。
- 前后不对称（前腿 71.77N vs 后腿 77.40N）但符合物理预期：真实 CoM 比 base 原点靠后 2.2mm，后腿理应多扛一点。
- 交叉核对 MPC 用的模型参数：mass=15.2064kg，CoM=[-0.0022,-0.0,0.2486]（y 方向零偏移，惯量矩阵
  Ixy≈Iyz≈0，只有 Ixz=-0.0165 不可忽略但不影响左右对称）。**结论：MPC 模型参数和仿真真实模型对得上，
  静态场景没有任何系统性偏置——翻倒不是模型标定问题，是动态/瞬态问题。**

**第二步：单次对角腿切换实验，锁定物理事件**

之前 walk 步态里，翻倒总是发生在"一条腿刚落地、另一条腿同一拍抬起"（duty=0.75 的步态零缓冲切换）附近。
搭了一个非循环的单次实验：四足站稳 → RL 单独摆动一次（真实 Raibert 落脚点 + 摆线轨迹）→ RL 触地的
同一 tick 让 FR 开始摆动（复刻真实观察到的同拍切换）→ FR 摆完落地 → 停止，不再循环。±100ms 高分辨率记录。

- 定位到**是刚落地的 RL 自己的执行器先饱和，不是刚抬起的 FR**：RL 小腿力矩触地时只有 3.2Nm，之后平滑爬升
  （3.2→6.0(+20ms)→15.4(+26ms)→26.6(+28ms)→**45.4Nm 饱和(+30ms)**），随后 RL 髋/大腿关节也相继饱和；
  FR 自己的力矩这段时间一直是个位数，直到 RL 已经失控后才在 +36~38ms 被动跟着饱和——是**继发**不是触发源。

**第三步：验证并证伪"落地姿态力学劣势"假说**

假说：RL 触地瞬间腿处于摆线轨迹末端的伸展姿态，力学劣势导致同样的力需要更大力矩。直接算了 RL 触地瞬间
vs 正常支撑中段的关节角、腿部子雅可比 `J_leg`（只取该腿自身 3 列）、以及由力矩上限反推的姿态承载力 Fz_max：

| | 中段支撑 | 触地瞬间 |
|---|---|---|
| 关节角(hip,thigh,calf) | 0.0°,50.9°,-104.0° | -0.0°,51.0°,-106.2° |
| calf 的 dτ/dFz | -0.1689 | -0.1736（只差3%） |
| 姿态 Fz_max（力矩上限反推） | 248.0N | 250.0N（几乎没变） |
| MPC 分配给 RL 的目标 Fz | 40.9N | 27.0N |
| 承载余量 | +207N | +223N |

**假说不成立**：触地瞬间的关节角、雅可比、承载力和正常支撑几乎一样，MPC 分配的静态力目标离承载上限还差一个
数量级。饱和不是"扛不动体重"，必须另有原因。

**第四步：力矩分解，定位到"姿态动态修正分量"**

给 `src/wbc.py` 的 `solve()` 返回值加了两个只读诊断字段 `qdd`（求解出的关节加速度）、`f_solved`（求解出的
接触力）——纯暴露内部已算出的量，不改变任何求解逻辑（self-test 仍 9/9 PASS）。用全身 EoM
`tau = (M@qddot)[关节行] + (h − Jc^T@f)[关节行]` 把 RL 小腿力矩拆成"静态扛重"(h−Jc^T@f 部分) 和
"姿态动态修正"(M@qddot 部分) 两项，逐点验证两项之和精确等于实际力矩。

结果：静态分量全程稳定在 ~3Nm（吻合第三步算出的 250N 承载力，远未触顶）；**动态分量从触地时的 6% 占比
一路涨到 +24ms 时的 100%+**，从这之后力矩几乎全部来自姿态动态修正项，直到 +30ms 撞上 45.4Nm 上限。
图见 `results/transition_diagnostic.png`（上图：力矩拆解；下图见下一步）。

**第五步：调弱姿态增益的直觉修法——被证伪，且方向相反**

- `w_ori` 权重减半（1000→500）：**完全无效**，饱和依旧精确发生在 +30ms，力矩轨迹几乎重合。原因：500 依然
  远大于第二名任务权重 w_swing=200，加权和里的优先级排序没变，QP 仍几乎精确满足姿态任务的目标加速度——
  权重只决定"这个任务在加权和里跟谁抢资源时占不占优"，不直接决定目标加速度本身，目标不变则力矩需求不变。
- 真正决定目标加速度大小的是 PD 增益 `kp_ori`/`kd_ori` 和硬上限 `a_ang_max`。但把 `kp_ori` 减半或
  `a_ang_max` 从 30 降到 10：**结果更差**——饱和提前到 +0ms（几乎触地瞬间就打满），窗口内 roll 峰值从
  基线的 1.14° 恶化到 10.84°（kp 减半）/ 25.36°（a_ang_max=10）。见 `results/transition_diagnostic.png` 下图。
- **解释**：RL 在落地前自己单独摆动的那 ~200ms 里，roll 已经在缓慢累积误差和角速度（此前测到从 -0.03°
  累积到 -0.146°）。调弱纠偏增益只是让摆动期间累积更多误差，转换瞬间要处理的姿态偏差更大、
  可用支撑腿更少（同拍恰好少一条腿），力矩需求不降反升。**不管增益强弱，只要"单腿摆动期间误差累积 +
  同拍双侧切换瞬间腿数减少"这个物理事件发生，迟早要被迫消化这份纠偏力矩，调增益只改变饱和早晚，不改变
  会不会饱和。**

**当前结论**：翻倒的直接触发机制已经清楚（落地腿力矩被姿态动态修正项顶爆），但"调弱增益"这条最直接的
修复路径已经证伪。候选的下一步方向（未验证，供下次讨论）：
  (a) 减少 RL 单腿摆动期间的误差累积（更短摆动周期/更快步频，或摆动期间也做部分纠偏而非等落地后才纠偏）；
  (b) 从根上避免"同拍双侧切换"（即避免 duty=0.75 的零缓冲切换，让步态有极短的四足/三足过渡缓冲，
      之前"接口时序平滑"那条路没做对具体实现，但没有专门针对"避免同拍双切"这个精确诱因去设计）；
  (c) 把纠偏力矩从单条刚落地的腿身上分摊到其余支撑腿（这需要动 WBC 的任务分配方式）。

### Phase 2④ - 集成回路搭好；trot 尚未稳定（诊断 + 部分修复，待定方向）

**做了什么**
- `scripts/trot_demo.py`：把三层串成闭环——gait 出接触/摆动相位 → MPC(~33Hz, dt=0.03,N=10) 出反力 →
  WBC(500Hz) 出力矩 → sim.step；含 stand 预热、速度 ramp、Raibert 落脚点、摆线摆动、日志/视频/早停摔倒检测。
- 沿途在 `src/wbc.py` 修了 3 处真实问题（都保留，各自使 self-test 仍 9/9 PASS）：
  1. **stance 接触改软任务**：原硬等式 `J·qddot=−Jdot·qvel` 会与力矩限幅冲突→QP 频繁不可行→输出乱力矩炸飞；
     改成高权重软任务后 qddot 自由、EoM 恒可解，`wbc_fail` 从数百降到 0。
  2. **非最优解回退**：QP 非 optimal 时回退上一步力矩，不再下发 OSQP 的巨大乱值。
  3. **任务加速度饱和**（`a_lin_max/a_ang_max/a_swing_max`）：强增益遇大误差时会索求爆炸力→launch；加 clip 后小误差仍刚、大误差被限。
- `scripts/trot_demo.py` 里修了落脚点参考点：Raibert 原来以 hip 关节位置(y=±0.046)为参考→落脚过窄→侧向翻；
  改用 base+yaw 旋转后的**名义足位**(y=±0.142)，恢复自然宽支撑。

**结果 / 现状（真实跑出）**
- ✅ 4 足站立：WBC-only 闭环 2s 漂移 0、max|pitch|0.17°。
- ❌ 平地 trot：稳不住。典型 `python scripts/trot_demo.py --secs5 --vx0`：~1.0–1.4s 内 max|roll|→60° 摔倒；
  在场景中 `wbc_fail=0`（可行性已解决）、无 launch（饱和已解决），失败模式是**纯控制**：2 对角支撑相里躯干慢翻。

**根因诊断（关键，供下次/换方案参考）**
- 隔离实验证明：即便**静态**只用 2 对角足站立（FL+RR），当前 WBC 也稳不住（~1–2s 翻到 30–160°），
  加大姿态增益(kp_ori 到 9000)也没用 → 不是调参问题，是结构问题。
- 机理：浮动基**无驱动**，其滚转力矩只能来自**接触力**；WBC 只能下发关节力矩 τ，接触力是 MuJoCo 被动涌现的。
  4 足是稳定平衡（略偏差也不倒），2 对角是**不稳定平衡**，需要精确的力控来镇定，而 τ→实际接触力的映射（软接触）
  达不到这个精度 → 反应式 WBC-base-PD 镇不住这个模态。MPC 开/关对比也证明当前 MPC 没能补上（no-MPC 同样翻）。

**下一步（待用户拍板，见"待决策"）**：trot 镇定需要更强手段，候选：
  (a) 让 **MPC 做主镇定**（预测式，horizon 内把不稳定模态压住）、WBC 主要**忠实跟踪 MPC 反力**而非自己做 base PD——
      需要给 MPC 正确的 horizon 内落脚点/接触序列 + 调姿态权重；
  (b) WBC 改**任务优先级/零空间投影**（接触>姿态>摆动）而非单 QP 软加权；
  (c) 降低目标：先只交付"4 足站立 + 原地缓慢踏步/极慢 trot"的可视化，把 Phase 3 对比实验的基线换成站立抗扰。

### Phase 2④ - 定位并修复两个真实 bug；walk 从 <1.5s 撑到 ~3.8s，仍会翻，转向任务优先级 WBC

接上一条记录（默认增益让站立恢复稳定）。这次从"用默认增益重测 walk/trot"接着做，一路又踩了几个坑，
其中两个是真实、可验证的 bug（已修复），最后一个是负结果（尝试过但没用，也记下来避免下次重复踩）。

**做了什么 / 踩了什么坑**

1. **复现基线**：默认 `WBCGains()` + `gait_name="stand"` 全程稳（roll 0、h=27cm）。符合预期。
2. **测 walk（3 足静稳支撑）仍会摔**：~1.3–1.7s 内 roll 冲到 60°。说明"2 对角不稳定平衡"不是唯一根因
   （walk 全程至少 3 足支撑，理论上静稳，但照样翻）。
3. **真 bug #1 —— 摆动轨迹加速度不连续**：追出摆动腿刚抬起的瞬间基座会被"发射"（h 从 27cm 冲到 60+cm）。
   查了 `src/gait.py` 摆动腿竖直"钟形"曲线的数学式：`0.5·h·(1−cos(2πs))` 端点位置/速度确实为 0，
   但**端点加速度不为 0、反而在 s=0（抬腿瞬间）取最大值**（≈38.5 m/s²，约 4g）——摆动腿一进入摆动相
   就被指令以近 4g 加速度"踹地"。用 sympy 验证后换成 minimum-jerk 型五次多项式 `64·s³·(1−s)³`
   （s、(1−s) 各三重根，位置/速度/加速度在两端点都严格为 0），`src/gait.py` self-test 加了加速度为 0
   的断言（13/13 PASS）。
4. **真 bug #2 —— 摆动任务能"劫持"基座加速度**：修完 bug #1 后 walk 还是会摔（launch 依旧，只是延后）。
   继续追发现：摆动腿力矩饱和（卡在小腿 45.4Nm 上限）期间，基座反而被顶起。原因是 WBC 单个加权 QP 里，
   摆动任务的雅可比 `J[i]` 在基座 6 列（0:6）非零——对一个装在浮动基上的腿，抬高基座和抬高腿在"抬起该脚"
   这个代价函数眼里是等价的。当该腿自己的关节力矩不够用时，QP 会更便宜地选择"抬基座"来达成摆动任务，
   而不是老老实实抬腿。修复：摆动任务的代价矩阵里把基座那 6 列清零，强制摆动任务只能用该腿自己的 3 个
   关节加速度（stance 的零滑移任务保留基座耦合——那个是真实物理，站立腿的的确会被基座拖走，不该删）。
   修完后 walk（period=0.8, sh=0.05, 默认增益）存活时间从 ~1.3–1.7s 提升到 **3.82s**（约 5 个步态周期）。
5. **负结果（试过没用，记录以免重复）**：怀疑 yaw 长期漂移（观测到 35°+）是多周期后翻倒的根因，
   给 `scripts/trot_demo.py` 加了显式 yaw 积分跟踪（原来是"yaw_des=当前 yaw"，等于没有 yaw 反馈）。
   实测**更差**（3.82s → 1.58s 摔）——强行锁 yaw 会和 roll/pitch 修正抢占同一批执行器的力矩预算，
   在这套耦合系统里"纠正 yaw"不如"放任 yaw 漂移"稳。已撤销这个改动，恢复"yaw 跟随当前值"。
6. 又做了一轮增益/步态参数网格搜索（`w_contact`、`kp_swing`、步频、抬腿高度等），发现结果对单个标量
   权重**极度敏感**——比如 `w_contact=150` 能撑满 5s 但 roll 摆到 56°像在钢丝上走，稍微再联动调别的参数
   就重新崩溃。这本身是一个值得记的结论：**单一加权和 QP 的 WBC 面对"多接触切换 + 力矩饱和"时鲁棒性差，
   继续调权重数字收益递减**，应该换成任务优先级/分层 QP（hard hierarchy：先满足 EoM + 接触约束，
   姿态任务在零空间里做，摆动任务再在姿态任务的零空间里做），而不是全部塞进一个加权最小二乘。

**结果（真实跑出，最终采用配置：默认 `WBCGains()`，两个 bug 修复保留，yaw 改动已撤销）**
- `gait_name="stand"`：3.5s 全程 roll=0.00°、h=27.0cm 不变。视频 `results/stand_demo.mp4`（88 帧，非黑帧）。
- `gait_name="walk"`（period=0.8, step_height=0.05, vx=0 原地踏步）：t=3.82s（约 5 周期）时 roll 到 60° 摔倒，
  之前翻倒 max|pitch|=26.1°，`wbc_fail=0`（可行性正常，纯粹是控制质量问题）。视频 `results/walk_demo.mp4`
  （96 帧，如实记录到摔倒瞬间，没有剪掉失败部分）。

**下一步**：把 WBC 从"单一加权 QP"改成任务优先级/分层 QP（PROGRESS 待决策里原来的候选 (b)），
接触约束和 EoM 放最高优先级（硬约束，已经是），姿态任务在零空间内求解，摆动任务在姿态任务的零空间里再求解，
避免摆动任务和姿态任务互相在同一个加权和里抢基座加速度。这是接下来重新开一个会话时应该做的核心变更，
不要再继续在当前的单 QP 架构里调权重数字（边际收益已经很低，上面的网格搜索证明了这点）。

### Phase 2④ - 集成回路：一路排查到"默认增益站立稳、我的调参把 baseline 调坏了"这个关键突破

这条记录把整条排查链路留下来（面试时这段"踩坑—定位—纠偏"的故事比结果更有料）。

**目标**：把 gait+MPC(~33Hz)+WBC(500Hz)+摆动腿串成闭环，平地跑通 trot。`scripts/trot_demo.py` 已实现（stand 预热、速度 ramp、Raibert 落脚点、摆线摆动、日志/视频/摔倒早停）。

**踩坑与逐个定位（都留在代码里，`src/wbc.py` self-test 仍 9/9 PASS）**
1. 起步就侧翻 → 站立→trot 切换太突兀，加 0.5s stand 预热。
2. QP 频繁不可行、输出乱力矩炸飞 → 原来 stance 用硬等式接触约束 `J·qddot=−Jdot·qvel` 与力矩限幅冲突；
   改成**高权重软任务**后 qddot 自由、EoM 恒可解，`wbc_fail` 从几百降到 0。并加"非最优解回退上一步力矩"。
3. 一扰动就 launch 到 60cm → 强增益遇大误差索求爆炸力；加**任务加速度饱和** `a_lin/a_ang/a_swing`。
4. 侧向翻 → Raibert 落脚点原以 hip 关节(y=±0.046)为参考、落脚过窄；改用名义足位(y=±0.142)恢复宽支撑。
5. 反复怀疑 MPC/2 对角不稳定模态，做了大量隔离实验（静态 2 足、开/关 MPC、MPC 主镇定+WBC 跟力等），一直摔。

**关键突破（最后定位）**
- 决定性对照：用 `gait="stand"`（全程 4 足、无摆动）跑 demo。用**我覆盖的那套"调优"增益会摔**，
  但用**纯默认 `WBCGains()` 完全稳**（roll≈0、pitch≈0、h=27cm 保持到 1.2s+）。
- 结论：**demo 结构和 WBC 本身没问题，是我为了"调稳 2 足"而改的增益（降 kp_pos_z、动 a_*_max、w_force 等）反而把
  原本稳的 baseline 破坏了**——绕了一大圈，最有效的动作是"回退到默认增益"。
- 附带一个有用的量纲洞察：力跟踪残差是 O(100N)、加速度任务残差是 O(10)，所以 `w_force=1` 并不"弱"，
  MPC 力一旦震荡就会通过力跟踪项泄漏进来扰乱基座——调参时要按残差量纲配权重，别只看权重数字。

**下次（Sonnet 会话）接续，直接从这一步开始**
1. 环境：`bash scripts/setup_env.sh && python scripts/stand_test.py`（pod 重启依赖会丢）。
2. 复现稳定基线：`run_trot(gait_name="stand", wbc_gains=WBCGains())` 应全程稳。
3. **紧接着要跑的实验**（上次在这条命令处被打断）：用**默认 `WBCGains()`** 重测
   `gait_name="walk"`（period 0.8/1.0，静稳 3 足支撑，最可能先跑通）与 `gait_name="trot"`（period 0.4/0.5），vx=0 原地。
   预期 walk 能站住/踏步；若 walk 稳，再加 vx（前进）、再攻 trot。
4. 若 walk 也不稳，从"单腿摆动瞬态"入手（摆动腿抬起/落地时对基座的扰动、落地冲击），而不是再动基座增益。
5. 跑通后出视频到 `results/`，再进 Phase 3。
   —— 方向仍是方案 (a)（MPC 预测式镇定 + WBC 跟力），但先用默认增益把"能走"跑出来，再逐步把 MPC 权重接进来。

### Phase 2③ - WBC 关节力矩 QP src/wbc.py 完成

**做了什么**
- `src/wbc.py`：全身控制力矩 QP（决策 42 维 = qddot(18)+f(12)+tau(12)）。
  - 动力学量全走 MuJoCo：`mj_fullM`（质量矩阵）、`mj_jacSite`（足端雅可比）、`data.qfrc_bias`（科氏+重力）。
    `Jdot·qvel` 偏置项用 scratch MjData 有限差分（免去 cacc/重力约定坑）。
  - 硬约束：浮动基 EoM 等式；stance 足硬接触无滑移 `J·qddot=−Jdot·qvel`；swing 足反力=0；
    stance 足摩擦锥金字塔 + 单边法向力；关节力矩限幅。
  - 软任务（加权最小二乘）：base 姿态（机体系角加速度 PD）、base 高度（世界系，x/y 位置不跟踪只跟速度）、
    swing 足笛卡尔加速度跟踪、跟踪 MPC 反力、正则。OSQP 求解 + warm-start，非最优解回退上一步力矩。
  - 关键调参教训：base 姿态/高度任务权重必须**远大于**力跟踪/正则（≈近似硬任务），否则被稀释、
    躯干像倒立摆一样翻掉；弱增益反而不稳。最终 w_ori=1000/w_pos=500，kp_ori=(2000,2000,800)。
- self-test（`python -m src.wbc`）9/9 PASS。

**结果**（真实跑出）
- 站立单次解：EoM 残差 5e-15，跟踪 MPC 反力误差 0.31N，|tau|max=5.9Nm，求解 ~0.2ms。
- 闭环 WBC-only 站立 2s：末态高度 27.0cm（=home），漂移 0.00cm，max|roll|=0.00°，max|pitch|=0.17°，
  1000 步 0 次求解失败。（对照：默认弱增益初版会在 ~0.2s 内翻倒，据此定位到权重/增益问题。）
- trot 单腿摆动工况：求解最优，EoM 残差 1.45e-6（在 OSQP 容差内），|tau|max=12.9Nm。

**下一步**：Phase 2 步骤 ④ `scripts/trot_demo.py`——把 gait+MPC(~40Hz)+WBC(500Hz) 串成闭环，平地 trot 调参跑通出视频。

### Phase 2② - Convex MPC src/mpc.py 完成

**做了什么**
- `src/mpc.py`：MIT Cheetah 3 凸 MPC（SRBD 13 维状态 [θ,p,ω,v,g]）。
  - `ConvexMPC`：绕当前 yaw 线性化的连续 A/B → 一阶离散 Ad/Bd → 稠密 condense（决策变量只留各腿反力 U）；
    代价 = 状态跟踪 Q + 力正则 R；约束 = 每足每步线性化摩擦锥（金字塔 4 面）+ 单边法向力 [f_min,f_max]，
    swing 足 f_max=0 强制反力为 0。OSQP 求解，sparsity 不变时走 `update()` 热启动。
  - `make_reference(...)`：由速度指令积分出 (N,13) 期望轨迹（躯干保持水平、指定高度）。
  - `composite_inertia_from_model(...)`：从 MuJoCo 在 home 位形算全身质量/CoM/复合转动惯量（平行轴），
    避免只用 trunk 惯量的粗糙近似。实测 mass=15.206kg，I_diag≈[0.170,0.484,0.535] kg·m²。
- self-test（`python -m src.mpc`）11/11 PASS。

**结果**（真实跑出）
- 站立（4 足）：每足法向力 38.5–38.9N，Σfz=154.9N ≈ m·g(149.2N)，摩擦锥满足，左右平衡。
- trot（FL+RR 支撑）：2 足各 ~76N 撑起 m·g，摆动足反力 <1e-2 N。
- 前进指令 0.6m/s：净前向力 +59N（方向正确，用于加速）。
- 热启动：复用同一 OSQP 实例，单次求解 ~0.8ms（40Hz 实时余量充足）。

**下一步**：Phase 2 步骤 ③ `src/wbc.py`（关节力矩 QP：决策 [qddot,f,tau]，`mj_fullM`/`mj_jac`/`qfrc_bias`，
浮动基 EoM 等式约束 + 摩擦锥/力矩限幅，跟踪 MPC 反力 + base 姿态 + 摆动足加速度）。

### 2026-09-16 Phase 2① - 步态调度 src/gait.py 完成

**做了什么**
- 新会话恢复：pod 重启后 pip/apt 依赖丢失，`bash scripts/setup_env.sh` 重装（mujoco 3.13.0 / osqp 1.1.3），
  `python scripts/stand_test.py` 复跑 RESULT: PASS，git log 与本文件一致（3 commits，无需更正）。
- `src/gait.py`（纯 numpy，controller-agnostic）：
  - `GaitScheduler`：归一化周期时钟 + 每腿 offset/duty 拆 stance/swing 相位；含 trot/stand/walk/pace/bound 预设；
    `eval(t)` 出接触旗标+分段相位，`contact_sequence(t,H,dt)` 给 MPC 预测时域出 (H,4) 接触表。
  - `raibert_foothold(...)`：MIT Cheetah 3 落脚点 = hip + (T_stance/2)·v_actual + √(h/g)·(v−v_cmd)（neutral point + capture point）。
  - `swing_foot_reference(...)`：水平摆线插值 + 竖直抬腿 raised-cosine bell，返回 pos/vel/acc（供 WBC 摆动足加速度跟踪）。
- self-test（`python src/gait.py`）13 项断言全 PASS，出图 `results/gait_schedule.png`。

**结果**（真实跑出）
- trot 对角腿同相（FL==RR、FR==RL），反相对角，占空比每腿 0.500，任意时刻恰好 2 足触地。
- 摆动轨迹端点严格落在 lift-off/touchdown、端点竖直速度=0、中点抬腿高度=step_height（0.08m 实测 0.0800）。
- Raibert：静止零指令时落脚点在 hip 正下方；稳态前进 v=v_cmd=0.5m/s 时落脚点前探 T/2·v=0.0625m；
  v<v_cmd 时落脚点后移（capture 反馈方向正确）。
- 修正过程记录：初版 neutral point 误用 v_cmd，改为 MIT 标准的 actual velocity，并把测试断言从"前进指令即前探"
  纠正为物理正确的"稳态才前探、加速阶段后移"。

**下一步**：Phase 2 步骤 ② `src/mpc.py`（SRBD 13 维凸 MPC，OSQP 稀疏求解，质量/惯量从 MuJoCo 读）。

### 2026-09-15 Phase 2 - 开源实现选型完成，定方案 A（参考架构自实现）

**做了什么**
- 按 PLAN 要求，开工前做了 Convex MPC/WBC 开源实现的网络调研，评估 6+ 仓库（见上方"待决策（已解决）"表）。
- 结论并经用户确认：走**方案 A**——以 elijah/go2-convex-mpc（MPC 层）+ benaziel/quadruped_wbc（WBC 层）为参考，
  在自有 `sim_env` 上用 OSQP 自实现两层，动力学量走 MuJoCo `mj_fullM`/`mj_jac`，免重依赖。
- 补充可复现性：新增 `scripts/setup_env.sh`（一键装 apt GL + pip 依赖并自检），PROGRESS 增加"重开 Pod 恢复步骤"，
  写下 Phase 2 四步实现计划，便于新会话/重启 pod 后无缝衔接。

**下一步**：从 Phase 2 实现计划 ① `src/gait.py` 开始编码。

### 2026-09-15 Phase 1 - MuJoCo + Go2 环境跑通，站立 smoke test 通过

**做了什么**
- 环境：安装 mujoco 3.13.0 / osqp 1.1.3 / numpy / scipy / matplotlib / imageio[ffmpeg]（见 `requirements.txt`）。
  headless 渲染用软件 OSMesa 后端（系统装 libosmesa6 + libgl1-mesa-dri；EGL 在本 Pod 上 `/dev/dri` 权限受限，
  上下文销毁时报错，故默认走 osmesa，`sim_env.py` 里 `MUJOCO_GL` 已 setdefault）。
- 模型：选用 Unitree **Go2**（menagerie 适配最新、社区参考实现最多）。把 `unitree_go2` 模型 vendored 进
  `assets/unitree_go2/`（BSD-3，含 LICENSE），保证仓库自洽可复现。给每条腿 calf 加了 `*_foot` site，
  并新建 `assets/unitree_go2/go2_scene.xml`：include 原模型 + 地面 + `<sensor>` 块
  （IMU: framequat/gyro/accelerometer；base 位姿/线速度/角速度；4×足端 touch 力 + framepos）。
- 代码：`src/sim_env.py` —— `Go2Sim` 类，torque 控制接口。
  - 12 力矩下发（按 actuator ctrlrange 限幅，支持 control_dt 控制降频 / 多 substep）。
  - 状态读取 `RobotState`：base 位姿(quat+rpy)/线角速度、12 关节 pos/vel、4 足端接触 bool/法向力/世界坐标、IMU。
  - `apply_external_wrench`（Phase 3 推力扰动预留）、offscreen `render`、四元数工具函数。
  - 约定：腿序 [FL,FR,RL,RR]，每腿 [hip,thigh,calf]，四元数 scalar-first (w,x,y,z)。
- 验证：`scripts/stand_test.py` 关节空间 PD 保持 home 位姿。

**结果**（`python scripts/stand_test.py`，3s，control_dt=2ms=500Hz）
- 站立稳定：settled base 高度 ~25.6 cm，max|roll| 0.0°、max|pitch| 0.58°，4/4 足端持续接触，峰值力矩 7.1 Nm
  （远低于 ±23.7/±45.4 限幅）。RESULT: PASS。
- `results/stand_test.mp4` 渲染正常（非黑帧，mean≈84）。
- nq=19 / nv=18 / nu=12（12 个直驱 torque motor），传感器 14 项全部就位。

**下一步**：进入 Phase 2，先做开源实现选型调研（见"待决策"），交用户确认后再写 MPC/WBC。
