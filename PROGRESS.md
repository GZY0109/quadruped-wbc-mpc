# 进度追踪

按 `PROJECT_PLAN.md` 的 Phase 顺序记录。每完成一个里程碑就在这里加一条，commit 时带上这次改动对应的记录。

## 使用方式（给 Claude 自己看）

- 每次开始工作前，先读这份文件的"当前状态"和最近几条记录，再看 `git log --oneline -20` 确认实际代码进度和这里记的一致。
- 每完成一个可验证的里程碑（环境跑通/控制器能站稳/一组实验跑完/遇到卡点需要下次继续），追加一条记录，然后 commit + push。
- 遇到需要用户决策的问题（比如某个开源仓库能不能用、是否要换机器人模型），记录在"待决策"里，不要自己瞎猜决定重大方向。

---

## 当前状态

`Phase 2 - 进行中（①②③ 完成；④ 集成回路搭好但 trot 尚未稳定，见下方诊断）`

- Phase 1 ✅ 完成：MuJoCo + Go2 环境跑通，`src/sim_env.py`。
- Phase 2 选型 ✅ 方案 A（参考架构自实现，MuJoCo 原生，两层 OSQP）。
- Phase 2 ① ✅ `src/gait.py`；② ✅ `src/mpc.py`；③ ✅ `src/wbc.py`——三层各自单元测试全 PASS。
- Phase 2 ④ 🚧 `scripts/trot_demo.py`：gait+MPC(40Hz)+WBC(500Hz)+摆动腿闭环已搭好并能跑，
  **4 足站立稳**，但**平地 trot 尚不能持续**——每个"2 对角腿支撑"相里躯干缓慢翻滚，多周期累积后 ~1–1.4s 摔倒。
  已定位根因并修掉数个子问题（详见记录），仍缺对"2 对角支撑不稳定模态"的有效镇定。**待与用户确认下一步方向。**

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

- **[待决策 - Phase 2④ trot 镇定方向]** 平地 trot 稳不住（根因见记录：反应式单 QP WBC 镇不住 2 对角支撑不稳定模态）。
  三个候选方向 (a) MPC 主镇定 + WBC 跟力 / (b) 任务优先级 WBC / (c) 降低交付目标。**需用户拍板后再继续，不自行决定大方向。**


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
