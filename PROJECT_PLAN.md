# 项目技术方案：四足机器人全身运动控制（Convex MPC + WBC）复现

对应简历项目二目标：命中"运动控制算法工程师"JD关键词，产出可量化的对比实验结果。

## 背景与动机

简历项目一（SU2 伴随法气动优化）证明了"高维约束优化 + 梯度求解"的工程能力，这套方法论与 MPC 每步在线求解约束 QP 的核心逻辑同源。本项目的目的是把这个能力**落地到机器人运动控制的具体场景**：复现 MIT Cheetah 一脉的 Convex MPC（足端反力规划）+ WBC（全身关节力矩分配）分层控制架构，在 MuJoCo 中跑通四足机器人的稳定行走，并做跨地形/抗扰动的定量对比实验。

参考文献：Di Carlo et al. 2018 "Dynamic Locomotion in the MIT Cheetah 3"；Kim et al. 2019 "Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control"。

## Phase 1：MuJoCo 环境 + 基线模型（预计 2-3 天）

> **状态：✅ 已完成（2026-09-15）**。选用 Unitree Go2，模型 vendored 进 `assets/unitree_go2/`；
> `src/sim_env.py`（`Go2Sim`）封装完成，`scripts/stand_test.py` PD 站立测试 PASS。详见 `PROGRESS.md`。

- 机器人模型：用 `mujoco_menagerie` 里的 Unitree Go1/Go2 或 A1 MJCF 模型（已经过 MuJoCo 官方适配，省去建模时间）。
- 目标：跑通开环/简单站立控制，确认关节名、执行器、传感器（IMU、足端接触）读写正常。
- 交付物：`src/sim_env.py`（封装 MuJoCo 环境，暴露状态读取/力矩下发接口）。

## Phase 2：Convex MPC + WBC 控制器实现（预计 1-1.5 周）

> **状态：🚧 进行中**。选型已定（2026-09-15，用户确认）：**方案 A —— 参考架构自实现，MuJoCo 原生**。
> 以 [elijah-waichong-chan/go2-convex-mpc](https://github.com/elijah-waichong-chan/go2-convex-mpc)（MPC 层，MIT）
> 与 [benaziel/quadruped_wbc](https://github.com/benaziel/quadruped_wbc)（WBC 层）为参考，两层都用 OSQP 自实现，
> 动力学量走 MuJoCo `mj_fullM`/`mj_jac`，免掉 Pinocchio/CasADi/DDS。候选评估与四步实现计划见 `PROGRESS.md`。
> 尚未开始写控制器代码。

- **Convex MPC 层**：简化刚体动力学模型（Single Rigid Body Dynamics），在时域窗口内求解足端反力，QP 用 `osqp` 或 `qpOASES` 求解，状态量：base 位置/姿态/线速度/角速度（13维线性化模型，标准 MIT Cheetah 公式）。
- **WBC 层**：给定 MPC 输出的期望反力和摆动腿轨迹，求解全身关节力矩的二次规划（QP），处理接触约束、力矩限幅、摩擦锥约束。
- 步态：先做 trot（对角小跑），跑通后可选加 bound/pace 做对比。
- 判断优先用现成开源实现做参考/改造而非从零手写（节省时间、降低出结果周期）——**在开工时用网络搜索确认当前可用、维护良好的 MuJoCo 原生或易移植的 Convex MPC/WBC 开源仓库**（例如面向 Unitree A1/Go1/Go2 的控制器实现），评估后决定是"移植复现"还是"参考架构自己实现"，并在 PROGRESS.md 里记录选型理由。
- 交付物：`src/mpc.py`、`src/wbc.py`、能在平地上稳定 trot 行走的 demo。

## Phase 3：定量对比实验（预计 3-4 天）

设计对照实验，量化控制器鲁棒性，产出简历里承诺的具体数字：

| 维度 | 具体设置 |
|---|---|
| 地形 | 平地 / 随机粗糙地形（高度场扰动） / 斜坡（至少 3 种，可加台阶） |
| 外部扰动 | 侧向/纵向推力冲量，幅度分级（如 5N·s / 10N·s / 15N·s） |
| 对比基线 | 开环固定步态（无反馈力矩调节）作为 baseline |
| 指标 | base 姿态跟踪误差（roll/pitch，度或等效位移 cm）、行走成功率（N 次 trial 里保持不摔倒的比例）、恢复时间 |

每组设置跑 ≥10 次 trial（不同随机种子/扰动时刻），产出误差分布图和成功率对比表，存入 `results/`。

## Phase 4：Isaac Lab 迁移 + RL 对比（可选加分项，预计 3-5 天）

- 用 Isaac Lab 自带的 legged locomotion 任务（Go2/ANYmal），跑通官方 RL baseline（PPO, rsl_rl）训练一版策略。
- 用 Phase 3 同样的地形/扰动/指标体系评测这个 RL 策略，和 Phase 2-3 的经典 MPC+WBC 控制器做**同一套指标下的横向对比**（哪个抗扰动更强、哪个在未见过地形上泛化更好）。
- 这一步把简历里"模仿学习策略评测方法论"（项目三）和运动控制项目连起来，形成经典控制 vs 学习方法的完整故事线，比单纯复现更有区分度。
- 交付物：`experiments/isaac_lab_rl_baseline/`、跨方法对比表。

## Phase 5：收尾（1-2 天）

- 整理 `results/` 里的图表和至少一段行走视频/GIF。
- 更新仓库 README 的"复现结果摘要"表格，填真实数字。
- 用真实数字替换简历项目二的模板文本：

  ```
  基于MuJoCo复现四足机器人全身运动控制（Convex MPC + WBC）框架，在
  [N]种地形与外部扰动场景下实现稳定行走，姿态跟踪误差控制在[X]cm以内，
  相比开环控制基线成功率提升[X]pp，验证了MPC求解器在实时约束优化场景
  下的工程可行性
  ```

  如果做了 Phase 4，可以再加一句 RL 对比结论。

## 时间/成本预算参考

- 纯 GPU 计算需求其实不大（MuJoCo CPU 仿真为主，Isaac Lab 阶段才真正吃 GPU），4090 主要是为了 Phase 4 的并行 RL 训练和后续可能的视觉/感知扩展预留余量。
- 建议：Phase 1-3 完全可以先在自己电脑或便宜 CPU 实例上做，**Phase 4 快到再租 4090**，能省下不少按小时计费的成本。如果已经在用 4090 跑 Phase 1-3 也没问题，只是没必要。
