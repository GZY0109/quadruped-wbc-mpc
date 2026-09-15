# 进度追踪

按 `PROJECT_PLAN.md` 的 Phase 顺序记录。每完成一个里程碑就在这里加一条，commit 时带上这次改动对应的记录。

## 使用方式（给 Claude 自己看）

- 每次开始工作前，先读这份文件的"当前状态"和最近几条记录，再看 `git log --oneline -20` 确认实际代码进度和这里记的一致。
- 每完成一个可验证的里程碑（环境跑通/控制器能站稳/一组实验跑完/遇到卡点需要下次继续），追加一条记录，然后 commit + push。
- 遇到需要用户决策的问题（比如某个开源仓库能不能用、是否要换机器人模型），记录在"待决策"里，不要自己瞎猜决定重大方向。

---

## 当前状态

`Phase 1 - 已完成`。MuJoCo + Go2 环境跑通，`src/sim_env.py` 封装完成并通过站立 smoke test。
下一步进入 **Phase 2**：开工前先按 PROJECT_PLAN 要求做开源 Convex MPC/WBC 实现的网络调研与选型，
把候选和理由写进下面的"待决策"，再动手写 `src/mpc.py` / `src/wbc.py`。

## 待决策

- **[Phase 2 开工前] Convex MPC + WBC 参考实现选型**：尚未调研。进入 Phase 2 第一件事是网络搜索
  当前维护良好、MuJoCo 原生或易移植的 Go2/A1 Convex MPC + WBC 开源仓库，评估"移植复现 vs 参考架构自写"，
  列候选 + 理由到这里，交用户确认后再动手。

## 记录

<!-- 格式：### YYYY-MM-DD Phase X - 一句话摘要 \n 具体做了什么、结果如何、下一步是什么 -->

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
