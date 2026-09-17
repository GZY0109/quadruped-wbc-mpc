# Quadruped WBC/MPC Reproduction

四足机器人全身运动控制（Whole-Body Control）复现项目：先在 MuJoCo 中复现经典 Convex MPC + WBC 控制器（MIT Cheetah 一脉），做定量对比实验；再迁移到 Isaac Lab，补一组classical control vs. learned RL policy 的鲁棒性对比。

对应求职方向：运动控制算法工程师 / 规控算法工程师。完整技术方案见 [`PROJECT_PLAN.md`](./PROJECT_PLAN.md)，实时进度见 [`PROGRESS.md`](./PROGRESS.md)。

## 结构

```
assets/unitree_go2/   Go2 MJCF 模型（vendored from mujoco_menagerie, BSD-3）
                      + go2_scene.xml（自定义场景：地面 + IMU/足端传感器）
src/                  控制器与仿真代码（sim_env.py: 仿真环境封装）
scripts/              可运行脚本（stand_test.py: Phase 1 站立 smoke test）
experiments/          实验脚本与配置
results/              实验数据、图表、视频
docs/                 补充文档
```

## 环境

- Phase 1-3: Python + MuJoCo + OSQP（Convex MPC 求解），MuJoCo CPU 仿真为主
- Phase 4: Isaac Lab（GPU 并行仿真 + RL baseline）

安装：

```bash
# 系统 GL（headless 渲染，软件 OSMesa 后端）
sudo apt-get install -y libosmesa6 libgl1-mesa-dri
pip install -r requirements.txt
```

快速自测（关节 PD 站立，验证仿真/控制闭环）：

```bash
python scripts/stand_test.py           # 无头，打印站立稳定性报告
python scripts/stand_test.py --video   # 另存 results/stand_test.mp4
```

> 无头渲染默认走 OSMesa（`sim_env.py` 里 `MUJOCO_GL` 默认设为 `osmesa`），
> 无需手动配置；有显示环境可自行 `export MUJOCO_GL=egl/glfw` 覆盖。

## 复现结果摘要

**项目定位（Phase 2④ 之后确定）**：这个项目最终交付的不是"稳定行走的四足机器人"，而是**任务优先级分层
QP 相对单层加权和 QP 在抗扰动鲁棒性上的量化对比**——两种 WBC 架构在同一套 MuJoCo+Go2 仿真、同一套统计方法论
（多种子扰动 trial，而非单次确定性跑测）下的正面交锋。三个步态场景（原地踏步 walk / trot / 前进 walk）在
关节噪声扰动下最终都会摔倒，这是如实报告的负结果；但分层QP在几乎所有测试条件下都显著更鲁棒，这个方向性
结论贯穿了扰动幅度扫描和地形对比两组独立实验，不是单点巧合。

**站立场景**（stand，四足全程支撑，零扰动）：完全稳定，漂移为 0，是控制器唯一有真实稳定裕度的场景——也是
Phase 3 地形对比实验的基础。

**Phase 2④ 突破 + Phase 3(A) 扰动幅度扫描**（`scripts/disturbance_sweep.py`，160 次跑测，5 种子/点）：
单层QP在几乎整个扰动幅度网格（0~0.08 rad）上都是 100% 摔倒；分层QP在同样网格上，存活**时间**（而非摔倒率）
稳定高于单层QP——三个步态场景的均值提升比例分别是原地walk **3.22x**、trot **3.05x**、前进walk **2.15x**。
摔倒率曲线本身大多饱和在 100%、非单调（跟已知的"边际稳定系统对数值噪声混沌敏感"现象一致），不是一条干净的
S 型曲线；trot 场景是例外也是最干净的单点结果——零噪声下分层QP **0% 摔倒**（撑满 5s）vs 单层QP **100% 摔倒**
（1.36s）。诚实的反例：前进walk 在低扰动幅度（≤0.02rad）区间分层QP反而比单层QP更差，只有幅度≥0.04才转为
明显优势——改进不是无条件的。图表 `results/disturbance_sweep.png`，原始数据 `results/disturbance_sweep.json`。

**Phase 3(B) 地形对比**（`scripts/terrain_eval.py`，站立平衡场景，5 种子/配置，扰动幅度 0.15rad 专门针对
站立场景校准）：

| 地形 | 单层QP | 分层QP |
|---|---|---|
| 平地 | 1.85±1.65s，80%摔倒 | 4.09±1.13s，40%摔倒 |
| 5°斜坡 | 0.94±0.19s，100%摔倒 | 1.92±0.85s，100%摔倒 |
| 粗糙地形（~3cm起伏） | 2.13±1.70s，80%摔倒 | 3.24±2.17s，40%摔倒 |

控制器（`src/mpc.py`/`src/wbc.py`）的摩擦锥/落脚点逻辑全程假设水平地面法向量，代码未做任何地形适配——粗糙
地形（对称起伏）几乎不影响两种QP的表现（跟平地在统计误差内难以区分），但哪怕只有 5° 的**斜坡**（方向性倾斜）
就让两种QP的摔倒率都冲到 100%、存活时间腰斩，这个反差直接量化了"假设平地"这个建模简化在真实非平地形上的
代价。分层QP在三种地形上都保持对单层QP的优势（存活时间提升 1.5x~2.2x），改进方向可以跨地形复现。
图表 `results/terrain_eval.png`，代表性视频 `results/terrain_{flat,slope_5deg,rough}_representative.mp4`。

**Phase 4 RL baseline 对比**（Isaac Lab + rsl_rl PPO，Go2 rough-terrain 官方默认配置，1500 iterations /
4096 并行环境，3090 上实测 24 分钟收敛，`experiments/isaac_lab_rl_baseline/`）：把 Phase 3 的评测协议
搬到 RL 策略上，发现两种方法对"扰动"的敏感通道完全不同，不是简单的谁更鲁棒。**关节噪声幅度扫描**（跟
Phase 3(A) 同样的网格，0~0.08rad，甚至推到 1.0rad 这种物理上离谱的量级）对 RL 策略是退化轴——摔倒率
全程 0%（已用直接读关节角的方式确认扰动确实生效，不是噪声事件失效）：RL 策略输出关节位置目标，每
20ms 控制周期内的隐式高增益 PD 就能把关节纠偏，跟经典分层QP每拍要解一次力矩 QP 的恢复速度不是一个
量级。换成**持续施加的斜坡角度扫描**（5°~训练课程上限 22.9°）才拿到有意义的断点：

| 斜坡角度 | 存活时间 | 摔倒率 |
|---|---|---|
| 5° / 10° / 15° | 10.0±0.0s（撑满） | 0% |
| 20° | 8.59±2.82s | 20% |
| 22.9°（训练课程上限） | 5.36±2.52s | 80% |

RL 策略在训练分布内（≤22.9°）基本稳，越接近分布边缘才开始退化；经典分层QP在仅 5° 斜坡下已经 100%
摔倒。两者鲁棒性的机制不同（RL 怕分布外的持续性地形倾斜，经典QP怕小幅度瞬态关节扰动），不是"RL全面
更强"或"经典控制没用"这种简化结论。原始数据 `results/isaac_lab_rl_eval.json` /
`results/isaac_lab_rl_slope_sweep.json`。
