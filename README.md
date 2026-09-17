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

<!-- Phase 3 完成后填写完整版：不同地形/扰动下的姿态跟踪误差、成功率对比表 -->

**Phase 2④ 阶段性结果**（多次扰动统计，非单次跑测；方法论见 `PROGRESS.md`）：站立场景（stand，四足全程支撑）
稳定，漂移为 0。行走场景（walk/trot，涉及腿部切换）目前尚未达到稳定：把 WBC 从单一加权和 QP 改成任务优先级
分层 QP 后，原地踏步（walk）存活时间从 0.87±0.10s 提升到 2.69±1.31s（5 个初始扰动种子，两组分布几乎不重叠，
改进有统计意义），trot、有前进速度的 walk 也在同一量级（2.6~3.0s）——但三个场景在扰动下 100% 会摔，还不是
一个可以宣称"稳定行走"的结果。图表 `results/hierarchical_vs_flat.png`，代表性视频（取存活时间最接近均值的
一次试验，非巧合的最好结果）`results/walk_hierarchical_representative.mp4`。
