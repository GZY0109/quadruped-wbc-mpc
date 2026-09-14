# Quadruped WBC/MPC Reproduction

四足机器人全身运动控制（Whole-Body Control）复现项目：先在 MuJoCo 中复现经典 Convex MPC + WBC 控制器（MIT Cheetah 一脉），做定量对比实验；再迁移到 Isaac Lab，补一组classical control vs. learned RL policy 的鲁棒性对比。

对应求职方向：运动控制算法工程师 / 规控算法工程师。完整技术方案见 [`PROJECT_PLAN.md`](./PROJECT_PLAN.md)，实时进度见 [`PROGRESS.md`](./PROGRESS.md)。

## 结构

```
src/           控制器与仿真代码
experiments/   实验脚本与配置
results/       实验数据、图表、视频
docs/          补充文档
```

## 环境

- Phase 1: Python + MuJoCo + OSQP（Convex MPC 求解）
- Phase 2: Isaac Lab（GPU 并行仿真 + RL baseline）

## 复现结果摘要

<!-- Claude 在 Phase 3 完成后填写：不同地形/扰动下的姿态跟踪误差、成功率对比表 -->
