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

<!-- Phase 3 完成后填写：不同地形/扰动下的姿态跟踪误差、成功率对比表 -->
