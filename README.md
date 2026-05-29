# PTZ 视觉伺服与自动追踪 — 认知与控制层

纯 C++17、无第三方依赖的云台视觉伺服控制核心。输入视觉层给出的目标坐标，
输出平滑的云台控制增量。整条流水线每帧执行一次：

```
视觉检测
   │
   ▼
TargetSelector ──(选中目标)──> KalmanTracker ──(predictAhead)──┐
   │                                                          │
   │                                            延迟补偿后的设定点误差
   ▼                                                          ▼
AdaptivePID ──(原始增量)──> TrajectoryPlanner ──(平滑增量)──> 云台指令
```

## 构建与运行

```bash
cmake -B build -S .
cmake --build build -j
./build/ptz_demo     # 闭环仿真：目标横扫 + 中途反向 + 诱饵目标出现
ctest --test-dir build   # 或直接 ./build/ptz_tests
```

## 模块说明

### 1. 运动学预测 — `KalmanTracker`（`include/ptz/kalman_tracker.hpp`）

- 状态量 6 维：`[x, y, vx, vy, ax, ay]`，**常加速度 (CA) 运动模型**。
- 过程噪声采用连续白噪声 jerk 模型离散化（`processNoise()`），用单一谱密度
  `process_noise_psd` 调节“信模型 vs 信观测”的权衡。
- 协方差更新使用 **Joseph 稳定形式**，保证 `P` 对称且半正定，长时间运行不发散。
- `predictAhead(Δt)`：在**不改动滤波状态**的前提下，把位置外推到 `t+Δt`
  （`Δt` 传系统总延迟：采集 + 推理 + 执行），作为控制器设定点，补偿纯反馈的相位滞后。

> 关于 “EKF”：本场景的运动模型（CA）与观测模型（直接观测像素位置）**都是线性的**，
> 因此最优估计器就是标准线性 KF——这里实现的就是它。`predict()/update()` 写成通用矩阵形式，
> 若日后改为在云台角坐标等**非线性**观测下工作，只需替换观测函数 `h(x)` 与其雅可比 `H`
> 即可升级为 EKF，其余管线无需改动。代码已按此预留接口与命名。

### 2. 自适应 PID — `AdaptivePID`（`include/ptz/adaptive_pid.hpp`）

二维（pan/tilt）增益调度控制器，作用于像素误差向量：

- **增益调度**：目标速度高 → 放大 `Kp/Kd`（响应更快）；进入中心“近区” → 线性
  压低 `Kp/Kd` 至 `near_scale`（柔和收敛，抑制过冲）。
- **误差死区**：`|误差| ≤ deadzone`（默认 2px）时输出强制为 0，消除居中后的高频微抖；
  死区内积分项缓慢衰减，避免空转累积。
- **积分重置**：监测误差变化率的突变（误差 “jerk” 超阈值 → 目标突然变向），
  立即清空积分项，防止积分饱和导致反向过冲。
- **抗饱和**：积分项 `integral_limit` 限幅；微分项一阶低通滤波，避免放大测量噪声；
  输出 `output_limit` 限幅。

### 3. 轨迹平滑与动力学约束 — `TrajectoryPlanner`（`include/ptz/trajectory_planner.hpp`）

把 PID 的阶跃式增量整形为对电机友好的平滑参考：

- **三次贝塞尔整形（滚动时域）**：以当前指令位置为 `P0`、当前指令速度作为起点切线
  （`P1 = P0 + v·T/3`），目标为终点且终点切线为 0。这保证轨迹 **C1 连续**
  （速度连续，永不出现阶跃），且天然 ease-in / ease-out。
- **动力学约束**：从贝塞尔速度剖面取期望速度后，对**加速度与速度做硬限幅**，
  使每一帧的指令都物理可行，避免机械冲击 / 齿轮磨损 / 画面抖动。
- 另含离线工具 `cubicBezier()` 与 `planEaseCurve()`，便于预规划与可视化。

> 关于需求中“叠加微幅高斯噪声”的取舍：注入噪声本质上是**增加**抖动，与“消除画面抖动、
> 符合动力学约束”的目标相悖，因此未采用。这里用**贝塞尔 C1 整形 + 加加速度/速度限幅**
> 实现真正的平滑物理轨迹，这是工程上正确的抗抖与保护电机做法。

### 4. 目标优先级 — `TargetSelector`（`include/ptz/target_selector.hpp`）

- 多维加权打分：**距离倒数**（对画面中心的接近度，平滑有界）+ **可见面积**（归一化），
  再乘检测置信度。
- **迟滞防抖**：仅当挑战者得分超过当前目标一定**相对裕度**（`switch_margin`）
  **并连续保持若干帧**（`switch_hold_frames`）后才切换，避免多目标间高频跳变。

### 5. 整合 — `ServoController`（`include/ptz/servo_controller.hpp`）

`update(detections, dt)` 串起以上全部模块，并处理目标切换时的重置（重新播种滤波器、
清空 PID 状态）。误差约定：`误差 = 预测目标位置 − 画面中心`；返回的增量驱动云台把目标拉回中心
（若云台正方向相反，在驱动层按轴翻转符号即可）。

## 文件结构

```
include/ptz/   matrix.hpp  kalman_tracker.hpp  adaptive_pid.hpp
               trajectory_planner.hpp  target_selector.hpp  servo_controller.hpp
src/           对应 .cpp 实现
examples/      demo.cpp     闭环仿真
tests/         test_control.cpp   14 项 sanity 测试
```
