# LQR ROS2 Gazebo AUV Control

## 项目简介

本项目基于 **ROS2 Humble、Gazebo 和 C++** 实现 AUV 深度控制，并在此前 PID 控制项目的基础上进一步学习和实现 LQR（Linear Quadratic Regulator）控制。

项目中根据 AUV 的状态空间模型设计 LQR 控制器，以深度误差和垂向速度作为状态，通过状态反馈计算控制输出，并将控制力施加到 Gazebo 中的 AUV 模型，实现完整的受力闭环控制。

主要完成内容：

- 使用 C++ 实现 LQR 控制器
- 建立状态空间模型
- 使用深度误差和垂向速度作为反馈状态
- 基于 Q、R 权重设计 LQR 状态反馈
- Gazebo 真实深度与垂向速度反馈
- 控制输出转换为 Gazebo 外力
- 加入控制力限幅
- 加入控制输出变化率限制
- 加入深度和速度测量噪声
- 使用低通滤波处理测量信号
- CSV 数据记录
- Python 实验结果绘图
- 与 PID、MPC 控制结果进行统一对比

---

## 控制结构

```text
Target Depth
     ↓
Depth Error
     ↓
┌─────────────────┐
│  LQR Controller │ ← Vertical Velocity
└─────────────────┘
     ↓
Control Output
     ↓
Gazebo Force Node
     ↓
AUV
     ↓
Depth / Velocity Feedback
     └──────────────→ LQR
```

LQR 使用的状态为：

```text
x = [depth error, vertical velocity]ᵀ
```

控制律：

```text
u = -Kx
```

与 PID 主要根据误差进行反馈不同，LQR 同时考虑当前位置偏差和运动速度。

当 AUV 还没有到达目标深度，但是已经具有较大的下潜速度时，速度状态可以使控制器提前减小控制力甚至产生反向控制，从而降低超调。

---

# 控制输出限制

为了使控制结果更符合实际执行器特点，在 LQR 输出中加入了控制力和控制变化率限制。

```text
Control Force:

-5 N ≤ u ≤ 5 N
```

控制变化率：

```text
3 N/s
```

控制周期：

```text
dt = 0.1 s
```

因此每个控制周期允许的最大变化量为：

```text
|Δu| ≤ 0.3 N/step
```

LQR 控制代码中实际进行了输出限幅和变化率限制。:contentReference[oaicite:1]{index=1} :contentReference[oaicite:2]{index=2}

---

# 噪声与滤波

为了避免直接使用完全理想的 Gazebo 状态，在反馈过程中加入了测量噪声。

```text
Depth Noise:
±0.03 m

Velocity Noise:
±0.01 m/s
```

并对深度和速度测量值进行低通滤波。

LQR 实际控制时使用滤波后的：

```text
filtered_depth
filtered_velocity
```

而 CSV 中同时保存真实值、测量值和滤波值，方便观察噪声和滤波效果。:contentReference[oaicite:3]{index=3}

---

# 实验内容

## 1. LQR 深度跟踪实验

首先使用固定目标深度进行 LQR 闭环控制实验。

控制器根据：

```text
Depth Error
Vertical Velocity
```

实时计算控制输出，并通过 Gazebo Force Node 对 AUV 施加实际外力。

实验过程中主要观察：

```text
Target Depth
True Depth
Measured Depth
Filtered Depth
Depth Error
Vertical Velocity
LQR Control Output
Control Increment
```

AUV 能够在 Gazebo 受力环境中逐渐到达目标深度，并稳定在目标深度附近。

---

## 2. 噪声与低通滤波实验

在 Gazebo 真实状态基础上加入测量噪声，并记录：

```text
True Depth
Measured Depth
Filtered Depth

True Velocity
Measured Velocity
Filtered Velocity
```

通过曲线可以观察到测量值围绕真实状态产生随机波动，而经过低通滤波后反馈信号更加平滑。

对应实验数据保存为：

```text
auv_lqr_noise_10m.csv
```

LQR 控制节点默认目标深度为 10 m，并使用该 CSV 文件记录实验结果。:contentReference[oaicite:4]{index=4}

实验综合图可以放在这里：

```markdown
![LQR Noise and Filter](你的LQR综合图路径.png)
```

---

## 3. PID / LQR / MPC 对比实验

完成 LQR 后，将 PID、LQR 和 MPC 放在相同的 Gazebo 受力环境和相同控制约束下进行对比。

统一比较时间为：

```text
0 ~ 75.2 s
```

主要结果：

| 指标 | PID | LQR | MPC |
|---|---:|---:|---:|
| Rise Time / s | 未达到 | 18.6 | 17.5 |
| First Target / s | 未达到 | 33.6 | 26.7 |
| Settling Time / s | 未达到 | 29.5 | 34.0 |
| Maximum Depth / m | 17.749 | 20.072 | 20.584 |
| Overshoot | — | 0.358% | 2.921% |
| Final Error / m | 2.251 | 0.007 | 0.002 |
| RMSE / m | 10.209 | 7.667 | 7.635 |
| Maximum Control / N | 3.589 | 5.000 | 5.000 |
| Maximum Δu / N | 0.300 | 0.300 | 0.300 |
| Control Energy | 159.45 | 468.83 | 524.45 |

在当前参数条件下：

```text
PID
控制结构简单，但在统一实验时间内没有到达目标深度。

LQR
超调较小，状态反馈效果稳定，在响应速度和控制平稳性之间取得较好的平衡。

MPC
目标跟踪速度较快，同时能够直接将控制约束加入优化问题。
```

通过这一组实验，可以更加直观地理解三种控制算法在相同仿真环境下的区别。

对比图可以放在：

```markdown
![PID LQR MPC Comparison](你的三算法对比图路径.png)
```

---

# 主要程序

```text
lqr_controller_node.cpp
    LQR控制器
    状态反馈
    控制输出计算
    输出限幅
    输出变化率限制
    CSV数据记录

force_depth_feedback_node.cpp
    获取Gazebo真实深度
    获取垂向速度
    加入测量噪声
    进行低通滤波

gazebo_force_node.cpp
    接收LQR控制输出
    加入垂向阻尼
    将控制输出转换为Gazebo外力

simple_auv
    Gazebo中的AUV模型
```

ROS2 中主要数据关系：

```text
/target_depth
        ↓
lqr_controller_node

/auv/current_trueposition ──→
/auv/vertical_velocity   ───→ LQR

LQR
 ↓
/auv/output
 ↓
gazebo_force_node
 ↓
Gazebo
```

项目中深度采用：

```text
depth = -Gazebo z
```

因此深度增加表示 AUV 向下运动，最终由 `gazebo_force_node` 完成控制方向与 Gazebo 坐标方向之间的转换。:contentReference[oaicite:5]{index=5}

---

# 编译与运行

进入 LQR 工作空间：

```bash
cd ~/lqr_ros_gazebo_auvcontrol
```

编译：

```bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Gazebo 模型可以通过：

```bash
ros2 launch auv_gazebo simple_auv.launch.py
```

启动。该 `simple_auv.launch.py` 用于加载 Gazebo 中的 `simple_auv` 模型。:contentReference[oaicite:6]{index=6}

完整 LQR 实验建议使用项目中已经配置好的 LQR Launch 文件启动。

> 这里不要随便改 Launch 文件名。GitHub README 中应填写你项目 `launch/` 文件夹里的真实文件名。

---

# 数据记录与结果分析

LQR 控制实验通过 CSV 保存数据。

其中噪声实验使用：

```text
auv_lqr_noise_10m.csv
```

控制节点能够订阅真实深度、测量深度和滤波深度，并记录实验状态。:contentReference[oaicite:7]{index=7}

实验完成后使用 Python 对数据进行处理，主要绘制：

```text
Depth Response
Noise and Filtering
Depth Error
Vertical Velocity
LQR Control Output
Control Increment
```

这些曲线用于分析跟踪速度、超调、稳态误差、噪声影响以及控制输出是否满足限制。

---

# 学习与体会

通过这个项目，我开始从 PID 的误差反馈控制进一步接触基于状态空间模型的控制方法。

在实现 LQR 的过程中，我逐渐理解了状态变量、状态反馈、Q/R 权重和反馈增益 K 之间的关系。相比直接调节 PID 的 Kp、Ki、Kd，LQR 更重要的思路是先确定系统模型，再通过对不同状态和控制输入设置权重来得到反馈增益。

将 LQR 真正接入 ROS2 和 Gazebo 后，我也认识到，仅仅得到一个控制公式并不能代表控制系统已经完成。实际仿真中还需要处理状态反馈、垂向速度、坐标方向、外力施加、控制限幅、变化率限制、传感器噪声和滤波等问题。

通过 PID、LQR 和 MPC 的连续实现和对比，我对三种方法的区别也更加清楚：

```text
PID
基于误差进行反馈控制

LQR
基于系统模型进行状态反馈和最优权衡

MPC
基于模型预测进行在线优化，并显式处理约束
```

LQR 项目对我最大的帮助，是让我从“根据误差调参数”逐渐过渡到“根据系统模型设计控制器”，也为后续学习 MPC 和机器人控制打下了基础。
