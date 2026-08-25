import os
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# CSV文件
# ============================================================

csvFile = "auv_lqr_20m.csv"


# ============================================================
# 图片保存文件夹
# ============================================================

outputFolder = "lqr_noise_figures"

os.makedirs(
    outputFolder,
    exist_ok=True
)


# ============================================================
# 读取CSV
# ============================================================

data = pd.read_csv(csvFile)


# ============================================================
# 读取数据
# ============================================================

time = data["time"]

targetDepth = data["target_depth"]

trueDepth = data["current_depth"]
measuredDepth = data["measured_depth"]
filteredDepth = data["filtered_depth"]

trueVelocity = data["velocity"]
measuredVelocity = data["measured_velocity"]
filteredVelocity = data["filtered_velocity"]

error = data["error"]

output = data["output"]


# ============================================================
# 1. 深度对比
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    trueDepth,
    label="True Depth",
    linewidth=2
)

plt.plot(
    time,
    measuredDepth,
    label="Measured Depth",
    linewidth=1,
    alpha=0.7
)

plt.plot(
    time,
    filteredDepth,
    label="Filtered Depth",
    linewidth=2
)

plt.plot(
    time,
    targetDepth,
    "--",
    label="Target Depth",
    linewidth=2
)

plt.xlabel("Time (s)")
plt.ylabel("Depth (m)")

plt.title(
    "LQR Depth Response with Measurement Noise"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "01_depth_response.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 2. 测量噪声与滤波效果
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    measuredDepth - trueDepth,
    label="Measurement Noise"
)

plt.plot(
    time,
    filteredDepth - trueDepth,
    label="Filtered Error"
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Depth Difference (m)")

plt.title(
    "Depth Measurement Noise and Filtering"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "02_depth_noise_filter.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 3. 深度误差
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    error,
    linewidth=2
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Depth Error (m)")

plt.title(
    "LQR Depth Error"
)

plt.grid(True)

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "03_depth_error.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 4. 垂向速度对比
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    trueVelocity,
    label="True Velocity",
    linewidth=2
)

plt.plot(
    time,
    measuredVelocity,
    label="Measured Velocity",
    linewidth=1,
    alpha=0.7
)

plt.plot(
    time,
    filteredVelocity,
    label="Filtered Velocity",
    linewidth=2
)

plt.xlabel("Time (s)")
plt.ylabel("Vertical Velocity (m/s)")

plt.title(
    "LQR Vertical Velocity with Measurement Noise"
)

plt.grid(True)
plt.legend()

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "04_vertical_velocity.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 5. LQR控制输出
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    output,
    linewidth=2
)

plt.axhline(
    0.0,
    linestyle="--"
)

plt.xlabel("Time (s)")
plt.ylabel("Control Output (N)")

plt.title(
    "LQR Control Output"
)

plt.grid(True)

plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "05_control_output.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 6. 综合图
# ============================================================

fig, ax = plt.subplots(
    4,
    1,
    figsize=(12, 14)
)


# ------------------------------------------------------------
# 第1张：深度
# ------------------------------------------------------------

ax[0].plot(
    time,
    trueDepth,
    label="True Depth",
    linewidth=2
)

ax[0].plot(
    time,
    measuredDepth,
    label="Measured Depth",
    linewidth=1,
    alpha=0.7
)

ax[0].plot(
    time,
    filteredDepth,
    label="Filtered Depth",
    linewidth=2
)

ax[0].plot(
    time,
    targetDepth,
    "--",
    label="Target Depth",
    linewidth=2
)

ax[0].set_ylabel(
    "Depth (m)"
)

ax[0].set_title(
    "LQR Depth Response"
)

ax[0].grid(True)
ax[0].legend()


# ------------------------------------------------------------
# 第2张：误差
# ------------------------------------------------------------

ax[1].plot(
    time,
    error,
    linewidth=2
)

ax[1].axhline(
    0.0,
    linestyle="--"
)

ax[1].set_ylabel(
    "Error (m)"
)

ax[1].set_title(
    "Depth Error"
)

ax[1].grid(True)


# ------------------------------------------------------------
# 第3张：速度
# ------------------------------------------------------------

ax[2].plot(
    time,
    trueVelocity,
    label="True Velocity",
    linewidth=2
)

ax[2].plot(
    time,
    measuredVelocity,
    label="Measured Velocity",
    linewidth=1,
    alpha=0.7
)

ax[2].plot(
    time,
    filteredVelocity,
    label="Filtered Velocity",
    linewidth=2
)

ax[2].set_ylabel(
    "Velocity (m/s)"
)

ax[2].set_title(
    "Vertical Velocity"
)

ax[2].grid(True)
ax[2].legend()


# ------------------------------------------------------------
# 第4张：控制量
# ------------------------------------------------------------

ax[3].plot(
    time,
    output,
    linewidth=2
)

ax[3].axhline(
    0.0,
    linestyle="--"
)

ax[3].set_xlabel(
    "Time (s)"
)

ax[3].set_ylabel(
    "Control Output (N)"
)

ax[3].set_title(
    "LQR Control Output"
)

ax[3].grid(True)


plt.tight_layout()

plt.savefig(
    os.path.join(
        outputFolder,
        "06_lqr_noise_summary.png"
    ),
    dpi=300
)

plt.close()


# ============================================================
# 打印基本信息
# ============================================================

print()
print("========================================")
print("LQR Noise Experiment")
print("========================================")

print(
    "Target depth:",
    targetDepth.iloc[-1],
    "m"
)

print(
    "Final true depth:",
    trueDepth.iloc[-1],
    "m"
)

print(
    "Final depth error:",
    error.iloc[-1],
    "m"
)

print(
    "Maximum depth:",
    trueDepth.max(),
    "m"
)

print(
    "Maximum velocity:",
    trueVelocity.abs().max(),
    "m/s"
)

print(
    "Maximum control output:",
    output.abs().max(),
    "N"
)

print(
    "Maximum measurement depth error:",
    (measuredDepth - trueDepth).abs().max(),
    "m"
)

print(
    "Maximum filtered depth error:",
    (filteredDepth - trueDepth).abs().max(),
    "m"
)

print("========================================")
print()

print(
    "Figures saved in:",
    outputFolder
)