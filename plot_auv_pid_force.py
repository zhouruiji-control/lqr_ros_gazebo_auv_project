#!/usr/bin/env python3

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


# =========================================================
# 文件设置
# Python脚本和CSV都放在 ~/ros_auvproject 下
# =========================================================
script_dir = Path(__file__).resolve().parent

csv_file = script_dir / "auv_pid_force.csv"

depth_figure = script_dir / "auv_pid_force_depth.png"
error_figure = script_dir / "auv_pid_force_error.png"
output_figure = script_dir / "auv_pid_force_output.png"


# =========================================================
# 检查CSV是否存在
# =========================================================
if not csv_file.exists():
    raise FileNotFoundError(
        f"没有找到CSV文件：{csv_file}\n"
        "请确认 auv_pid_force.csv 和本脚本位于同一目录。"
    )


# =========================================================
# 读取CSV
# =========================================================
data = pd.read_csv(csv_file)

required_columns = [
    "time",
    "target_depth",
    "current_depth",
    "error",
    "output",
]

missing_columns = [
    column
    for column in required_columns
    if column not in data.columns
]

if missing_columns:
    raise ValueError(
        f"CSV缺少以下数据列：{missing_columns}\n"
        f"当前CSV列名为：{list(data.columns)}"
    )


# 将数据转换为数值，异常数据转为NaN
for column in required_columns:
    data[column] = pd.to_numeric(
        data[column],
        errors="coerce",
    )

# 删除关键列中存在空值的行
data = data.dropna(
    subset=required_columns
).reset_index(drop=True)

if data.empty:
    raise ValueError("CSV中没有可以用于绘图的有效数据。")


# =========================================================
# 图1：目标深度和实际深度
# =========================================================
plt.figure(figsize=(9, 5.5))

plt.plot(
    data["time"],
    data["target_depth"],
    linestyle="--",
    linewidth=2.0,
    label="Target depth",
)

plt.plot(
    data["time"],
    data["current_depth"],
    linewidth=2.0,
    label="Current depth",
)

plt.xlabel(
    "Time (s)",
    fontsize=13,
)

plt.ylabel(
    "Depth (m)",
    fontsize=13,
)

plt.title(
    "AUV Depth Response",
    fontsize=15,
)

plt.tick_params(
    axis="both",
    labelsize=11,
)

plt.grid(
    True,
    linestyle="--",
    alpha=0.5,
)

plt.legend(
    fontsize=11,
)

# 当前定义为深度向下为正。
# 需要让20 m显示在图的下方时，取消下一行注释。
# plt.gca().invert_yaxis()

plt.tight_layout()

plt.savefig(
    depth_figure,
    dpi=300,
    bbox_inches="tight",
)

plt.close()


# =========================================================
# 图2：深度误差
# =========================================================
plt.figure(figsize=(9, 5.5))

plt.plot(
    data["time"],
    data["error"],
    linewidth=2.0,
    label="Depth error",
)

plt.axhline(
    y=0.0,
    linestyle="--",
    linewidth=1.2,
)

plt.xlabel(
    "Time (s)",
    fontsize=13,
)

plt.ylabel(
    "Error (m)",
    fontsize=13,
)

plt.title(
    "AUV Depth Tracking Error",
    fontsize=15,
)

plt.tick_params(
    axis="both",
    labelsize=11,
)

plt.grid(
    True,
    linestyle="--",
    alpha=0.5,
)

plt.legend(
    fontsize=11,
)

plt.tight_layout()

plt.savefig(
    error_figure,
    dpi=300,
    bbox_inches="tight",
)

plt.close()


# =========================================================
# 图3：PID控制输出
# =========================================================
plt.figure(figsize=(9, 5.5))

plt.plot(
    data["time"],
    data["output"],
    linewidth=2.0,
    label="PID output",
)

plt.axhline(
    y=0.0,
    linestyle="--",
    linewidth=1.2,
)

# 你的PID输出限幅是 ±5
plt.axhline(
    y=5.0,
    linestyle=":",
    linewidth=1.0,
    label="Output limit",
)

plt.axhline(
    y=-5.0,
    linestyle=":",
    linewidth=1.0,
)

plt.xlabel(
    "Time (s)",
    fontsize=13,
)

plt.ylabel(
    "Control output",
    fontsize=13,
)

plt.title(
    "PID Control Output",
    fontsize=15,
)

plt.tick_params(
    axis="both",
    labelsize=11,
)

plt.grid(
    True,
    linestyle="--",
    alpha=0.5,
)

plt.legend(
    fontsize=11,
)

plt.tight_layout()

plt.savefig(
    output_figure,
    dpi=300,
    bbox_inches="tight",
)

plt.close()


# =========================================================
# 输出简单实验结果
# =========================================================
final_target = data["target_depth"].iloc[-1]
final_depth = data["current_depth"].iloc[-1]
final_error = data["error"].iloc[-1]

maximum_depth = data["current_depth"].max()
minimum_depth = data["current_depth"].min()

maximum_output = data["output"].max()
minimum_output = data["output"].min()

print("=========================================")
print("AUV受力版实验绘图完成")
print("=========================================")
print(f"数据点数量：{len(data)}")
print(f"实验时长：{data['time'].iloc[-1]:.2f} s")
print(f"最终目标深度：{final_target:.3f} m")
print(f"最终实际深度：{final_depth:.3f} m")
print(f"最终深度误差：{final_error:.3f} m")
print(f"最大实际深度：{maximum_depth:.3f} m")
print(f"最小实际深度：{minimum_depth:.3f} m")
print(f"最大控制输出：{maximum_output:.3f}")
print(f"最小控制输出：{minimum_output:.3f}")
print()
print("图片已保存：")
print(depth_figure)
print(error_figure)
print(output_figure)   