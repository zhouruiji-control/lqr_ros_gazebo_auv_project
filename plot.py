import os
import pandas as pd
import matplotlib.pyplot as plt


# 三组目标深度
target_list = [10, 15, 20]

# CSV所在文件夹
# plot.py和CSV都在ros_auvproject根目录时，可以使用 "."
csv_directory = "."

# 图片总保存文件夹
output_directory = "experiment_figures"

# 自动创建图片文件夹
os.makedirs(output_directory, exist_ok=True)


for target in target_list:

    # =========================
    # 1. 自动确定CSV文件名
    # =========================
    csv_filename = f"auv_pid_{target}m.csv"
    csv_path = os.path.join(csv_directory, csv_filename)

    # 检查CSV是否存在
    if not os.path.exists(csv_path):
        print(f"找不到文件：{csv_path}，跳过 {target} m 工况")
        continue

    print(f"正在处理：{csv_path}")

    # 读取CSV
    data = pd.read_csv(csv_path)

    # 去掉表头前后可能存在的空格
    data.columns = data.columns.str.strip()

    time = data["time"]

    # 每个工况建立单独文件夹
    condition_directory = os.path.join(
        output_directory,
        f"{target}m"
    )

    os.makedirs(condition_directory, exist_ok=True)

    # ==================================================
    # 图1：深度响应曲线
    # ==================================================
    plt.figure(figsize=(10, 6))

    plt.plot(
        time,
        data["current_trueposition"],
        label="True Depth"
    )

    plt.axhline(
        y=target,
        linestyle="--",
        label=f"Target Depth ({target} m)"
    )

    plt.xlabel("Time / s")
    plt.ylabel("Depth / m")
    plt.title(f"AUV Depth Response - Target {target} m")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    save_path = os.path.join(
        condition_directory,
        f"depth_response_{target}m.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    # ==================================================
    # 图2：深度误差曲线
    # ==================================================
    plt.figure(figsize=(10, 6))

    plt.plot(
        time,
        data["error"],
        label="Depth Error"
    )

    plt.axhline(
        y=0,
        linestyle="--",
        label="Zero Error"
    )

    plt.xlabel("Time / s")
    plt.ylabel("Error / m")
    plt.title(f"Depth Error - Target {target} m")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    save_path = os.path.join(
        condition_directory,
        f"depth_error_{target}m.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    # ==================================================
    # 图3：控制输出曲线
    # ==================================================
    plt.figure(figsize=(10, 6))

    plt.plot(
        time,
        data["output"],
        label="Control Output"
    )

    plt.xlabel("Time / s")
    plt.ylabel("Control Output")
    plt.title(f"PID Control Output - Target {target} m")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    save_path = os.path.join(
        condition_directory,
        f"control_output_{target}m.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    # ==================================================
    # 图4：测量深度与滤波深度曲线
    # ==================================================
    plt.figure(figsize=(10, 6))

    plt.plot(
        time,
        data["measure_position"],
        label="Measured Depth",
        alpha=0.6
    )

    plt.plot(
        time,
        data["lvbo_posiition"],
        label="Filtered Depth"
    )

    plt.plot(
        time,
        data["current_trueposition"],
        label="True Depth"
    )

    plt.axhline(
        y=target,
        linestyle="--",
        label=f"Target Depth ({target} m)"
    )

    plt.xlabel("Time / s")
    plt.ylabel("Depth / m")
    plt.title(
        f"Measured and Filtered Depth - Target {target} m"
    )
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    save_path = os.path.join(
        condition_directory,
        f"measurement_filter_{target}m.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    # ==================================================
    # 图5：PID三项曲线
    # ==================================================
    plt.figure(figsize=(10, 6))

    plt.plot(
        time,
        data["p_term"],
        label="P Term"
    )

    plt.plot(
        time,
        data["i_term"],
        label="I Term"
    )

    plt.plot(
        time,
        data["d_term"],
        label="D Term"
    )

    plt.xlabel("Time / s")
    plt.ylabel("PID Terms")
    plt.title(f"PID Terms - Target {target} m")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    save_path = os.path.join(
        condition_directory,
        f"pid_terms_{target}m.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    # ==================================================
    # 图6：所有曲线放在一张大图中
    # ==================================================
    fig, axs = plt.subplots(
        5,
        1,
        figsize=(12, 20),
        sharex=True
    )

    # 深度响应
    axs[0].plot(
        time,
        data["current_trueposition"],
        label="True Depth"
    )

    axs[0].axhline(
        y=target,
        linestyle="--",
        label=f"Target Depth ({target} m)"
    )

    axs[0].set_ylabel("Depth / m")
    axs[0].set_title("AUV Depth Response")
    axs[0].legend()
    axs[0].grid(True)

    # 深度误差
    axs[1].plot(
        time,
        data["error"],
        label="Depth Error"
    )

    axs[1].axhline(
        y=0,
        linestyle="--"
    )

    axs[1].set_ylabel("Error / m")
    axs[1].set_title("Depth Error")
    axs[1].legend()
    axs[1].grid(True)

    # 控制输出
    axs[2].plot(
        time,
        data["output"],
        label="Control Output"
    )

    axs[2].set_ylabel("Output")
    axs[2].set_title("PID Control Output")
    axs[2].legend()
    axs[2].grid(True)

    # 测量值与滤波值
    axs[3].plot(
        time,
        data["measure_position"],
        label="Measured Depth",
        alpha=0.6
    )

    axs[3].plot(
        time,
        data["lvbo_posiition"],
        label="Filtered Depth"
    )

    axs[3].plot(
        time,
        data["current_trueposition"],
        label="True Depth"
    )

    axs[3].set_ylabel("Depth / m")
    axs[3].set_title("Measured and Filtered Depth")
    axs[3].legend()
    axs[3].grid(True)

    # PID三项
    axs[4].plot(
        time,
        data["p_term"],
        label="P Term"
    )

    axs[4].plot(
        time,
        data["i_term"],
        label="I Term"
    )

    axs[4].plot(
        time,
        data["d_term"],
        label="D Term"
    )

    axs[4].set_xlabel("Time / s")
    axs[4].set_ylabel("PID Terms")
    axs[4].set_title("PID Terms")
    axs[4].legend()
    axs[4].grid(True)

    fig.suptitle(
        f"AUV PID Control Results - Target {target} m",
        fontsize=16
    )

    plt.tight_layout(rect=[0, 0, 1, 0.98])

    save_path = os.path.join(
        condition_directory,
        f"auv_pid_all_figures_{target}m.png"
    )

    plt.savefig(
        save_path,
        dpi=300,
        bbox_inches="tight"
    )

    plt.close()

    print(f"{target} m 工况图片保存完成：{condition_directory}")


print("三组工况处理结束。")