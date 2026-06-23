import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV
data = pd.read_csv("auv_pid_result.csv")

# 去掉表头空格
data.columns = data.columns.str.strip()

time = data["time"]
target = 10.0

# 创建一个大窗口，里面放 5 张图
fig, axs = plt.subplots(5, 1, figsize=(12, 18), sharex=True)

# =========================
# 图1：深度响应
# =========================
axs[0].plot(time, data["current_trueposition"], label="True Position")
axs[0].plot(time, data["measure_position"], label="Measured Position", alpha=0.5)
axs[0].plot(time, data["lvbo_posiition"], label="Filtered Position")
axs[0].axhline(y=target, linestyle="--", label="Target Depth")
axs[0].set_ylabel("Depth")
axs[0].set_title("AUV Depth Response")
axs[0].legend()
axs[0].grid(True)

# =========================
# 图2：误差
# =========================
axs[1].plot(time, data["error"], label="Error")
axs[1].axhline(y=0, linestyle="--")
axs[1].set_ylabel("Error")
axs[1].set_title("Depth Error")
axs[1].legend()
axs[1].grid(True)

# =========================
# 图3：控制输出
# =========================
axs[2].plot(time, data["output"], label="Control Output")
axs[2].set_ylabel("Output")
axs[2].set_title("PID Control Output")
axs[2].legend()
axs[2].grid(True)

# =========================
# 图4：速度
# =========================
axs[3].plot(time, data["velocity"], label="Velocity")
axs[3].set_ylabel("Velocity")
axs[3].set_title("AUV Velocity")
axs[3].legend()
axs[3].grid(True)

# =========================
# 图5：PID 三项
# =========================
axs[4].plot(time, data["p_term"], label="P term")
axs[4].plot(time, data["i_term"], label="I term")
axs[4].plot(time, data["d_term"], label="D term")
axs[4].set_xlabel("Time / s")
axs[4].set_ylabel("PID Terms")
axs[4].set_title("PID Terms")
axs[4].legend()
axs[4].grid(True)

# 自动调整间距
plt.tight_layout()

# 保存整张大图
plt.savefig("auv_pid_all_figures.png", dpi=300)

# 一次性显示
plt.show()