import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

column_map = {
    "運行時間(s)": "timestamp",
    "角度(deg)": "angle",
    "絕對Y(cm)": "knee_y",
    "絕對Z(cm)": "knee_z",
}

df = pd.read_csv("knee_data_20251227_103045.csv")
df = df.rename(columns=column_map)
df = df.sort_values("timestamp").reset_index(drop=True)

# 資料完整度
ts_min = df["timestamp"].min()
ts_max = df["timestamp"].max()
duration = ts_max - ts_min
theoretical = int(duration * 50)
completeness = len(df) / theoretical * 100

print("=" * 60)
print("📊 knee_data_20251227_103045.csv 完整分析")
print("=" * 60)
print(f"總筆數: {len(df)}")
print(f"時長: {duration:.1f} 秒")
print(f"理論筆數 (50Hz): {theoretical}")
print(f"完整度: {completeness:.1f}%")

# 取樣率
time_diffs = df["timestamp"].diff().dropna()
print(f"\n⏱️ 取樣率: {1/time_diffs.mean():.1f} Hz")
print(f"平均間隔: {time_diffs.mean()*1000:.1f} ms")

# 檢查間隙
large_gaps = time_diffs[time_diffs > 0.05]
print(f"間隙 (>50ms): {len(large_gaps)} 處")

# 角度
print(f"\n📐 角度統計:")
print(f'  最小: {df["angle"].min():.1f}°')
print(f'  最大: {df["angle"].max():.1f}°')
print(f'  平均: {df["angle"].mean():.1f}°')

# 峰值
peaks, _ = find_peaks(df["angle"], height=30, distance=15, prominence=10)
print(f"\n🦵 偵測到 {len(peaks)} 次抬腳:")
for i, peak_idx in enumerate(peaks):
    t = df.loc[peak_idx, "timestamp"]
    a = df.loc[peak_idx, "angle"]
    status = "✅" if a >= 85 else ("⚠️" if a >= 60 else "❌")
    print(f"  第{i+1}次: {t:.2f}s, 角度={a:.1f}° {status}")

count_ok = sum(1 for p in peaks if df.loc[p, "angle"] >= 85)
count_warn = sum(1 for p in peaks if 60 <= df.loc[p, "angle"] < 85)
count_fail = sum(1 for p in peaks if df.loc[p, "angle"] < 60)
print(f"\n統計: ✅達標({count_ok}) | ⚠️接近({count_warn}) | ❌不足({count_fail})")

# 繪製圖表
fig, axes = plt.subplots(2, 1, figsize=(14, 8))

# 角度時間圖
ax1 = axes[0]
ax1.plot(df["timestamp"], df["angle"], "b-", linewidth=1)
ax1.axhline(y=90, color="r", linestyle="--", alpha=0.5, label="90° target")
ax1.axhline(y=60, color="orange", linestyle=":", alpha=0.5, label="60° threshold")

for peak_idx in peaks:
    angle_val = df.loc[peak_idx, "angle"]
    ax1.plot(df.loc[peak_idx, "timestamp"], angle_val, "ro", markersize=8)
    ax1.annotate(
        f"{angle_val:.0f}°",
        (df.loc[peak_idx, "timestamp"], angle_val + 3),
        ha="center",
        fontsize=9,
    )

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Knee Angle (deg)")
ax1.set_title("knee_data_20251227_103045.csv - Knee Drive Analysis")
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_ylim(-5, 100)

# 軌跡圖
ax2 = axes[1]
ax2.plot(df["knee_y"], df["knee_z"], "b-", linewidth=0.5, alpha=0.6)
ax2.scatter(
    df["knee_y"].iloc[0],
    df["knee_z"].iloc[0],
    c="green",
    s=100,
    marker="o",
    label="Start",
    zorder=5,
)
ax2.scatter(
    df["knee_y"].iloc[-1],
    df["knee_z"].iloc[-1],
    c="red",
    s=100,
    marker="s",
    label="End",
    zorder=5,
)

for peak_idx in peaks:
    ax2.scatter(
        df.loc[peak_idx, "knee_y"],
        df.loc[peak_idx, "knee_z"],
        c="orange",
        s=80,
        marker="^",
        zorder=4,
    )

ax2.set_xlabel("Knee Y - Forward (cm)")
ax2.set_ylabel("Knee Z - Up (cm)")
ax2.set_title("Knee Trajectory (Side View)")
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.axis("equal")

plt.tight_layout()
plt.savefig("latest_analysis.png", dpi=150)
print("\n📈 圖表已儲存至 latest_analysis.png")
plt.show()
