# 分析最新膝蓋運動資料
import pandas as pd
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt

# 讀取最新資料
df = pd.read_csv("knee_data_20251227_083705.csv")

# 欄位名稱對應 (中文 -> 英文簡稱)
column_map = {
    "時間戳記": "timestamp_str",
    "運行時間(s)": "timestamp",
    "角度(deg)": "angle",
    "ΔX(cm)": "delta_x",
    "ΔY(cm)": "delta_y",
    "ΔZ(cm)": "delta_z",
    "絕對X(cm)": "knee_x",
    "絕對Y(cm)": "knee_y",
    "絕對Z(cm)": "knee_z",
    "是否穩定": "stable",
    "加速度X(g)": "accel_x",
    "加速度Y(g)": "accel_y",
    "加速度Z(g)": "accel_z",
    "陀螺儀X(deg/s)": "gyro_x",
    "陀螺儀Y(deg/s)": "gyro_y",
    "陀螺儀Z(deg/s)": "gyro_z",
}
df = df.rename(columns=column_map)

print("=" * 60)
print("📊 資料完整度檢查 - knee_data_20251227_083705.csv")
print("=" * 60)

# 基本資訊
print(f"\n📈 基本統計:")
print(f"  總筆數: {len(df)}")
print(f"  欄位: {list(df.columns)}")
ts_min = df["timestamp"].min()
ts_max = df["timestamp"].max()
print(f"  時間範圍: {ts_min:.2f}s ~ {ts_max:.2f}s")
print(f"  總時長: {ts_max - ts_min:.2f} 秒")

# 取樣率分析
time_diffs = df["timestamp"].diff().dropna()
print(f"\n⏱️ 取樣率分析:")
print(f"  平均間隔: {time_diffs.mean()*1000:.1f} ms")
print(f"  標準差: {time_diffs.std()*1000:.1f} ms")
print(f"  實際取樣率: {1/time_diffs.mean():.1f} Hz")
theoretical = int((ts_max - ts_min) * 50)
print(f"  理論筆數 (50Hz): {theoretical}")
completeness = len(df) / theoretical * 100 if theoretical > 0 else 0
print(f"  資料完整度: {completeness:.1f}%")

# 檢查大間隙
large_gaps = time_diffs[time_diffs > 0.1]
print(f"\n⚠️ 資料間隙 (>100ms): {len(large_gaps)} 處")
if len(large_gaps) > 0:
    for idx in list(large_gaps.index)[:5]:
        print(
            f'  - 時間 {df.loc[idx, "timestamp"]:.2f}s: 間隙 {large_gaps[idx]*1000:.0f}ms'
        )

# 角度分析
print(f"\n📐 角度統計:")
print(f'  最小角度: {df["angle"].min():.1f}°')
print(f'  最大角度: {df["angle"].max():.1f}°')
print(f'  平均角度: {df["angle"].mean():.1f}°')

# 偵測抬腳動作 (峰值偵測)
print("\n" + "=" * 60)
print("🦵 抬腳動作偵測")
print("=" * 60)

# 找出角度峰值 (抬腳最高點)
peaks, properties = find_peaks(df["angle"], height=60, distance=25, prominence=20)

print(f"\n偵測到 {len(peaks)} 次抬腳動作:")
print("-" * 50)

lift_data = []
for i, peak_idx in enumerate(peaks):
    peak_time = df.loc[peak_idx, "timestamp"]
    peak_angle = df.loc[peak_idx, "angle"]

    # 找出這次抬腳的起始和結束
    # 往前找最低點
    start_search = max(0, peak_idx - 50)
    start_idx = df.loc[start_search:peak_idx, "angle"].idxmin()
    start_time = df.loc[start_idx, "timestamp"]
    start_angle = df.loc[start_idx, "angle"]

    # 往後找最低點
    end_search = min(len(df) - 1, peak_idx + 50)
    end_idx = df.loc[peak_idx:end_search, "angle"].idxmin()
    end_time = df.loc[end_idx, "timestamp"]
    end_angle = df.loc[end_idx, "angle"]

    # 計算抬腳時間
    lift_duration = peak_time - start_time
    down_duration = end_time - peak_time
    total_duration = end_time - start_time

    # 判斷是否達到 90 度
    is_90deg = "✅" if peak_angle >= 85 else "⚠️"

    print(f"\n第 {i+1} 次抬腳 {is_90deg}")
    print(f"  峰值角度: {peak_angle:.1f}°")
    print(f"  時間: {start_time:.2f}s → {peak_time:.2f}s → {end_time:.2f}s")
    print(f"  抬腳時間: {lift_duration:.2f}s | 放下時間: {down_duration:.2f}s")
    print(f"  總週期: {total_duration:.2f}s")

    lift_data.append(
        {
            "lift_num": i + 1,
            "peak_angle": peak_angle,
            "start_time": start_time,
            "peak_time": peak_time,
            "end_time": end_time,
            "lift_duration": lift_duration,
            "down_duration": down_duration,
        }
    )

# 統計摘要
print("\n" + "=" * 60)
print("📊 抬腳動作摘要")
print("=" * 60)
if lift_data:
    angles = [d["peak_angle"] for d in lift_data]
    durations = [d["lift_duration"] + d["down_duration"] for d in lift_data]

    print(f"  總抬腳次數: {len(lift_data)}")
    print(f"  平均峰值角度: {np.mean(angles):.1f}° ± {np.std(angles):.1f}°")
    print(f"  角度範圍: {min(angles):.1f}° ~ {max(angles):.1f}°")
    print(f"  平均週期: {np.mean(durations):.2f}s")

    # 計算步頻
    if len(lift_data) > 1:
        step_times = [
            lift_data[i + 1]["peak_time"] - lift_data[i]["peak_time"]
            for i in range(len(lift_data) - 1)
        ]
        cadence = 60 / np.mean(step_times)
        print(f"  步頻: {cadence:.1f} 步/分鐘")

    # 檢查 90 度達成率
    count_90 = sum(1 for a in angles if a >= 85)
    print(
        f"\n  ✅ 達到 90° 的抬腳: {count_90}/{len(lift_data)} ({count_90/len(lift_data)*100:.0f}%)"
    )

# 繪製圖表
fig, axes = plt.subplots(2, 1, figsize=(14, 8))

# 上圖：角度時間圖
ax1 = axes[0]
ax1.plot(df["timestamp"], df["angle"], "b-", linewidth=0.8, label="Knee Angle")
ax1.axhline(y=90, color="r", linestyle="--", alpha=0.5, label="90° Target")
ax1.axhline(y=85, color="orange", linestyle=":", alpha=0.5, label="85° Threshold")

# 標記峰值
for i, peak_idx in enumerate(peaks):
    ax1.plot(
        df.loc[peak_idx, "timestamp"], df.loc[peak_idx, "angle"], "ro", markersize=8
    )
    ax1.annotate(
        f'{df.loc[peak_idx, "angle"]:.0f}°',
        (df.loc[peak_idx, "timestamp"], df.loc[peak_idx, "angle"]),
        textcoords="offset points",
        xytext=(0, 10),
        ha="center",
        fontsize=9,
    )

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Knee Angle (°)")
ax1.set_title("Knee Drive Analysis - 90° Walking Pattern")
ax1.legend(loc="upper right")
ax1.grid(True, alpha=0.3)
ax1.set_ylim(-10, max(df["angle"].max() + 20, 100))

# 下圖：膝蓋 Y-Z 軌跡
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

# 標記抬腳最高點
for i, peak_idx in enumerate(peaks):
    ax2.scatter(
        df.loc[peak_idx, "knee_y"],
        df.loc[peak_idx, "knee_z"],
        c="orange",
        s=80,
        marker="^",
        zorder=4,
    )

ax2.set_xlabel("Knee Y (Forward) [cm]")
ax2.set_ylabel("Knee Z (Up) [cm]")
ax2.set_title("Knee Trajectory (Side View)")
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.axis("equal")

plt.tight_layout()
plt.savefig("latest_analysis.png", dpi=150)
print("\n📈 圖表已儲存至 latest_analysis.png")
plt.show()
