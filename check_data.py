#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""分析最新的膝蓋抬腿資料 - 聚焦分析站立→抬腳→站立→抬腳序列"""

import pandas as pd
import numpy as np
from scipy.signal import find_peaks
import matplotlib.pyplot as plt

# 讀取最新資料
filename = "knee_data_20251227_083705.csv"
df = pd.read_csv(filename)
print(f"分析檔案: {filename}")

print("=" * 60)
print("📊 資料基本資訊")
print("=" * 60)
print(f"總筆數: {len(df)}")
time_min = df["運行時間(s)"].min()
time_max = df["運行時間(s)"].max()
print(f"取樣時間範圍: {time_min:.2f}s ~ {time_max:.2f}s")
print(f"總持續時間: {time_max - time_min:.2f} 秒")
print(f"取樣率: {len(df) / (time_max - time_min):.1f} Hz")

# 聚焦分析前60秒的資料 (這應該包含你的 站立→抬腳→站立→抬腳 動作)
df_early = df[df["運行時間(s)"] < 70].copy()
print(f"\n前70秒資料筆數: {len(df_early)}")

print("\n" + "=" * 60)
print("📐 角度統計")
print("=" * 60)
print(f'最小角度: {df["角度(deg)"].min():.1f} 度')
print(f'最大角度: {df["角度(deg)"].max():.1f} 度')
print(f'平均角度: {df["角度(deg)"].mean():.1f} 度')

# 找出抬腳的波峰 (局部最大值)
angles = df["角度(deg)"].values
times = df["運行時間(s)"].values

# 找波峰 (抬腳最高點) - 設定最小高度 70 度 (針對90度抬腳)
peaks, peak_props = find_peaks(angles, height=70, distance=50, prominence=20)
# 找波谷 (站立最低點)
valleys, valley_props = find_peaks(-angles, height=-20, distance=50, prominence=15)

print("\n" + "=" * 60)
print("🦵 90度抬腳動作分析 (預期3次)")
print("=" * 60)
print(f"偵測到 {len(peaks)} 次高抬腳動作")

for i, peak in enumerate(peaks):
    y = df["絕對Y(cm)"].iloc[peak]
    z = df["絕對Z(cm)"].iloc[peak]
    angle_ok = "✓" if angles[peak] >= 85 else "⚠️"
    print(
        f"  第 {i+1} 次抬腳: 時間={times[peak]:.2f}s, 角度={angles[peak]:.1f}度 {angle_ok}"
    )

print("\n" + "=" * 60)
print("🧍 站立位置分析 (預期4次)")
print("=" * 60)
print(f"偵測到 {len(valleys)} 個站立位置")

for i, valley in enumerate(valleys[:10]):
    print(f"  站立 {i+1}: 時間={times[valley]:.2f}s, 角度={angles[valley]:.1f} 度")

# 檢查動作順序
print("\n" + "=" * 60)
print("📋 動作順序檢查 (預期: 站立→抬腳→站立→抬腳→站立→抬腳)")
print("=" * 60)

# 合併波峰和波谷，按時間排序
all_events = []
for p in peaks:
    all_events.append((times[p], "抬腳", angles[p]))
for v in valleys:
    all_events.append((times[v], "站立", angles[v]))

all_events.sort(key=lambda x: x[0])

print("偵測到的動作序列:")
for i, (t, action, angle) in enumerate(all_events[:15]):
    symbol = "⬆️" if action == "抬腳" else "⬇️"
    check = (
        "✓ 90度達標"
        if (action == "抬腳" and angle >= 85)
        else ("✓" if action == "站立" else "⚠️ 未達90度")
    )
    print(f"  {i+1}. {symbol} {t:.2f}s: {action} (角度: {angle:.1f}度) {check}")

# 資料完整度檢查
print("\n" + "=" * 60)
print("✅ 資料完整度檢查")
print("=" * 60)

# 檢查取樣間隔
time_diffs = np.diff(times)
avg_interval = np.mean(time_diffs)
max_gap = np.max(time_diffs)
min_gap = np.min(time_diffs)

print(f"平均取樣間隔: {avg_interval*1000:.1f} ms (預期 20ms)")
print(f"最大間隔: {max_gap*1000:.1f} ms")
print(f"最小間隔: {min_gap*1000:.1f} ms")

# 檢查是否有資料遺失
expected_samples = int((time_max - time_min) * 50)  # 50Hz
actual_samples = len(df)
completeness = (actual_samples / expected_samples) * 100

print(f"預期筆數 (50Hz): {expected_samples}")
print(f"實際筆數: {actual_samples}")
print(f"資料完整度: {completeness:.1f}%")

# 檢查穩定狀態比例
stable_count = (df["是否穩定"] == "是").sum()
stable_ratio = stable_count / len(df) * 100
print(f"穩定狀態比例: {stable_ratio:.1f}%")

# 軌跡合理性檢查
print("\n" + "=" * 60)
print("🎯 軌跡合理性檢查")
print("=" * 60)

if len(peaks) >= 1:
    for i, peak in enumerate(peaks):
        y_pos = df["絕對Y(cm)"].iloc[peak]
        z_pos = df["絕對Z(cm)"].iloc[peak]
        angle = angles[peak]
        print(
            f"  抬腳 {i+1}: 角度={angle:.1f}度, Y={y_pos:.1f}cm(前後), Z={z_pos:.1f}cm(上下)"
        )

        # 檢查合理性: 90度抬腳時 Y 應該約40cm, Z 應該約0cm
        angle_ok = "✓" if angle >= 85 else "⚠️"
        y_ok = "✓" if y_pos > 35 else "⚠️"
        z_ok = "✓" if z_pos > -10 else "⚠️"
        print(
            f"    角度{angle_ok}(>=85°)  膝蓋前伸{y_ok}(Y>35cm)  膝蓋上抬{z_ok}(Z>-10cm)"
        )

# 繪製圖表 - 聚焦分析前 60 秒
print("\n" + "=" * 60)
print("📈 繪製動作軌跡圖表 (前60秒)")
print("=" * 60)

# 篩選前 60 秒的資料
df_plot = df[df["運行時間(s)"] < 70].copy()
angles_plot = df_plot["角度(deg)"].values
times_plot = df_plot["運行時間(s)"].values

# 找此區間的波峰波谷
peaks_plot, _ = find_peaks(angles_plot, height=25, distance=50, prominence=8)
valleys_plot, _ = find_peaks(-angles_plot, height=-15, distance=50, prominence=5)

fig, axes = plt.subplots(3, 1, figsize=(14, 10))

# 圖1: 角度隨時間變化
ax1 = axes[0]
ax1.plot(times_plot, angles_plot, "b-", linewidth=1, label="Thigh Angle")
ax1.scatter(
    times_plot[peaks_plot],
    angles_plot[peaks_plot],
    color="red",
    s=100,
    zorder=5,
    label="Leg Lift (Peak)",
    marker="^",
)
ax1.scatter(
    times_plot[valleys_plot],
    angles_plot[valleys_plot],
    color="green",
    s=100,
    zorder=5,
    label="Standing (Valley)",
    marker="v",
)
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Angle (deg)")
ax1.set_title("Knee Drive Motion Analysis - Angle vs Time (First 60s)")
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.axhline(y=0, color="gray", linestyle="--", alpha=0.5)

# 標註動作順序
for i, peak in enumerate(peaks_plot):
    ax1.annotate(
        f"Lift {i+1}\n{angles_plot[peak]:.0f}°",
        xy=(times_plot[peak], angles_plot[peak]),
        xytext=(0, 15),
        textcoords="offset points",
        ha="center",
        fontsize=8,
        color="red",
    )
for i, valley in enumerate(valleys_plot):
    ax1.annotate(
        f"Stand {i+1}\n{angles_plot[valley]:.0f}°",
        xy=(times_plot[valley], angles_plot[valley]),
        xytext=(0, -20),
        textcoords="offset points",
        ha="center",
        fontsize=8,
        color="green",
    )

# 圖2: Y-Z 軌跡 (膝蓋位置)
ax2 = axes[1]
y_plot = df_plot["絕對Y(cm)"].values
z_plot = df_plot["絕對Z(cm)"].values
scatter = ax2.scatter(y_plot, z_plot, c=times_plot, cmap="viridis", s=5, alpha=0.7)
ax2.scatter(
    y_plot[peaks_plot],
    z_plot[peaks_plot],
    color="red",
    s=150,
    zorder=5,
    label="Leg Lift",
    marker="^",
    edgecolors="black",
)
ax2.scatter(
    y_plot[valleys_plot],
    z_plot[valleys_plot],
    color="green",
    s=150,
    zorder=5,
    label="Standing",
    marker="v",
    edgecolors="black",
)
ax2.scatter(
    [0], [-40], color="purple", s=200, marker="o", label="Hip (Origin)", zorder=6
)
ax2.set_xlabel("Y Position (cm) - Forward/Backward")
ax2.set_ylabel("Z Position (cm) - Up/Down")
ax2.set_title("Knee Trajectory in Y-Z Plane")
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_aspect("equal")
plt.colorbar(scatter, ax=ax2, label="Time (s)")

# 圖3: 陀螺儀資料
ax3 = axes[2]
ax3.plot(times_plot, df_plot["陀螺儀X(deg/s)"].values, "r-", alpha=0.7, label="Gyro X")
ax3.plot(times_plot, df_plot["陀螺儀Y(deg/s)"].values, "g-", alpha=0.7, label="Gyro Y")
ax3.plot(times_plot, df_plot["陀螺儀Z(deg/s)"].values, "b-", alpha=0.7, label="Gyro Z")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Angular Velocity (deg/s)")
ax3.set_title("Gyroscope Data")
ax3.legend()
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("motion_analysis_early.png", dpi=150)
print("圖表已儲存至 motion_analysis_early.png")
plt.show()

# 詳細分析動作序列
print("\n" + "=" * 60)
print("📋 詳細動作序列分析 (前60秒)")
print("=" * 60)

all_events = []
for p in peaks_plot:
    all_events.append((times_plot[p], "抬腳", angles_plot[p], y_plot[p], z_plot[p]))
for v in valleys_plot:
    all_events.append((times_plot[v], "站立", angles_plot[v], y_plot[v], z_plot[v]))

all_events.sort(key=lambda x: x[0])

print("\n時間     動作    角度     膝蓋Y    膝蓋Z")
print("-" * 50)
for t, action, angle, y, z in all_events:
    symbol = "⬆️" if action == "抬腳" else "⬇️"
    print(f"{t:6.2f}s  {symbol}{action}  {angle:5.1f}°  {y:6.1f}cm  {z:6.1f}cm")

# 計算動作之間的時間間隔和角度變化
if len(all_events) >= 2:
    print("\n" + "=" * 60)
    print("⏱️ 動作間隔分析")
    print("=" * 60)
    for i in range(1, len(all_events)):
        prev = all_events[i - 1]
        curr = all_events[i]
        dt = curr[0] - prev[0]
        da = curr[2] - prev[2]
        print(f"{prev[1]} → {curr[1]}: 間隔 {dt:.2f}s, 角度變化 {da:+.1f}°")
