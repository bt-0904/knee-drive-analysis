#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""分析 4 次 90 度抬腳動作"""

import pandas as pd
import numpy as np
from scipy.signal import find_peaks

# 檢查最新的檔案
import os

files = sorted(
    [
        f
        for f in os.listdir(".")
        if f.startswith("knee_data_20251227") and f.endswith(".csv")
    ]
)

print("今天的檔案:")
for f in files:
    df_temp = pd.read_csv(f)
    angle_max = df_temp["角度(deg)"].max()
    print(f"  {f}: {len(df_temp)}筆, 最大角度={angle_max:.1f}度")

# 使用角度最高且筆數合理的檔案
filename = "knee_data_20251227_083519.csv"
df = pd.read_csv(filename)

print()
print("=" * 70)
print(f"📊 分析檔案: {filename}")
print("=" * 70)
print(f"總筆數: {len(df):,}")

t_min = df["運行時間(s)"].min()
t_max = df["運行時間(s)"].max()
print(f"時間範圍: {t_min:.2f}s ~ {t_max:.2f}s (持續 {t_max-t_min:.1f} 秒)")
print(f"取樣率: {len(df) / (t_max - t_min):.1f} Hz")

angle_col = "角度(deg)"
print(f"角度範圍: {df[angle_col].min():.1f} ~ {df[angle_col].max():.1f} 度")

angles = df[angle_col].values
times = df["運行時間(s)"].values

# 找 90 度以上的抬腳 (預期 4 次)
peaks, _ = find_peaks(angles, height=80, distance=50, prominence=30)

print()
print("=" * 70)
print("🦵 90度抬腳偵測 (預期 4 次)")
print("=" * 70)
print(f"偵測到 {len(peaks)} 次高抬腳動作:")
print()
print(f'{"次數":^6} {"時間":^12} {"角度":^12} {"膝蓋Y":^14} {"膝蓋Z":^14} {"狀態":^12}')
print("-" * 70)

for i, p in enumerate(peaks):
    t = times[p]
    angle = angles[p]
    y = df["絕對Y(cm)"].iloc[p]
    z = df["絕對Z(cm)"].iloc[p]
    status = "✓ 達標" if angle >= 85 else "⚠️ 未達標"
    print(
        f"{i+1:^6} {t:^12.2f}s {angle:^12.1f}° {y:^14.1f}cm {z:^14.1f}cm {status:^12}"
    )

# 找站立位置
valleys, _ = find_peaks(-angles, height=-15, distance=50, prominence=20)

print()
print("=" * 70)
print("🧍 站立位置偵測 (預期 5 次: 開始 + 4 次中間)")
print("=" * 70)
print(f"偵測到 {len(valleys)} 個站立位置")

# 動作序列
print()
print("=" * 70)
print("📋 完整動作序列")
print("   預期: 校正→站立→抬腳→站立→抬腳→站立→抬腳→站立→抬腳")
print("=" * 70)

events = []
for p in peaks:
    events.append((times[p], "⬆️ 抬腳", angles[p]))
for v in valleys:
    events.append((times[v], "⬇️ 站立", angles[v]))
events.sort(key=lambda x: x[0])

for i, (t, action, angle) in enumerate(events):
    check = ""
    if "抬腳" in action:
        check = " ✓ 90度達標" if angle >= 85 else " ⚠️ 未達90度"
    print(f"  {i+1:2}. {t:6.2f}s: {action} ({angle:5.1f}°){check}")

# 軌跡檢查
print()
print("=" * 70)
print("🎯 90度抬腳軌跡驗證")
print("=" * 70)
print("理論值: 90度時 Y≈40cm(前伸), Z≈0cm(與髖同高)")
print()

all_ok = True
for i, p in enumerate(peaks):
    angle = angles[p]
    y = df["絕對Y(cm)"].iloc[p]
    z = df["絕對Z(cm)"].iloc[p]

    angle_ok = angle >= 85
    y_ok = y > 35
    z_ok = z > -10

    status = "✓" if (angle_ok and y_ok and z_ok) else "⚠️"
    if not (angle_ok and y_ok and z_ok):
        all_ok = False

    a_mark = "✓" if angle_ok else "⚠️"
    y_mark = "✓" if y_ok else "⚠️"
    z_mark = "✓" if z_ok else "⚠️"

    print(f"  抬腳 {i+1}: {status}")
    print(f"    角度: {angle:.1f}° {a_mark}(>=85)")
    print(f"    膝蓋Y: {y:.1f}cm {y_mark}(>35cm)")
    print(f"    膝蓋Z: {z:.1f}cm {z_mark}(>-10cm)")

# 資料完整度
print()
print("=" * 70)
print("✅ 資料完整度檢查")
print("=" * 70)
time_diffs = np.diff(times)
positive_diffs = time_diffs[time_diffs > 0]
print(f"平均取樣間隔: {np.mean(positive_diffs)*1000:.1f} ms (預期 20ms)")
print(f"最大間隔: {np.max(positive_diffs)*1000:.1f} ms")
expected = int((t_max - t_min) * 50)
completeness = len(df) / expected * 100 if expected > 0 else 0
print(f"資料完整度: {completeness:.1f}%")
stable_ratio = (df["是否穩定"] == "是").sum() / len(df) * 100
print(f"穩定狀態比例: {stable_ratio:.1f}%")

print()
print("=" * 70)
if len(peaks) == 4 and all_ok:
    print("🎉 結論: 4 次 90 度抬腳全部偵測成功，軌跡正確！")
elif len(peaks) == 4:
    print("⚠️ 結論: 偵測到 4 次抬腳，但部分軌跡需確認")
else:
    print(f"⚠️ 結論: 偵測到 {len(peaks)} 次抬腳 (預期 4 次)")
print("=" * 70)
