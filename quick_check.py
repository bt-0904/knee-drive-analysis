import pandas as pd
import numpy as np
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

print("=" * 60)
print("📊 knee_data_20251227_103045.csv 詳細分析")
print("=" * 60)

ts_min = df["timestamp"].min()
ts_max = df["timestamp"].max()
duration = ts_max - ts_min

print(f"總筆數: {len(df)}")
print(f"時間範圍: {ts_min:.2f}s ~ {ts_max:.2f}s (共 {duration:.1f}s)")

time_diffs = df["timestamp"].diff().dropna()
print(f"平均間隔: {time_diffs.mean()*1000:.1f} ms")
print(f"實際取樣率: {1/time_diffs.mean():.1f} Hz")

# 角度統計
print(f"\n📐 角度統計:")
print(f'  最小: {df["angle"].min():.1f}°')
print(f'  最大: {df["angle"].max():.1f}°')
print(f'  平均: {df["angle"].mean():.1f}°')

# 降低門檻找峰值 (30度以上)
peaks, props = find_peaks(df["angle"], height=30, distance=15, prominence=10)
print(f"\n🦵 偵測到 {len(peaks)} 次抬腳 (門檻 30°):")

for i, peak_idx in enumerate(peaks):
    t = df.loc[peak_idx, "timestamp"]
    a = df.loc[peak_idx, "angle"]
    status = "✅" if a >= 85 else ("⚠️" if a >= 60 else "❌")
    print(f"  第{i+1}次: {t:.2f}s, 角度={a:.1f}° {status}")

print("\n角度達標說明: ✅>=85° | ⚠️>=60° | ❌<60°")
