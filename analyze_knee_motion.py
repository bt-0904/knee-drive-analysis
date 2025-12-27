# -*- coding: utf-8 -*-
"""
膝蓋抬腿動作分析腳本+
分析 knee_data_20251129_090804.csv 中 09:10 之後的資料
視覺化抬腳起伏變化
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import numpy as np

# 設定中文字體（Windows 系統使用微軟正黑體）
plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei", "SimHei", "Arial Unicode MS"]
plt.rcParams["axes.unicode_minus"] = False


def load_and_filter_data(filepath, start_time_str="09:10:00"):
    """載入 CSV 並篩選指定時間之後的資料"""
    # 讀取 CSV
    df = pd.read_csv(filepath)

    # 轉換時間戳記為 datetime
    df["時間戳記"] = pd.to_datetime(df["時間戳記"])

    # 篩選 09:10 之後的資料
    start_time = (
        df["時間戳記"].iloc[0].replace(hour=9, minute=10, second=0, microsecond=0)
    )
    df_filtered = df[df["時間戳記"] >= start_time].copy()

    print(f"📊 資料總筆數: {len(df)}")
    print(f"📊 篩選後筆數: {len(df_filtered)}")
    print(
        f"📊 時間範圍: {df_filtered['時間戳記'].iloc[0]} ~ {df_filtered['時間戳記'].iloc[-1]}"
    )

    return df_filtered


def analyze_leg_raise(df):
    """分析抬腿數據"""
    angle = df["角度(deg)"].values

    # 基本統計
    stats = {
        "平均角度": np.mean(angle),
        "最大角度": np.max(angle),
        "最小角度": np.min(angle),
        "角度標準差": np.std(angle),
        "角度範圍": np.max(angle) - np.min(angle),
    }

    print("\n" + "=" * 50)
    print("📈 抬腿動作統計分析")
    print("=" * 50)
    for key, value in stats.items():
        print(f"  {key}: {value:.2f}°")

    return stats


def visualize_knee_motion(df, output_path="knee_motion_analysis.png"):
    """視覺化膝蓋運動數據"""

    # 建立時間軸（使用運行時間或索引）
    time_seconds = df["運行時間(s)"].values

    # 建立圖表 - 4 個子圖
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle("🦵 膝蓋抬腿動作分析 (09:10 ~ 結束)", fontsize=16, fontweight="bold")

    # ===== 圖1: 大腿角度變化 =====
    ax1 = axes[0]
    ax1.plot(
        time_seconds, df["角度(deg)"], color="#2196F3", linewidth=1.5, label="大腿角度"
    )
    ax1.axhline(y=0, color="gray", linestyle="--", alpha=0.5, label="站立基準 (0°)")
    ax1.axhline(y=90, color="red", linestyle="--", alpha=0.5, label="水平位置 (90°)")
    ax1.fill_between(time_seconds, 0, df["角度(deg)"], alpha=0.3, color="#2196F3")
    ax1.set_ylabel("角度 (度)", fontsize=11)
    ax1.set_title("大腿抬起角度變化", fontsize=12)
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(-10, 180)

    # ===== 圖2: 膝蓋垂直位置 (Z軸) =====
    ax2 = axes[1]
    ax2.plot(
        time_seconds,
        df["絕對Z(cm)"],
        color="#4CAF50",
        linewidth=1.5,
        label="膝蓋高度 Z",
    )
    ax2.axhline(y=0, color="gray", linestyle="--", alpha=0.5, label="髖關節高度")
    ax2.fill_between(
        time_seconds,
        0,
        df["絕對Z(cm)"],
        where=(df["絕對Z(cm)"] > 0),
        alpha=0.3,
        color="#4CAF50",
        label="高於髖關節",
    )
    ax2.fill_between(
        time_seconds,
        0,
        df["絕對Z(cm)"],
        where=(df["絕對Z(cm)"] <= 0),
        alpha=0.3,
        color="#FF9800",
        label="低於髖關節",
    )
    ax2.set_ylabel("高度 (cm)", fontsize=11)
    ax2.set_title("膝蓋垂直位置 (相對於髖關節)", fontsize=12)
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    # ===== 圖3: 膝蓋前後位移 (Y軸) =====
    ax3 = axes[2]
    ax3.plot(
        time_seconds,
        df["絕對Y(cm)"],
        color="#9C27B0",
        linewidth=1.5,
        label="膝蓋前後位移 Y",
    )
    ax3.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
    ax3.fill_between(time_seconds, 0, df["絕對Y(cm)"], alpha=0.3, color="#9C27B0")
    ax3.set_ylabel("前後位移 (cm)", fontsize=11)
    ax3.set_title("膝蓋前後位移 (正值=向前)", fontsize=12)
    ax3.legend(loc="upper right")
    ax3.grid(True, alpha=0.3)

    # ===== 圖4: 陀螺儀角速度 =====
    ax4 = axes[3]
    ax4.plot(
        time_seconds,
        df["陀螺儀X(deg/s)"],
        color="#F44336",
        linewidth=1,
        alpha=0.8,
        label="X軸 (前後擺動)",
    )
    ax4.plot(
        time_seconds,
        df["陀螺儀Y(deg/s)"],
        color="#2196F3",
        linewidth=1,
        alpha=0.8,
        label="Y軸 (左右擺動)",
    )
    ax4.plot(
        time_seconds,
        df["陀螺儀Z(deg/s)"],
        color="#4CAF50",
        linewidth=1,
        alpha=0.8,
        label="Z軸 (旋轉)",
    )
    ax4.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
    ax4.set_ylabel("角速度 (°/s)", fontsize=11)
    ax4.set_xlabel("運行時間 (秒)", fontsize=11)
    ax4.set_title("陀螺儀角速度 (動作劇烈程度)", fontsize=12)
    ax4.legend(loc="upper right")
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"\n✅ 圖表已儲存至: {output_path}")
    plt.show()


def visualize_3d_trajectory(df, output_path="knee_trajectory_3d.png"):
    """視覺化膝蓋 3D 軌跡"""

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    # 繪製軌跡
    x = df["絕對X(cm)"].values
    y = df["絕對Y(cm)"].values
    z = df["絕對Z(cm)"].values

    # 使用顏色表示時間進程
    colors = np.linspace(0, 1, len(x))
    scatter = ax.scatter(x, y, z, c=colors, cmap="viridis", s=20, alpha=0.7)

    # 連接線
    ax.plot(x, y, z, color="gray", alpha=0.3, linewidth=0.5)

    # 標記髖關節原點
    ax.scatter([0], [0], [0], color="red", s=200, marker="*", label="髖關節原點")

    # 設定軸標籤
    ax.set_xlabel("X (cm) - 左右", fontsize=11)
    ax.set_ylabel("Y (cm) - 前後", fontsize=11)
    ax.set_zlabel("Z (cm) - 上下", fontsize=11)
    ax.set_title("🦵 膝蓋 3D 運動軌跡", fontsize=14, fontweight="bold")

    # 色條
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.6, label="時間進程")

    ax.legend(loc="upper left")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"✅ 3D 軌跡圖已儲存至: {output_path}")
    plt.show()


def detect_leg_raises(df, threshold=60):
    """偵測高抬腿動作（角度超過閾值）"""

    high_raises = df[df["角度(deg)"] > threshold]

    print("\n" + "=" * 50)
    print(f"🦵 高抬腿偵測 (角度 > {threshold}°)")
    print("=" * 50)
    print(f"  偵測到 {len(high_raises)} 次高抬腿動作")

    if len(high_raises) > 0:
        print(f"  最高抬腿角度: {high_raises['角度(deg)'].max():.1f}°")
        print(f"  高抬腿平均角度: {high_raises['角度(deg)'].mean():.1f}°")

    return high_raises


# ===== 主程式 =====
if __name__ == "__main__":
    # 設定檔案路徑
    csv_file = "knee_data_20251129_090804.csv"

    print("=" * 60)
    print("🦵 膝蓋抬腿動作分析系統")
    print("=" * 60)

    # 1. 載入並篩選資料
    df = load_and_filter_data(csv_file, start_time_str="09:10:00")

    # 2. 統計分析
    stats = analyze_leg_raise(df)

    # 3. 偵測高抬腿
    high_raises = detect_leg_raises(df, threshold=60)

    # 4. 視覺化
    print("\n📊 正在生成視覺化圖表...")
    visualize_knee_motion(df, output_path="knee_motion_analysis.png")

    # 5. 3D 軌跡圖
    visualize_3d_trajectory(df, output_path="knee_trajectory_3d.png")

    print("\n✅ 分析完成！")
