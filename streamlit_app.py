# -*- coding: utf-8 -*-
"""
膝蓋抬腿動作分析 - Streamlit 網頁應用
支援 CSV 檔案分析、上傳儲存、刪除管理
"""

import streamlit as st
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import glob
import os
import re

# Plotly for interactive charts
from plotly.subplots import make_subplots
import plotly.graph_objects as go

# ===== 翻譯字典 =====
TRANSLATIONS = {
    "zh": {
        # 頁面標題
        "page_title": "膝蓋動作分析系統",
        "app_title": "🦵 膝蓋動作分析系統",
        # 語言切換
        "lang_toggle": "🌐 English",
        # 側邊欄 - 檔案管理
        "file_management": "📂 檔案管理",
        "select_source": "選擇資料來源",
        "source_existing": "📂 現有檔案",
        "source_upload": "📤 上傳新檔案",
        "select_file": "選擇檔案",
        "delete_btn": "🗑️ 刪除此檔案",
        "confirm_delete_msg": "⚠️ 確定要刪除 **{}**？",
        "confirm_delete_btn": "✅ 確認刪除",
        "cancel_btn": "❌ 取消",
        "delete_success": "✅ 檔案已刪除！",
        "delete_fail": "❌ 刪除失敗: {}",
        "no_csv": "📭 找不到任何 CSV 檔案",
        "select_csv": "選擇 CSV 檔案",
        "upload_success": "✅ 已載入: {}",
        "save_btn": "💾 儲存到紀錄",
        "save_success": "✅ 已儲存為: {}",
        "save_fail": "❌ 儲存失敗: {}",
        # 側邊欄 - 時間範圍與參數
        "time_range_header": "⏱️ 時間範圍",
        "time_range_label": "選擇時間範圍 (秒)",
        "analysis_params": "🎯 分析參數",
        "threshold_label": "高抬腿閾值 (度)",
        # 主區域
        "stats_header": "📊 統計指標",
        "avg_angle": "平均角度",
        "max_angle": "最大角度",
        "min_angle": "最小角度",
        "std_dev": "標準差",
        "data_count": "資料筆數",
        # Tabs
        "tab_motion": "📈 動作分析",
        "tab_3d": "🌐 3D 軌跡",
        "tab_raw": "📋 原始資料",
        # Tab1 - 動作分析
        "motion_chart_title": "📈 4 合 1 動作分析圖表",
        "motion_chart_hint": "💡 提示：可使用滑鼠滾輪縮放、拖曳平移，雙擊重置視圖。每個子圖可獨立縮放 Y 軸，X 軸（時間軸）連動。",
        "high_raise_header": "🦵 高抬腿偵測 (角度 > {}°)",
        "detect_count": "偵測次數",
        "detect_max": "最高角度",
        "detect_avg": "平均高抬角度",
        "no_detection": "未偵測到角度超過 {}° 的高抬腿動作",
        # Tab2 - 3D 軌跡
        "traj_title": "🌐 膝蓋 3D 運動軌跡",
        "traj_hint": "💡 提示：可使用滑鼠拖曳旋轉視角、滾輪縮放，雙擊重置視圖。",
        # Tab3 - 原始資料
        "raw_title": "📋 原始資料預覽",
        "download_btn": "📥 下載篩選後的 CSV",
        # 錯誤 / 空白狀態
        "no_file_info": "👈 請從左側選擇或上傳 CSV 檔案開始分析",
        "no_data_error": "❌ 無法載入資料或資料為空",
        "process_error": "❌ 處理資料時發生錯誤: {}",
        "usage_md": """
### 📖 使用說明

1. **分析現有檔案**：從左側下拉選單選擇已記錄的 CSV 檔案
2. **上傳新檔案**：切換到「上傳新檔案」模式，拖放 CSV 檔案
3. **儲存紀錄**：上傳的檔案可選擇儲存到專案資料夾
4. **調整參數**：使用時間範圍滑桿篩選資料
5. **刪除舊檔案**：選擇檔案後點擊刪除按鈕
 
### 📊 圖表說明

- **動作分析**：4 合 1 圖表顯示角度、Z軸、Y軸、角速度
- **3D 軌跡**：膝蓋在空間中的運動路徑
- **原始資料**：完整資料表格，可排序與下載
""",
        # Plotly 圖表文字
        "chart_main_title": "膝蓋抬腿動作分析",
        "sub_angle": "大腿抬起角度變化",
        "sub_z": "膝蓋垂直位置 (相對於髖關節)",
        "sub_y": "膝蓋前後位移 (正值=向前)",
        "sub_gyro": "角速度 (陀螺儀資料)",
        "sub_flex": "角度變化率 (Flex Sensor 模式)",
        "sub_gyro_imu": "陀螺儀角速度",
        "legend_thigh": "大腿角度",
        "legend_above": "高於髖關節",
        "legend_below": "低於髖關節",
        "legend_y": "膝蓋前後位移 Y",
        "legend_rate": "角度變化率",
        "legend_gx": "陀螺儀 X軸",
        "legend_gy": "陀螺儀 Y軸",
        "legend_gz": "陀螺儀 Z軸",
        "flex_annotation": "Flex Sensor 資料<br>(無陀螺儀)",
        "yaxis_angle": "角度 (度)",
        "yaxis_height": "高度 (cm)",
        "yaxis_disp": "前後位移 (cm)",
        "yaxis_rate": "角度變化率 (°/s)",
        "yaxis_gyro": "角速度 (°/s)",
        "xaxis_time": "運行時間 (秒)",
        "colorbar_time": "時間進程",
        "traj_chart_title": "膝蓋 3D 運動軌跡",
        "traj_legend_knee": "膝蓋軌跡",
        "traj_legend_line": "軌跡線",
        "traj_legend_hip": "髖關節原點",
        "traj_x": "X (cm) - 左右",
        "traj_y": "Y (cm) - 前後",
        "traj_z": "Z (cm) - 上下",
        # 字型
        "font_family": "Microsoft JhengHei, sans-serif",
        "mpl_font": "Microsoft JhengHei",
    },
    "en": {
        # Page title
        "page_title": "Knee Motion Analysis",
        "app_title": "🦵 Knee Motion Analysis System",
        # Language toggle
        "lang_toggle": "🌐 中文",
        # Sidebar - File management
        "file_management": "📂 File Management",
        "select_source": "Select Data Source",
        "source_existing": "📂 Existing Files",
        "source_upload": "📤 Upload New File",
        "select_file": "Select File",
        "delete_btn": "🗑️ Delete This File",
        "confirm_delete_msg": "⚠️ Are you sure you want to delete **{}**?",
        "confirm_delete_btn": "✅ Confirm Delete",
        "cancel_btn": "❌ Cancel",
        "delete_success": "✅ File deleted!",
        "delete_fail": "❌ Delete failed: {}",
        "no_csv": "📭 No CSV files found",
        "select_csv": "Select CSV File",
        "upload_success": "✅ Loaded: {}",
        "save_btn": "💾 Save to Records",
        "save_success": "✅ Saved as: {}",
        "save_fail": "❌ Save failed: {}",
        # Sidebar - Time range & params
        "time_range_header": "⏱️ Time Range",
        "time_range_label": "Select Time Range (s)",
        "analysis_params": "🎯 Analysis Parameters",
        "threshold_label": "High Knee-Drive Threshold (°)",
        # Main area
        "stats_header": "📊 Statistics",
        "avg_angle": "Avg Angle",
        "max_angle": "Max Angle",
        "min_angle": "Min Angle",
        "std_dev": "Std Dev",
        "data_count": "Data Points",
        # Tabs
        "tab_motion": "📈 Motion Analysis",
        "tab_3d": "🌐 3D Trajectory",
        "tab_raw": "📋 Raw Data",
        # Tab1 - Motion Analysis
        "motion_chart_title": "📈 4-in-1 Motion Analysis Chart",
        "motion_chart_hint": "💡 Tip: Use mouse wheel to zoom, drag to pan, double-click to reset. Each subplot can zoom Y-axis independently; X-axis (time) is linked.",
        "high_raise_header": "🦵 High Knee-Drive Detection (Angle > {}°)",
        "detect_count": "Detected",
        "detect_max": "Peak Angle",
        "detect_avg": "Avg Raised Angle",
        "no_detection": "No knee-drive events detected above {}°",
        # Tab2 - 3D Trajectory
        "traj_title": "🌐 Knee 3D Motion Trajectory",
        "traj_hint": "💡 Tip: Drag to rotate, scroll to zoom, double-click to reset view.",
        # Tab3 - Raw Data
        "raw_title": "📋 Raw Data Preview",
        "download_btn": "📥 Download Filtered CSV",
        # Error / empty state
        "no_file_info": "👈 Please select or upload a CSV file from the left sidebar to start analysis",
        "no_data_error": "❌ Failed to load data or data is empty",
        "process_error": "❌ Error processing data: {}",
        "usage_md": """
### 📖 Usage Guide

1. **Analyse existing files**: Select a recorded CSV file from the left dropdown
2. **Upload new file**: Switch to "Upload New File" mode and drag & drop a CSV
3. **Save records**: Uploaded files can be saved to the project folder
4. **Adjust parameters**: Use the time range slider to filter data
5. **Delete old files**: Select a file then click the delete button

### 📊 Chart Guide

- **Motion Analysis**: 4-in-1 chart showing angle, Z-axis, Y-axis, and angular velocity
- **3D Trajectory**: Knee movement path in 3D space
- **Raw Data**: Full data table with sorting and download support
""",
        # Plotly chart text
        "chart_main_title": "Knee Drive Motion Analysis",
        "sub_angle": "Thigh Elevation Angle",
        "sub_z": "Knee Vertical Position (relative to hip)",
        "sub_y": "Knee Fore-Aft Displacement (positive = forward)",
        "sub_gyro": "Angular Velocity (Gyroscope Data)",
        "sub_flex": "Angular Rate (Flex Sensor Mode)",
        "sub_gyro_imu": "Gyroscope Angular Velocity",
        "legend_thigh": "Thigh Angle",
        "legend_above": "Above Hip",
        "legend_below": "Below Hip",
        "legend_y": "Knee Fore-Aft Disp. Y",
        "legend_rate": "Angular Rate",
        "legend_gx": "Gyro X-Axis",
        "legend_gy": "Gyro Y-Axis",
        "legend_gz": "Gyro Z-Axis",
        "flex_annotation": "Flex Sensor Data<br>(No Gyroscope)",
        "yaxis_angle": "Angle (°)",
        "yaxis_height": "Height (cm)",
        "yaxis_disp": "Fore-Aft Disp. (cm)",
        "yaxis_rate": "Angular Rate (°/s)",
        "yaxis_gyro": "Angular Velocity (°/s)",
        "xaxis_time": "Elapsed Time (s)",
        "colorbar_time": "Time Progress",
        "traj_chart_title": "Knee 3D Motion Trajectory",
        "traj_legend_knee": "Knee Trajectory",
        "traj_legend_line": "Trajectory Line",
        "traj_legend_hip": "Hip Origin",
        "traj_x": "X (cm) - Lateral",
        "traj_y": "Y (cm) - Fore-Aft",
        "traj_z": "Z (cm) - Vertical",
        # Font
        "font_family": "Times New Roman, serif",
        "mpl_font": "Times New Roman",
    },
}


def t(key: str) -> str:
    """回傳目前語言的翻譯字串"""
    lang = st.session_state.get("language", "zh")
    return TRANSLATIONS[lang].get(key, TRANSLATIONS["zh"].get(key, key))


# ===== 頁面設定 =====
st.set_page_config(
    page_title="膝蓋動作分析系統",
    page_icon="🦵",
    layout="wide",
    initial_sidebar_state="expanded",
)

# 字型設定（在 main() 內依語言動態切換）
plt.rcParams["axes.unicode_minus"] = False


# ===== 資料處理函式 =====
@st.cache_data
def load_csv_data(filepath_or_buffer, start_time_seconds=0, end_time_seconds=None):
    """載入 CSV 並篩選時間範圍"""
    df = pd.read_csv(filepath_or_buffer)

    # 轉換時間戳記為 datetime
    if "時間戳記" in df.columns:
        df["時間戳記"] = pd.to_datetime(df["時間戳記"])

    # 時間範圍篩選
    if "運行時間(s)" in df.columns:
        if end_time_seconds is None:
            end_time_seconds = df["運行時間(s)"].max()
        df = df[
            (df["運行時間(s)"] >= start_time_seconds)
            & (df["運行時間(s)"] <= end_time_seconds)
        ].copy()

    return df


def calculate_statistics(df):
    """計算統計指標"""
    if "角度(deg)" not in df.columns or len(df) == 0:
        return None

    angle = df["角度(deg)"].values
    return {
        "平均角度": np.mean(angle),
        "最大角度": np.max(angle),
        "最小角度": np.min(angle),
        "角度標準差": np.std(angle),
        "角度範圍": np.max(angle) - np.min(angle),
        "資料筆數": len(df),
    }


def find_csv_files(pattern="knee_data_*.csv"):
    """尋找所有符合 pattern 的 CSV 檔案"""
    csv_files = glob.glob(pattern)
    csv_files = [f for f in csv_files if "calibration" not in f]

    # 依檔名時間戳記排序（新到舊）
    def extract_timestamp(filename):
        match = re.search(r"knee_data_(\d{8}_\d{6})\.csv", filename)
        return match.group(1) if match else "00000000_000000"

    csv_files.sort(key=extract_timestamp, reverse=True)
    return csv_files


def get_file_info(filepath):
    """取得檔案資訊"""
    if os.path.exists(filepath):
        stat = os.stat(filepath)
        size_kb = stat.st_size / 1024
        modified = datetime.fromtimestamp(stat.st_mtime)
        return {
            "size": f"{size_kb:.1f} KB",
            "modified": modified.strftime("%Y-%m-%d %H:%M"),
        }
    return {"size": "N/A", "modified": "N/A"}


# ===== 視覺化函式 =====
def create_analysis_plot(df):
    """建立 4 合 1 互動式分析圖表 (Plotly)"""
    time_seconds = df["運行時間(s)"].values

    # 建立 4x1 子圖，X 軸連動
    fig = make_subplots(
        rows=4,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=(
            t("sub_angle"),
            t("sub_z"),
            t("sub_y"),
            t("sub_gyro"),  # 預設標題，Flex Sensor 模式會動態更新
        ),
    )

    # ===== 圖1: 大腿角度變化 =====
    # 填充區域
    fig.add_trace(
        go.Scatter(
            x=time_seconds,
            y=df["角度(deg)"],
            fill="tozeroy",
            fillcolor="rgba(33, 150, 243, 0.3)",
            line=dict(color="#2196F3", width=1.5),
            name=t("legend_thigh"),
            showlegend=True,
        ),
        row=1,
        col=1,
    )
    # 站立基準線 (0°)
    fig.add_hline(y=0, line_dash="dash", line_color="gray", opacity=0.5, row=1, col=1)
    # 水平位置線 (90°)
    fig.add_hline(y=90, line_dash="dash", line_color="red", opacity=0.5, row=1, col=1)

    # ===== 圖2: 膝蓋垂直位置 (Z軸) =====
    z_values = df["絕對Z(cm)"].values
    # 高於髖關節 (綠色)
    z_above = np.where(z_values > 0, z_values, 0)
    # 低於髖關節 (橘色)
    z_below = np.where(z_values <= 0, z_values, 0)

    fig.add_trace(
        go.Scatter(
            x=time_seconds,
            y=z_above,
            fill="tozeroy",
            fillcolor="rgba(76, 175, 80, 0.3)",
            line=dict(color="#4CAF50", width=1.5),
            name=t("legend_above"),
            showlegend=True,
        ),
        row=2,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=time_seconds,
            y=z_below,
            fill="tozeroy",
            fillcolor="rgba(255, 152, 0, 0.3)",
            line=dict(color="#FF9800", width=1.5),
            name=t("legend_below"),
            showlegend=True,
        ),
        row=2,
        col=1,
    )
    fig.add_hline(y=0, line_dash="dash", line_color="gray", opacity=0.5, row=2, col=1)

    # ===== 圖3: 膝蓋前後位移 (Y軸) =====
    fig.add_trace(
        go.Scatter(
            x=time_seconds,
            y=df["絕對Y(cm)"],
            fill="tozeroy",
            fillcolor="rgba(156, 39, 176, 0.3)",
            line=dict(color="#9C27B0", width=1.5),
            name=t("legend_y"),
            showlegend=True,
        ),
        row=3,
        col=1,
    )
    fig.add_hline(y=0, line_dash="dash", line_color="gray", opacity=0.5, row=3, col=1)

    # ===== 圖4: 角度變化率 / 陀螺儀 =====
    # 檢測是否為 Flex Sensor 資料
    gyro_sum = (
        abs(df["陀螺儀X(deg/s)"].sum())
        + abs(df["陀螺儀Y(deg/s)"].sum())
        + abs(df["陀螺儀Z(deg/s)"].sum())
    )
    is_flex_sensor = gyro_sum < 1.0

    if is_flex_sensor:
        # Flex Sensor 模式：顯示角度變化率
        angle_diff = np.diff(df["角度(deg)"].values, prepend=df["角度(deg)"].values[0])
        time_diff = np.diff(time_seconds, prepend=time_seconds[0])
        time_diff[time_diff == 0] = 0.02
        angular_velocity = angle_diff / time_diff

        fig.add_trace(
            go.Scatter(
                x=time_seconds,
                y=angular_velocity,
                line=dict(color="#FF5722", width=1),
                name=t("legend_rate"),
                showlegend=True,
            ),
            row=4,
            col=1,
        )
        # 更新第4圖標題
        fig.layout.annotations[3].update(text=t("sub_flex"))
        # 加入 Flex Sensor 標註
        fig.add_annotation(
            x=0.99,
            y=0.02,
            xref="x4 domain",
            yref="y4 domain",
            text=t("flex_annotation"),
            showarrow=False,
            font=dict(size=10),
            bgcolor="wheat",
            opacity=0.8,
            xanchor="right",
            yanchor="bottom",
        )
    else:
        # IMU 模式
        fig.add_trace(
            go.Scatter(
                x=time_seconds,
                y=df["陀螺儀X(deg/s)"],
                line=dict(color="#F44336", width=1),
                name=t("legend_gx"),
                showlegend=True,
            ),
            row=4,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=time_seconds,
                y=df["陀螺儀Y(deg/s)"],
                line=dict(color="#2196F3", width=1),
                name=t("legend_gy"),
                showlegend=True,
            ),
            row=4,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=time_seconds,
                y=df["陀螺儀Z(deg/s)"],
                line=dict(color="#4CAF50", width=1),
                name=t("legend_gz"),
                showlegend=True,
            ),
            row=4,
            col=1,
        )
        fig.layout.annotations[3].update(text=t("sub_gyro_imu"))

    fig.add_hline(y=0, line_dash="dash", line_color="gray", opacity=0.5, row=4, col=1)

    # ===== 整體佈局設定 =====
    font_family = t("font_family")
    fig.update_layout(
        title=dict(
            text=t("chart_main_title"),
            font=dict(size=18, family=font_family),
            x=0.5,
        ),
        font=dict(family=font_family, size=12),
        height=900,
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        hovermode="x unified",
    )

    # Y 軸標籤
    fig.update_yaxes(title_text=t("yaxis_angle"), row=1, col=1, range=[-10, 180])
    fig.update_yaxes(title_text=t("yaxis_height"), row=2, col=1)
    fig.update_yaxes(title_text=t("yaxis_disp"), row=3, col=1)
    if is_flex_sensor:
        fig.update_yaxes(title_text=t("yaxis_rate"), row=4, col=1)
    else:
        fig.update_yaxes(title_text=t("yaxis_gyro"), row=4, col=1)

    # X 軸標籤 (只在最下方顯示)
    fig.update_xaxes(title_text=t("xaxis_time"), row=4, col=1)

    return fig


def create_3d_trajectory(df):
    """建立互動式 3D 軌跡圖 (Plotly)"""
    x = df["絕對X(cm)"].values
    y = df["絕對Y(cm)"].values
    z = df["絕對Z(cm)"].values

    # 時間進程顏色
    colors = np.linspace(0, 1, len(x))

    font_family = t("font_family")
    fig = go.Figure()

    # 軌跡點 (散點圖)
    fig.add_trace(
        go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="markers",
            marker=dict(
                size=4,
                color=colors,
                colorscale="Viridis",
                opacity=0.7,
                colorbar=dict(title=t("colorbar_time"), thickness=15, len=0.6),
            ),
            name=t("traj_legend_knee"),
        )
    )

    # 連接線
    fig.add_trace(
        go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="lines",
            line=dict(color="gray", width=1),
            opacity=0.3,
            name=t("traj_legend_line"),
            showlegend=False,
        )
    )

    # 髖關節原點
    fig.add_trace(
        go.Scatter3d(
            x=[0],
            y=[0],
            z=[0],
            mode="markers",
            marker=dict(size=12, color="red", symbol="diamond"),
            name=t("traj_legend_hip"),
        )
    )

    fig.update_layout(
        title=dict(
            text=t("traj_chart_title"),
            font=dict(size=18, family=font_family),
            x=0.5,
        ),
        font=dict(family=font_family, size=12),
        scene=dict(
            xaxis_title=t("traj_x"),
            yaxis_title=t("traj_y"),
            zaxis_title=t("traj_z"),
            aspectmode="data",
        ),
        height=700,
        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
    )

    return fig


def detect_high_raises(df, threshold=60):
    """偵測高抬腿動作"""
    if "角度(deg)" not in df.columns:
        return pd.DataFrame()
    return df[df["角度(deg)"] > threshold]


# ===== 主應用程式 =====
def main():
    # 初始化語言設定
    if "language" not in st.session_state:
        st.session_state.language = "zh"

    # ===== 側邊欄 =====
    with st.sidebar:
        # ── 語言切換（最頂端）──
        # Toggle 關閉 = 中文，開啟 = 英文
        is_english = st.toggle(
            "🌐 English",
            value=(st.session_state.language == "en"),
            key="lang_toggle_widget",
        )
        st.session_state.language = "en" if is_english else "zh"
        st.divider()

        # 動態設定 Matplotlib 字型
        mpl_font = t("mpl_font")
        plt.rcParams["font.sans-serif"] = [mpl_font, "Arial Unicode MS"]

        st.header(t("file_management"))

        # 資料來源選擇
        source = st.radio(
            t("select_source"), [t("source_existing"), t("source_upload")]
        )

        selected_file = None
        df = None

        if source == t("source_existing"):
            # 列出現有 CSV 檔案
            csv_files = find_csv_files()

            if csv_files:
                # 顯示檔案選擇器
                selected_file = st.selectbox(
                    t("select_file"),
                    csv_files,
                    format_func=lambda x: f"{os.path.basename(x)}",
                )

                # 顯示檔案資訊
                if selected_file:
                    info = get_file_info(selected_file)
                    st.caption(f"📊 大小: {info['size']} | 🕐 {info['modified']}")

                    # 刪除按鈕
                    st.markdown("---")
                    if st.button(t("delete_btn"), type="secondary"):
                        st.session_state.confirm_delete = selected_file

                    # 確認刪除對話框
                    if st.session_state.get("confirm_delete") == selected_file:
                        st.warning(
                            t("confirm_delete_msg").format(
                                os.path.basename(selected_file)
                            )
                        )
                        col1, col2 = st.columns(2)
                        with col1:
                            if st.button(t("confirm_delete_btn"), type="primary"):
                                try:
                                    os.remove(selected_file)
                                    st.success(t("delete_success"))
                                    st.session_state.confirm_delete = None
                                    st.rerun()
                                except Exception as e:
                                    st.error(t("delete_fail").format(e))
                        with col2:
                            if st.button(t("cancel_btn")):
                                st.session_state.confirm_delete = None
                                st.rerun()
            else:
                st.warning(t("no_csv"))

        else:  # 上傳新檔案
            uploaded_file = st.file_uploader(t("select_csv"), type=["csv"])

            if uploaded_file:
                st.success(t("upload_success").format(uploaded_file.name))

                # 儲存按鈕
                if st.button(t("save_btn"), type="primary"):
                    # 產生新檔名
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    new_filename = f"knee_data_{timestamp}.csv"

                    try:
                        # 讀取上傳內容並儲存
                        content = uploaded_file.getvalue()
                        with open(new_filename, "wb") as f:
                            f.write(content)
                        st.success(t("save_success").format(new_filename))
                        st.rerun()
                    except Exception as e:
                        st.error(t("save_fail").format(e))

                # 設定為當前分析檔案
                selected_file = uploaded_file

        st.markdown("---")

        # 時間範圍篩選（載入資料後才顯示）
        time_range = None
        if selected_file:
            st.header(t("time_range_header"))
            # 先載入資料取得時間範圍
            try:
                if isinstance(selected_file, str):
                    df_temp = pd.read_csv(selected_file)
                else:
                    df_temp = pd.read_csv(selected_file)
                    selected_file.seek(0)  # 重置檔案指標

                if "運行時間(s)" in df_temp.columns:
                    max_time = float(df_temp["運行時間(s)"].max())
                    time_range = st.slider(
                        t("time_range_label"),
                        min_value=0.0,
                        max_value=max_time,
                        value=(0.0, max_time),
                        step=0.1,
                    )
            except Exception:
                pass

        # 高抬腿閾值
        st.header(t("analysis_params"))
        threshold = st.slider(t("threshold_label"), 30, 90, 60, 5)

    # ===== 主區域 =====
    st.title(t("app_title"))
    st.markdown("---")

    if selected_file:
        try:
            # 載入資料
            if time_range:
                df = load_csv_data(selected_file, time_range[0], time_range[1])
            else:
                df = load_csv_data(selected_file)

            if df is None or len(df) == 0:
                st.error(t("no_data_error"))
                return

            # ===== 統計指標卡片 =====
            stats = calculate_statistics(df)
            if stats:
                st.subheader(t("stats_header"))
                col1, col2, col3, col4, col5 = st.columns(5)

                with col1:
                    st.metric(t("avg_angle"), f"{stats['平均角度']:.1f}°")
                with col2:
                    st.metric(t("max_angle"), f"{stats['最大角度']:.1f}°")
                with col3:
                    st.metric(t("min_angle"), f"{stats['最小角度']:.1f}°")
                with col4:
                    st.metric(t("std_dev"), f"{stats['角度標準差']:.1f}°")
                with col5:
                    st.metric(t("data_count"), f"{stats['資料筆數']}")

            st.markdown("---")

            # ===== 分析圖表 =====
            tab1, tab2, tab3 = st.tabs([t("tab_motion"), t("tab_3d"), t("tab_raw")])

            with tab1:
                st.subheader(t("motion_chart_title"))
                st.caption(t("motion_chart_hint"))
                fig1 = create_analysis_plot(df)
                st.plotly_chart(fig1, use_container_width=True)

                # 高抬腿偵測
                st.markdown("---")
                st.subheader(t("high_raise_header").format(threshold))
                high_raises = detect_high_raises(df, threshold)

                if len(high_raises) > 0:
                    col1, col2, col3 = st.columns(3)
                    with col1:
                        st.metric(t("detect_count"), f"{len(high_raises)} 次")
                    with col2:
                        st.metric(
                            t("detect_max"), f"{high_raises['角度(deg)'].max():.1f}°"
                        )
                    with col3:
                        st.metric(
                            t("detect_avg"), f"{high_raises['角度(deg)'].mean():.1f}°"
                        )
                else:
                    st.info(t("no_detection").format(threshold))

            with tab2:
                st.subheader(t("traj_title"))
                st.caption(t("traj_hint"))
                fig2 = create_3d_trajectory(df)
                st.plotly_chart(fig2, use_container_width=True)

            with tab3:
                st.subheader(t("raw_title"))
                st.dataframe(df, width="stretch", height=400)

                # 下載按鈕
                csv_data = df.to_csv(index=False).encode("utf-8-sig")
                st.download_button(
                    label=t("download_btn"),
                    data=csv_data,
                    file_name=f"knee_data_filtered_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                    mime="text/csv",
                )

        except Exception as e:
            st.error(t("process_error").format(e))
            st.exception(e)

    else:
        # 沒有選擇檔案時顯示說明
        st.info(t("no_file_info"))
        st.markdown(t("usage_md"))


if __name__ == "__main__":
    main()
