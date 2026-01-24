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

# ===== 頁面設定 =====
st.set_page_config(
    page_title="膝蓋動作分析系統",
    page_icon="🦵",
    layout="wide",
    initial_sidebar_state="expanded",
)

# 設定中文字體（Windows 系統使用微軟正黑體）
plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei", "SimHei", "Arial Unicode MS"]
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
            "大腿抬起角度變化",
            "膝蓋垂直位置 (相對於髖關節)",
            "膝蓋前後位移 (正值=向前)",
            "角速度 (陀螺儀資料)",  # 預設標題，Flex Sensor 模式會動態更新
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
            name="大腿角度",
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
            name="高於髖關節",
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
            name="低於髖關節",
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
            name="膝蓋前後位移 Y",
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
                name="角度變化率",
                showlegend=True,
            ),
            row=4,
            col=1,
        )
        # 更新第4圖標題
        fig.layout.annotations[3].update(text="角度變化率 (Flex Sensor 模式)")
        # 加入 Flex Sensor 標註
        fig.add_annotation(
            x=0.99,
            y=0.02,
            xref="x4 domain",
            yref="y4 domain",
            text="Flex Sensor 資料<br>(無陀螺儀)",
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
                name="陀螺儀 X軸",
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
                name="陀螺儀 Y軸",
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
                name="陀螺儀 Z軸",
                showlegend=True,
            ),
            row=4,
            col=1,
        )
        fig.layout.annotations[3].update(text="陀螺儀角速度")

    fig.add_hline(y=0, line_dash="dash", line_color="gray", opacity=0.5, row=4, col=1)

    # ===== 整體佈局設定 =====
    fig.update_layout(
        title=dict(
            text="膝蓋抬腿動作分析",
            font=dict(size=18, family="Microsoft JhengHei, sans-serif"),
            x=0.5,
        ),
        font=dict(family="Microsoft JhengHei, sans-serif", size=12),
        height=900,
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        hovermode="x unified",
    )

    # Y 軸標籤
    fig.update_yaxes(title_text="角度 (度)", row=1, col=1, range=[-10, 180])
    fig.update_yaxes(title_text="高度 (cm)", row=2, col=1)
    fig.update_yaxes(title_text="前後位移 (cm)", row=3, col=1)
    if is_flex_sensor:
        fig.update_yaxes(title_text="角度變化率 (°/s)", row=4, col=1)
    else:
        fig.update_yaxes(title_text="角速度 (°/s)", row=4, col=1)

    # X 軸標籤 (只在最下方顯示)
    fig.update_xaxes(title_text="運行時間 (秒)", row=4, col=1)

    return fig


def create_3d_trajectory(df):
    """建立互動式 3D 軌跡圖 (Plotly)"""
    x = df["絕對X(cm)"].values
    y = df["絕對Y(cm)"].values
    z = df["絕對Z(cm)"].values

    # 時間進程顏色
    colors = np.linspace(0, 1, len(x))

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
                colorbar=dict(title="時間進程", thickness=15, len=0.6),
            ),
            name="膝蓋軌跡",
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
            name="軌跡線",
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
            name="髖關節原點",
        )
    )

    fig.update_layout(
        title=dict(
            text="膝蓋 3D 運動軌跡",
            font=dict(size=18, family="Microsoft JhengHei, sans-serif"),
            x=0.5,
        ),
        font=dict(family="Microsoft JhengHei, sans-serif", size=12),
        scene=dict(
            xaxis_title="X (cm) - 左右",
            yaxis_title="Y (cm) - 前後",
            zaxis_title="Z (cm) - 上下",
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
    st.title("🦵 膝蓋動作分析系統")
    st.markdown("---")

    # ===== 側邊欄 =====
    with st.sidebar:
        st.header("📂 檔案管理")

        # 資料來源選擇
        source = st.radio("選擇資料來源", ["📂 現有檔案", "📤 上傳新檔案"])

        selected_file = None
        df = None

        if source == "📂 現有檔案":
            # 列出現有 CSV 檔案
            csv_files = find_csv_files()

            if csv_files:
                # 顯示檔案選擇器
                selected_file = st.selectbox(
                    "選擇檔案",
                    csv_files,
                    format_func=lambda x: f"{os.path.basename(x)}",
                )

                # 顯示檔案資訊
                if selected_file:
                    info = get_file_info(selected_file)
                    st.caption(f"📊 大小: {info['size']} | 🕐 {info['modified']}")

                    # 刪除按鈕
                    st.markdown("---")
                    if st.button("🗑️ 刪除此檔案", type="secondary"):
                        st.session_state.confirm_delete = selected_file

                    # 確認刪除對話框
                    if st.session_state.get("confirm_delete") == selected_file:
                        st.warning(
                            f"⚠️ 確定要刪除 **{os.path.basename(selected_file)}**？"
                        )
                        col1, col2 = st.columns(2)
                        with col1:
                            if st.button("✅ 確認刪除", type="primary"):
                                try:
                                    os.remove(selected_file)
                                    st.success("✅ 檔案已刪除！")
                                    st.session_state.confirm_delete = None
                                    st.rerun()
                                except Exception as e:
                                    st.error(f"❌ 刪除失敗: {e}")
                        with col2:
                            if st.button("❌ 取消"):
                                st.session_state.confirm_delete = None
                                st.rerun()
            else:
                st.warning("📭 找不到任何 CSV 檔案")

        else:  # 上傳新檔案
            uploaded_file = st.file_uploader("選擇 CSV 檔案", type=["csv"])

            if uploaded_file:
                st.success(f"✅ 已載入: {uploaded_file.name}")

                # 儲存按鈕
                if st.button("💾 儲存到紀錄", type="primary"):
                    # 產生新檔名
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    new_filename = f"knee_data_{timestamp}.csv"

                    try:
                        # 讀取上傳內容並儲存
                        content = uploaded_file.getvalue()
                        with open(new_filename, "wb") as f:
                            f.write(content)
                        st.success(f"✅ 已儲存為: {new_filename}")
                        st.rerun()
                    except Exception as e:
                        st.error(f"❌ 儲存失敗: {e}")

                # 設定為當前分析檔案
                selected_file = uploaded_file

        st.markdown("---")

        # 時間範圍篩選（載入資料後才顯示）
        time_range = None
        if selected_file:
            st.header("⏱️ 時間範圍")
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
                        "選擇時間範圍 (秒)",
                        min_value=0.0,
                        max_value=max_time,
                        value=(0.0, max_time),
                        step=0.1,
                    )
            except Exception:
                pass

        # 高抬腿閾值
        st.header("🎯 分析參數")
        threshold = st.slider("高抬腿閾值 (度)", 30, 90, 60, 5)

    # ===== 主區域 =====
    if selected_file:
        try:
            # 載入資料
            if time_range:
                df = load_csv_data(selected_file, time_range[0], time_range[1])
            else:
                df = load_csv_data(selected_file)

            if df is None or len(df) == 0:
                st.error("❌ 無法載入資料或資料為空")
                return

            # ===== 統計指標卡片 =====
            stats = calculate_statistics(df)
            if stats:
                st.subheader("📊 統計指標")
                col1, col2, col3, col4, col5 = st.columns(5)

                with col1:
                    st.metric("平均角度", f"{stats['平均角度']:.1f}°")
                with col2:
                    st.metric("最大角度", f"{stats['最大角度']:.1f}°")
                with col3:
                    st.metric("最小角度", f"{stats['最小角度']:.1f}°")
                with col4:
                    st.metric("標準差", f"{stats['角度標準差']:.1f}°")
                with col5:
                    st.metric("資料筆數", f"{stats['資料筆數']}")

            st.markdown("---")

            # ===== 分析圖表 =====
            tab1, tab2, tab3 = st.tabs(["📈 動作分析", "🌐 3D 軌跡", "📋 原始資料"])

            with tab1:
                st.subheader("📈 4 合 1 動作分析圖表")
                st.caption(
                    "💡 提示：可使用滑鼠滾輪縮放、拖曳平移，雙擊重置視圖。每個子圖可獨立縮放 Y 軸，X 軸（時間軸）連動。"
                )
                fig1 = create_analysis_plot(df)
                st.plotly_chart(fig1, use_container_width=True)

                # 高抬腿偵測
                st.markdown("---")
                st.subheader(f"🦵 高抬腿偵測 (角度 > {threshold}°)")
                high_raises = detect_high_raises(df, threshold)

                if len(high_raises) > 0:
                    col1, col2, col3 = st.columns(3)
                    with col1:
                        st.metric("偵測次數", f"{len(high_raises)} 次")
                    with col2:
                        st.metric("最高角度", f"{high_raises['角度(deg)'].max():.1f}°")
                    with col3:
                        st.metric(
                            "平均高抬角度", f"{high_raises['角度(deg)'].mean():.1f}°"
                        )
                else:
                    st.info(f"未偵測到角度超過 {threshold}° 的高抬腿動作")

            with tab2:
                st.subheader("🌐 膝蓋 3D 運動軌跡")
                st.caption("💡 提示：可使用滑鼠拖曳旋轉視角、滾輪縮放，雙擊重置視圖。")
                fig2 = create_3d_trajectory(df)
                st.plotly_chart(fig2, use_container_width=True)

            with tab3:
                st.subheader("📋 原始資料預覽")
                st.dataframe(df, width="stretch", height=400)

                # 下載按鈕
                csv_data = df.to_csv(index=False).encode("utf-8-sig")
                st.download_button(
                    label="📥 下載篩選後的 CSV",
                    data=csv_data,
                    file_name=f"knee_data_filtered_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                    mime="text/csv",
                )

        except Exception as e:
            st.error(f"❌ 處理資料時發生錯誤: {e}")
            st.exception(e)

    else:
        # 沒有選擇檔案時顯示說明
        st.info("👈 請從左側選擇或上傳 CSV 檔案開始分析")

        st.markdown(
            """
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
        """
        )


if __name__ == "__main__":
    main()
