"""
MQTT 資料接收器 - 膝蓋抬腿分析系統
接收 ESP32 透過 MQTT 傳送的感測器資料並儲存為 CSV
"""

import paho.mqtt.client as mqtt
import json
import csv
import os
from datetime import datetime
import time

# ===== MQTT 設定 =====
MQTT_BROKER = "mqtt.singularinnovation-ai.com"
MQTT_PORT = 1883
MQTT_USER = "singular"
MQTT_PASSWORD = "Singular#1234"
MQTT_TOPIC = "knee-drive/data"

# ===== CSV 檔案設定 =====
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_FILENAME = f"knee_data_{timestamp}.csv"

# CSV 欄位
CSV_HEADERS = [
    "時間戳記",
    "運行時間(s)",
    "角度(deg)",
    "ΔX(cm)",
    "ΔY(cm)",
    "ΔZ(cm)",
    "絕對X(cm)",
    "絕對Y(cm)",
    "絕對Z(cm)",
    "是否穩定",
    "加速度X(g)",
    "加速度Y(g)",
    "加速度Z(g)",
    "陀螺儀X(deg/s)",
    "陀螺儀Y(deg/s)",
    "陀螺儀Z(deg/s)",
]

# ===== 全域變數 =====
csv_file = None
csv_writer = None
data_count = 0
start_time = None


# ===== MQTT 回調函數 =====
def on_connect(client, userdata, flags, rc):
    """連線成功時的回調"""
    if rc == 0:
        print("✓ 成功連接到 MQTT Broker")
        print(f"  伺服器: {MQTT_BROKER}:{MQTT_PORT}")
        print(f"  訂閱主題: {MQTT_TOPIC}")
        client.subscribe(MQTT_TOPIC)
        print("\n等待 ESP32 資料...")
        print("=" * 60)
    else:
        print(f"✗ 連線失敗，錯誤碼: {rc}")


def on_message(client, userdata, msg):
    """收到訊息時的回調"""
    global csv_writer, data_count, start_time

    try:
        # 解析 JSON 資料
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)

        # 檢查是否為校正訊息
        msg_type = data.get("type", "data")

        if msg_type == "connection_test":
            # ESP32 連線測試訊息
            print("\n" + "=" * 60)
            print("🔗 ESP32 已連線！")
            print(f"   裝置: {data.get('device', 'Unknown')}")
            print(f"   狀態: {data.get('status', 'Unknown')}")
            print("=" * 60)
            print("\n等待校正開始...")
            return

        if msg_type == "calibration":
            # 校正進度訊息
            step = data.get("step", "")
            action = data.get("action", "")
            progress = data.get("progress", 0)
            samples = data.get("samples", 0)
            accel = data.get("accel", {})

            # 清除當前行並顯示校正狀態
            print("\r" + " " * 80, end="\r")  # 清除行

            action_text = {
                "waiting": "⏳ 準備中",
                "stand": "📍 站立",
                "lift": "🦵 抬腳",
                "analyzing": "📊 分析中",
            }.get(action, action)

            # 顯示進度條
            if progress > 0:
                bar_length = 20
                filled = int(bar_length * progress / 100)
                bar = "█" * filled + "░" * (bar_length - filled)
                print(
                    f"校正 [{step}] {action_text} [{bar}] {progress}% | 樣本:{samples}"
                )
            else:
                print(
                    f"校正 [{step}] {action_text} | 加速度: X={accel.get('x', 0):.2f} Y={accel.get('y', 0):.2f} Z={accel.get('z', 0):.2f}"
                )
            return

        elif msg_type == "calibration_complete":
            # 校正完成訊息
            print("\n" + "=" * 60)
            print("✓ 校正完成！")
            print(f"  主軸: {['X', 'Y', 'Z'][data.get('primary_axis', 0)]}")
            print(f"  重力軸: {['X', 'Y', 'Z'][data.get('gravity_axis', 0)]}")
            print(
                f"  軸向符號: {'+1 (抬腳增加)' if data.get('axis_sign', 1) > 0 else '-1 (抬腳減少)'}"
            )
            print(f"  站立平均值: {data.get('stand_avg', 0):.3f}")
            print(f"  抬腳平均值: {data.get('lift_avg', 0):.3f}")
            print("=" * 60)
            print("\n開始接收測量資料...")
            return

        # 記錄開始時間
        if start_time is None:
            start_time = time.time()

        # 提取資料
        timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        elapsed = data.get("elapsed_time", 0)
        angle = data.get("angle", 0)
        stable = data.get("stable", False)

        delta = data.get("delta", {})
        absolute = data.get("absolute", {})
        accel = data.get("accel", {})
        gyro = data.get("gyro", {})

        # 寫入 CSV
        row = [
            timestamp_str,
            f"{elapsed:.2f}",
            f"{angle:.1f}",
            f"{delta.get('x', 0):.1f}",
            f"{delta.get('y', 0):.1f}",
            f"{delta.get('z', 0):.1f}",
            f"{absolute.get('x', 0):.1f}",
            f"{absolute.get('y', 0):.1f}",
            f"{absolute.get('z', 0):.1f}",
            "是" if stable else "否",
            f"{accel.get('x', 0):.3f}",
            f"{accel.get('y', 0):.3f}",
            f"{accel.get('z', 0):.3f}",
            f"{gyro.get('x', 0):.1f}",
            f"{gyro.get('y', 0):.1f}",
            f"{gyro.get('z', 0):.1f}",
        ]

        csv_writer.writerow(row)

        data_count += 1

        # 每 5 筆寫入一次檔案（加速處理）
        if data_count % 5 == 0:
            csv_file.flush()

        # 即時顯示（每筆都顯示）
        elapsed_total = time.time() - start_time
        print(
            f"[{data_count:4d}] {angle:5.1f}° | "
            f"Y:{delta.get('y', 0):6.1f} Z:{delta.get('z', 0):6.1f} | "
            f"{'✓' if stable else '⏳'} | "
            f"{elapsed_total:.1f}s",
            end="\r",  # 使用 \r 讓顯示在同一行更新（更快）
        )

        # 每 10 筆換行一次，避免畫面混亂
        if data_count % 10 == 0:
            print()  # 換行

    except json.JSONDecodeError as e:
        print(f"✗ JSON 解析錯誤: {e}")
    except Exception as e:
        print(f"✗ 處理資料時發生錯誤: {e}")


def on_disconnect(client, userdata, rc):
    """斷線時的回調"""
    if rc != 0:
        print(f"\n⚠️ 意外斷線，錯誤碼: {rc}")
        print("嘗試重新連線...")


# ===== 主程式 =====
def main():
    global csv_file, csv_writer

    print("=" * 60)
    print("膝蓋抬腿分析系統 - MQTT 資料接收器")
    print("=" * 60)
    print(f"CSV 檔案: {CSV_FILENAME}")
    print(f"按 Ctrl+C 停止記錄")
    print("=" * 60)

    # 建立 CSV 檔案
    csv_file = open(CSV_FILENAME, "w", newline="", encoding="utf-8-sig")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(CSV_HEADERS)

    # 建立 MQTT 客戶端
    client = mqtt.Client(client_id=f"KneeAnalyzer-{int(time.time())}")
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD)

    # 優化 MQTT 接收速度
    client.max_inflight_messages_set(20)  # 增加並行訊息數
    client.max_queued_messages_set(0)  # 不限制佇列大小

    # 設定回調函數
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    try:
        # 連接到 MQTT Broker
        print(f"\n正在連接到 {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)

        # 開始接收資料（高速模式）
        client.loop_forever()

    except KeyboardInterrupt:
        print("\n\n使用者中斷，正在停止...")
    except Exception as e:
        print(f"\n✗ 發生錯誤: {e}")
    finally:
        # 清理資源
        client.disconnect()
        if csv_file:
            csv_file.close()

        print("\n" + "=" * 60)
        print(f"✓ 資料記錄完成")
        print(f"  共記錄: {data_count} 筆資料")
        print(f"  檔案位置: {os.path.abspath(CSV_FILENAME)}")
        if start_time:
            total_time = time.time() - start_time
            print(f"  記錄時長: {total_time:.1f} 秒")
            if data_count > 0:
                print(f"  平均取樣率: {data_count/total_time:.1f} Hz")
        print("=" * 60)


if __name__ == "__main__":
    main()
