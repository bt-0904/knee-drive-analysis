# 膝蓋抬腿分析系統 - 無線資料採集使用說明

## 📋 系統概述

這是一個使用 **ESP32-C3 + Flex Sensor（彎曲感測器）** 的膝蓋抬腿動作分析系統，透過 Wi-Fi + MQTT 進行無線資料傳輸。

**系統架構**：

```
ESP32-C3 + Flex Sensor (綁在大腿)
    ↓ Wi-Fi
手機熱點 (Bt)
    ↓ 4G/5G
MQTT Broker (mqtt.singularinnovation-ai.com)
    ↓ 網際網路
電腦 (執行 Python 接收腳本)
```

---

## 🔧 硬體需求

1. **ESP32-C3 Super Mini**
2. **Flex Sensor（彎曲感測器）**
3. **10KΩ 電阻**（分壓電路用）
4. **行動電源**（供電給 ESP32）
5. **手機**（開啟熱點，帶在身上）
6. **電腦**（執行 Python 接收腳本）

**接線方式（分壓電路）**：

```
3.3V ──────┬────── Flex Sensor ──────┬────── GPIO4 (ADC)
           │                         │
           └─────── 10KΩ ────────────┴────── GND
```

| 連接點                         | ESP32-C3 |
| ------------------------------ | -------- |
| Flex Sensor 一端               | 3.3V     |
| Flex Sensor 另一端 + 10KΩ 一端 | GPIO4    |
| 10KΩ 另一端                    | GND      |

---

## 📱 使用步驟

### **步驟 1：手機設定熱點**

#### Android:

1. 設定 → 網路與網際網路 → 無線基地台與網路共用
2. 開啟「可攜式無線基地台」
3. 設定：
   - **網路名稱 (SSID)**: `Bt`
   - **密碼**: `bt_980904`

#### iOS:

1. 設定 → 個人熱點
2. 開啟「允許其他人加入」
3. Wi-Fi 密碼設為：`bt_980904`
4. 重新命名為：`Bt`（設定 → 一般 → 關於本機 → 名稱）

---

### **步驟 2：上傳程式到 ESP32**

1. 用 USB 連接 ESP32 到電腦
2. 在 VS Code 中開啟專案
3. 點擊 PlatformIO 側邊欄的 **Upload** 按鈕
4. 等待上傳完成

**如果上傳失敗**：

- 按住 **BOOT** 按鈕
- 按下 **RESET** 按鈕
- 釋放兩個按鈕，重新上傳

---

### **步驟 3：首次使用校正（重要！）**

首次使用或更換 Flex Sensor 時，需要進行校正以確保角度計算準確。

#### 校正流程：

1. **開啟 Serial Monitor**（PlatformIO 側邊欄 → Monitor）
2. **記錄平直時的 ADC 值**：
   - 將 Flex Sensor 保持平直（0°）
   - 觀察序列埠輸出的 `ADC` 欄位
   - 記錄穩定的數值（例如：2358）
3. **記錄彎曲時的 ADC 值**：
   - 將 Flex Sensor 彎曲至約 90°
   - 記錄穩定的數值（例如：1737）
4. **更新程式碼**：
   編輯 `src/main.cpp`（約第 48-49 行）：
   ```cpp
   #define OFFSET_FLAT_ADC 2358  // ← 你的平直 ADC 值
   #define OFFSET_BENT_ADC 1737  // ← 你的彎曲 ADC 值
   ```
5. **重新上傳程式**

#### 校正驗證：

上傳後觀察 Serial Monitor：

- 站立時應顯示約 **0°**
- 膝蓋水平時應顯示約 **90°**

> 💡 **提示**：如果角度方向相反（站立時 90°），交換兩個常數的值

---

### **步驟 4：測試連線（選用）**

1. 點擊 PlatformIO 的 **Monitor** 按鈕
2. 觀察序列埠輸出，應該會看到：

   ```
   ✓ Wi-Fi 連線成功！
   IP 位址: 192.168.x.x
   ✓ MQTT 連線成功！

   [samplingTask] 已啟動 (50Hz)
   [transmitTask] 已啟動 (2Hz)
   ```

3. 確認連線正常後，可以關閉 Monitor

---

### **步驟 5：在電腦執行接收腳本**

#### 安裝 Python 套件（首次使用）：

```bash
pip install paho-mqtt matplotlib pandas numpy
```

#### 執行接收腳本：

```bash
python mqtt_receiver.py
```

**預期輸出**：

```
============================================================
膝蓋抬腿分析系統 - MQTT 資料接收器
============================================================
CSV 檔案: knee_data_20260124_143025.csv
按 Ctrl+C 停止記錄
============================================================

正在連接到 mqtt.singularinnovation-ai.com:1883...
✓ 成功連接到 MQTT Broker
  伺服器: mqtt.singularinnovation-ai.com:1883
  訂閱主題: knee-drive/data

等待 ESP32 資料...
============================================================
[批次] 收到 25 筆資料 | 角度: 12.5° ~ 45.3° | 總計: 25 筆
[批次] 收到 25 筆資料 | 角度: 43.2° ~ 52.1° | 總計: 50 筆
```

---

### **步驟 6：開始跑步測試**

1. **準備階段**：
   - 將 Flex Sensor 固定在大腿上（沿大腿方向）
   - ESP32 固定在大腿或腰帶上
   - 行動電源供電
   - 手機放在口袋或臂套（距離 ESP32 5-10 公尺內）
   - 電腦執行 `mqtt_receiver.py`

2. **開始測試**：
   - ESP32 開機後會自動連接 Wi-Fi 並開始傳輸
   - **不需要校正等待時間**，開機即可使用
   - 開始跑步或原地抬腿
   - 電腦會即時顯示資料並自動儲存 CSV

3. **結束記錄**：
   - 在電腦上按 **Ctrl+C** 停止記錄
   - CSV 檔案會自動儲存

---

## 📊 資料格式

### **CSV 檔案欄位**

| 欄位        | 說明           | 單位  |
| ----------- | -------------- | ----- |
| 時間戳記    | 資料接收時間   | -     |
| 運行時間(s) | ESP32 運行時間 | 秒    |
| 角度(deg)   | 大腿抬起角度   | 度    |
| ΔY(cm)      | 膝蓋前後位移   | 公分  |
| ΔZ(cm)      | 膝蓋上下位移   | 公分  |
| 是否穩定    | 數值穩定度     | 是/否 |

### **MQTT JSON 資料格式（批次版）**

```json
{
  "type": "batch",
  "n": 25,
  "d": [
    {
      "t": 1.23,
      "a": 45.0,
      "s": 1,
      "dy": 28.3,
      "dz": -12.5
    },
    {
      "t": 1.25,
      "a": 46.2,
      "s": 1,
      "dy": 29.1,
      "dz": -11.8
    }
  ]
}
```

**欄位縮寫說明**：

| 欄位 | 全名      | 說明                 |
| ---- | --------- | -------------------- |
| `t`  | timestamp | 運行時間（秒）       |
| `a`  | angle     | 大腿角度（度）       |
| `s`  | stable    | 是否穩定（0/1）      |
| `dy` | deltaY    | 膝蓋前後位移（公分） |
| `dz` | deltaZ    | 膝蓋上下位移（公分） |

---

## ⚙️ 進階設定

### **修改 Wi-Fi 設定**

編輯 `src/main.cpp`（約第 82-83 行）：

```cpp
const char* WIFI_SSID = "你的WiFi名稱";
const char* WIFI_PASSWORD = "你的WiFi密碼";
```

### **修改 MQTT Topic**

編輯 `src/main.cpp`（約第 90 行）：

```cpp
const char* MQTT_TOPIC = "knee-drive/data";  // 改成你的 Topic
```

同時修改 `mqtt_receiver.py`：

```python
MQTT_TOPIC = "knee-drive/data"  # 改成相同的 Topic
```

### **調整取樣/傳輸頻率**

編輯 `src/main.cpp`（約第 32-35 行）：

```cpp
#define SAMPLE_INTERVAL_MS 20   // 取樣間隔 (50Hz = 20ms)
#define SEND_INTERVAL_MS 500    // 發送間隔 (2Hz = 500ms)
#define BATCH_SIZE 25           // 每批次筆數
#define RING_BUFFER_SIZE 200    // 緩衝區大小
```

### **修改大腿長度**

編輯 `src/main.cpp`（約第 56 行）：

```cpp
#define THIGH_LENGTH 45.0  // 大腿長度 (公分)
```

---

## 🔍 故障排除

### **問題 1：ADC 讀數不變化**

**檢查項目**：

- Flex Sensor 接線是否正確
- 10KΩ 電阻是否連接
- GPIO4 腳位接觸是否良好

**解決方法**：

- 重新檢查分壓電路接線
- 確認 3.3V 和 GND 連接正確
- 嘗試更換 Flex Sensor

---

### **問題 2：角度顯示不準確**

**可能原因**：

- 校正值不正確
- Flex Sensor 安裝位置偏移

**解決方法**：

1. 重新進行校正程序
2. 確認 `OFFSET_FLAT_ADC` 和 `OFFSET_BENT_ADC` 數值正確
3. 確保 Flex Sensor 固定牢靠

---

### **問題 3：ESP32 無法連接 Wi-Fi**

**檢查項目**：

- 手機熱點是否已開啟
- SSID 和密碼是否正確（區分大小寫）
- ESP32 是否在手機附近（5-10 公尺內）

**解決方法**：

- 重新檢查 `main.cpp` 中的 Wi-Fi 設定
- 重新上傳程式到 ESP32
- 重啟手機熱點

---

### **問題 4：Python 腳本無法連接 MQTT**

**檢查項目**：

- 電腦是否有網路連線
- MQTT 伺服器是否正常運作

**解決方法**：

```bash
# 測試 MQTT 伺服器連線
ping mqtt.singularinnovation-ai.com
```

---

### **問題 5：收不到資料**

**檢查項目**：

- ESP32 序列埠是否顯示「資料已發送」
- Python 腳本是否顯示「等待 ESP32 資料...」
- Topic 名稱是否一致

**解決方法**：

- 確認 ESP32 和 Python 使用相同的 Topic
- 重啟 ESP32 和 Python 腳本

---

### **問題 6：資料延遲或遺失**

**可能原因**：

- 手機訊號不佳
- Ring Buffer 溢出

**解決方法**：

- 確保手機有良好的 4G/5G 訊號
- 縮短 ESP32 與手機的距離
- 觀察序列埠輸出的 `dropped` 計數

---

## 📈 資料分析

### **使用內建分析腳本**

```bash
python analyze_knee_motion.py
```

會自動：

1. 尋找最新的 CSV 檔案
2. 計算統計數據（平均角度、最大角度）
3. 產生視覺化圖表（角度變化、膝蓋軌跡等）

### **使用 Python 自訂分析**

```python
import pandas as pd
import matplotlib.pyplot as plt

# 讀取 CSV
df = pd.read_csv('knee_data_20260124_143025.csv')

# 繪製角度變化圖
plt.figure(figsize=(12, 6))
plt.plot(df['運行時間(s)'], df['角度(deg)'])
plt.xlabel('時間 (秒)')
plt.ylabel('大腿角度 (度)')
plt.title('跑步時膝蓋抬腿角度變化')
plt.grid(True)
plt.show()

# 計算統計數據
print(f"平均抬腿角度: {df['角度(deg)'].mean():.1f}°")
print(f"最大抬腿角度: {df['角度(deg)'].max():.1f}°")
```

### **使用 Excel 分析**

1. 開啟 CSV 檔案
2. 選取資料範圍
3. 插入 → 圖表 → 折線圖
4. 分析趨勢與統計數據

---

## 🎯 使用場景

### **場景 1：跑步機測試**

- 電腦放在跑步機旁
- 即時監控抬腿動作
- 適合短距離測試

### **場景 2：操場跑步**

- 手機放在口袋/臂套
- ESP32 + Flex Sensor 綁在大腿
- 可自由移動（Wi-Fi 範圍內）

### **場景 3：室內訓練**

- 連接家中 Wi-Fi
- 長時間資料記錄
- 適合詳細分析

---

## 📞 技術支援

如有問題，請檢查：

1. ESP32 序列埠輸出（Serial Monitor）
2. Python 腳本輸出訊息
3. Wi-Fi 和 MQTT 連線狀態
4. Flex Sensor 接線和校正值

---

## 📝 版本資訊

- **版本**: 2.0.0
- **更新日期**: 2026-01-24
- **相容硬體**: ESP32-C3 Super Mini + Flex Sensor
- **通訊協定**: Wi-Fi + MQTT（批次傳輸）
- **資料格式**: JSON（批次）→ CSV
- **取樣頻率**: 50Hz 取樣 + 2Hz 批次傳輸

---

**祝你測試順利！🏃‍♂️💨**
