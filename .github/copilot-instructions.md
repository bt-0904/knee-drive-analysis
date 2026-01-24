# Copilot Instructions - Knee Drive Analysis

## 專案概述

嵌入式運動分析系統：ESP32-C3 + **Flex Sensor** 即時追蹤跑步膝蓋抬腿動作，透過 MQTT 無線傳輸至 Python 接收端儲存為 CSV。

**資料流**：Flex Sensor ADC (50Hz) → FreeRTOS Ring Buffer → MQTT 批次 (2Hz) → Python → CSV

## 🚫 AI 禁止操作

1. **禁止執行 `pio`/`platformio` 終端指令** - 使用 PlatformIO 側邊欄 UI 操作
2. **禁止修改 build flags** - `ARDUINO_USB_MODE=1` 和 `ARDUINO_USB_CDC_ON_BOOT=1` 必須保留
3. **禁止使用 GPIO8/GPIO9 作為 I2C/ADC** - 這些是 strapping pins，會導致開機失敗（但 GPIO8 可用於 LED）
4. **lib_deps 格式** - 使用 `author/library@^version` 格式

## 硬體關鍵限制

| 項目            | 設定        | 原因                     |
| --------------- | ----------- | ------------------------ |
| Flex Sensor ADC | **GPIO4**   | ADC1_CH4，避開 strapping |
| LED 指示燈      | **GPIO8**   | 內建 LED，LOW=亮         |
| 序列埠          | 115200 baud | USB CDC 模式             |
| Ring Buffer     | 200 筆      | 可承受 4 秒網路中斷      |
| 批次傳輸        | 25 筆/500ms | 降低 MQTT overhead       |

**Flex Sensor 接線**：

```
3.3V ------+------ Flex Sensor ------+------ GPIO4 (ADC)
           |                         |
           +------- 10K Ohm ---------+------ GND
```

## 架構：FreeRTOS 雙任務設計

```
┌─────────────────┐    Ring Buffer    ┌─────────────────┐
│  samplingTask   │ ──────────────→ │  transmitTask   │
│  (50Hz, Core 0) │   Mutex保護       │  (2Hz, Core 1)  │
│  優先權 3       │   200筆容量       │  優先權 1       │
│  Stack: 4KB     │                   │  Stack: 8KB     │
└─────────────────┘                   └─────────────────┘
```

**關鍵常數** ([src/main.cpp#L14-L17](src/main.cpp#L14-L17))：

- `SAMPLE_INTERVAL_MS=20` (50Hz 取樣)
- `SEND_INTERVAL_MS=500` (2Hz 發送)
- `BATCH_SIZE=25` (每批次筆數)
- `RING_BUFFER_SIZE=200` (緩衝區容量)

## 動態校正系統（4 階段，約 15 秒）

**校正狀態機** ([src/main.cpp#L155-L163](src/main.cpp#L155-L163))：

## 2 點校正系統（Flex Sensor 簡化版，約 9 秒）

**校正狀態機**：

| 階段 | 狀態         | 時間 | LED 行為     | 使用者動作           |
| ---- | ------------ | ---- | ------------ | -------------------- |
| 0    | CAL_INIT     | 3 秒 | 慢閃 (1s)    | 準備好               |
| 1    | CAL_FLAT     | 3 秒 | **熄滅**     | Flex Sensor 保持平直 |
| 2    | CAL_BENT     | 3 秒 | 快閃 (500ms) | Flex Sensor 彎曲 90° |
| 3    | CAL_COMPLETE | -    | 熄滅         | 開始測量             |

校正參數：

- **calibFlatAdc**：平直時 ADC 讀數（約 2358）
- **calibBentAdc**：彎曲時 ADC 讀數（約 1737）
- ADC 值越低 = 角度越大（電阻反比）

## MQTT 資料格式（批次版，相容模式）

```json
{"type":"batch","n":25,"d":[
  {"t":1.23,"a":45.0,"s":1,"dy":28.3,"dz":-12.5},
  ...
]}
```

欄位縮寫：`t`=timestamp, `a`=angle, `s`=stable, `dy/dz`=delta

## 程式碼慣例

- **註解語言**：英文（避免編碼問題）
- **數值格式**：`printf("%6.1f", value)` 對齊輸出
- **錯誤處理**：初始化失敗進入 `while(1) delay(1000)` 無限迴圈
- **LED 控制**：`LED_ON=LOW`, `LED_OFF=HIGH`（低電位觸發）

## 核心演算法

**ADC 轉角度（線性映射）**：

```cpp
float angle = (float)(calibFlatAdc - rawValue) / (calibFlatAdc - calibBentAdc) * 90.0;
```

**座標系統**：原點=髖關節，0°=站立，90°=膝蓋水平

```cpp
kneeY = THIGH_LENGTH * sin(angleRad);  // 前後
kneeZ = -THIGH_LENGTH * cos(angleRad); // 上下
```

## 開發工作流程

1. **編譯/上傳**：使用 PlatformIO 側邊欄按鈕（非終端機）
2. **首次上傳**：按住 BOOT → 按 RESET → 放開 BOOT → 上傳
3. **除錯**：Serial Monitor 查看 ADC 讀數、校正進度、任務狀態
4. **資料接收**：`python mqtt_receiver.py` 自動產生 `knee_data_YYYYMMDD_HHMMSS.csv`
5. **分析視覺化**：`python analyze_knee_motion.py`

## 函式庫配置

```ini
# platformio.ini
lib_deps =
    knolleary/PubSubClient@^2.8
    bblanchon/ArduinoJson@^7.0.0
```

---

**最後更新**: 2026-01-03
