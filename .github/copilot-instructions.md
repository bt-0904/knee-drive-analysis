# Copilot Instructions - Knee Drive Analysis

## 專案概述

嵌入式運動分析系統：ESP32-C3 + ICM-20948 IMU 即時追蹤跑步膝蓋抬腿動作，透過 MQTT 無線傳輸至 Python 接收端儲存為 CSV。

**資料流**：IMU (50Hz) → FreeRTOS Ring Buffer → MQTT 批次 (2Hz) → Python → CSV

## 🚫 AI 禁止操作

1. **禁止執行 `pio`/`platformio` 終端指令** - 使用 PlatformIO 側邊欄 UI 操作
2. **禁止修改 build flags** - `ARDUINO_USB_MODE=1` 和 `ARDUINO_USB_CDC_ON_BOOT=1` 必須保留
3. **禁止使用 GPIO8/GPIO9 作為 I2C** - 這些是 strapping pins，會導致開機失敗（但 GPIO8 可用於 LED）
4. **lib_deps 格式** - SparkFun 需用 GitHub URL，其他可用 `author/library@^version`

## 硬體關鍵限制

| 項目        | 設定                     | 原因                      |
| ----------- | ------------------------ | ------------------------- |
| I2C 接線    | **SDA=GPIO5, SCL=GPIO6** | GPIO8/9 是 strapping pins |
| LED 指示燈  | **GPIO8**                | 內建 LED，LOW=亮          |
| 序列埠      | 115200 baud              | USB CDC 模式              |
| Ring Buffer | 200 筆                   | 可承受 4 秒網路中斷       |
| 批次傳輸    | 25 筆/500ms              | 降低 MQTT overhead        |

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

| 階段 | 狀態          | 時間 | LED 行為       | 使用者動作 |
| ---- | ------------- | ---- | -------------- | ---------- |
| 0    | CAL_INIT      | 3 秒 | 慢閃 (1s)      | 準備站好   |
| 1    | CAL_STAND_1   | 3 秒 | **熄滅**       | 站立不動   |
| 2    | CAL_LIFT_1    | 3 秒 | 快閃 (500ms)   | 抬腳至水平 |
| 3    | CAL_STAND_2   | 3 秒 | **熄滅**       | 站立不動   |
| 4    | CAL_LIFT_2    | 3 秒 | 快閃 (500ms)   | 抬腳至水平 |
| 5    | CAL_ANALYZING | 瞬間 | 超快閃 (300ms) | 等待       |
| 6    | CAL_COMPLETE  | -    | 熄滅           | 開始測量   |

> ⚠️ 使用者會觀察到 **3 次「閃爍 → 熄滅」轉換**（準備 → 站 1、抬 1→ 站 2、抬 2→ 完成），這是正常的。

校正自動偵測：

- **primaryAxis**：主要運動軸（膝蓋抬起方向）
- **gravityAxis**：重力軸（站立時的垂直方向）
- **axisSign**：角度正負方向

## MQTT 資料格式（批次版）

```json
{"type":"batch","n":25,"d":[
  {"t":1.23,"a":12.5,"s":1,"dy":5.2,"dz":8.3,"ax":0.12,"ay":0.04,"az":0.98,"gx":2.5,"gy":-1.3,"gz":0.8},
  ...
]}
```

欄位縮寫：`t`=timestamp, `a`=angle, `s`=stable, `dy/dz`=delta, `ax/ay/az`=accel, `gx/gy/gz`=gyro

## 程式碼慣例

- **註解語言**：繁體中文
- **序列輸出**：Unicode 框線 (`╔═╗║╚═╝`) + 狀態符號 (`✓✗⚠️⏳`)
- **數值格式**：`printf("%6.1f", value)` 對齊輸出
- **錯誤處理**：初始化失敗進入 `while(1) delay(1000)` 無限迴圈
- **LED 控制**：`LED_ON=LOW`, `LED_OFF=HIGH`（低電位觸發）

## 核心演算法

**互補濾波**：

```cpp
float alpha = 0.90;
thighAngle = alpha * gyroAngle + (1 - alpha) * accelAngle;
```

**座標系統**：原點=髖關節，0°=站立，90°=膝蓋水平

```cpp
kneeY = THIGH_LENGTH * sin(angleRad);  // 前後
kneeZ = -THIGH_LENGTH * cos(angleRad); // 上下
```

## 開發工作流程

1. **編譯/上傳**：使用 PlatformIO 側邊欄按鈕（非終端機）
2. **首次上傳**：按住 BOOT → 按 RESET → 放開 BOOT → 上傳
3. **除錯**：Serial Monitor 查看 I2C 掃描、校正進度、任務狀態
4. **資料接收**：`python mqtt_receiver.py` 自動產生 `knee_data_YYYYMMDD_HHMMSS.csv`
5. **分析視覺化**：`python analyze_knee_motion.py`

## 函式庫配置範例

```ini
# platformio.ini
lib_deps =
    https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary.git  # 必須用 GitHub URL
    knolleary/PubSubClient@^2.8
    bblanchon/ArduinoJson@^7.0.0
```

---

**最後更新**: 2026-01-03
