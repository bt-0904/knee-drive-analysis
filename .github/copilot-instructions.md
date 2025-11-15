# Copilot Instructions - Knee Drive Analysis

## 專案架構概述

這是一個**嵌入式 + 雲端**的運動分析系統，使用 ESP32-C3 微控制器即時追蹤跑步時的膝蓋抬腿動作。系統分為三層：

1. **感測層** (ESP32-C3 + ICM-20948)：綁在大腿上，每秒採集 100 次 9 軸慣性資料
2. **傳輸層** (Wi-Fi → 手機熱點 → MQTT Broker)：透過 JSON 格式無線傳輸資料到雲端
3. **接收層** (Python `mqtt_receiver.py`)：在電腦上即時接收並儲存為 CSV 格式供分析

**核心演算法**：使用**互補濾波器** (96% 陀螺儀 + 4% 加速度計) 融合感測器數據，計算大腿角度與膝蓋 3D 座標。系統開機後自動進行 3 秒站立校正，將站立姿勢設為 (0, 0, 0) 基準點。

## 硬體架構與關鍵限制

- **主控板**: ESP32-C3 Super Mini (RISC-V 架構，**內建 USB CDC**，無需外接串口晶片)
- **感測器**: ICM-20948 (9 軸 IMU，I2C 位址自動偵測 0x68/0x69)
- **I2C 接線**: **必須使用 GPIO5 (SDA) / GPIO6 (SCL)**
  - ⚠️ **禁止使用 GPIO8/GPIO9**：這些是 strapping pins，會導致開機失敗
  - I2C 時脈：100kHz (標準模式，降低干擾)
- **無線架構**: ESP32 → 手機熱點 (預設 SSID: `Bt`) → MQTT Broker → 電腦
- **MQTT Buffer**: 512 bytes (因 JSON payload 約 300-400 bytes)

## PlatformIO 開發工作流程

此專案**強制使用 PlatformIO**，完全透過 VS Code 擴充功能操作，**禁止執行命令列指令**。

### 標準操作方式

- **編譯**: 點擊 PlatformIO 側邊欄的 **Build** 按鈕
- **上傳**: 點擊 **Upload** 按鈕（首次上傳需手動進入 bootloader，見下方）
- **序列埠監控**: 點擊 **Monitor** 按鈕（波特率自動設為 115200）
- **清理建置**: 點擊 **Clean** 按鈕

⚠️ **AI 禁止操作**：不可使用 `run_in_terminal` 執行 `pio`、`platformio`、`pio device monitor` 等指令

### 首次上傳 Bootloader 步驟

ESP32-C3 Super Mini 首次上傳需手動進入 bootloader 模式：

1. 按住 **BOOT** 按鈕（保持按下）
2. 按下 **RESET** 按鈕（然後釋放）
3. 釋放 **BOOT** 按鈕
4. 點擊 PlatformIO Upload 開始上傳
5. 上傳完成後按 **RESET** 重啟

### 建置配置 (platformio.ini)

```ini
[env:esp32-c3-supermini]
board = dfrobot_beetle_esp32c3  # 相容 ESP32-C3 Super Mini
framework = arduino
monitor_speed = 115200
build_flags =
    -D ARDUINO_USB_MODE=1          # 啟用 USB 功能
    -D ARDUINO_USB_CDC_ON_BOOT=1   # 開機時啟用 USB CDC
```

**關鍵點**：這兩個 build flags 是必須的，讓 ESP32-C3 在開機時自動啟用 USB 序列埠。

## 相依套件管理規範

⚠️ **關鍵限制**：在 `platformio.ini` 的 `lib_deps` 中，**必須使用完整 GitHub URL**，**禁止使用 PlatformIO Registry 套件名稱**。

**正確範例**：
```ini
lib_deps =
    https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary.git
    knolleary/PubSubClient@^2.8
    bblanchon/ArduinoJson@^7.0.0
```

**錯誤範例**（禁止）：
```ini
lib_deps =
    adafruit/Adafruit MPU6050@^2.2.6  # ❌ 不可使用 PlatformIO Registry 名稱
```

**匯入新函式庫步驟**：
1. 在 GitHub 搜尋目標函式庫
2. 確認函式庫品質（stars、最近更新、Arduino 相容性）
3. 複製 GitHub 倉庫 URL（格式：`https://github.com/owner/repo.git`）
4. 加入 `platformio.ini` 的 `lib_deps` 區塊

## 程式碼架構與設計模式

### Arduino 框架結構

```cpp
// src/main.cpp 的標準架構
void setup() {
    // 初始化（只執行一次）
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    connectWiFi();
    imu.begin(Wire, 1);  // 嘗試 I2C 位址 0x69
}

void loop() {
    // 主迴圈（持續執行）
    if (imu.dataReady()) {
        // 讀取感測器 → 計算角度 → 發送 MQTT → 顯示結果
    }
    delay(10);  // 100Hz 取樣頻率
}
```

### 座標系統與演算法

- **原點**: 髖關節 (0, 0, 0)
- **大腿長度**: 45 cm（可調整常數 `THIGH_LENGTH`）
- **角度定義**: 0° = 大腿垂直向下（站立），90° = 大腿水平向前
- **座標轉換**:
  ```cpp
  kneeY = THIGH_LENGTH * sin(angleRad);   // 前後位移
  kneeZ = -THIGH_LENGTH * cos(angleRad);  // 上下位移（負值表示在髖關節下方）
  ```

### 互補濾波器實作

```cpp
// 融合加速度計（長期穩定）與陀螺儀（短期精確）
float alpha = 0.96;
float accelAngle = atan2(accelY, accelZ) * 180.0 / PI;
float gyroAngle = thighAngle + gyroX * deltaTime;
thighAngle = alpha * gyroAngle + (1 - alpha) * accelAngle;
```

**設計理由**：加速度計受重力影響穩定但易受振動干擾；陀螺儀短期精確但長期累積誤差。96:4 的比例平衡兩者優勢。

### 自動校正機制

開機後前 3 秒收集站立姿勢數據，計算平均角度作為校正偏移量：

```cpp
if (!isCalibrated && calibrationElapsed < 3000) {
    calibrationSum += thighAngle;
    calibrationCount++;
} else {
    calibrationAngle = calibrationSum / calibrationCount;
    float calibratedAngle = thighAngle - calibrationAngle;
}
```

校正完成後，所有角度都會扣除初始偏移，使站立姿勢回歸 0°。

## 無線傳輸架構

### Wi-Fi 連線流程

1. ESP32 開機後連接手機熱點（預設 SSID: `Bt`，密碼: `bt_980904`）
2. 手機熱點透過 4G/5G 連接外網
3. ESP32 透過手機上網連接 MQTT Broker (`mqtt.singularinnovation-ai.com:1883`)

**設定位置**：`src/main.cpp` 第 9-17 行

### MQTT 資料格式

每筆資料封裝為 JSON（約 300-400 bytes）：

```json
{
  "timestamp": 3.45,
  "elapsed_time": 3.45,
  "angle": 12.5,
  "stable": true,
  "delta": {"x": 0.0, "y": 5.2, "z": 8.3},
  "absolute": {"x": 0.0, "y": 5.2, "z": -43.7},
  "accel": {"x": 0.123, "y": 0.045, "z": 0.987},
  "gyro": {"x": 2.5, "y": -1.3, "z": 0.8}
}
```

**傳輸設定**：
- Topic: `knee-drive/data`（可在 `main.cpp` 修改）
- QoS: 0（最多傳送一次，不確認，降低延遲）
- Keep-alive: 60 秒
- Buffer: 512 bytes（需設定 `mqttClient.setBufferSize(512)`）

### Python 接收腳本 (mqtt_receiver.py)

- 訂閱同一 MQTT Topic
- 解析 JSON 並即時儲存為 CSV（格式見 `CSV_HEADERS` 陣列）
- 檔名自動加上時間戳記（如 `knee_data_20251108_101037.csv`）
- 按 Ctrl+C 停止記錄並顯示統計資訊

## 程式碼慣例與風格

1. **註解語言**: 繁體中文
2. **序列輸出**: 使用 Unicode 字元繪製表格框線（如 `║`, `╔`, `╚`）
3. **狀態指示器**: ✓（成功）、✗（失敗）、⚠️（警告）、⏳（處理中）
4. **數值格式化**: 使用 `printf` 對齊輸出（如 `%6.1f` 表示寬度 6、小數點 1 位）
5. **錯誤處理**: I2C 初始化失敗後進入無限迴圈 `while(1) delay(1000)`，避免無效資料
6. **除錯方法**: 主要使用 `Serial.println()`，不依賴硬體除錯器（JTAG）

## 專案檔案結構

```
src/main.cpp                      # 主程式（Arduino setup/loop）
mqtt_receiver.py                  # Python MQTT 接收腳本
platformio.ini                    # 建置配置與套件定義
knee_data_YYYYMMDD_HHMMSS.csv    # 自動生成的資料檔案（時間戳記命名）
README.md                         # 專案說明
USAGE.md                          # 詳細使用指南
```

**重要目錄**（PlatformIO 自動管理）：
- `.pio/` - 建置快取與編譯產物
- `include/` - 自訂標頭檔（目前未使用）
- `lib/` - 專案特定函式庫（目前未使用）

## 關鍵技術規格

| 項目 | 規格 |
|------|------|
| 取樣頻率 | 100 Hz (實際約 10 Hz，因 `delay(500)`) |
| 加速度範圍 | ±2g（預設，ICM-20948 可調至 ±16g） |
| 陀螺儀範圍 | ±250°/s（預設，可調至 ±2000°/s） |
| I2C 時脈 | 100 kHz（標準模式） |
| 序列埠速率 | 115200 baud |
| MQTT Buffer | 512 bytes |
| 校正時間 | 3 秒（收集約 300 筆資料取平均） |
| 穩定判定 | 連續 30 次角度變化 < 0.5° |

## 常見問題與解決方法

| 問題 | 檢查項目 | 解決方案 |
|------|---------|---------|
| 上傳失敗 | USB 連線 | 手動進入 bootloader（按 BOOT+RESET） |
| 序列埠無輸出 | `monitor_speed` | 確認設為 115200 |
| I2C 初始化失敗 | 接線與位址 | 確認 SDA=GPIO5, SCL=GPIO6；檢查 VCC/GND |
| Wi-Fi 連不上 | SSID/密碼 | 檢查手機熱點設定，確認大小寫 |
| MQTT 發送失敗 | Buffer 大小 | 確認 `setBufferSize(512)` 已設定 |
| 資料飄移 | 校正失敗 | 重啟 ESP32，站立 3 秒完成校正 |

## 開發狀態與待辦事項

**已完成** ✅：
- ICM-20948 整合（I2C 自動偵測）
- 互補濾波器與座標計算
- Wi-Fi + MQTT 無線傳輸
- 自動校正系統
- Python 接收與 CSV 儲存

**進行中** 🚧：
- 跑步效率評估演算法

**待開發**：
- 網頁即時視覺化（WebSocket）
- 歷史資料分析工具（Python/Jupyter）
- 多使用者管理

## AI 開發協助規範

### 禁止操作

1. ❌ **禁止執行終端機指令**：不可使用 `run_in_terminal` 執行 `pio`、`platformio`、`python` 等指令
2. ❌ **禁止修改 build flags**：`ARDUINO_USB_MODE=1` 和 `ARDUINO_USB_CDC_ON_BOOT=1` 是必須的
3. ❌ **禁止使用 GPIO8/GPIO9**：這會導致 ESP32-C3 開機失敗

### 建議操作

1. ✅ **使用 Web Search 查證技術資料**：查詢 ESP32-C3、ICM-20948、Arduino 函式庫文件
2. ✅ **修改程式碼時保持註解語言一致**：使用繁體中文
3. ✅ **新增函式庫時使用 GitHub URL**：避免使用 PlatformIO Registry 名稱

### 除錯流程

1. 檢查 PlatformIO 編譯輸出
2. 透過 Serial Monitor 查看即時輸出
3. 確認 I2C 掃描結果（`scanI2C()` 函式）
4. 驗證 Wi-Fi 與 MQTT 連線狀態
5. 使用 `mqtt_receiver.py` 確認資料接收

---

**最後更新**: 2025-11-15
