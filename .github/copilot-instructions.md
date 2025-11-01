# Copilot Instructions - Knee Drive Analysis

## 專案概述

這是一個使用 ESP32-C3 與 MPU-6050 IMU 感測器分析跑步膝蓋抬腿動作的嵌入式系統專案。目標是量化分析不同抬腿高度對跑步效率的影響。

## 硬體架構

- **主控板**: ESP32-C3 Super Mini (RISC-V 架構，內建 USB)
- **感測器**: MPU-6050 (6 軸 IMU：3 軸加速度計 + 3 軸陀螺儀)
- **通訊**: I2C (SCL=GPIO9, SDA=GPIO8)
- **電源**: 3.3V

## 開發環境

此專案使用 **PlatformIO** 而非 Arduino IDE：

```bash
# 編譯專案
pio run

# 上傳到開發板
pio run --target upload

# 開啟序列埠監控 (115200 baud)
pio device monitor

# 清理建置檔案
pio run --target clean
```

**重要**:

- PlatformIO 會自動管理相依套件，無需手動安裝 Arduino 函式庫
- **禁止使用 PowerShell 指令**：不可直接在程式碼中執行 PowerShell 命令（如 `platformio`、`pio` 等），應使用 VS Code 的 PlatformIO 擴充功能操作

## 建置配置 (platformio.ini)

- **Board**: `dfrobot_beetle_esp32c3` (相容 ESP32-C3 Super Mini)
- **Framework**: Arduino
- **關鍵建置旗標**:
  - `ARDUINO_USB_MODE=1` - 啟用 USB 功能
  - `ARDUINO_USB_CDC_ON_BOOT=1` - 開機時啟用 USB CDC (序列埠通訊)

## 專案結構慣例

```
src/main.cpp          # 主程式進入點 (setup/loop)
include/              # 自訂標頭檔 (.h)
lib/                  # 專案特定函式庫 (非 PlatformIO registry)
test/                 # 單元測試
platformio.ini        # 建置配置與相依套件定義
```

## 程式碼慣例

1. **語言**: 使用繁體中文進行註解與文件撰寫
2. **框架**: Arduino 框架 (使用 `setup()` 和 `loop()` 結構)
3. **序列輸出**: 使用 `Serial` 物件，速率固定為 115200 baud
4. **感測器取樣**: 目標頻率 100 Hz

## AI 協助開發規範

### 禁止的操作

- **禁止執行終端機指令**：不可使用 `run_in_terminal` 工具執行 `pio`、`platformio` 等指令
- **使用 VS Code UI**：所有編譯、上傳、監控操作應透過 VS Code 的 PlatformIO 擴充功能完成

### 資料查證與搜尋

- **優先使用 Web Search**：需要查詢技術資料、函式庫文件、硬體規格時，應使用 `web search` 工具進行即時資料查證
- **適用場景**：
  - 查詢 ESP32-C3 或 MPU-6050 技術規格
  - 尋找 Arduino 函式庫使用範例
  - 查證 I2C 通訊協定細節
  - 搜尋感測器校準方法
  - 確認最新的函式庫版本與相容性

### 函式庫匯入

- **必須使用 GitHub 超連結**：在 `platformio.ini` 中使用完整的 GitHub 倉庫 URL
- **禁止使用官方套件名稱**：不可使用 PlatformIO Registry 的套件名稱

## 開發工作流程

### 首次上傳注意事項

ESP32-C3 Super Mini 首次上傳可能需要手動進入 bootloader：

1. 按住 **BOOT** 按鈕
2. 按下 **RESET** 按鈕
3. 釋放兩個按鈕，開始上傳
4. 上傳完成後按 **RESET** 重啟

### 除錯方法

- 使用 `Serial.println()` 進行除錯輸出
- 透過 VS Code PlatformIO 的 Serial Monitor 查看即時輸出
- PlatformIO 的 Debug 功能需要額外硬體 (JTAG)
- **注意**：不要在終端機手動執行 `pio device monitor` 指令

## 專案階段與功能

**已完成**: 基本專案架構  
**進行中**: MPU-6050 感測器整合  
**待開發**:

- 膝蓋運動軌跡計算演算法
- 抬腿高度量化分析
- Wi-Fi 數據傳輸
- 網頁介面視覺化

## 關鍵技術規格

- MPU-6050 加速度範圍: ±2g/±4g/±8g/±16g (可調)
- MPU-6050 陀螺儀範圍: ±250/±500/±1000/±2000 °/s (可調)
- I2C 時脈速度: 標準 100kHz 或快速 400kHz
- 目標取樣頻率: 100 Hz

## 新增相依套件

**函式庫匯入規範**：

- **必須使用 GitHub 超連結**：在 `platformio.ini` 中使用完整的 GitHub 倉庫 URL
- **禁止使用官方套件名稱**：不可使用 PlatformIO Registry 的套件名稱（如 `adafruit/Adafruit MPU6050`）

**正確範例**（使用 GitHub URL）：

```ini
lib_deps =
    https://github.com/ElectronicCats/mpu6050.git
    https://github.com/adafruit/Adafruit_Sensor.git
```

**錯誤範例**（禁止使用）：

```ini
lib_deps =
    adafruit/Adafruit MPU6050@^2.2.6
    adafruit/Adafruit Unified Sensor@^1.1.14
```

**匯入步驟**：

1. 在 GitHub 搜尋所需函式庫
2. 確認函式庫品質（star 數、更新頻率、相容性）
3. 複製 GitHub 倉庫 URL（`.git` 結尾）
4. 加入 `platformio.ini` 的 `lib_deps` 區塊

## 常見問題

- **上傳失敗**: 檢查 USB 連線，嘗試手動進入 bootloader
- **序列埠無輸出**: 確認 `monitor_speed` 設定為 115200
- **編譯錯誤**: 使用 VS Code PlatformIO 的 Clean 功能清理後重新建置
- **需要技術資料**: 使用 Web Search 查詢最新文件與範例

**重要提醒**：所有操作應透過 VS Code 的 PlatformIO 擴充功能，避免直接執行命令列指令。
