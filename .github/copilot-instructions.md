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

**重要**: PlatformIO 會自動管理相依套件，無需手動安裝 Arduino 函式庫。

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

## 開發工作流程

### 首次上傳注意事項

ESP32-C3 Super Mini 首次上傳可能需要手動進入 bootloader：

1. 按住 **BOOT** 按鈕
2. 按下 **RESET** 按鈕
3. 釋放兩個按鈕，開始上傳
4. 上傳完成後按 **RESET** 重啟

### 除錯方法

- 使用 `Serial.println()` 進行除錯輸出
- 透過 `pio device monitor` 查看即時輸出
- PlatformIO 的 Debug 功能需要額外硬體 (JTAG)

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

在 `platformio.ini` 的 `[env]` 區塊加入：

```ini
lib_deps =
    adafruit/Adafruit MPU6050@^2.2.6
    adafruit/Adafruit Unified Sensor@^1.1.14
```

## 常見問題

- **上傳失敗**: 檢查 USB 連線，嘗試手動進入 bootloader
- **序列埠無輸出**: 確認 `monitor_speed` 設定為 115200
- **編譯錯誤**: 執行 `pio run --target clean` 清理後重新建置
