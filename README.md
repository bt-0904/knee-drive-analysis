# 膝蓋抬腿動作分析系統 (Knee Drive Analysis)

## 專案簡介

本專案旨在分析跑步時抬腿高低對跑步效率的影響。透過 IMU 感測器收集膝蓋運動數據，以量化分析不同抬腿高度與跑步效率之間的關係。

## 硬體配置

### 開發板

- **ESP32-C3 Super Mini**
  - 基於 RISC-V 架構的 ESP32-C3 晶片
  - 支援 Wi-Fi 和 Bluetooth
  - 內建 USB 功能

### 感測器

- **MPU-6050**
  - 6 軸慣性測量單元 (IMU)
  - 3 軸加速度計 + 3 軸陀螺儀
  - I2C 通訊介面

### 接線配置

| MPU-6050 | ESP32-C3 Super Mini |
| -------- | ------------------- |
| SCL      | GPIO 9              |
| SDA      | GPIO 8              |
| VCC      | 3.3V                |
| GND      | GND                 |

## 軟體環境

- **開發平台**: PlatformIO
- **框架**: Arduino
- **主控晶片**: ESP32-C3
- **程式語言**: C++

## 專案結構

```
knee-drive-analysis/
├── platformio.ini          # PlatformIO 配置文件
├── src/
│   └── main.cpp           # 主程式
├── include/               # 標頭檔
├── lib/                   # 專案函式庫
└── test/                  # 測試程式
```

## 開始使用

### 環境需求

1. 安裝 [Visual Studio Code](https://code.visualstudio.com/)
2. 安裝 [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) 擴充功能

### 編譯與上傳

1. 克隆或下載本專案
2. 在 VS Code 中開啟專案資料夾
3. 連接 ESP32-C3 Super Mini 到電腦
4. 點擊 PlatformIO 工具列的「Upload」按鈕

**注意事項**：

- 首次上傳時，可能需要手動進入 bootloader 模式
- 按住 **BOOT** 按鈕，然後按 **RESET** 按鈕
- 上傳完成後，按 **RESET** 按鈕重啟開發板

### 監控輸出

點擊 PlatformIO 工具列的「Monitor」按鈕，即可查看序列埠輸出。

## 功能特點

- [x] MPU-6050 感測器數據讀取
- [ ] 膝蓋運動軌跡計算
- [ ] 抬腿高度量化分析
- [ ] 跑步效率評估演算法
- [ ] 即時數據顯示
- [ ] Wi-Fi 數據傳輸
- [ ] 網頁介面視覺化

## 研究目標

1. **數據收集**：記錄不同跑步速度下的膝蓋運動數據
2. **特徵提取**：分析抬腿高度、頻率、加速度等關鍵指標
3. **效率評估**：建立抬腿高度與跑步效率的關聯模型
4. **最佳化建議**：根據分析結果提供個人化的跑步姿勢建議

## 技術規格

- **取樣頻率**: 100 Hz
- **加速度範圍**: ±2g / ±4g / ±8g / ±16g (可設定)
- **陀螺儀範圍**: ±250°/s / ±500°/s / ±1000°/s / ±2000°/s (可設定)
- **通訊速率**: 115200 baud

## 授權

本專案採用 MIT 授權條款。
