#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h" // SparkFun ICM-20948 函式庫

// ICM-20948 物件 (使用 I2C)
ICM_20948_I2C imu;

// I2C 設定
#define SDA_PIN 5 // ESP32-C3 GPIO5
#define SCL_PIN 6 // ESP32-C3 GPIO6

// 大腿參數（感測器綁在大腿上）
#define THIGH_LENGTH 45.0 // 大腿長度 (公分)，從髖關節到膝蓋

// 角度追蹤變數（互補濾波）
float thighAngle = 0.0; // 大腿抬起角度（0° = 垂直向下，90° = 水平向前）
float roll = 0.0;       // 橫滾角（左右偏移）
unsigned long lastTime = 0;

// 膝蓋座標（相對於髖關節）
float kneeX = 0.0; // 左右位置
float kneeY = 0.0; // 前後位置
float kneeZ = 0.0; // 上下位置（負值表示在髖關節下方）

// I2C 掃描函數
void scanI2C()
{
  Serial.println("正在掃描 I2C 裝置...");
  byte count = 0;

  for (byte i = 1; i < 127; i++)
  {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0)
    {
      Serial.print("發現裝置位址: 0x");
      if (i < 16)
        Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }

  if (count == 0)
    Serial.println("⚠️ 未發現任何 I2C 裝置！請檢查接線。");
  else
    Serial.printf("✓ 共發現 %d 個 I2C 裝置\n", count);
  Serial.println();
}

void setup()
{
  // 初始化序列埠 (115200 baud)
  Serial.begin(115200);
  delay(1000); // 等待穩定
  Serial.println("\n=== ICM-20948 感測器初始化 ===");

  // 初始化 I2C (ESP32-C3 Super Mini: SDA=GPIO5, SCL=GPIO6)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 降低 I2C 時脈至 100kHz (標準模式)
  delay(100);

  // 掃描 I2C 裝置
  scanI2C();

  // 嘗試兩個可能的 I2C 位址
  Serial.println("正在初始化 ICM-20948...");

  bool initialized = false;
  uint8_t addressToTry[] = {1, 0}; // 先試 0x69，再試 0x68
  int addressIndex = 0;

  while (!initialized && addressIndex < 2)
  {
    uint8_t currentAddress = addressToTry[addressIndex];
    Serial.printf("嘗試位址: 0x%02X...\n", currentAddress ? 0x69 : 0x68);

    imu.begin(Wire, currentAddress);
    delay(100);

    Serial.print("  初始化狀態: ");
    Serial.println(imu.statusString());

    if (imu.status == ICM_20948_Stat_Ok)
    {
      initialized = true;
      Serial.printf("✓ ICM-20948 連線成功！(位址: 0x%02X)\n", currentAddress ? 0x69 : 0x68);
    }
    else
    {
      addressIndex++;
      if (addressIndex < 2)
      {
        Serial.println("  失敗，嘗試下一個位址...");
        delay(500);
      }
    }
  }

  if (!initialized)
  {
    Serial.println("\n✗ ICM-20948 初始化失敗！");
    Serial.println("\n請檢查：");
    Serial.println("1. VCC → 3.3V");
    Serial.println("2. GND → GND");
    Serial.println("3. SDA → GPIO5");
    Serial.println("4. SCL → GPIO6");
    Serial.println("5. 模組是否正常供電（檢查電源燈）");
    Serial.println("\n程式停止運行。");
    while (1)
      delay(1000);
  }

  Serial.println("\n=== 大腿抬起角度追蹤系統 ===");
  Serial.println("髖關節位置 = 原點 (0, 0, 0)");
  Serial.printf("大腿長度 = %.1f cm\n", THIGH_LENGTH);
  Serial.println("\n追蹤目標：膝蓋位置變化");
  Serial.println("  0° = 大腿垂直向下（站立）");
  Serial.println(" 90° = 大腿水平向前（膝蓋抬到最高）");
  Serial.println("\n開始讀取...\n");

  lastTime = millis();
  delay(2000);
}

void loop()
{
  // 檢查是否有新數據
  if (imu.dataReady())
  {
    // 計算時間差
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // 轉換為秒
    lastTime = currentTime;

    // 讀取感測器數據
    imu.getAGMT();

    // 加速度數據 (單位: g)
    float accelX = imu.accX();
    float accelY = imu.accY();
    float accelZ = imu.accZ();

    // 陀螺儀數據 (單位: °/s)
    float gyroX = imu.gyrX();
    float gyroY = imu.gyrY();
    float gyroZ = imu.gyrZ();

    // ===== 計算大腿抬起角度（互補濾波）=====

    // 方法 1：用加速度計算角度（利用重力方向）
    // 假設感測器安裝在大腿上，Y 軸沿大腿方向
    float accelAngle = atan2(accelY, accelZ) * 180.0 / PI;

    // 方法 2：用陀螺儀積分（角速度 × 時間）
    float gyroAngle = thighAngle + gyroX * deltaTime;

    // 方法 3：互補濾波（融合兩者）
    float alpha = 0.96; // 96% 信任陀螺儀，4% 信任加速度計
    thighAngle = alpha * gyroAngle + (1 - alpha) * accelAngle;

    // 確保角度在 -180° 到 180° 範圍內
    if (thighAngle > 180.0)
      thighAngle -= 360.0;
    if (thighAngle < -180.0)
      thighAngle += 360.0;

    // ===== 計算膝蓋座標（相對於髖關節）=====
    // 原點 = 髖關節 (0, 0, 0)
    // 目標 = 膝蓋位置 (x, y, z)

    // 轉換角度為弧度
    float angleRad = thighAngle * PI / 180.0;

    // 座標計算（極坐標轉直角座標）
    // 假設：0° = 大腿垂直向下，90° = 大腿水平向前
    kneeX = 0.0;                           // 左右不動（簡化）
    kneeY = THIGH_LENGTH * sin(angleRad);  // 前後位移（正值 = 往前）
    kneeZ = -THIGH_LENGTH * cos(angleRad); // 上下位移（負值 = 往下，0° 時在髖關節正下方）

    // ===== 顯示結果 =====
    Serial.println("╔════════════════════════════════════════╗");
    Serial.printf("║ 大腿抬起角度： %6.1f°              ║\n", abs(thighAngle));
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║          膝蓋座標 (cm)                 ║");
    Serial.printf("║   X (左右): %7.1f                  ║\n", kneeX);
    Serial.printf("║   Y (前後): %7.1f                  ║\n", kneeY);
    Serial.printf("║   Z (上下): %7.1f                  ║\n", kneeZ);
    Serial.println("╠════════════════════════════════════════╣");

    // 顯示對應的抬腿程度
    float absAngle = abs(thighAngle);
    Serial.print("║ 狀態： ");
    if (absAngle < 10)
    {
      Serial.println("站立（大腿垂直）          ║");
    }
    else if (absAngle < 30)
    {
      Serial.println("輕微抬腿                  ║");
    }
    else if (absAngle < 60)
    {
      Serial.println("中度抬腿                  ║");
    }
    else if (absAngle < 85)
    {
      Serial.println("高抬腿                    ║");
    }
    else if (absAngle < 100)
    {
      Serial.println("膝蓋抬到最高點（接近水平）║");
    }
    else
    {
      Serial.println("過度抬腿（超過水平）      ║");
    }

    Serial.println("╠════════════════════════════════════════╣");
    Serial.printf("║ 加速度 | X:%6.3f Y:%6.3f Z:%6.3f ║\n", accelX, accelY, accelZ);
    Serial.printf("║ 陀螺儀 | X:%6.1f Y:%6.1f Z:%6.1f   ║\n", gyroX, gyroY, gyroZ);
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
  }

  // 100Hz 取樣頻率 (每10ms讀取一次)
  delay(500); // 改為 100ms，每秒更新 10 次
}