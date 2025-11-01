#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>

// MPU6050 物件
MPU6050 mpu;

// 加速度最大值追蹤
float maxAccelX = 0.0;
float maxAccelY = 0.0;
float maxAccelZ = 0.0;
float maxAccelTotal = 0.0; // 三軸合成加速度最大值

// 膝蓋座標追蹤（位置積分）
float velocityX = 0.0; // 速度 (m/s)
float velocityY = 0.0;
float velocityZ = 0.0;
float positionX = 0.0; // 位置 (cm)
float positionY = 0.0;
float positionZ = 0.0;

// 最大位移追蹤
float maxPosX = 0.0;
float maxPosY = 0.0;
float maxPosZ = 0.0;

// 時間追蹤
unsigned long lastUpdateTime = 0;

void setup()
{
  // 初始化序列埠 (115200 baud)
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10); // 等待序列埠準備就緒
  }
  Serial.println("\n=== MPU-6050 感測器初始化 ===");

  // 初始化 I2C (ESP32-C3 Super Mini: SDA=GPIO8, SCL=GPIO9)
  Wire.begin(8, 9);

  // 初始化 MPU6050
  Serial.println("正在初始化 MPU6050...");
  mpu.initialize();

  // 檢查連線狀態
  if (mpu.testConnection())
  {
    Serial.println("✓ MPU6050 連線成功！");
  }
  else
  {
    Serial.println("✗ MPU6050 連線失敗，請檢查接線");
    while (1)
      ; // 停止程式
  }

  // 設定感測器範圍
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // ±2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250); // ±250°/s

  Serial.println("\n--- 感測器設定 ---");
  Serial.println("加速度範圍: ±2g");
  Serial.println("陀螺儀範圍: ±250°/s");
  Serial.println("\n開始讀取資料...\n");

  delay(1000);

  // 初始化時間
  lastUpdateTime = millis();
}

void loop()
{
  // 記錄開始時間
  unsigned long startTime = millis();

  // 計算時間差（秒）
  float deltaTime = (startTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = startTime;

  // 讀取加速度與陀螺儀原始數據
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // 轉換為實際單位
  float accelX = ax / 16384.0; // 轉換為 g (±2g範圍)
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float gyroX = gx / 131.0; // 轉換為 °/s (±250°/s範圍)
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // 讀取溫度
  int16_t temperature = mpu.getTemperature();
  float tempC = temperature / 340.0 + 36.53; // 轉換為攝氏度

  // 更新加速度最大值（使用絕對值）
  float absAccelX = abs(accelX);
  float absAccelY = abs(accelY);
  float absAccelZ = abs(accelZ);

  if (absAccelX > maxAccelX)
    maxAccelX = absAccelX;
  if (absAccelY > maxAccelY)
    maxAccelY = absAccelY;
  if (absAccelZ > maxAccelZ)
    maxAccelZ = absAccelZ;

  // 計算三軸合成加速度（向量長度）
  float accelTotal = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
  if (accelTotal > maxAccelTotal)
    maxAccelTotal = accelTotal;

  // ===== 膝蓋座標計算 =====
  // 移除重力影響（假設 Z 軸向上，靜止時 Z = 1g）
  float accelX_noG = accelX;
  float accelY_noG = accelY;
  float accelZ_noG = accelZ - 1.0; // 移除重力加速度

  // 轉換單位：g → m/s² (1g = 9.81 m/s²)
  float accelX_ms2 = accelX_noG * 9.81;
  float accelY_ms2 = accelY_noG * 9.81;
  float accelZ_ms2 = accelZ_noG * 9.81;

  // 積分計算速度 (v = v0 + a*dt)
  velocityX += accelX_ms2 * deltaTime;
  velocityY += accelY_ms2 * deltaTime;
  velocityZ += accelZ_ms2 * deltaTime;

  // 速度衰減（補償積分漂移）
  velocityX *= 0.98;
  velocityY *= 0.98;
  velocityZ *= 0.98;

  // 積分計算位置 (s = s0 + v*dt) - 轉換為公分
  positionX += velocityX * deltaTime * 100.0;
  positionY += velocityY * deltaTime * 100.0;
  positionZ += velocityZ * deltaTime * 100.0;

  // 更新最大位移（使用絕對值）
  float absPosX = abs(positionX);
  float absPosY = abs(positionY);
  float absPosZ = abs(positionZ);

  if (absPosX > maxPosX)
    maxPosX = absPosX;
  if (absPosY > maxPosY)
    maxPosY = absPosY;
  if (absPosZ > maxPosZ)
    maxPosZ = absPosZ;

  // 只顯示 Pos 和 Gyro
  Serial.print("\r");
  Serial.printf("Pos[cm] X:%6.1f Y:%6.1f Z:%6.1f | ", positionX, positionY, positionZ);
  Serial.printf("Gyro[°/s] X:%6.1f Y:%6.1f Z:%6.1f     ", gyroX, gyroY, gyroZ);

  // 100Hz 取樣頻率 (每10ms讀取一次)
  delay(10);
}