#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h" // SparkFun ICM-20948 函式庫
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===== FreeRTOS 標頭 =====
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ===== FreeRTOS 任務配置 =====
#define SAMPLE_INTERVAL_MS 20 // 取樣間隔 20ms = 50Hz
#define SEND_INTERVAL_MS 500  // 發送間隔 500ms = 2Hz
#define BATCH_SIZE 25         // 每批次最多 25 筆
#define RING_BUFFER_SIZE 200  // 環形緩衝區大小（可承受 4 秒網路中斷）

#define SAMPLING_STACK_SIZE 4096 // 取樣任務 Stack
#define TRANSMIT_STACK_SIZE 8192 // 傳輸任務 Stack（JSON 序列化需要較多空間）
#define SAMPLING_PRIORITY 3      // 取樣任務優先級（高）
#define TRANSMIT_PRIORITY 1      // 傳輸任務優先級（低）

// ===== 感測器樣本結構（縮短欄位以節省記憶體）=====
struct SensorSample
{
  uint32_t timestamp;           // millis() 時間戳記
  float angle;                  // 校正後角度
  float deltaY;                 // 前後位移
  float deltaZ;                 // 上下位移
  float accelX, accelY, accelZ; // 加速度
  float gyroX, gyroY, gyroZ;    // 陀螺儀
  bool stable;                  // 穩定度
};

// ===== 環形緩衝區 =====
SensorSample ringBuffer[RING_BUFFER_SIZE];
volatile uint16_t bufferHead = 0;  // 寫入位置
volatile uint16_t bufferTail = 0;  // 讀取位置
volatile uint16_t bufferCount = 0; // 目前資料筆數

// ===== FreeRTOS 同步物件 =====
SemaphoreHandle_t bufferMutex = NULL;
TaskHandle_t samplingTaskHandle = NULL;
TaskHandle_t transmitTaskHandle = NULL;

// ===== 統計變數 =====
volatile uint32_t totalSampleCount = 0;
volatile uint32_t totalSentCount = 0;
volatile uint32_t droppedCount = 0;

// Wi-Fi 設定
const char *WIFI_SSID = "Bt";
const char *WIFI_PASSWORD = "bt_980904";

// MQTT 設定
const char *MQTT_SERVER = "mqtt.singularinnovation-ai.com";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "singular";
const char *MQTT_PASSWORD = "Singular#1234";
const char *MQTT_TOPIC = "knee-drive/data";

// ICM-20948 物件 (使用 I2C)
ICM_20948_I2C imu;

// Wi-Fi 和 MQTT 客戶端
WiFiClient espClient;
PubSubClient mqttClient(espClient);

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

// ===== 環形緩衝區操作函數 =====
bool pushSample(const SensorSample *sample)
{
  if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE)
  {
    if (bufferCount < RING_BUFFER_SIZE)
    {
      ringBuffer[bufferHead] = *sample;
      bufferHead = (bufferHead + 1) % RING_BUFFER_SIZE;
      bufferCount++;
      totalSampleCount++;
      xSemaphoreGive(bufferMutex);
      return true;
    }
    else
    {
      // 緩衝區真的滿了，丟棄舊資料
      droppedCount++;
      xSemaphoreGive(bufferMutex);
      return false;
    }
  }
  // Mutex 超時 - 不算資料丟失，下次再試
  return true; // 返回 true 避免誤報警告
}

uint16_t popSamples(SensorSample *dest, uint16_t maxCount)
{
  uint16_t popped = 0;
  if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(50)) == pdTRUE)
  {
    while (bufferCount > 0 && popped < maxCount)
    {
      dest[popped] = ringBuffer[bufferTail];
      bufferTail = (bufferTail + 1) % RING_BUFFER_SIZE;
      bufferCount--;
      popped++;
    }
    xSemaphoreGive(bufferMutex);
  }
  return popped;
}

uint16_t getBufferCount()
{
  uint16_t count = 0;
  if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(5)) == pdTRUE)
  {
    count = bufferCount;
    xSemaphoreGive(bufferMutex);
  }
  return count;
}

// ===== FreeRTOS 任務函數宣告 =====
void samplingTask(void *parameter);
void transmitTask(void *parameter);

// ===== 新版動態校正系統 =====
// 校正狀態機
enum CalibrationState
{
  CAL_INIT,      // 初始化，等待使用者準備
  CAL_STAND_1,   // 第一次站立
  CAL_LIFT_1,    // 第一次抬腳
  CAL_STAND_2,   // 第二次站立
  CAL_LIFT_2,    // 第二次抬腳
  CAL_ANALYZING, // 分析數據
  CAL_COMPLETE   // 校正完成
};

CalibrationState calState = CAL_INIT;
bool isCalibrated = false;

// 校正數據收集
struct CalibrationData
{
  float accelX[200]; // 存放加速度數據
  float accelY[200];
  float accelZ[200];
  int count;
};

CalibrationData standData1, liftData1, standData2, liftData2;

// 校正分析結果
struct SensorOrientation
{
  int primaryAxis;   // 主要感測軸 (0=X, 1=Y, 2=Z)
  int secondaryAxis; // 次要感測軸 (重力軸)
  float axisSign;    // 軸向正負 (+1 或 -1)
  float standAvg;    // 站立時主軸平均值
  float liftAvg;     // 抬腳時主軸平均值
  float gravityAxis; // 重力軸平均值
};

SensorOrientation sensorOrient;

// 校正計時與階段控制
unsigned long calibrationStartTime = 0;
unsigned long stateStartTime = 0;
const unsigned long STATE_DURATION = 3000; // 每個階段 3 秒
int dataIndex = 0;

// 計算後的校正參數
float calibrationAngle = 0.0;

// 初始座標（用於相對位移計算）
float initialKneeY = 0.0;
float initialKneeZ = 0.0;

// 穩定度檢測變數
float prevAngle = 0.0;
int stableCount = 0;
bool isStable = false;
unsigned long startTime = 0;

// 低通濾波器變數（用於平滑角度輸出）
float smoothedAngle = 0.0;
const float SMOOTHING_FACTOR = 0.3; // 0.0-1.0，越小越平滑

// ===== 校正輔助函數 =====
void initCalibrationData(CalibrationData &data)
{
  data.count = 0;
  for (int i = 0; i < 200; i++)
  {
    data.accelX[i] = 0;
    data.accelY[i] = 0;
    data.accelZ[i] = 0;
  }
}

void addCalibrationSample(CalibrationData &data, float ax, float ay, float az)
{
  if (data.count < 200)
  {
    data.accelX[data.count] = ax;
    data.accelY[data.count] = ay;
    data.accelZ[data.count] = az;
    data.count++;
  }
}

// 計算軸向平均值
void calculateAxisAverages(CalibrationData &data, float &avgX, float &avgY, float &avgZ)
{
  avgX = avgY = avgZ = 0;
  if (data.count == 0)
    return;

  for (int i = 0; i < data.count; i++)
  {
    avgX += data.accelX[i];
    avgY += data.accelY[i];
    avgZ += data.accelZ[i];
  }
  avgX /= data.count;
  avgY /= data.count;
  avgZ /= data.count;
}

// 分析感測器方向
void analyzeSensorOrientation()
{
  float stand1X, stand1Y, stand1Z;
  float lift1X, lift1Y, lift1Z;
  float stand2X, stand2Y, stand2Z;
  float lift2X, lift2Y, lift2Z;

  calculateAxisAverages(standData1, stand1X, stand1Y, stand1Z);
  calculateAxisAverages(liftData1, lift1X, lift1Y, lift1Z);
  calculateAxisAverages(standData2, stand2X, stand2Y, stand2Z);
  calculateAxisAverages(liftData2, lift2X, lift2Y, lift2Z);

  // 計算站立和抬腳的平均值
  float standAvgX = (stand1X + stand2X) / 2.0;
  float standAvgY = (stand1Y + stand2Y) / 2.0;
  float standAvgZ = (stand1Z + stand2Z) / 2.0;

  float liftAvgX = (lift1X + lift2X) / 2.0;
  float liftAvgY = (lift1Y + lift2Y) / 2.0;
  float liftAvgZ = (lift1Z + lift2Z) / 2.0;

  // 計算站立到抬腳的變化量
  float deltaX = abs(liftAvgX - standAvgX);
  float deltaY = abs(liftAvgY - standAvgY);
  float deltaZ = abs(liftAvgZ - standAvgZ);

  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║        📊 感測器數據分析結果            ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ 站立時加速度平均值：                   ║");
  Serial.printf("║   X: %7.3f  Y: %7.3f  Z: %7.3f  ║\n", standAvgX, standAvgY, standAvgZ);
  Serial.println("║ 抬腳時加速度平均值：                   ║");
  Serial.printf("║   X: %7.3f  Y: %7.3f  Z: %7.3f  ║\n", liftAvgX, liftAvgY, liftAvgZ);
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ 站立→抬腳 變化量：                    ║");
  Serial.printf("║   ΔX: %6.3f  ΔY: %6.3f  ΔZ: %6.3f ║\n", deltaX, deltaY, deltaZ);
  Serial.println("╠════════════════════════════════════════╣");

  // 判斷主要感測軸（變化最大的軸）
  // 同時計算抬腳時角度應該增加還是減少
  float standAngleX = atan2(standAvgX, standAvgZ) * 180.0 / PI;
  float liftAngleX = atan2(liftAvgX, liftAvgZ) * 180.0 / PI;
  float standAngleY = atan2(standAvgY, standAvgZ) * 180.0 / PI;
  float liftAngleY = atan2(liftAvgY, liftAvgZ) * 180.0 / PI;
  float standAngleZ = atan2(standAvgZ, standAvgY) * 180.0 / PI;
  float liftAngleZ = atan2(liftAvgZ, liftAvgY) * 180.0 / PI;

  Serial.println("║ 計算的角度變化：                       ║");
  Serial.printf("║   X軸角度: 站%.1f° → 抬%.1f° (Δ%.1f°) ║\n",
                standAngleX, liftAngleX, liftAngleX - standAngleX);
  Serial.printf("║   Y軸角度: 站%.1f° → 抬%.1f° (Δ%.1f°) ║\n",
                standAngleY, liftAngleY, liftAngleY - standAngleY);
  Serial.println("╠════════════════════════════════════════╣");

  if (deltaX >= deltaY && deltaX >= deltaZ)
  {
    sensorOrient.primaryAxis = 0; // X 軸
    // 判斷抬腳時角度是否增加，如果減少則需要反轉
    sensorOrient.axisSign = (liftAngleX > standAngleX) ? 1.0 : -1.0;
    sensorOrient.standAvg = standAvgX;
    sensorOrient.liftAvg = liftAvgX;
    Serial.println("║ 🎯 主要感測軸：X 軸                    ║");
  }
  else if (deltaY >= deltaX && deltaY >= deltaZ)
  {
    sensorOrient.primaryAxis = 1; // Y 軸
    sensorOrient.axisSign = (liftAngleY > standAngleY) ? 1.0 : -1.0;
    sensorOrient.standAvg = standAvgY;
    sensorOrient.liftAvg = liftAvgY;
    Serial.println("║ 🎯 主要感測軸：Y 軸                    ║");
  }
  else
  {
    sensorOrient.primaryAxis = 2; // Z 軸
    sensorOrient.axisSign = (liftAngleZ > standAngleZ) ? 1.0 : -1.0;
    sensorOrient.standAvg = standAvgZ;
    sensorOrient.liftAvg = liftAvgZ;
    Serial.println("║ 🎯 主要感測軸：Z 軸                    ║");
  }

  // 判斷重力軸（站立時絕對值最大的軸）
  float absStandX = abs(standAvgX);
  float absStandY = abs(standAvgY);
  float absStandZ = abs(standAvgZ);

  if (absStandZ >= absStandX && absStandZ >= absStandY)
  {
    sensorOrient.secondaryAxis = 2;
    sensorOrient.gravityAxis = standAvgZ;
    Serial.println("║ 🌍 重力軸：Z 軸                        ║");
  }
  else if (absStandY >= absStandX && absStandY >= absStandZ)
  {
    sensorOrient.secondaryAxis = 1;
    sensorOrient.gravityAxis = standAvgY;
    Serial.println("║ 🌍 重力軸：Y 軸                        ║");
  }
  else
  {
    sensorOrient.secondaryAxis = 0;
    sensorOrient.gravityAxis = standAvgX;
    Serial.println("║ 🌍 重力軸：X 軸                        ║");
  }

  Serial.printf("║ 軸向符號：%s                         ║\n",
                sensorOrient.axisSign > 0 ? "+1（抬腳增加）" : "-1（抬腳減少）");
  Serial.println("╚════════════════════════════════════════╝\n");
}

// ===== MQTT 校正狀態發送函數 =====
void sendCalibrationStatus(const char *step, const char *action, int progress, int samples,
                           float accelX, float accelY, float accelZ)
{
  if (!mqttClient.connected())
    return;

  JsonDocument doc;
  doc["type"] = "calibration";
  doc["step"] = step;
  doc["action"] = action;
  doc["progress"] = progress;
  doc["samples"] = samples;

  JsonObject accel = doc["accel"].to<JsonObject>();
  accel["x"] = accelX;
  accel["y"] = accelY;
  accel["z"] = accelZ;

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  mqttClient.publish(MQTT_TOPIC, jsonBuffer, false);
}

// 發送校正分析結果到 MQTT
void sendCalibrationResult()
{
  if (!mqttClient.connected())
    return;

  JsonDocument doc;
  doc["type"] = "calibration_complete";
  doc["primary_axis"] = sensorOrient.primaryAxis;
  doc["gravity_axis"] = sensorOrient.secondaryAxis;
  doc["axis_sign"] = sensorOrient.axisSign;
  doc["stand_avg"] = sensorOrient.standAvg;
  doc["lift_avg"] = sensorOrient.liftAvg;
  doc["gravity_value"] = sensorOrient.gravityAxis;

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  mqttClient.publish(MQTT_TOPIC, jsonBuffer, false);
}

// Wi-Fi 連線函數
void connectWiFi()
{
  Serial.println("\n========================================");
  Serial.println("正在連接 Wi-Fi...");
  Serial.printf("SSID: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\n✓ Wi-Fi 連線成功！");
    Serial.printf("IP 位址: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("訊號強度: %d dBm\n", WiFi.RSSI());
  }
  else
  {
    Serial.println("\n✗ Wi-Fi 連線失敗！");
    Serial.println("請檢查 SSID 和密碼設定");
  }
  Serial.println("========================================\n");
}

// MQTT 重連函數
void reconnectMQTT()
{
  if (!mqttClient.connected())
  {
    Serial.print("正在連接 MQTT...");

    // 產生唯一的客戶端 ID
    String clientId = "ESP32-KneeDrive-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD))
    {
      Serial.println(" ✓ 連線成功！");
      Serial.printf("Topic: %s\n", MQTT_TOPIC);
    }
    else
    {
      Serial.printf(" ✗ 失敗，狀態碼: %d\n", mqttClient.state());
    }
  }
}

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

// ===== FreeRTOS 取樣任務：50Hz 高精度感測器讀取 =====
void samplingTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_INTERVAL_MS);

  Serial.println("[SamplingTask] 任務啟動，開始 50Hz 取樣");

  while (true)
  {
    // 精確 20ms 間隔（使用 vTaskDelayUntil 確保穩定頻率）
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // 檢查 IMU 是否有新數據
    if (!imu.dataReady())
    {
      continue;
    }

    // 計算時間差
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // 讀取感測器數據
    imu.getAGMT();

    float accelX = imu.accX();
    float accelY = imu.accY();
    float accelZ = imu.accZ();
    float gyroX = imu.gyrX();
    float gyroY = imu.gyrY();
    float gyroZ = imu.gyrZ();

    // ===== 校正階段處理 =====
    if (!isCalibrated)
    {
      unsigned long stateElapsed = currentTime - stateStartTime;

      switch (calState)
      {
      case CAL_INIT:
        if (currentTime >= stateStartTime)
        {
          calState = CAL_STAND_1;
          stateStartTime = currentTime;
          Serial.println("\n╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  📍 步驟 1/4：站立                                       ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
        }
        break;

      case CAL_STAND_1:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(standData1, accelX, accelY, accelZ);
        }
        else
        {
          calState = CAL_LIFT_1;
          stateStartTime = currentTime;
          Serial.println("\n╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  🦵 步驟 2/4：抬腳                                       ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
        }
        break;

      case CAL_LIFT_1:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(liftData1, accelX, accelY, accelZ);
        }
        else
        {
          calState = CAL_STAND_2;
          stateStartTime = currentTime;
          Serial.println("\n╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  📍 步驟 3/4：再次站立                                   ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
        }
        break;

      case CAL_STAND_2:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(standData2, accelX, accelY, accelZ);
        }
        else
        {
          calState = CAL_LIFT_2;
          stateStartTime = currentTime;
          Serial.println("\n╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  🦵 步驟 4/4：最後一次抬腳                               ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
        }
        break;

      case CAL_LIFT_2:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(liftData2, accelX, accelY, accelZ);
        }
        else
        {
          calState = CAL_ANALYZING;
          Serial.println("\n╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  🎉 數據收集完成！正在分析...                            ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
        }
        break;

      case CAL_ANALYZING:
        analyzeSensorOrientation();
        calState = CAL_COMPLETE;
        isCalibrated = true;
        startTime = currentTime;
        initialKneeY = 0;
        initialKneeZ = -THIGH_LENGTH;
        sendCalibrationResult();
        Serial.println("\n✓ 校正完成！開始正常測量...\n");
        break;

      case CAL_COMPLETE:
        break;
      }
      continue; // 校正期間不存儲數據到 ring buffer
    }

    // ===== 正常測量模式：計算角度 =====
    float accelAngle = 0.0;
    float gyroRate = 0.0;
    float primaryAccel = 0.0;
    float gravityAccel = 0.0;

    // 根據校正結果選擇軸向
    if (sensorOrient.primaryAxis == 0)
    {
      primaryAccel = accelX;
      gyroRate = (sensorOrient.secondaryAxis == 2) ? gyroY : gyroZ;
    }
    else if (sensorOrient.primaryAxis == 1)
    {
      primaryAccel = accelY;
      gyroRate = (sensorOrient.secondaryAxis == 2) ? gyroX : gyroZ;
    }
    else
    {
      primaryAccel = accelZ;
      gyroRate = (sensorOrient.secondaryAxis == 1) ? gyroX : gyroY;
    }

    if (sensorOrient.secondaryAxis == 0)
      gravityAccel = accelX;
    else if (sensorOrient.secondaryAxis == 1)
      gravityAccel = accelY;
    else
      gravityAccel = accelZ;

    // 計算加速度角度
    accelAngle = atan2(primaryAccel, gravityAccel) * 180.0 / PI;

    // 陀螺儀積分
    float gyroAngle = thighAngle + gyroRate * deltaTime;

    // 互補濾波
    float alpha = 0.90;
    thighAngle = alpha * gyroAngle + (1 - alpha) * accelAngle;

    // 角度標準化
    if (thighAngle > 180.0)
      thighAngle -= 360.0;
    if (thighAngle < -180.0)
      thighAngle += 360.0;

    // 應用校正
    float standAngle = atan2(sensorOrient.standAvg, sensorOrient.gravityAxis) * 180.0 / PI;
    float rawCalibratedAngle = thighAngle - standAngle;

    while (rawCalibratedAngle > 180.0)
      rawCalibratedAngle -= 360.0;
    while (rawCalibratedAngle < -180.0)
      rawCalibratedAngle += 360.0;

    float calibratedAngle = rawCalibratedAngle * sensorOrient.axisSign;
    float displayAngle = abs(calibratedAngle);
    if (displayAngle > 120.0)
      displayAngle = 180.0 - displayAngle;

    // 低通濾波
    float angleDelta = abs(displayAngle - smoothedAngle);
    if (angleDelta > 30.0 && smoothedAngle != 0.0)
    {
      smoothedAngle = smoothedAngle * 0.95 + displayAngle * 0.05;
    }
    else
    {
      smoothedAngle = smoothedAngle * (1.0 - SMOOTHING_FACTOR) + displayAngle * SMOOTHING_FACTOR;
    }

    calibratedAngle = smoothedAngle;

    // 計算膝蓋座標
    float angleRad = calibratedAngle * PI / 180.0;
    kneeX = 0.0;
    kneeY = THIGH_LENGTH * sin(angleRad);
    kneeZ = -THIGH_LENGTH * cos(angleRad);

    float deltaY = kneeY - initialKneeY;
    float deltaZ = kneeZ - initialKneeZ;

    // 穩定度檢測
    float angleDiff = abs(calibratedAngle - prevAngle);
    if (angleDiff < 0.5)
    {
      stableCount++;
    }
    else
    {
      stableCount = 0;
      isStable = false;
    }
    if (stableCount >= 30 && !isStable)
    {
      isStable = true;
    }
    prevAngle = calibratedAngle;

    // ===== 儲存到 Ring Buffer =====
    SensorSample sample;
    sample.timestamp = currentTime;
    sample.angle = abs(calibratedAngle);
    sample.deltaY = deltaY;
    sample.deltaZ = deltaZ;
    sample.accelX = accelX;
    sample.accelY = accelY;
    sample.accelZ = accelZ;
    sample.gyroX = gyroX;
    sample.gyroY = gyroY;
    sample.gyroZ = gyroZ;
    sample.stable = isStable;

    if (!pushSample(&sample))
    {
      // Buffer 真的滿了，記錄警告（但不阻塞）
      static unsigned long lastWarning = 0;
      if (currentTime - lastWarning > 5000)
      {
        Serial.printf("⚠️ Ring buffer 已滿 (%d/%d)，丟棄: %lu 筆\n",
                      bufferCount, RING_BUFFER_SIZE, droppedCount);
        lastWarning = currentTime;
      }
    }
  }
}

// ===== FreeRTOS 傳輸任務：2Hz 批次 MQTT 發送 =====
void transmitTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SEND_INTERVAL_MS);

  Serial.println("[TransmitTask] 任務啟動，開始 2Hz 批次傳輸");

  SensorSample batchBuffer[BATCH_SIZE];

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // 維護 WiFi 連線
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("⚠️ Wi-Fi 斷線，嘗試重連...");
      connectWiFi();
      continue;
    }

    // 維護 MQTT 連線
    if (!mqttClient.connected())
    {
      reconnectMQTT();
      if (!mqttClient.connected())
      {
        continue;
      }
    }

    mqttClient.loop();

    // 從 Ring Buffer 取出數據
    uint16_t count = popSamples(batchBuffer, BATCH_SIZE);

    if (count == 0)
    {
      continue; // 沒有數據可發送
    }

    // 建立批次 JSON（使用縮短的欄位名稱）
    JsonDocument doc;
    doc["type"] = "batch";
    doc["n"] = count;

    JsonArray dataArray = doc["d"].to<JsonArray>();

    for (uint16_t i = 0; i < count; i++)
    {
      JsonObject item = dataArray.add<JsonObject>();
      item["t"] = batchBuffer[i].timestamp / 1000.0;             // 時間戳（秒）
      item["a"] = round(batchBuffer[i].angle * 10) / 10.0;       // 角度（保留1位小數）
      item["s"] = batchBuffer[i].stable ? 1 : 0;                 // 穩定（0/1）
      item["dy"] = round(batchBuffer[i].deltaY * 10) / 10.0;     // deltaY
      item["dz"] = round(batchBuffer[i].deltaZ * 10) / 10.0;     // deltaZ
      item["ax"] = round(batchBuffer[i].accelX * 1000) / 1000.0; // 加速度X
      item["ay"] = round(batchBuffer[i].accelY * 1000) / 1000.0;
      item["az"] = round(batchBuffer[i].accelZ * 1000) / 1000.0;
      item["gx"] = round(batchBuffer[i].gyroX * 10) / 10.0; // 陀螺儀X
      item["gy"] = round(batchBuffer[i].gyroY * 10) / 10.0;
      item["gz"] = round(batchBuffer[i].gyroZ * 10) / 10.0;
    }

    // 序列化並發送
    char jsonBuffer[4096];
    size_t jsonSize = serializeJson(doc, jsonBuffer);

    bool success = mqttClient.publish(MQTT_TOPIC, jsonBuffer, false);

    if (success)
    {
      Serial.printf("✓ 批次發送 %d 筆 (%d bytes)，緩衝區剩餘: %d\n",
                    count, jsonSize, getBufferCount());
    }
    else
    {
      Serial.printf("✗ MQTT 發送失敗 (狀態: %d)\n", mqttClient.state());
    }
  }
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

  // 初始化校正數據結構
  initCalibrationData(standData1);
  initCalibrationData(liftData1);
  initCalibrationData(standData2);
  initCalibrationData(liftData2);

  // ===== 先連接 Wi-Fi 和 MQTT =====
  // 連接 Wi-Fi
  connectWiFi();

  // 設定 MQTT 伺服器
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(4096); // 擴大為 4KB 以容納批次 JSON
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(30);

  // 等待 MQTT 連線成功
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║  📡 正在建立 MQTT 連線...                          ║");
  Serial.println("╚════════════════════════════════════════════════════╝");

  bool mqttConnected = false;
  int mqttAttempts = 0;

  while (!mqttConnected && mqttAttempts < 5)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      reconnectMQTT();
      if (mqttClient.connected())
      {
        mqttConnected = true;

        // 發送測試訊息確認連線
        JsonDocument testDoc;
        testDoc["type"] = "connection_test";
        testDoc["status"] = "connected";
        testDoc["device"] = "ESP32-KneeDrive";

        char testBuffer[128];
        serializeJson(testDoc, testBuffer);

        if (mqttClient.publish(MQTT_TOPIC, testBuffer, false))
        {
          Serial.println("\n✓ MQTT 連線成功！測試訊息已發送");
        }
        else
        {
          Serial.println("\n⚠️ MQTT 連線成功，但測試訊息發送失敗");
        }
      }
    }

    if (!mqttConnected)
    {
      mqttAttempts++;
      Serial.printf("  嘗試 %d/5...\n", mqttAttempts);
      delay(2000);
    }
  }

  if (!mqttConnected)
  {
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║  ⚠️ MQTT 連線失敗！                                ║");
    Serial.println("║                                                    ║");
    Serial.println("║  校正資訊將無法傳送到接收端                        ║");
    Serial.println("║  請檢查：                                          ║");
    Serial.println("║  1. Wi-Fi 熱點是否開啟                             ║");
    Serial.println("║  2. MQTT 伺服器是否正常                            ║");
    Serial.println("║                                                    ║");
    Serial.println("║  5 秒後將繼續進行校正（僅序列埠輸出）              ║");
    Serial.println("╚════════════════════════════════════════════════════╝\n");
    delay(5000);
  }
  else
  {
    Serial.println("\n╔════════════════════════════════════════════════════╗");
    Serial.println("║  ✓ MQTT 連線成功！                                 ║");
    Serial.println("║                                                    ║");
    Serial.println("║  請在接收端執行 mqtt_receiver.py 準備接收          ║");
    Serial.println("╚════════════════════════════════════════════════════╝\n");
    delay(2000);
  }

  // ===== 顯示校正說明 =====
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║  🔧 動態校正系統啟動                              ║");
  Serial.println("║                                                    ║");
  Serial.println("║  請依照以下步驟進行校正：                          ║");
  Serial.println("║  1️⃣  站立 3 秒（大腿垂直向下）                    ║");
  Serial.println("║  2️⃣  抬腳 3 秒（膝蓋抬高）                        ║");
  Serial.println("║  3️⃣  站立 3 秒                                    ║");
  Serial.println("║  4️⃣  抬腳 3 秒                                    ║");
  Serial.println("║                                                    ║");
  Serial.println("║  系統會分析感測器安裝方向並自動校正                ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");
  Serial.println("⏳ 3 秒後開始校正...\n");

  lastTime = millis();
  startTime = millis();
  calibrationStartTime = millis();
  stateStartTime = millis() + 3000; // 3 秒後開始第一階段
  calState = CAL_INIT;
  delay(100);

  // ===== 初始化 FreeRTOS 同步物件 =====
  bufferMutex = xSemaphoreCreateMutex();
  if (bufferMutex == NULL)
  {
    Serial.println("✗ Mutex 建立失敗！");
    while (1)
      delay(1000);
  }
  Serial.println("✓ FreeRTOS Mutex 初始化完成");

  // ===== 建立 FreeRTOS 任務 =====
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║  🚀 FreeRTOS 雙任務架構啟動                        ║");
  Serial.println("║                                                    ║");
  Serial.printf("║  取樣頻率: %d Hz (每 %d ms)                        ║\n", 1000 / SAMPLE_INTERVAL_MS, SAMPLE_INTERVAL_MS);
  Serial.printf("║  發送頻率: %d Hz (每 %d ms)                        ║\n", 1000 / SEND_INTERVAL_MS, SEND_INTERVAL_MS);
  Serial.printf("║  批次大小: %d 筆                                   ║\n", BATCH_SIZE);
  Serial.printf("║  緩衝區: %d 筆 (%.1f 秒)                           ║\n", RING_BUFFER_SIZE, (float)RING_BUFFER_SIZE * SAMPLE_INTERVAL_MS / 1000);
  Serial.println("╚════════════════════════════════════════════════════╝\n");

  // 建立取樣任務（高優先級）
  BaseType_t result = xTaskCreatePinnedToCore(
      samplingTask,        // 任務函數
      "SamplingTask",      // 任務名稱
      SAMPLING_STACK_SIZE, // Stack 大小
      NULL,                // 參數
      SAMPLING_PRIORITY,   // 優先級
      &samplingTaskHandle, // Handle
      0                    // Core 0 (ESP32-C3 只有 Core 0)
  );

  if (result == pdPASS)
  {
    Serial.println("✓ 取樣任務建立成功 (50Hz, 優先級 3)");
  }
  else
  {
    Serial.println("✗ 取樣任務建立失敗！");
  }

  // 建立傳輸任務（低優先級）
  result = xTaskCreatePinnedToCore(
      transmitTask,        // 任務函數
      "TransmitTask",      // 任務名稱
      TRANSMIT_STACK_SIZE, // Stack 大小
      NULL,                // 參數
      TRANSMIT_PRIORITY,   // 優先級
      &transmitTaskHandle, // Handle
      0                    // Core 0
  );

  if (result == pdPASS)
  {
    Serial.println("✓ 傳輸任務建立成功 (2Hz, 優先級 1)");
  }
  else
  {
    Serial.println("✗ 傳輸任務建立失敗！");
  }

  Serial.println("\n開始運行...\n");
}

void loop()
{
  // ===== FreeRTOS 架構：主迴圈閒置 =====
  // 所有工作都由 samplingTask 和 transmitTask 執行
  // loop() 只需維持 Arduino 框架運行
  vTaskDelay(pdMS_TO_TICKS(1000));
}