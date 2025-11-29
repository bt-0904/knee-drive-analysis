#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h" // SparkFun ICM-20948 函式庫
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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
  mqttClient.setBufferSize(512);
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

    // ===== 動態校正狀態機 =====
    if (!isCalibrated)
    {
      unsigned long stateElapsed = currentTime - stateStartTime;

      switch (calState)
      {
      case CAL_INIT:
        // 等待初始 3 秒
        if (currentTime >= stateStartTime)
        {
          calState = CAL_STAND_1;
          stateStartTime = currentTime;
          Serial.println();
          Serial.println("╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  ┌─────────────────────────────────────────────────────┐ ║");
          Serial.println("║  │  📍 步驟 1/4：站立                                  │ ║");
          Serial.println("║  └─────────────────────────────────────────────────────┘ ║");
          Serial.println("║                                                          ║");
          Serial.println("║  👤 動作：雙腳自然站立，大腿保持垂直向下                ║");
          Serial.println("║                                                          ║");
          Serial.println("║  ⚠️  注意：請保持身體穩定，不要晃動                      ║");
          Serial.println("║                                                          ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
          // 發送 MQTT 狀態
          sendCalibrationStatus("1/4", "stand", 0, 0, accelX, accelY, accelZ);
        }
        else
        {
          int remaining = (stateStartTime - currentTime) / 1000 + 1;
          static int lastRemaining = 0;
          if (remaining != lastRemaining)
          {
            Serial.println();
            Serial.println("┌────────────────────────────────────┐");
            Serial.printf("│  ⏳ 校正即將開始... %d 秒           │\n", remaining);
            Serial.println("│                                    │");
            Serial.println("│  🧍 請先保持站立姿勢準備好         │");
            Serial.println("└────────────────────────────────────┘");
            // 發送準備狀態
            sendCalibrationStatus("準備中", "waiting", remaining, 0, accelX, accelY, accelZ);
            lastRemaining = remaining;
          }
        }
        break;

      case CAL_STAND_1:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(standData1, accelX, accelY, accelZ);
          int remainingSec = (STATE_DURATION - stateElapsed) / 1000 + 1;
          static int lastStand1Sec = 0;
          if (remainingSec != lastStand1Sec)
          {
            // 進度條
            int progress = (stateElapsed * 100) / STATE_DURATION;
            Serial.print("📍 站立中 [");
            for (int i = 0; i < 20; i++)
            {
              Serial.print(i < (progress / 5) ? "█" : "░");
            }
            Serial.printf("] %d秒  (樣本:%d)\n", remainingSec, standData1.count);
            // 發送 MQTT 狀態
            sendCalibrationStatus("1/4", "stand", progress, standData1.count, accelX, accelY, accelZ);
            lastStand1Sec = remainingSec;
          }
        }
        else
        {
          calState = CAL_LIFT_1;
          stateStartTime = currentTime;
          Serial.println();
          Serial.println("╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  ┌─────────────────────────────────────────────────────┐ ║");
          Serial.println("║  │  🦵 步驟 2/4：抬腳                                  │ ║");
          Serial.println("║  └─────────────────────────────────────────────────────┘ ║");
          Serial.println("║                                                          ║");
          Serial.println("║  👤 動作：抬起一隻腳，膝蓋盡量抬高（約 45-90 度）       ║");
          Serial.println("║                                                          ║");
          Serial.println("║  💡 提示：可以扶著牆壁保持平衡                          ║");
          Serial.println("║                                                          ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
          // 發送步驟切換通知
          sendCalibrationStatus("2/4", "lift", 0, 0, accelX, accelY, accelZ);
        }
        break;

      case CAL_LIFT_1:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(liftData1, accelX, accelY, accelZ);
          int remainingSec = (STATE_DURATION - stateElapsed) / 1000 + 1;
          static int lastLift1Sec = 0;
          if (remainingSec != lastLift1Sec)
          {
            int progress = (stateElapsed * 100) / STATE_DURATION;
            Serial.print("🦵 抬腳中 [");
            for (int i = 0; i < 20; i++)
            {
              Serial.print(i < (progress / 5) ? "█" : "░");
            }
            Serial.printf("] %d秒  (樣本:%d)\n", remainingSec, liftData1.count);
            // 發送 MQTT 狀態
            sendCalibrationStatus("2/4", "lift", progress, liftData1.count, accelX, accelY, accelZ);
            lastLift1Sec = remainingSec;
          }
        }
        else
        {
          calState = CAL_STAND_2;
          stateStartTime = currentTime;
          Serial.println();
          Serial.println("╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  ┌─────────────────────────────────────────────────────┐ ║");
          Serial.println("║  │  📍 步驟 3/4：再次站立                              │ ║");
          Serial.println("║  └─────────────────────────────────────────────────────┘ ║");
          Serial.println("║                                                          ║");
          Serial.println("║  👤 動作：放下腳，回到自然站立姿勢                      ║");
          Serial.println("║                                                          ║");
          Serial.println("║  ⚠️  注意：這是第二次站立數據收集，請保持穩定           ║");
          Serial.println("║                                                          ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
          // 發送步驟切換通知
          sendCalibrationStatus("3/4", "stand", 0, 0, accelX, accelY, accelZ);
        }
        break;

      case CAL_STAND_2:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(standData2, accelX, accelY, accelZ);
          int remainingSec = (STATE_DURATION - stateElapsed) / 1000 + 1;
          static int lastStand2Sec = 0;
          if (remainingSec != lastStand2Sec)
          {
            int progress = (stateElapsed * 100) / STATE_DURATION;
            Serial.print("📍 站立中 [");
            for (int i = 0; i < 20; i++)
            {
              Serial.print(i < (progress / 5) ? "█" : "░");
            }
            Serial.printf("] %d秒  (樣本:%d)\n", remainingSec, standData2.count);
            // 發送 MQTT 狀態
            sendCalibrationStatus("3/4", "stand", progress, standData2.count, accelX, accelY, accelZ);
            lastStand2Sec = remainingSec;
          }
        }
        else
        {
          calState = CAL_LIFT_2;
          stateStartTime = currentTime;
          Serial.println();
          Serial.println("╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  ┌─────────────────────────────────────────────────────┐ ║");
          Serial.println("║  │  🦵 步驟 4/4：最後一次抬腳                          │ ║");
          Serial.println("║  └─────────────────────────────────────────────────────┘ ║");
          Serial.println("║                                                          ║");
          Serial.println("║  👤 動作：再次抬起膝蓋，與第一次抬腳高度相近            ║");
          Serial.println("║                                                          ║");
          Serial.println("║  🎉 這是最後一步！完成後將自動分析數據                  ║");
          Serial.println("║                                                          ║");
          Serial.println("╚══════════════════════════════════════════════════════════╝");
          // 發送步驟切換通知
          sendCalibrationStatus("4/4", "lift", 0, 0, accelX, accelY, accelZ);
        }
        break;

      case CAL_LIFT_2:
        if (stateElapsed < STATE_DURATION)
        {
          addCalibrationSample(liftData2, accelX, accelY, accelZ);
          int remainingSec = (STATE_DURATION - stateElapsed) / 1000 + 1;
          static int lastLift2Sec = 0;
          if (remainingSec != lastLift2Sec)
          {
            int progress = (stateElapsed * 100) / STATE_DURATION;
            Serial.print("🦵 抬腳中 [");
            for (int i = 0; i < 20; i++)
            {
              Serial.print(i < (progress / 5) ? "█" : "░");
            }
            Serial.printf("] %d秒  (樣本:%d)\n", remainingSec, liftData2.count);
            // 發送 MQTT 狀態
            sendCalibrationStatus("4/4", "lift", progress, liftData2.count, accelX, accelY, accelZ);
            lastLift2Sec = remainingSec;
          }
        }
        else
        {
          calState = CAL_ANALYZING;
          Serial.println();
          Serial.println("╔══════════════════════════════════════════════════════════╗");
          Serial.println("║  🎉 數據收集完成！                                       ║");
          Serial.println("║                                                          ║");
          Serial.println("║  📊 正在分析感測器安裝方向...                            ║");
          Serial.println("║                                                          ║");
          Serial.printf("║  📈 收集統計：站立 %d+%d 筆，抬腳 %d+%d 筆             ║\n",
                        standData1.count, standData2.count, liftData1.count, liftData2.count);
          Serial.println("╚══════════════════════════════════════════════════════════╝");
          Serial.println();
          // 發送分析中狀態
          sendCalibrationStatus("分析中", "analyzing", 100,
                                standData1.count + standData2.count + liftData1.count + liftData2.count,
                                accelX, accelY, accelZ);
        }
        break;

      case CAL_ANALYZING:
        // 分析感測器方向
        analyzeSensorOrientation();

        // 設定校正完成
        calState = CAL_COMPLETE;
        isCalibrated = true;
        startTime = currentTime;

        // 設定初始座標
        initialKneeY = 0;
        initialKneeZ = -THIGH_LENGTH;

        // 發送校正結果到 MQTT
        sendCalibrationResult();

        Serial.println("╔════════════════════════════════════════════════════╗");
        Serial.println("║  ✓ 校正完成！系統已偵測感測器安裝方向              ║");
        Serial.println("║                                                    ║");
        Serial.println("║  📍 座標顯示說明：                                  ║");
        Serial.println("║     • 站立時角度應接近 0°                          ║");
        Serial.println("║     • 抬腿時角度會增加                             ║");
        Serial.println("╚════════════════════════════════════════════════════╝\n");
        Serial.println("開始正常測量...\n");
        break;

      case CAL_COMPLETE:
        // 不應該執行到這裡
        break;
      }

      delay(50); // 校正期間降低取樣頻率
      return;
    }

    // ===== 使用自適應角度計算（根據校正結果）=====
    float accelAngle = 0.0;
    float gyroRate = 0.0;

    // 根據偵測到的主軸和重力軸計算角度
    // 使用主軸（抬腳時變化最大的軸）和重力軸來計算傾斜角度
    float primaryAccel = 0.0;
    float gravityAccel = 0.0;

    // 獲取主軸加速度值
    if (sensorOrient.primaryAxis == 0)
    {
      primaryAccel = accelX;
      // 根據主軸選擇適當的陀螺儀軸
      if (sensorOrient.secondaryAxis == 2)
        gyroRate = gyroY;
      else if (sensorOrient.secondaryAxis == 1)
        gyroRate = gyroZ;
      else
        gyroRate = gyroY;
    }
    else if (sensorOrient.primaryAxis == 1)
    {
      primaryAccel = accelY;
      if (sensorOrient.secondaryAxis == 2)
        gyroRate = gyroX;
      else if (sensorOrient.secondaryAxis == 0)
        gyroRate = gyroZ;
      else
        gyroRate = gyroX;
    }
    else
    {
      primaryAccel = accelZ;
      if (sensorOrient.secondaryAxis == 1)
        gyroRate = gyroX;
      else if (sensorOrient.secondaryAxis == 0)
        gyroRate = gyroY;
      else
        gyroRate = gyroX;
    }

    // 獲取重力軸加速度值
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

    // 互補濾波（融合兩者）
    // 降低 alpha 值，讓加速度計有更多權重來減少漂移
    float alpha = 0.90; // 90% 信任陀螺儀，10% 信任加速度計
    thighAngle = alpha * gyroAngle + (1 - alpha) * accelAngle;

    // 確保角度在 -180° 到 180° 範圍內
    if (thighAngle > 180.0)
      thighAngle -= 360.0;
    if (thighAngle < -180.0)
      thighAngle += 360.0;

    // ===== 應用校正（使站立時為 0 度，抬腳時為正角度）=====
    // 計算站立時的基準角度
    float standAngle = atan2(sensorOrient.standAvg, sensorOrient.gravityAxis) * 180.0 / PI;

    // 計算校正後的角度：當前角度 - 站立基準角度
    float rawCalibratedAngle = thighAngle - standAngle;

    // 將角度標準化到 -180 到 180 範圍
    while (rawCalibratedAngle > 180.0)
      rawCalibratedAngle -= 360.0;
    while (rawCalibratedAngle < -180.0)
      rawCalibratedAngle += 360.0;

    // 根據 axisSign 調整方向（確保抬腳時角度增加）
    float calibratedAngle = rawCalibratedAngle * sensorOrient.axisSign;

    // 將校正後的角度限制在合理的抬腳範圍內 (0° ~ 120°)
    // 站立時應該接近 0°，抬腳最多約 90-100°
    // 使用絕對值來顯示抬腳角度
    float displayAngle = abs(calibratedAngle);

    // 如果角度超過 120°，可能是計算錯誤，進行修正
    if (displayAngle > 120.0)
    {
      displayAngle = 180.0 - displayAngle;
    }

    // ===== 低通濾波器（平滑角度輸出，減少突然跳動）=====
    // 檢測異常跳動：如果角度突然變化超過 30 度，忽略這個值
    float angleDelta = abs(displayAngle - smoothedAngle);
    if (angleDelta > 30.0 && smoothedAngle != 0.0)
    {
      // 異常跳動，使用較小的更新權重
      smoothedAngle = smoothedAngle * 0.95 + displayAngle * 0.05;
    }
    else
    {
      // 正常更新：使用低通濾波
      smoothedAngle = smoothedAngle * (1.0 - SMOOTHING_FACTOR) + displayAngle * SMOOTHING_FACTOR;
    }

    // 使用平滑後的角度作為最終顯示值
    calibratedAngle = smoothedAngle;

    // ===== 計算膝蓋座標（相對於髖關節）=====
    float angleRad = calibratedAngle * PI / 180.0;
    kneeX = 0.0;
    kneeY = THIGH_LENGTH * sin(angleRad);
    kneeZ = -THIGH_LENGTH * cos(angleRad);

    // 計算相對於初始位置的變化
    float deltaY = kneeY - initialKneeY;
    float deltaZ = kneeZ - initialKneeZ;

    // ===== 穩定度檢測 =====
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
      Serial.println("\n✓ 數值已穩定！現在可以進行準確測量。\n");
    }

    prevAngle = calibratedAngle;

    // 計算已運行時間
    float elapsedTime = (currentTime - startTime) / 1000.0;

    // ===== 顯示結果 =====
    Serial.println("╔════════════════════════════════════════╗");
    Serial.printf("║ 大腿抬起角度： %6.1f°              ║\n", abs(calibratedAngle));
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║     相對位移 (相對初始位置)           ║");
    Serial.printf("║   ΔX (左右): %7.1f                 ║\n", 0.0);
    Serial.printf("║   ΔY (前後): %7.1f                 ║\n", deltaY);
    Serial.printf("║   ΔZ (上下): %7.1f                 ║\n", deltaZ);
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║     絕對座標 (相對髖關節)             ║");
    Serial.printf("║   X: %7.1f  Y: %7.1f  Z: %7.1f  ║\n", kneeX, kneeY, kneeZ);
    Serial.println("╠════════════════════════════════════════╣");

    // 顯示穩定度指示器
    Serial.print("║ 穩定度： ");
    if (isStable)
    {
      Serial.println("✓ 已穩定                ║");
    }
    else
    {
      Serial.printf("⏳ 穩定中... (%d/30)        ║\n", stableCount);
    }

    // 顯示運行時間
    Serial.printf("║ 運行時間： %.1f 秒", elapsedTime);
    if (elapsedTime > 60)
    {
      Serial.println(" (建議重啟) ║");
    }
    else
    {
      Serial.println("              ║");
    }
    Serial.println("╠════════════════════════════════════════╣");

    // 顯示對應的抬腿程度
    float absAngle = abs(calibratedAngle);
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
    Serial.printf("║ 主軸:%d  重力軸:%d  符號:%.0f         ║\n",
                  sensorOrient.primaryAxis, sensorOrient.secondaryAxis, sensorOrient.axisSign);
    Serial.println("╚════════════════════════════════════════╝");

    // ===== MQTT 資料傳輸 =====
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("⚠️ Wi-Fi 斷線，嘗試重連...");
      connectWiFi();
    }

    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED)
    {
      reconnectMQTT();
    }

    if (mqttClient.connected())
    {
      JsonDocument doc;
      doc["timestamp"] = currentTime / 1000.0;
      doc["elapsed_time"] = elapsedTime;
      doc["angle"] = abs(calibratedAngle);
      doc["stable"] = isStable;

      JsonObject delta = doc["delta"].to<JsonObject>();
      delta["x"] = 0.0;
      delta["y"] = deltaY;
      delta["z"] = deltaZ;

      JsonObject absolute = doc["absolute"].to<JsonObject>();
      absolute["x"] = kneeX;
      absolute["y"] = kneeY;
      absolute["z"] = kneeZ;

      JsonObject accel = doc["accel"].to<JsonObject>();
      accel["x"] = accelX;
      accel["y"] = accelY;
      accel["z"] = accelZ;

      JsonObject gyro = doc["gyro"].to<JsonObject>();
      gyro["x"] = gyroX;
      gyro["y"] = gyroY;
      gyro["z"] = gyroZ;

      // 加入感測器方向資訊
      JsonObject orientation = doc["orientation"].to<JsonObject>();
      orientation["primary_axis"] = sensorOrient.primaryAxis;
      orientation["gravity_axis"] = sensorOrient.secondaryAxis;
      orientation["axis_sign"] = sensorOrient.axisSign;

      char jsonBuffer[512];
      size_t jsonSize = serializeJson(doc, jsonBuffer);

      Serial.printf("JSON 大小: %d bytes\n", jsonSize);

      bool success = mqttClient.publish(MQTT_TOPIC, jsonBuffer, false);

      if (success)
      {
        Serial.println("✓ 資料已發送到 MQTT");
      }
      else
      {
        Serial.println("✗ MQTT 發送失敗");
        Serial.printf("  MQTT 狀態: %d\n", mqttClient.state());
      }
    }
    else
    {
      Serial.println("⚠️ MQTT 未連線，資料未發送");
    }

    mqttClient.loop();
    Serial.println();
  }

  // 100Hz 取樣頻率
  delay(500);
}