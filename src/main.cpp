#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h" // SparkFun ICM-20948 函式庫
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Wi-Fi 設定
const char* WIFI_SSID = "Bt";
const char* WIFI_PASSWORD = "bt_980904";

// MQTT 設定
const char* MQTT_SERVER = "mqtt.singularinnovation-ai.com";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "singular";
const char* MQTT_PASSWORD = "Singular#1234";
const char* MQTT_TOPIC = "knee-drive/data";

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

// 校正變數
bool isCalibrated = false;
float calibrationAngle = 0.0; // 校正用的初始角度
float calibrationSum = 0.0;   // 累積角度總和
int calibrationCount = 0;     // 校正樣本數量
unsigned long calibrationStartTime = 0;

// 初始座標（用於相對位移計算）
float initialKneeY = 0.0;
float initialKneeZ = 0.0;

// 穩定度檢測變數
float prevAngle = 0.0;
int stableCount = 0;
bool isStable = false;
unsigned long startTime = 0;

// Wi-Fi 連線函數
void connectWiFi() {
  Serial.println("\n========================================");
  Serial.println("正在連接 Wi-Fi...");
  Serial.printf("SSID: %s\n", WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ Wi-Fi 連線成功！");
    Serial.printf("IP 位址: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("訊號強度: %d dBm\n", WiFi.RSSI());
  } else {
    Serial.println("\n✗ Wi-Fi 連線失敗！");
    Serial.println("請檢查 SSID 和密碼設定");
  }
  Serial.println("========================================\n");
}

// MQTT 重連函數
void reconnectMQTT() {
  if (!mqttClient.connected()) {
    Serial.print("正在連接 MQTT...");
    
    // 產生唯一的客戶端 ID
    String clientId = "ESP32-KneeDrive-";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println(" ✓ 連線成功！");
      Serial.printf("Topic: %s\n", MQTT_TOPIC);
    } else {
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
  Serial.println("\n⏱️  數值穩定建議時間：");
  Serial.println("  • 靜態測量（站立）：等待 3-5 秒");
  Serial.println("  • 動態測量（抬腿）：0.5-1 秒即可");
  Serial.println("  • 長時間追蹤：建議不超過 60 秒\n");

  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║  🔧 自動校正中...                     ║");
  Serial.println("║  請保持站立姿勢不動 3 秒               ║");
  Serial.println("╚════════════════════════════════════════╝\n");

  // 連接 Wi-Fi
  connectWiFi();
  
  // 設定 MQTT 伺服器
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(512);  // 增加 MQTT buffer 大小到 512 bytes
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(30);
  
  // 初次連接 MQTT
  if (WiFi.status() == WL_CONNECTED) {
    reconnectMQTT();
  }

  lastTime = millis();
  startTime = millis();
  calibrationStartTime = millis();
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

    // ===== 自動校正（前 3 秒）=====
    if (!isCalibrated)
    {
      unsigned long calibrationElapsed = currentTime - calibrationStartTime;

      if (calibrationElapsed < 3000) // 前 3 秒收集數據
      {
        calibrationSum += thighAngle;
        calibrationCount++;

        // 顯示校正進度
        if (calibrationCount % 10 == 0)
        { // 每 10 次顯示一次
          Serial.printf("⏳ 校正中... %.1f 秒 (樣本數: %d)\n",
                        calibrationElapsed / 1000.0, calibrationCount);
        }
      }
      else // 3 秒後完成校正
      {
        calibrationAngle = calibrationSum / calibrationCount;
        isCalibrated = true;

        // 計算並記錄初始座標
        float initAngleRad = 0.0;                         // 校正後角度為 0
        initialKneeY = THIGH_LENGTH * sin(initAngleRad);  // 應該接近 0
        initialKneeZ = -THIGH_LENGTH * cos(initAngleRad); // 應該是 -45

        Serial.println("\n╔════════════════════════════════════════╗");
        Serial.println("║  ✓ 校正完成！                         ║");
        Serial.printf("║  校正角度偏移: %6.2f°              ║\n", calibrationAngle);
        Serial.println("║  初始座標已設定為 (0, 0, 0)           ║");
        Serial.printf("║  實際初始位置: Y=%.1f, Z=%.1f cm   ║\n", initialKneeY, initialKneeZ);
        Serial.println("╚════════════════════════════════════════╝\n");
        Serial.println("📍 座標顯示說明：");
        Serial.println("   • 顯示的是「相對於初始位置」的變化");
        Serial.println("   • 站立時應該接近 (0, 0, 0)");
        Serial.println("   • 抬腿時會看到 Y 和 Z 的變化\n");
        Serial.println("開始正常測量...\n");
      }
    }

    // ===== 應用校正 =====
    float calibratedAngle = thighAngle - calibrationAngle;

    // ===== 計算膝蓋座標（相對於髖關節）=====
    // 原點 = 髖關節 (0, 0, 0)
    // 目標 = 膝蓋位置 (x, y, z)

    // 轉換角度為弧度（使用校正後的角度）
    float angleRad = calibratedAngle * PI / 180.0;

    // 座標計算（極坐標轉直角座標）
    // 假設：0° = 大腿垂直向下，90° = 大腿水平向前
    kneeX = 0.0;                           // 左右不動（簡化）
    kneeY = THIGH_LENGTH * sin(angleRad);  // 前後位移（正值 = 往前）
    kneeZ = -THIGH_LENGTH * cos(angleRad); // 上下位移（負值 = 往下，0° 時在髖關節正下方）

    // ===== 計算相對於初始位置的變化 =====
    float deltaY = kneeY - initialKneeY; // 相對前後位移
    float deltaZ = kneeZ - initialKneeZ; // 相對上下位移

    // ===== 顯示數據（只在校正完成後）=====
    if (!isCalibrated)
    {
      delay(100);
      return; // 校正期間不顯示其他數據
    }

    // ===== 穩定度檢測 =====
    float angleDiff = abs(calibratedAngle - prevAngle);
    if (angleDiff < 0.5)
    { // 角度變化小於 0.5 度
      stableCount++;
    }
    else
    {
      stableCount = 0;
      isStable = false;
    }

    // 連續 30 次測量（約 3 秒）角度穩定，判定為穩定
    if (stableCount >= 30 && !isStable)
    {
      isStable = true;
      Serial.println("\n✓ 數值已穩定！現在可以進行準確測量。\n");
    }

    prevAngle = calibratedAngle; // 使用校正後的角度

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

    // ===== MQTT 資料傳輸 =====
    // 檢查 Wi-Fi 連線
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("⚠️ Wi-Fi 斷線，嘗試重連...");
      connectWiFi();
    }
    
    // 檢查 MQTT 連線
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
      reconnectMQTT();
    }
    
    // 發布資料到 MQTT
    if (mqttClient.connected()) {
      // 建立 JSON 文件
      JsonDocument doc;
      doc["timestamp"] = currentTime / 1000.0;
      doc["elapsed_time"] = elapsedTime;
      doc["angle"] = abs(calibratedAngle);
      doc["stable"] = isStable;
      
      // 相對位移
      JsonObject delta = doc["delta"].to<JsonObject>();
      delta["x"] = 0.0;
      delta["y"] = deltaY;
      delta["z"] = deltaZ;
      
      // 絕對座標
      JsonObject absolute = doc["absolute"].to<JsonObject>();
      absolute["x"] = kneeX;
      absolute["y"] = kneeY;
      absolute["z"] = kneeZ;
      
      // 原始感測器數據
      JsonObject accel = doc["accel"].to<JsonObject>();
      accel["x"] = accelX;
      accel["y"] = accelY;
      accel["z"] = accelZ;
      
      JsonObject gyro = doc["gyro"].to<JsonObject>();
      gyro["x"] = gyroX;
      gyro["y"] = gyroY;
      gyro["z"] = gyroZ;
      
      // 序列化 JSON
      char jsonBuffer[512];
      size_t jsonSize = serializeJson(doc, jsonBuffer);
      
      // 顯示 JSON 大小（除錯用）
      Serial.printf("JSON 大小: %d bytes\n", jsonSize);
      
      // 發布到 MQTT (QoS 0 = 最多一次，不需確認)
      bool success = mqttClient.publish(MQTT_TOPIC, jsonBuffer, false);
      
      if (success) {
        Serial.println("✓ 資料已發送到 MQTT");
      } else {
        Serial.println("✗ MQTT 發送失敗");
        Serial.printf("  原因: Buffer 大小不足或連線問題\n");
        Serial.printf("  MQTT 狀態: %d\n", mqttClient.state());
      }
    } else {
      Serial.println("⚠️ MQTT 未連線，資料未發送");
    }
    
    // 保持 MQTT 連線
    mqttClient.loop();
    Serial.println();
  }

  // 100Hz 取樣頻率 (每10ms讀取一次)
  delay(500); // 改為 100ms，每秒更新 10 次
}