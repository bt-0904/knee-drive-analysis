/**
 * Flex Sensor 膝蓋抬腿分析系統
 *
 * 硬體接線：
 *   3.3V ------+------ Flex Sensor ------+------ GPIO4 (ADC)
 *              |                         |
 *              +------- 10K Ohm ---------+------ GND
 *
 * 功能：
 *   - FreeRTOS 雙任務架構（取樣 50Hz + 傳輸 2Hz）
 *   - 固定 offset 校正（預先量測的 ADC 基準值）
 *   - MQTT 批次傳輸至 Python 接收端
 *   - 與 IMU 版本資料格式相容
 *
 * 校正說明：
 *   如需重新校正，請修改以下常數：
 *   - OFFSET_FLAT_ADC：Flex Sensor 平直時（0度）的 ADC 讀數
 *   - OFFSET_BENT_ADC：Flex Sensor 彎曲時（90度）的 ADC 讀數
 *   可透過 Serial Monitor 觀察實際 ADC 值來取得這些數值
 */

#include <Arduino.h>
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
#define TRANSMIT_STACK_SIZE 8192 // 傳輸任務 Stack
#define SAMPLING_PRIORITY 3      // 取樣任務優先級（高）
#define TRANSMIT_PRIORITY 1      // 傳輸任務優先級（低）

// ===== Flex Sensor 硬體配置 =====
#define FLEX_SENSOR_PIN 4 // ADC 輸入腳位 (GPIO4 - ADC1_CH4)
#define LED_PIN 8         // 內建 LED (LOW=亮, HIGH=滅)
#define LED_ON LOW
#define LED_OFF HIGH

// ===== 固定校正 Offset（預先量測的基準值） =====
// 如需重新校正：觀察 Serial Monitor 的 ADC 值，更新這兩個常數
#define OFFSET_FLAT_ADC 2358 // Flex Sensor 平直時（0度）的 ADC 讀數
#define OFFSET_BENT_ADC 1737 // Flex Sensor 彎曲時（90度）的 ADC 讀數

// ===== 移動平均濾波配置 =====
#define FILTER_SIZE 5 // 移動平均點數

// ===== 大腿參數 =====
#define THIGH_LENGTH 45.0 // 大腿長度 (公分)

// ===== 感測器樣本結構 =====
struct SensorSample
{
  uint32_t timestamp;
  float angle;
  float deltaY;
  float deltaZ;
  int rawAdc;
  bool stable;
};

// ===== 環形緩衝區 =====
SensorSample ringBuffer[RING_BUFFER_SIZE];
volatile uint16_t bufferHead = 0;
volatile uint16_t bufferTail = 0;
volatile uint16_t bufferCount = 0;

// ===== FreeRTOS 同步物件 =====
SemaphoreHandle_t bufferMutex = NULL;
TaskHandle_t samplingTaskHandle = NULL;
TaskHandle_t transmitTaskHandle = NULL;

// ===== 統計變數 =====
volatile uint32_t totalSampleCount = 0;
volatile uint32_t totalSentCount = 0;
volatile uint32_t droppedCount = 0;

// ===== Wi-Fi 設定 =====
const char *WIFI_SSID = "Bt";
const char *WIFI_PASSWORD = "bt_980904";

// ===== MQTT 設定 =====
const char *MQTT_SERVER = "mqtt.singularinnovation-ai.com";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "singular";
const char *MQTT_PASSWORD = "Singular#1234";
const char *MQTT_TOPIC = "knee-drive/data";

// ===== Wi-Fi 和 MQTT 客戶端 =====
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ===== 固定校正值（使用 Offset 常數） =====
int calibFlatAdc = OFFSET_FLAT_ADC;
int calibBentAdc = OFFSET_BENT_ADC;

// ===== 角度追蹤變數 =====
float currentAngle = 0.0;
unsigned long lastTime = 0;
unsigned long startTime = 0;

float kneeY = 0.0;
float kneeZ = 0.0;
float initialKneeY = 0.0;
float initialKneeZ = 0.0;

float prevAngle = 0.0;
int stableCount = 0;
bool isStable = false;

int filterBuffer[FILTER_SIZE];
int filterIndex = 0;
bool filterReady = false;

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
      droppedCount++;
      xSemaphoreGive(bufferMutex);
      return false;
    }
  }
  return true;
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

// ===== Flex Sensor 讀取函數 =====
int readFilteredAdc()
{
  int rawValue = analogRead(FLEX_SENSOR_PIN);

  filterBuffer[filterIndex] = rawValue;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;

  if (!filterReady && filterIndex == 0)
  {
    filterReady = true;
  }

  if (filterReady)
  {
    long sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
    {
      sum += filterBuffer[i];
    }
    return sum / FILTER_SIZE;
  }

  return rawValue;
}

float adcToAngle(int rawValue)
{
  if (rawValue > calibFlatAdc)
    rawValue = calibFlatAdc;
  if (rawValue < calibBentAdc)
    rawValue = calibBentAdc;

  float angle = (float)(calibFlatAdc - rawValue) / (calibFlatAdc - calibBentAdc) * 90.0;

  if (angle < 0)
    angle = 0;
  if (angle > 90)
    angle = 90;

  return angle;
}

// ===== Wi-Fi 連線函數 =====
void connectWiFi()
{
  Serial.println("\n========================================");
  Serial.println("Connecting to Wi-Fi...");
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
    Serial.println("\nWi-Fi Connected!");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  }
  else
  {
    Serial.println("\nWi-Fi Connection Failed!");
  }
  Serial.println("========================================\n");
}

// ===== MQTT 重連函數 =====
void reconnectMQTT()
{
  if (!mqttClient.connected())
  {
    Serial.print("Connecting to MQTT...");

    String clientId = "ESP32-FlexSensor-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD))
    {
      Serial.println(" Connected!");
      Serial.printf("Topic: %s\n", MQTT_TOPIC);
    }
    else
    {
      Serial.printf(" Failed, state: %d\n", mqttClient.state());
    }
  }
}

// ===== FreeRTOS 任務函數宣告 =====
void samplingTask(void *parameter);
void transmitTask(void *parameter);

// ===== FreeRTOS 取樣任務 =====
void samplingTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_INTERVAL_MS);

  Serial.println("[SamplingTask] Started, 50Hz sampling");
  Serial.println("Using fixed offset calibration values");
  Serial.printf("  FLAT ADC: %d (0 deg)\n", calibFlatAdc);
  Serial.printf("  BENT ADC: %d (90 deg)\n", calibBentAdc);
  Serial.printf("  ADC Range: %d\n", calibFlatAdc - calibBentAdc);

  // 設定初始位置
  initialKneeY = 0;
  initialKneeZ = -THIGH_LENGTH;
  startTime = millis();

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    unsigned long currentTime = millis();
    int rawAdc = readFilteredAdc();

    // ===== 測量模式（使用固定 offset） =====
    currentAngle = adcToAngle(rawAdc);

    float angleRad = currentAngle * PI / 180.0;
    kneeY = THIGH_LENGTH * sin(angleRad);
    kneeZ = -THIGH_LENGTH * cos(angleRad);

    float deltaY = kneeY - initialKneeY;
    float deltaZ = kneeZ - initialKneeZ;

    float angleDiff = abs(currentAngle - prevAngle);
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
    prevAngle = currentAngle;

    // ===== 儲存到 Ring Buffer =====
    SensorSample sample;
    sample.timestamp = currentTime;
    sample.angle = currentAngle;
    sample.deltaY = deltaY;
    sample.deltaZ = deltaZ;
    sample.rawAdc = rawAdc;
    sample.stable = isStable;

    if (!pushSample(&sample))
    {
      static unsigned long lastWarning = 0;
      if (currentTime - lastWarning > 5000)
      {
        Serial.printf("WARNING: Ring buffer full (%d/%d), dropped: %lu\n",
                      bufferCount, RING_BUFFER_SIZE, droppedCount);
        lastWarning = currentTime;
      }
    }
  }
}

// ===== FreeRTOS 傳輸任務 =====
void transmitTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SEND_INTERVAL_MS);

  Serial.println("[TransmitTask] Started, 2Hz batch transmit");

  SensorSample batchBuffer[BATCH_SIZE];

  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("Wi-Fi disconnected, reconnecting...");
      connectWiFi();
      continue;
    }

    if (!mqttClient.connected())
    {
      reconnectMQTT();
      if (!mqttClient.connected())
      {
        continue;
      }
    }

    mqttClient.loop();

    uint16_t count = popSamples(batchBuffer, BATCH_SIZE);

    if (count == 0)
    {
      continue;
    }

    // 建立批次 JSON（相容 IMU 版本格式）
    JsonDocument doc;
    doc["type"] = "batch";
    doc["n"] = count;

    JsonArray dataArray = doc["d"].to<JsonArray>();

    for (uint16_t i = 0; i < count; i++)
    {
      JsonObject item = dataArray.add<JsonObject>();
      item["t"] = batchBuffer[i].timestamp / 1000.0;
      item["a"] = round(batchBuffer[i].angle * 10) / 10.0;
      item["s"] = batchBuffer[i].stable ? 1 : 0;
      item["dy"] = round(batchBuffer[i].deltaY * 10) / 10.0;
      item["dz"] = round(batchBuffer[i].deltaZ * 10) / 10.0;
      // 相容模式：加速度和陀螺儀填 0
      item["ax"] = 0;
      item["ay"] = 0;
      item["az"] = 0;
      item["gx"] = 0;
      item["gy"] = 0;
      item["gz"] = 0;
    }

    char jsonBuffer[4096];
    size_t jsonSize = serializeJson(doc, jsonBuffer);

    bool success = mqttClient.publish(MQTT_TOPIC, jsonBuffer, false);

    if (success)
    {
      totalSentCount += count;
      Serial.printf("Batch sent: %d samples (%d bytes), buffer: %d\n",
                    count, jsonSize, getBufferCount());
    }
    else
    {
      Serial.printf("MQTT publish failed (state: %d)\n", mqttClient.state());
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  Flex Sensor Knee Analysis System");
  Serial.println("  FreeRTOS + MQTT Batch Transmit");
  Serial.println("========================================");
  Serial.println();

  // 初始化 LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  Serial.printf("LED initialized (GPIO%d)\n", LED_PIN);

  // 初始化 ADC
  analogReadResolution(12);
  Serial.printf("ADC initialized (GPIO%d, 12-bit)\n", FLEX_SENSOR_PIN);

  // 初始化濾波器
  for (int i = 0; i < FILTER_SIZE; i++)
  {
    filterBuffer[i] = analogRead(FLEX_SENSOR_PIN);
  }
  Serial.printf("Moving average filter initialized (%d points)\n", FILTER_SIZE);

  // 測試讀取
  int testAdc = analogRead(FLEX_SENSOR_PIN);
  float testVoltage = testAdc * 3.3 / 4095.0;
  Serial.printf("Flex Sensor test: ADC=%d, Voltage=%.2fV\n", testAdc, testVoltage);

  Serial.println();
  Serial.println("=== Thigh Angle Tracking System ===");
  Serial.printf("Thigh length = %.1f cm\n", THIGH_LENGTH);
  Serial.println("  0 deg = Standing (vertical)");
  Serial.println(" 90 deg = Knee raised (horizontal)");

  // 連接 Wi-Fi
  connectWiFi();

  // 設定 MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(4096);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(30);

  Serial.println("\nConnecting to MQTT...");

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

        JsonDocument testDoc;
        testDoc["type"] = "connection_test";
        testDoc["status"] = "connected";
        testDoc["device"] = "ESP32-FlexSensor";
        testDoc["sensor"] = "flex_sensor";

        char testBuffer[128];
        serializeJson(testDoc, testBuffer);

        if (mqttClient.publish(MQTT_TOPIC, testBuffer, false))
        {
          Serial.println("MQTT connected! Test message sent.");
        }
      }
    }

    if (!mqttConnected)
    {
      mqttAttempts++;
      Serial.printf("  Attempt %d/5...\n", mqttAttempts);
      delay(2000);
    }
  }

  if (!mqttConnected)
  {
    Serial.println("MQTT connection failed, continuing in 5 seconds...");
    delay(5000);
  }
  else
  {
    Serial.println("Run mqtt_receiver.py on PC to receive data");
    delay(2000);
  }

  // 固定 Offset 校正說明
  Serial.println("\n========================================");
  Serial.println("  Fixed Offset Calibration Mode");
  Serial.println("========================================");
  Serial.printf("  FLAT ADC: %d (0 deg)\n", OFFSET_FLAT_ADC);
  Serial.printf("  BENT ADC: %d (90 deg)\n", OFFSET_BENT_ADC);
  Serial.printf("  ADC Range: %d\n", OFFSET_FLAT_ADC - OFFSET_BENT_ADC);
  Serial.println("----------------------------------------");
  Serial.println("  To recalibrate: update OFFSET_FLAT_ADC");
  Serial.println("  and OFFSET_BENT_ADC in source code");
  Serial.println("========================================\n");

  lastTime = millis();

  // 初始化 FreeRTOS
  bufferMutex = xSemaphoreCreateMutex();
  if (bufferMutex == NULL)
  {
    Serial.println("Mutex creation failed!");
    while (1)
      delay(1000);
  }
  Serial.println("FreeRTOS Mutex initialized");

  Serial.println("\n========================================");
  Serial.println("  FreeRTOS Dual-Task Architecture");
  Serial.println("========================================");
  Serial.printf("  Sampling: %d Hz (every %d ms)\n", 1000 / SAMPLE_INTERVAL_MS, SAMPLE_INTERVAL_MS);
  Serial.printf("  Transmit: %d Hz (every %d ms)\n", 1000 / SEND_INTERVAL_MS, SEND_INTERVAL_MS);
  Serial.printf("  Batch size: %d samples\n", BATCH_SIZE);
  Serial.printf("  Buffer: %d samples (%.1f sec)\n", RING_BUFFER_SIZE, (float)RING_BUFFER_SIZE * SAMPLE_INTERVAL_MS / 1000);
  Serial.println("========================================\n");

  // 建立取樣任務
  BaseType_t result = xTaskCreatePinnedToCore(
      samplingTask,
      "SamplingTask",
      SAMPLING_STACK_SIZE,
      NULL,
      SAMPLING_PRIORITY,
      &samplingTaskHandle,
      0);

  if (result == pdPASS)
  {
    Serial.println("Sampling task created (50Hz, priority 3)");
  }
  else
  {
    Serial.println("Sampling task creation failed!");
  }

  // 建立傳輸任務
  result = xTaskCreatePinnedToCore(
      transmitTask,
      "TransmitTask",
      TRANSMIT_STACK_SIZE,
      NULL,
      TRANSMIT_PRIORITY,
      &transmitTaskHandle,
      0);

  if (result == pdPASS)
  {
    Serial.println("Transmit task created (2Hz, priority 1)");
  }
  else
  {
    Serial.println("Transmit task creation failed!");
  }

  Serial.println("\nRunning...\n");
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
