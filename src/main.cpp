#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "DHT.h"
#include "esp_sleep.h"

// ---------- Your constants ----------
#define LED_DELAY    1000
#define SENSOR_DELAY 2000   // DHT22 needs ~2s between reads to reduce NaN
#define POT_DELAY    100
#define STEP_SIZE    3

// ---------- Modes ----------
enum PowerMode  { normalMode, lowEnergyMode };
enum SensorMode { temperatureMode, humidityMode };
enum ScaleMode  { thresholdMode, absoluteMode };

// ---------- Your pins (UNCHANGED) ----------
const int ledPins[5] = {33, 25, 26, 27, 14};
const int potPin = 34;
const int sensorPin = 23;

// ---------- Buttons pins (AS YOU REQUESTED) ----------
const int btnSensorPin = 0;  // TEMP/HUM
const int btnScalePin  = 4;  // THR/ABS
const int btnPowerPin  = 15; // NORMAL/LOW ENERGY

// ---------- Globals ----------
PowerMode  powerMode  = normalMode;
SensorMode sensorMode = temperatureMode;
ScaleMode  scaleMode  = absoluteMode;

DHT dht(sensorPin, DHT22);

// ---- Send BOTH readings so serial can show temp + hum always ----
struct SensorData {
  float tempC;
  float humPct;
  bool  tempOK;
  bool  humOK;
};

QueueHandle_t sensorQueue;
QueueHandle_t potQueue; // mapped threshold only (int), no raw ADC now

// ---- Button notification bits ----
static const uint32_t BTN_SENSOR_BIT = (1u << 0);
static const uint32_t BTN_SCALE_BIT  = (1u << 1);
static const uint32_t BTN_POWER_BIT  = (1u << 2);

TaskHandle_t buttonTaskHandle = nullptr;

// ---------- Helpers ----------
void setLedBar(uint8_t level) { // 0..5
  if (level > 5) level = 5;
  for (int i = 0; i < 5; i++) {
    digitalWrite(ledPins[i], (i < level) ? HIGH : LOW);
  }
}

void enterLightSleep10s() {
  esp_sleep_enable_timer_wakeup(10ULL * 1000000ULL);
  Serial.flush();
  esp_light_sleep_start();
}

static void printStatus(const SensorData &sd, int threshold, float usedVal) {
  Serial.println("========================================");
  Serial.println(" Smart Environment Monitor (ESP32)");
  Serial.println("----------------------------------------");

  Serial.print("Temperature (C): ");
  if (sd.tempOK) Serial.println(sd.tempC, 1);
  else Serial.println("N/A");

  Serial.print("Humidity    (%): ");
  if (sd.humOK) Serial.println(sd.humPct, 1);
  else Serial.println("N/A");

  Serial.print("Pot threshold: ");
  Serial.println(threshold);

  Serial.print("BTN1 Sensor mode: ");
  Serial.println((sensorMode == temperatureMode) ? "TEMPERATURE" : "HUMIDITY");

  Serial.print("BTN2 Scale mode : ");
  Serial.println((scaleMode == absoluteMode) ? "ABSOLUTE" : "THRESHOLD");

  Serial.print("BTN3 Power mode : ");
  Serial.println((powerMode == normalMode) ? "NORMAL" : "LOW ENERGY");

  Serial.print("Value used for LEDs: ");
  if (!isnan(usedVal)) Serial.println(usedVal, 1);
  else Serial.println("N/A");

  Serial.println("========================================");
}

// ---------- ISRs ----------
void IRAM_ATTR isrSensor() {
  BaseType_t hp = pdFALSE;
  xTaskNotifyFromISR(buttonTaskHandle, BTN_SENSOR_BIT, eSetBits, &hp);
  if (hp) portYIELD_FROM_ISR();
}
void IRAM_ATTR isrScale() {
  BaseType_t hp = pdFALSE;
  xTaskNotifyFromISR(buttonTaskHandle, BTN_SCALE_BIT, eSetBits, &hp);
  if (hp) portYIELD_FROM_ISR();
}
void IRAM_ATTR isrPower() {
  BaseType_t hp = pdFALSE;
  xTaskNotifyFromISR(buttonTaskHandle, BTN_POWER_BIT, eSetBits, &hp);
  if (hp) portYIELD_FROM_ISR();
}

// ---------- Tasks ----------
void vSensorRead(void *pv) {
  static SensorData sd = {NAN, NAN, false, false};

  while (true) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    // NaN prevention: only update if valid
    if (!isnan(t)) { sd.tempC = t; sd.tempOK = true; }
    if (!isnan(h)) { sd.humPct = h; sd.humOK  = true; }

    xQueueOverwrite(sensorQueue, &sd);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY));
  }
}

void vPotRead(void *pv) {
  int thr;

  while (true) {
    int raw = analogRead(potPin);

    // Your exact ranges (UNCHANGED):
    thr = (sensorMode == temperatureMode)
            ? map(raw, 0, 4095, 19, 26)
            : map(raw, 0, 4095, 35, 66);

    xQueueOverwrite(potQueue, &thr);
    vTaskDelay(pdMS_TO_TICKS(POT_DELAY));
  }
}

void vLedView(void *pv) {
  SensorData sd;
  int threshold = 0;

  // Wait for first values (prevents startup junk)
  xQueueReceive(sensorQueue, &sd, portMAX_DELAY);
  xQueueReceive(potQueue, &threshold, portMAX_DELAY);

  while (true) {
    // Get latest values without removing them
    xQueuePeek(sensorQueue, &sd, 0);
    xQueuePeek(potQueue, &threshold, 0);

    // Choose which sensor value to use
    float sensorVal = (sensorMode == temperatureMode) ? sd.tempC : sd.humPct;
    bool  ok        = (sensorMode == temperatureMode) ? sd.tempOK : sd.humOK;

    // -------- LED update FIRST (prevents visible blink while printing) --------
    uint8_t ledLevel = 0;

    if (!ok || isnan(sensorVal)) {
      ledLevel = 0;
    } else {
      int factorHT = (sensorMode == temperatureMode) ? 1 : 2;

      if (scaleMode == absoluteMode) {
        float level = threshold - 2 * STEP_SIZE;
        ledLevel = 0;
        for (int i = 0; i < 5; i++) {
          if (sensorVal >= level) {
            ledLevel++;
            level += STEP_SIZE;
          } else break;
        }
      } else { // thresholdMode
        float thr = (float)threshold;
        float v = sensorVal;

        if (v < thr - 5 * factorHT)      ledLevel = 1;
        else if (v < thr - 1 * factorHT) ledLevel = 2;
        else if (v < thr + 1 * factorHT) ledLevel = 3;
        else if (v < thr + 5 * factorHT) ledLevel = 4;
        else                             ledLevel = 5;
      }
    }

    // Set LEDs in one consistent step (no clearing-then-printing blink)
    setLedBar(ledLevel);

    // -------- Serial print AFTER LED update --------
    printStatus(sd, threshold, (ok ? sensorVal : NAN));

    if (powerMode == lowEnergyMode) enterLightSleep10s();
    else vTaskDelay(pdMS_TO_TICKS(LED_DELAY));
  }
}

// Debounce + apply button actions
void vButtons(void *pv) {
  uint32_t bits;
  TickType_t lastSensor = 0, lastScale = 0, lastPower = 0;
  const TickType_t debounce = pdMS_TO_TICKS(50);

  while (true) {
    xTaskNotifyWait(0, 0xFFFFFFFF, &bits, portMAX_DELAY);
    TickType_t now = xTaskGetTickCount();

    if ((bits & BTN_SENSOR_BIT) && (now - lastSensor > debounce)) {
      lastSensor = now;
      sensorMode = (sensorMode == temperatureMode) ? humidityMode : temperatureMode;
    }
    if ((bits & BTN_SCALE_BIT) && (now - lastScale > debounce)) {
      lastScale = now;
      scaleMode = (scaleMode == absoluteMode) ? thresholdMode : absoluteMode;
    }
    if ((bits & BTN_POWER_BIT) && (now - lastPower > debounce)) {
      lastPower = now;
      powerMode = (powerMode == normalMode) ? lowEnergyMode : normalMode;
    }
  }
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 5; i++) pinMode(ledPins[i], OUTPUT);
  pinMode(potPin, INPUT);
  pinMode(sensorPin, INPUT);

  // Buttons: internal pullup, press = LOW
  pinMode(btnSensorPin, INPUT_PULLUP);
  pinMode(btnScalePin,  INPUT_PULLUP);
  pinMode(btnPowerPin,  INPUT_PULLUP);

  dht.begin();

  sensorQueue = xQueueCreate(1, sizeof(SensorData));
  potQueue    = xQueueCreate(1, sizeof(int));

  xTaskCreate(vSensorRead, "Sensor", 2048, nullptr, 2, nullptr);
  xTaskCreate(vPotRead,    "Pot",    2048, nullptr, 1, nullptr);
  xTaskCreate(vLedView,    "LED",    2048, nullptr, 2, nullptr);

  xTaskCreate(vButtons, "Buttons", 2048, nullptr, 3, &buttonTaskHandle);

  attachInterrupt(digitalPinToInterrupt(btnSensorPin), isrSensor, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnScalePin),  isrScale,  FALLING);
  attachInterrupt(digitalPinToInterrupt(btnPowerPin),  isrPower,  FALLING);
}

void loop() {}
