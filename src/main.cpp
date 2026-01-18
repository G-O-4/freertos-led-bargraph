#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "DHT.h"

#define HIGH_PRIORETY 2
#define MED_PRIORETY 1
#define LOW_PRIORETY 0

#define LED_DELAY 1000
#define SENSOR_DELAY 500
#define POT_DELAY 100

#define STEP_SIZE 3

// Helper enums for modes, they help the readablity of the program
enum PowerMode {normalMode, lowEnergyMode};
enum SensorMode {temperatureMode, humidityMode};
enum ScaleMode {thresholdMode, absoluteMode};

// Pins
const int ledPins[5] = {33, 25, 26, 27, 14};
const int potPin = 34;
const int sensorPin = 23;

// Modes
PowerMode powerMode = normalMode;
SensorMode sensorMode = temperatureMode; 
ScaleMode scaleMode = absoluteMode;

// Global Vars
QueueHandle_t sensorQueue;
QueueHandle_t potQueue;

// Functions
void vSensorRead(void *pvParameters);
void vPotRead(void *pvParameters);
void vLedView(void *pvParamters);
DHT dht(sensorPin, DHT22);

void setup() {
  // Putting leds to output
  for (int led: ledPins){
    pinMode(led, OUTPUT);
  }

  // Setting Sensor and Pot
  pinMode(potPin, INPUT);
  pinMode(sensorPin, INPUT);

  // Init Serial
  Serial.begin(115200);

  // Init dht
  dht.begin();

  // Queues
  sensorQueue = xQueueCreate(1, sizeof(float));
  potQueue = xQueueCreate(1, sizeof(int));

  // Create Tasks
  xTaskCreate(vSensorRead, "Sensor", 2048, nullptr, MED_PRIORETY, nullptr);
  xTaskCreate(vPotRead, "Potentiometer", 2048, nullptr, LOW_PRIORETY, nullptr);
  xTaskCreate(vLedView, "Led Output", 2048, nullptr, HIGH_PRIORETY, nullptr);
}

void loop(){
  // Empty
} 

void vSensorRead(void *pvParameters){
  float t, h;

  while (true){
    switch (sensorMode) {
    case temperatureMode:
      t = dht.readTemperature();
      xQueueOverwrite(sensorQueue, &t);
      break;
    
    case humidityMode:
      h = dht.readHumidity();
      xQueueOverwrite(sensorQueue, &h);
      break;

    default:
      break;
    }
  vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY));
  }
}


void vPotRead(void *pvParameters){
  int potVal;

  while (true){
    potVal = analogRead(potPin);
    potVal = (sensorMode == temperatureMode)? map(potVal, 0, 4095, 19, 26) : map(potVal, 0, 4095, 35, 66);
    xQueueOverwrite(potQueue, &potVal);
    vTaskDelay(pdMS_TO_TICKS(POT_DELAY));
  }
}


void vLedView(void *pvParameters){
  float recievedSensor;
  int recievedPot, iteratorVal;
  int factorHT = (sensorMode == temperatureMode)? 1 : 2;
  int comparisonMap[4] = {-5*factorHT, -factorHT, factorHT, 5*factorHT};

  while (true){
    xQueueReceive(potQueue, &recievedPot, portMAX_DELAY);
    xQueueReceive(sensorQueue, &recievedSensor, portMAX_DELAY);
    iteratorVal = recievedPot - 2 * STEP_SIZE;    

    Serial.println(recievedPot);
    Serial.println(recievedSensor);
    Serial.println("---------------------");

    switch (scaleMode) {
    case  absoluteMode:
      for (int led: ledPins){
        if (iteratorVal <= recievedSensor) {
          digitalWrite(led, HIGH);
          iteratorVal += STEP_SIZE;
        }
        else
          break;
      }
      break;
    
    case thresholdMode:
      for (int i=0; i<4; i++){
        digitalWrite(ledPins[i], HIGH);
        if (comparisonMap[i] + recievedPot > recievedSensor)
          break;
      }

    default:
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(LED_DELAY));
  }
}