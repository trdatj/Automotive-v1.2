#include <SPI.h>
#include <mcp2515.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

const int PIN_LEFT = 26;
const int PIN_RIGHT = 27;
const int PIN_COS = 25;
const int PIN_PHA = 33;
const int PIN_LED_NODE1 = 17;
const int PIN_LED_NODE2 = 16;
const int PIN_LED_NODE3 = 4;
const int PIN_LED_NODE4 = 12;
const int PIN_LED_NODE5 = 13;

SemaphoreHandle_t dataMutex;

struct can_frame canMsg;
struct can_frame txMsg;
MCP2515 CAN(5);

struct SystemState {
  bool left = false;
  bool right = false;
  bool cos = false;
  bool pha = false;
  bool hazard = false;

  int targetSpeed = 0;
  float realSpeed = 0.0;
  int rpm = 0;
  int distance = 0;
  int gapLevel = 3;
  int cruiseState = 0;
  int gear = 0;

  unsigned long lastRxTime1 = 0;
  unsigned long lastRxTime2 = 0;
  unsigned long lastRxTime3 = 0;
  unsigned long lastRxTime4 = 0;
  unsigned long lastRxTime5 = 0;
} sysState;

bool blinkState = LOW;
const int BLINK_INTERVAL = 300;
const int BLINK_SPEED_NODE = 300;

#define BIT_PHA 0
#define BIT_COS 1
#define BIT_LEFT 2
#define BIT_RIGHT 3
#define BIT_HAZARD 4
#define BIT_CRUISE_MAIN 0

// ---- task 1: Đọc CAN ----
void TaskCANReceive(void *parameter) {
  struct can_frame canMsg;
  uint8_t lastByte1 = 0;

  while (1) {
    if (CAN.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);

      if (canMsg.can_id == 0x01) {
        sysState.lastRxTime5 = millis();

        uint8_t currentByte1 = canMsg.data[1];

        if (((currentByte1 >> BIT_PHA) & 1) && !((lastByte1 >> BIT_PHA) & 1)) sysState.pha = !sysState.pha;
        if (((currentByte1 >> BIT_COS) & 1) && !((lastByte1 >> BIT_COS) & 1)) sysState.cos = !sysState.cos;

        // Logic Xi nhan / Hazard
        if (((currentByte1 >> BIT_HAZARD) & 1) && !((lastByte1 >> BIT_HAZARD) & 1)) {
          sysState.hazard = !sysState.hazard;
          if (sysState.hazard) {
            sysState.left = false;
            sysState.right = false;
          }
        }

        if (!sysState.hazard) {  // Chỉ chỉnh xi nhan khi không bật Hazard
          if (((currentByte1 >> BIT_LEFT) & 1) && !((lastByte1 >> BIT_LEFT) & 1)) {
            sysState.left = !sysState.left;
            if (sysState.left) sysState.right = false;
          }
          if (((currentByte1 >> BIT_RIGHT) & 1) && !((lastByte1 >> BIT_RIGHT) & 1)) {
            sysState.right = !sysState.right;
            if (sysState.right) sysState.left = false;
          }
        }

        lastByte1 = currentByte1;

        // Đọc Distance
        uint8_t distLow = canMsg.data[4];
        uint8_t distHigh = canMsg.data[5];
        sysState.distance = (distHigh << 8) | distLow;

        // Đọc ADAS
        sysState.gapLevel = canMsg.data[6];
        sysState.targetSpeed = canMsg.data[7];
        bool currentBtn = (canMsg.data[0] >> BIT_CRUISE_MAIN) & 1;

        static bool lastBtn = false;

        if (currentBtn && !lastBtn) {
          if (sysState.cruiseState == 0) {
            sysState.cruiseState = 1;  // Đang Tắt -> Chuyển sang READY
          } else {
            sysState.cruiseState = 0;  // Đang Bật -> Chuyển sang TẮT
          }
        }
        lastBtn = currentBtn;

        if (sysState.cruiseState != 0) {
          if (sysState.targetSpeed > 0) {
            sysState.cruiseState = 2;  // Có tốc độ -> ACTIVE (Xanh)
          } else {
            if (sysState.distance > 0 && sysState.distance < 50) {
            } else {
              if (sysState.cruiseState == 2) {
                sysState.cruiseState = 1;
              }
            }
          }
        }

        uint16_t joyVal = (canMsg.data[3] << 8) | canMsg.data[2];
        static unsigned long p_timer = 0;

        if (sysState.cruiseState == 2) {
          sysState.gear = 3;
          p_timer = millis();
        }

        else {
          if (joyVal < 2800) {
            sysState.gear = 3;  // tiến
            p_timer = millis();
          } else if (joyVal > 2980) {
            sysState.gear = 1;  // lùi
            p_timer = millis();
          } else {
            if (millis() - p_timer > 3000) {
              sysState.gear = 0;  // đỗ
            }
          }
        }
      }

      if (canMsg.can_id == 0x02) {
        sysState.lastRxTime4 = millis();

        uint32_t raw_rpm = ((uint32_t)canMsg.data[0] << 24) | ((uint32_t)canMsg.data[1] << 16) | ((uint32_t)canMsg.data[2] << 8) | canMsg.data[3];
        sysState.rpm = (int)raw_rpm;

        int raw_speed_int = (canMsg.data[4] << 8) | canMsg.data[5];
        sysState.realSpeed = raw_speed_int / 100.0;
      }

      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ---- task điều khiển led
void TaskControl(void *parameter) {
  unsigned long lastBlinkTime = 0;

  while (1) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastBlinkTime >= BLINK_INTERVAL) {
      lastBlinkTime = currentMillis;
      blinkState = !blinkState;
    }
    xSemaphoreTake(dataMutex, portMAX_DELAY);

    bool sHazard = sysState.hazard;
    bool sLeft = sysState.left;
    bool sRight = sysState.right;
    bool sCos = sysState.cos;
    bool sPha = sysState.pha;

    unsigned long t1 = sysState.lastRxTime1;
    unsigned long t2 = sysState.lastRxTime2;
    unsigned long t3 = sysState.lastRxTime3;
    unsigned long t4 = sysState.lastRxTime4;
    unsigned long t5 = sysState.lastRxTime5;

    xSemaphoreGive(dataMutex);

    digitalWrite(PIN_COS, sCos ? HIGH : LOW);
    digitalWrite(PIN_PHA, sPha ? HIGH : LOW);

    if (sHazard) {
      digitalWrite(PIN_LEFT, blinkState);
      digitalWrite(PIN_RIGHT, blinkState);
    } else {
      digitalWrite(PIN_LEFT, sLeft ? blinkState : LOW);
      digitalWrite(PIN_RIGHT, sRight ? blinkState : LOW);
    }

    auto updateNodeLed = [&](int pin, unsigned long lastRx) {
      if (currentMillis - lastRx < 1000) {
        bool ledOn = (currentMillis / BLINK_SPEED_NODE) % 2 == 0;
        digitalWrite(pin, ledOn ? HIGH : LOW);
      } else {
        digitalWrite(pin, LOW);
      }
    };

    updateNodeLed(PIN_LED_NODE1, t1);
    updateNodeLed(PIN_LED_NODE2, t2);
    updateNodeLed(PIN_LED_NODE3, t3);
    updateNodeLed(PIN_LED_NODE4, t4);
    updateNodeLed(PIN_LED_NODE5, t5);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ---- debug ----
void TaskSerial(void *parameter) {
  char txBuffer[128];
  static uint8_t currentRoadLimit = 120;
  while (1) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int dist = sysState.distance;
    int gap = sysState.gapLevel;
    int cState = sysState.cruiseState;
    bool p = sysState.pha;
    bool c = sysState.cos;
    bool xnt = sysState.left;
    bool xnp = sysState.right;
    bool hazard = sysState.hazard;
    float speed = sysState.realSpeed;
    int rpm = sysState.rpm;
    int tSpeed = sysState.targetSpeed;
    xSemaphoreGive(dataMutex);

    if (hazard) {
      xnt = true;
      xnp = true;
    }

    // Serial.printf("D:%d cm | Gap:%d | Cruise:%d | Pha:%d | Cos:%d | XNT:%d | XNP:%d | HAZARD:%d | SPEED:%.2f\n",
    //               dist, gap, cState, p, c, xnt, xnp, hazard, speed, rpm);

    Serial.printf("DATA:%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", speed, rpm, dist, p, c, xnt, xnp, cState, tSpeed, gap);
    Serial.printf("GEAR:%d\n", sysState.gear);

    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();

      bool updateCAN = false;

      if (input.startsWith("CMD_LIMIT_")) {
        if (input.indexOf("40") != -1) currentRoadLimit = 40;
        else if (input.indexOf("50") != -1) currentRoadLimit = 50;
        else if (input.indexOf("60") != -1) currentRoadLimit = 60;
        else if (input.indexOf("80") != -1) currentRoadLimit = 80;
      }

      else if (input == "STOP") {
        currentRoadLimit = 0;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        sysState.cruiseState = 0;
        sysState.targetSpeed = 0;
        xSemaphoreGive(dataMutex);
      }

      else if (input == "CMD_CONSTRUCT") {
        currentRoadLimit = 0;
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        sysState.cruiseState = 0;
        sysState.targetSpeed = 0;

        xSemaphoreGive(dataMutex);
      }
    }
    txMsg.can_id = 0x03;
    txMsg.can_dlc = 8;
    memset(txMsg.data, 0, 8);

    txMsg.data[6] = currentRoadLimit;

    CAN.sendMessage(&txMsg);


    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SPI.begin();
  CAN.reset();
  CAN.setBitrate(CAN_250KBPS, MCP_8MHZ);
  CAN.setNormalMode();

  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  pinMode(PIN_COS, OUTPUT);
  pinMode(PIN_PHA, OUTPUT);
  pinMode(PIN_LED_NODE1, OUTPUT);
  pinMode(PIN_LED_NODE2, OUTPUT);
  pinMode(PIN_LED_NODE3, OUTPUT);
  pinMode(PIN_LED_NODE4, OUTPUT);
  pinMode(PIN_LED_NODE5, OUTPUT);

  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LEFT, HIGH);
    digitalWrite(PIN_RIGHT, HIGH);
    delay(300);
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);
    delay(300);
  }

  dataMutex = xSemaphoreCreateMutex();

  Serial.println("ESP32 FreeRTOS System Starting...");

  xTaskCreatePinnedToCore(TaskCANReceive, "CAN_Rx", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "Control", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskSerial, "Serial_Tx", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelete(NULL);
}
