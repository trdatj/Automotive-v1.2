#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 CAN(5);

// --- KHAI BÁO CHÂN LED ---
const int PIN_LEFT = 26;
const int PIN_RIGHT = 27;
const int PIN_COS = 25;
const int PIN_PHA = 33;

const int PIN_LED_NODE1 = 17;
const int PIN_LED_NODE2 = 16;
const int PIN_LED_NODE3 = 4;
const int PIN_LED_NODE4 = 12;
const int PIN_LED_NODE5 = 13;

unsigned long timerLed1 = 0;
unsigned long timerLed2 = 0;
unsigned long timerLed3 = 0;
unsigned long timerLed4 = 0;
unsigned long timerLed5 = 0;

const int BLINK_SPEED = 300;
unsigned long lastRxTime1 = 0;
unsigned long lastRxTime2 = 0;
unsigned long lastRxTime3 = 0;
unsigned long lastRxTime4 = 0;
unsigned long lastRxTime5 = 0;

// --- BIẾN TRẠNG THÁI (ON/OFF) ---
bool stateLeft = false;
bool stateRight = false;
bool stateCos = false;
bool statePha = false;
bool stateHazard = false;

unsigned long lastBlinkTime = 0;
bool blinkState = LOW;
const int BLINK_INTERVAL = 300;

// --- khai báo macro bit ---
#define BIT_PHA 0
#define BIT_COS 1
#define BIT_LEFT 2
#define BIT_RIGHT 3
#define BIT_HAZARD 4

// --- BIẾN LƯU TRẠNG THÁI NÚT LẦN TRƯỚC ---
uint8_t lastByte1 = 0;
uint8_t lastByte0 = 0;

unsigned long lastLimitPressTime = 0;  // Biến lưu thời gian lần bấm cuối
bool isLimitActive = false;            // Biến lưu trạng thái bật/tắt

// --- ADAS buttons
enum CruiseState {
  CRUISE_OFF,    // Tắt hẳn
  CRUISE_READY,  // Bật màn hình, chờ SET (Màu Trắng/Vàng)
  CRUISE_ACTIVE  // Đang chạy tự động (Màu Xanh lá)
};

CruiseState currentCruiseState = CRUISE_OFF;
int targetSpeed = 0;
int realSpeed = 0;  // Nhận từ ID 0x02
int distance = 0;   // Nhận từ ID 0x01
int gapLevel = 3;   // Nhận từ ID 0x01
int cruiseState = 0;

#define BIT_CRUISE_MAIN 0
#define BIT_SET 1
#define BIT_RES 2
#define BIT_GAP 3
#define BIT_LIMIT 4

int receivedDistance = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  CAN.reset();
  CAN.setBitrate(CAN_250KBPS, MCP_8MHZ);
  CAN.setNormalMode();

  pinMode(PIN_LEFT, OUTPUT);
  pinMode(PIN_RIGHT, OUTPUT);
  pinMode(PIN_COS, OUTPUT);
  pinMode(PIN_PHA, OUTPUT);

  digitalWrite(PIN_LEFT, LOW);
  digitalWrite(PIN_RIGHT, LOW);
  digitalWrite(PIN_COS, LOW);
  digitalWrite(PIN_PHA, LOW);

  pinMode(PIN_LED_NODE1, OUTPUT);
  pinMode(PIN_LED_NODE2, OUTPUT);
  pinMode(PIN_LED_NODE3, OUTPUT);
  pinMode(PIN_LED_NODE4, OUTPUT);
  pinMode(PIN_LED_NODE5, OUTPUT);

  digitalWrite(PIN_LED_NODE1, LOW);
  digitalWrite(PIN_LED_NODE2, LOW);
  digitalWrite(PIN_LED_NODE3, LOW);
  digitalWrite(PIN_LED_NODE4, LOW);
  digitalWrite(PIN_LED_NODE5, LOW);

  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LEFT, HIGH);
    digitalWrite(PIN_RIGHT, HIGH);
    delay(300);
    digitalWrite(PIN_LEFT, LOW);
    digitalWrite(PIN_RIGHT, LOW);
    delay(300);
  }

  Serial.println("ESP32 System Ready...");
}

void loop() {
  if (millis() - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = millis();
    blinkState = !blinkState;
  }

  // if (millis() - timerLed1 > 50) digitalWrite(PIN_LED_NODE1, LOW);
  // if (millis() - timerLed2 > 50) digitalWrite(PIN_LED_NODE2, LOW);
  // if (millis() - timerLed3 > 50) digitalWrite(PIN_LED_NODE3, LOW);

  unsigned long currentMillis = millis();

  if (currentMillis - lastRxTime1 < 1000) {
    bool ledState = (currentMillis / BLINK_SPEED) % 2 == 0;
    digitalWrite(PIN_LED_NODE1, ledState ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_NODE1, LOW);
  }

  // LED NODE 2
  if (currentMillis - lastRxTime2 < 1000) {
    bool ledState = (currentMillis / BLINK_SPEED) % 2 == 0;
    digitalWrite(PIN_LED_NODE2, ledState ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_NODE2, LOW);
  }

  if (currentMillis - lastRxTime3 < 1000) {
    bool ledState = (currentMillis / BLINK_SPEED) % 2 == 0;
    digitalWrite(PIN_LED_NODE3, ledState ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_NODE3, LOW);
  }

  if (currentMillis - lastRxTime4 < 1000) {
    bool ledState = (currentMillis / BLINK_SPEED) % 2 == 0;
    digitalWrite(PIN_LED_NODE4, ledState ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_NODE4, LOW);
  }

  if (currentMillis - lastRxTime5 < 1000) {
    bool ledState = (currentMillis / BLINK_SPEED) % 2 == 0;
    digitalWrite(PIN_LED_NODE5, ledState ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_NODE5, LOW);
  }

  if (CAN.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0x01) {
      lastRxTime5 = millis();

      uint8_t currentByte1 = canMsg.data[1];

      if (((currentByte1 >> BIT_PHA) & 1) && !((lastByte1 >> BIT_PHA) & 1)) {
        Serial.println(statePha ? "PHA:OFF" : "PHA:ON");
        statePha = !statePha;
      }

      if (((currentByte1 >> BIT_COS) & 1) && !((lastByte1 >> BIT_COS) & 1)) {
        Serial.println(stateCos ? "COS:OFF" : "COS:ON");
        stateCos = !stateCos;
      }

      if (((currentByte1 >> BIT_LEFT) & 1) && !((lastByte1 >> BIT_LEFT) & 1)) {
        Serial.println(stateLeft ? "LEFT:OFF" : "LEFT:ON");
        stateLeft = !stateLeft;
        if (stateLeft) stateRight = false;
      }

      if (((currentByte1 >> BIT_RIGHT) & 1) && !((lastByte1 >> BIT_RIGHT) & 1)) {
        Serial.println(stateRight ? "RIGHT:OFF" : "RIGHT:ON");
        stateRight = !stateRight;
        if (stateRight) stateLeft = false;
      }

      if (((currentByte1 >> BIT_HAZARD) & 1) && !((lastByte1 >> BIT_HAZARD) & 1)) {
        Serial.println(stateHazard ? "HAZARD:OFF" : "HAZARD:ON");
        stateHazard = !stateHazard;
        if (stateHazard) {
          stateLeft = false;
          stateRight = false;
        }
      }

      // Serial.print("Byte1: ");
      // Serial.println(currentByte1, BIN);

      lastByte1 = currentByte1;

      // -- distance ---
      uint8_t distLow = canMsg.data[4];   // Byte thấp
      uint8_t distHigh = canMsg.data[5];  // Byte cao
      receivedDistance = (distHigh << 8) | distLow;

      if (millis() - lastPrintTime > 200) {
        Serial.print(" | Distance: ");
        Serial.print(receivedDistance);
        Serial.println(" cm");

        lastPrintTime = millis();
      }

      // --- ADAS ---
      uint8_t currentByte0 = canMsg.data[0];

      gapLevel = canMsg.data[6];
      Serial.print("GAP CHANGED: Level ");
      Serial.print(gapLevel);

      targetSpeed = canMsg.data[7];

      // Suy luận trạng thái Cruise Control để hiển thị màu icon
      // Nếu Target Speed > 0 -> Chắc chắn đang ACTIVE (Xanh lá)
      // Nếu Target Speed == 0 nhưng nút Cruise Main (Bit 0) đang ON -> READY (Trắng/Vàng)
      // Còn lại -> OFF

      //Active cruise control
      bool isCruiseMainOn = (currentByte0 >> BIT_CRUISE_MAIN) & 1;  // Bit này STM32 gửi trạng thái switch, không phải nút nhấn nhả
      // Lưu ý: Nếu code STM32 gửi dạng nút nhấn nhả (pulse), ta cần logic latch ở đây.
      // Nhưng đơn giản nhất:
      if (targetSpeed > 0) {
        cruiseState = 2;  // ACTIVE
      } else if (isCruiseMainOn || (targetSpeed == 0 && cruiseState == 2)) {
        // Logic này hơi phụ thuộc code STM32 gửi gì ở Bit 0.
        // Giả sử bro toggle bit 0 bên STM32 khi cruise ready:
        cruiseState = 1;  // READY
      } else {
        cruiseState = 0;  // OFF
      }
      // Serial.print("Byte0: ");
      // Serial.println(currentByte0, BIN);

      lastByte0 = currentByte0;
    }

    if (canMsg.can_id == 0x02) {
      //if (canMsg.header.dlc >= 6) {
      lastRxTime4 = millis();

      // uint32_t encoder_freq = 0;
      // encoder_freq |= (uint32_t)canMsg.data[0] << 24;
      // encoder_freq |= (uint32_t)canMsg.data[1] << 16;
      // encoder_freq |= (uint32_t)canMsg.data[2] << 8;
      // encoder_freq |= (uint32_t)canMsg.data[3];

      // uint16_t speed_raw = 0;
      // speed_raw |= (uint16_t)canMsg.data[4] << 8;
      // speed_raw |= (uint16_t)canMsg.data[5];

      // float speed_kmh = speed_raw / 100.0;
      // // Serial.print("SPEED:");
      // // Serial.println(speed_kmh);
      // //}

      // int pot_val = map((long)speed_kmh, 0, 200, 0, 4095);

      // Serial.print("POT_VAL:");
      // Serial.println(pot_val);
      //}
    }
  }
  if (stateHazard) {
    //digitalWrite(PIN_HAZARD, blinkState);
    digitalWrite(PIN_LEFT, blinkState);
    digitalWrite(PIN_RIGHT, blinkState);
  } else {
    //digitalWrite(PIN_HAZARD, LOW);

    digitalWrite(PIN_LEFT, stateLeft ? blinkState : LOW);
    digitalWrite(PIN_RIGHT, stateRight ? blinkState : LOW);
  }

  digitalWrite(PIN_COS, stateCos ? HIGH : LOW);
  digitalWrite(PIN_PHA, statePha ? HIGH : LOW);
}