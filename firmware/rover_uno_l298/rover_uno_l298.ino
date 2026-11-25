// rover_uno_l298 - tak 로버 모터 펌웨어 (아두이노 우노 + HW-095(L298N) 2개)
//
// 2026-11-25 배선 (사용자 제공):
//   왼쪽 HW-095 : IN1=D11  IN2=D10  IN3=D9   IN4=D8
//   오른쪽 HW-095: IN1=D2   IN2=D3   IN3=D4   IN4=D5
//   ENA/ENB 는 점퍼가 꽂힌 상태(항상 켜짐)라고 가정하고, IN 핀에 PWM 을 건다.
//
// 모터 번호:  0 = 왼쪽 A (IN1/IN2)   1 = 왼쪽 B (IN3/IN4)
//             2 = 오른쪽 A (IN1/IN2) 3 = 오른쪽 B (IN3/IN4)
//
// 시리얼 115200 baud, 줄 단위('\n'):
//   부팅 직후           -> "L298 READY"
//   "M <left> <right>"  : 왼쪽/오른쪽 바퀴 쌍 속도, -255..255 (응답 없음)
//   "W a b c d"         : 모터 0..3 을 각각 -255..255 로 (바퀴별 시험, 메카넘용)
//   "S"                 : 즉시 정지
//   "D a b c d"         : 모터 0..3 의 방향 부호(+1/-1) 설정, EEPROM 에 저장 -> "OK D ..."
//   "?"                 : 상태 한 줄 -> "ST l r | d0 d1 d2 d3 | age_ms"
// 안전: 마지막 M/W 명령 후 300 ms 동안 새 명령이 없으면 스스로 정지한다.

#include <Arduino.h>
#include <EEPROM.h>

// 모터별 핀: pwmPin 은 PWM 가능한 핀, otherPin 은 나머지 핀.
// pwmIsIn1 = true 면 pwmPin 이 그 모터의 첫 번째 입력(IN1 또는 IN3)이다.
struct Motor {
  uint8_t pwmPin, otherPin;
  bool pwmIsFirst;
};

static const Motor MOTORS[4] = {
  {11, 10, true},   // 왼쪽 A : IN1=D11(PWM) IN2=D10
  {9, 8, true},     // 왼쪽 B : IN3=D9(PWM)  IN4=D8
  {3, 2, false},    // 오른쪽 A: IN1=D2       IN2=D3(PWM)
  {5, 4, false},    // 오른쪽 B: IN3=D4       IN4=D5(PWM)
};

static int8_t dirSign[4] = {1, 1, 1, 1};
static int currentPwm[4] = {0, 0, 0, 0};
static unsigned long lastCommandMs = 0;
static bool moving = false;

static char line[40];
static uint8_t lineLen = 0;

// value > 0: 첫 번째 입력 HIGH 쪽으로 구동, value < 0: 반대, 0: 정지(브레이크)
static void driveMotor(uint8_t index, int value) {
  const Motor &m = MOTORS[index];
  value = constrain(value, -255, 255) * dirSign[index];
  currentPwm[index] = value;
  if (value == 0) {
    digitalWrite(m.otherPin, LOW);
    analogWrite(m.pwmPin, 0);
    return;
  }
  uint8_t duty = (uint8_t)abs(value);
  bool firstHigh = value > 0;               // 첫 번째 입력을 HIGH 로 쓰는 방향인가
  if (firstHigh == m.pwmIsFirst) {
    // PWM 핀이 구동 쪽: 반대 핀 LOW, PWM = duty
    digitalWrite(m.otherPin, LOW);
    analogWrite(m.pwmPin, duty);
  } else {
    // 반대 핀이 구동 쪽: 반대 핀 HIGH, PWM 은 뒤집어서 (LOW 인 동안 구동)
    digitalWrite(m.otherPin, HIGH);
    analogWrite(m.pwmPin, 255 - duty);
  }
}

static void stopAll() {
  for (uint8_t i = 0; i < 4; i++) driveMotor(i, 0);
  moving = false;
}

static bool parseInts(const char *text, long *out, uint8_t count) {
  const char *p = text;
  for (uint8_t i = 0; i < count; i++) {
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    out[i] = v;
    p = end;
  }
  while (*p == ' ') p++;
  return *p == 0;
}

static void handleLine() {
  long v[4];
  if (line[0] == 'M' && line[1] == ' ' && parseInts(line + 2, v, 2)) {
    driveMotor(0, (int)v[0]);
    driveMotor(1, (int)v[0]);
    driveMotor(2, (int)v[1]);
    driveMotor(3, (int)v[1]);
    lastCommandMs = millis();
    moving = v[0] != 0 || v[1] != 0;
  } else if (line[0] == 'W' && line[1] == ' ' && parseInts(line + 2, v, 4)) {
    for (uint8_t i = 0; i < 4; i++) driveMotor(i, (int)v[i]);
    lastCommandMs = millis();
    moving = v[0] || v[1] || v[2] || v[3];
  } else if (line[0] == 'S' && line[1] == 0) {
    stopAll();
  } else if (line[0] == 'D' && line[1] == ' ' && parseInts(line + 2, v, 4)) {
    for (uint8_t i = 0; i < 4; i++) {
      if (v[i] != 1 && v[i] != -1) {
        Serial.println(F("ERR D needs +1/-1"));
        return;
      }
    }
    stopAll();
    EEPROM.update(10, 0x5A);
    for (uint8_t i = 0; i < 4; i++) {
      dirSign[i] = (int8_t)v[i];
      EEPROM.update(11 + i, (uint8_t)(v[i] == 1 ? 1 : 2));
    }
    Serial.print(F("OK D"));
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(' ');
      Serial.print(dirSign[i]);
    }
    Serial.println();
  } else if (line[0] == '?' && line[1] == 0) {
    Serial.print(F("ST "));
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(currentPwm[i]);
      Serial.print(' ');
    }
    Serial.print(F("| "));
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(dirSign[i]);
      Serial.print(' ');
    }
    Serial.print(F("| "));
    Serial.println(millis() - lastCommandMs);
  }
}

void setup() {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(MOTORS[i].pwmPin, LOW);
    digitalWrite(MOTORS[i].otherPin, LOW);
    pinMode(MOTORS[i].pwmPin, OUTPUT);
    pinMode(MOTORS[i].otherPin, OUTPUT);
  }
  if (EEPROM.read(10) == 0x5A) {
    for (uint8_t i = 0; i < 4; i++) dirSign[i] = EEPROM.read(11 + i) == 2 ? -1 : 1;
  }
  Serial.begin(115200);
  Serial.println(F("L298 READY"));
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[lineLen] = 0;
      if (lineLen) handleLine();
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = ch;
    } else {
      lineLen = 0;
    }
  }
  if (moving && millis() - lastCommandMs > 300) stopAll();   // 명령이 끊기면 정지
}
