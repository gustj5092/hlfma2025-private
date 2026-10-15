// driving_encoder.ino — driving.ino + 뒷바퀴 홀센서 엔코더 + 상태 피드백 + 전자석/비상정지 자리
//
// 보드: Arduino Mega 2560, 모터드라이버 MD30C(PWM+DIR) x3, 조향 가변저항 A2
// 기존 driving.ino와 PC->Mega 명령 포맷은 그대로입니다 ("s{조향스텝}l{앞PWM}r{뒤PWM}\n").
// 추가된 것:
//   PC -> Mega :  "M0" / "M1"   전자석 OFF / ON          (대문자 M, 한 줄에 단독)
//                 "R"           엔코더 카운트 0으로 리셋  (대문자 R)
//   Mega -> PC :  "S <enc_count> <steer_step> <estop> <pot_raw> <magnet>\n"  50 ms마다
//                 enc_count  : 누적 홀센서 펄스 (방향 없음, PC가 명령 방향으로 부호를 붙임)
//                 steer_step : 가변저항에서 환산한 현재 조향 스텝 (-7..+7, +=우)
//                 estop      : 1이면 비상정지 입력 활성
//                 pot_raw    : 가변저항 ADC 원값 (캘리브레이션용)
//                 magnet     : 전자석 출력 상태
// 핀 번호와 극성은 아래 "Pins" 블록에만 모아 두었습니다. 배선 후 여기만 고치면 됩니다.

// ===== Pins (MD30C) =====
static const int FRONT_PWM = 5;
static const int FRONT_DIR = 22;
static const int REAR_PWM  = 6;
static const int REAR_DIR  = 24;
static const int STEER_PWM = 9;
static const int STEER_DIR = 26;

// ===== Pins (new) =====
static const int ENCODER_PIN = 2;      // 홀센서 신호. Mega 외부 인터럽트 핀: 2, 3, 18, 19, 20, 21
static const int MAGNET_PIN  = 8;      // 전자석 MOSFET 게이트 (HIGH = ON). 아직 미배선이면 그대로 둬도 됨
static const int ESTOP_PIN   = -1;     // 비상정지 입력 (INPUT_PULLUP, LOW = 정지). 없으면 -1

// ===== DIR polarity =====
static bool FRONT_DIR_FWD = LOW;
static bool REAR_DIR_FWD  = LOW;
static bool STEER_DIR_RT  = LOW;

// ===== POT (steering sensor) =====
const int POT = A2;
const int RES_LEFT   = 1023;   // 가장 왼쪽에서 읽힌 값
const int RES_CENTER = 580;    // 정확한 센터 실측
const int RES_RIGHT  = 8;      // 가장 오른쪽에서 읽힌 값
const int MAX_STEERING_STEP = 7;

// ===== Encoder =====
const unsigned long ENCODER_MIN_PULSE_US = 3000;  // 이보다 짧은 간격의 펄스는 바운스로 무시 (70 mm/펄스, 1 m/s여도 70 ms 간격)
const bool ENCODER_USE_PULLUP = true;             // 오픈컬렉터 홀센서면 true

// ===== Params =====
const unsigned int COMMAND_INTERVAL = 50;        // ms, 모터/조향 갱신 주기
const unsigned int STATUS_INTERVAL  = 50;        // ms, "S ..." 송신 주기
const int MAX_SPEED_CHANGE_PER_INTERVAL = 20;
const unsigned long RX_TIMEOUT = 300;            // ms, 이 시간 동안 명령이 없으면 즉시 정지 (USB 끊김 대비)
const bool MAGNET_OFF_ON_ESTOP = true;           // 비상정지 시 전자석도 끔 (브리프 "모든 출력 0")
// 주의: RX_TIMEOUT(통신 끊김)에는 모터만 세우고 전자석은 유지합니다. 카트를 물고 있을 때 놓치지 않기 위함.

// === 조향 P 제어 ===
const float STEER_KP = 30.0;
const int STEER_DEAD_BAND = 1;
const int MIN_STEER_SPEED = 40;

// ===== State =====
int angle_cmd = 0;                 // -MAX..+MAX
int target_front_speed = 0;
int target_rear_speed = 0;
int current_front_speed = 0;
int current_rear_speed = 0;
unsigned long lastCommandTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastRxTime = 0;
bool magnet_on = false;
bool estop_active = false;
int steer_step_now = 0;
int pot_raw_now = 0;

volatile unsigned long enc_count = 0;
volatile unsigned long enc_last_us = 0;

// ==== Encoder ISR ====
void onEncoderPulse() {
  unsigned long now_us = micros();
  if (now_us - enc_last_us < ENCODER_MIN_PULSE_US) return;
  enc_last_us = now_us;
  enc_count++;
}

unsigned long readEncoder() {
  noInterrupts();
  unsigned long c = enc_count;
  interrupts();
  return c;
}

void resetEncoder() {
  noInterrupts();
  enc_count = 0;
  interrupts();
}

// ==== MD30C drive ====
void md30c_drive(int pwmPin, int dirPin, int signedSpeed, bool dirPositiveLevel) {
  int duty = abs(signedSpeed);
  if (duty > 255) duty = 255;
  bool pos = (signedSpeed >= 0);
  digitalWrite(dirPin, pos ? dirPositiveLevel : !dirPositiveLevel);
  analogWrite(pwmPin, duty);  // 0 -> brake
}
void brake(int pwmPin) { analogWrite(pwmPin, 0); }
void setFrontMotorSpeed(int spd){ md30c_drive(FRONT_PWM, FRONT_DIR, spd, FRONT_DIR_FWD); }
void setRearMotorSpeed (int spd){ md30c_drive(REAR_PWM,  REAR_DIR,  spd, REAR_DIR_FWD ); }
void steerBrake(){ brake(STEER_PWM); }

void setMagnet(bool on) {
  magnet_on = on;
  digitalWrite(MAGNET_PIN, on ? HIGH : LOW);
}

// ===== Serial parser =====
const unsigned int MAX_INPUT = 24;

void processMotion(const char *data) {
  int sIndex=-1, fIndex=-1, rIndex=-1;
  for (int i=0; data[i]!='\0'; i++){
    if (data[i]=='s') sIndex=i;
    else if (data[i]=='l') fIndex=i;
    else if (data[i]=='r') rIndex=i;
  }
  if (sIndex!=-1 && fIndex!=-1 && rIndex!=-1){
    int newAngle      = atoi(data + sIndex + 1);
    int newFrontSpeed = atoi(data + fIndex + 1);
    int newRearSpeed  = atoi(data + rIndex + 1);
    angle_cmd   = constrain(newAngle, -MAX_STEERING_STEP, MAX_STEERING_STEP);
    target_front_speed = constrain(newFrontSpeed, -255, 255);
    target_rear_speed  = constrain(newRearSpeed,  -255, 255);
    lastRxTime = millis();
  }
}

void processData(const char *data) {
  if (data[0] == '\0') return;
  if (data[0] == 'M') {              // "M0" / "M1" / "M 1"
    const char *p = data + 1;
    while (*p == ' ') p++;
    setMagnet(*p == '1');
    lastRxTime = millis();           // 전자석 명령도 통신 생존 신호로 취급
    return;
  }
  if (data[0] == 'R') {              // 엔코더 리셋
    resetEncoder();
    return;
  }
  processMotion(data);
}

void processIncomingByte(const byte b){
  static char line[MAX_INPUT]; static unsigned int pos=0;
  switch(b){
    case '\n': line[pos]=0; processData(line); pos=0; break;
    case '\r': break;
    default: if (pos < (MAX_INPUT-1)) line[pos++]=b; break;
  }
}

// ===== Centered piecewise map with clamping =====
int map_centered(int res_raw) {
  int res = res_raw;
  if (RES_LEFT >= RES_RIGHT) {
    if (res > RES_LEFT)  res = RES_LEFT;
    if (res < RES_RIGHT) res = RES_RIGHT;
  } else {
    if (res < RES_LEFT)  res = RES_LEFT;
    if (res > RES_RIGHT) res = RES_RIGHT;
  }
  long step;
  if (res >= RES_CENTER) {
    step = map(res, RES_CENTER, RES_LEFT, 0, -MAX_STEERING_STEP);
  } else {
    step = map(res, RES_CENTER, RES_RIGHT, 0, +MAX_STEERING_STEP);
  }
  return (int)constrain(step, -MAX_STEERING_STEP, MAX_STEERING_STEP);
}

int rampToward(int current, int target) {
  if (current < target) {
    current += MAX_SPEED_CHANGE_PER_INTERVAL;
    if (current > target) current = target;
  } else if (current > target) {
    current -= MAX_SPEED_CHANGE_PER_INTERVAL;
    if (current < target) current = target;
  }
  return current;
}

void sendStatus() {
  Serial.print("S ");
  Serial.print(readEncoder());
  Serial.print(' ');
  Serial.print(steer_step_now);
  Serial.print(' ');
  Serial.print(estop_active ? 1 : 0);
  Serial.print(' ');
  Serial.print(pot_raw_now);
  Serial.print(' ');
  Serial.println(magnet_on ? 1 : 0);
}

// ===== Setup & Loop =====
void setup() {
  Serial.begin(115200);

  pinMode(POT, INPUT);
  pinMode(FRONT_PWM, OUTPUT); pinMode(FRONT_DIR, OUTPUT);
  pinMode(REAR_PWM,  OUTPUT); pinMode(REAR_DIR,  OUTPUT);
  pinMode(STEER_PWM, OUTPUT); pinMode(STEER_DIR, OUTPUT);
  pinMode(MAGNET_PIN, OUTPUT);
  setMagnet(false);

  pinMode(ENCODER_PIN, ENCODER_USE_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), onEncoderPulse, FALLING);
  if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, INPUT_PULLUP);

  brake(FRONT_PWM); brake(REAR_PWM); brake(STEER_PWM);
  digitalWrite(FRONT_DIR, LOW); digitalWrite(REAR_DIR, LOW); digitalWrite(STEER_DIR, LOW);

  Serial.println("driving_encoder ready: s/l/r motion, M0/M1 magnet, R reset; status 'S enc step estop pot magnet'");
}

void loop() {
  unsigned long now = millis();
  while (Serial.available() > 0) processIncomingByte(Serial.read());

  if (now - lastCommandTime >= COMMAND_INTERVAL) {
    lastCommandTime = now;

    estop_active = (ESTOP_PIN >= 0) && (digitalRead(ESTOP_PIN) == LOW);

    // 명령 끊김: 램프 없이 즉시 정지 (조향은 현재 목표 유지)
    if (millis() - lastRxTime > RX_TIMEOUT) {
      target_front_speed = target_rear_speed = 0;
      current_front_speed = current_rear_speed = 0;
    }

    pot_raw_now = analogRead(POT);
    steer_step_now = map_centered(pot_raw_now);

    if (estop_active) {
      target_front_speed = target_rear_speed = 0;
      current_front_speed = current_rear_speed = 0;
      steerBrake();
      setFrontMotorSpeed(0);
      setRearMotorSpeed(0);
      if (MAGNET_OFF_ON_ESTOP && magnet_on) setMagnet(false);
    } else {
      int err = angle_cmd - steer_step_now;
      if (abs(err) <= STEER_DEAD_BAND) {
        steerBrake();
      } else {
        int steer_speed = (int)(err * STEER_KP);
        steer_speed = constrain(steer_speed, -255, 255);
        if (steer_speed > 0 && steer_speed < MIN_STEER_SPEED) steer_speed = MIN_STEER_SPEED;
        else if (steer_speed < 0 && steer_speed > -MIN_STEER_SPEED) steer_speed = -MIN_STEER_SPEED;
        md30c_drive(STEER_PWM, STEER_DIR, steer_speed, STEER_DIR_RT);
      }

      current_front_speed = rampToward(current_front_speed, target_front_speed);
      current_rear_speed  = rampToward(current_rear_speed,  target_rear_speed);
      setFrontMotorSpeed(current_front_speed);
      setRearMotorSpeed(current_rear_speed);
    }
  }

  if (now - lastStatusTime >= STATUS_INTERVAL) {
    lastStatusTime = now;
    sendStatus();
  }
}
