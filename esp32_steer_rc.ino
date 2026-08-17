/*
  ESP32 RC + Jetson USB-C (C to C)

  PPM sync lock:
    ppmArmed / lastEdgeUs / 6ch 프레임 / warmup 2프레임

  [0]=CH1 조향
  [1]=CH2 스로틀
  [4]=CH5 모드
  [5]=CH6 E-STOP

  MANUAL (CH5<=1300):
    CH1 → 서보
    CH2 + CH5 → Jetson
    Jetson S: 명령 무시

  AUTO (CH5>=1700):
    RC CH1/CH2 구동 무시
    Jetson S: → 서보

  E-STOP (CH6>=1500):
    서보 중앙 복귀
    RC/Jetson 조향 명령 무시

  Jetson TX:
    RC,ch1,ch2,ch5,ch6,target_angle_deg,servo_command_deg

    target_angle_deg:
      MANUAL/AUTO에서 결정된 스무딩 전 목표 조향각

    servo_command_deg:
      스무딩과 각도 제한 후 실제 servo.write()에 전달한 정수 각도

  Jetson RX:
    S:-1.0..1.0
*/

#include <ESP32Servo.h>

Servo steeringServo;

// USB-C 포트와 연결된 Serial 하나로 젯슨과 주고받는다.
#define JetsonSerial Serial

// ==================================================
// 핀 설정
// ==================================================

const int PPM_PIN   = 16;
const int SERVO_PIN = 18;

// ==================================================
// PPM 설정
// ==================================================

// FS-iA6B PPM 6채널
const int CHANNELS = 6;
const int EXPECTED_CHANNELS = 6;

const int PPM_CH1 = 0;
const int PPM_CH2 = 1;
const int PPM_CH5 = 4;
const int PPM_CH6 = 5;

const uint32_t PPM_SYNC_GAP_US = 3000;

// 정상 채널 값 허용 범위
const uint16_t PPM_MIN_VALID_US = 800;
const uint16_t PPM_MAX_VALID_US = 2200;

// PPM 손실 판단 시간
const unsigned long PPM_TIMEOUT_MS = 100;

// ==================================================
// Jetson USB-C 설정
// ==================================================

const long JETSON_BAUD = 115200;

const unsigned long SEND_INTERVAL_MS = 20;
const unsigned long UART_TIMEOUT_MS  = 300;

/*
  디버그 로그도 젯슨과 같은 USB 선로로 나간다.
  실주행에서는 false로 두고, 시리얼 모니터로 볼 때만 true.
*/
const bool DEBUG_LOG = false;

// ==================================================
// 조향 설정
// ==================================================

const int CENTER_ANGLE = 90;
const int LEFT_ANGLE   = 40;
const int RIGHT_ANGLE  = 140;

// 1.0이면 즉시 목표각으로 이동한다.
// 진단 중에는 0.2 정도로 제한
const float SMOOTH_FACTOR = 0.20f;

// ==================================================
// 모드 설정
// ==================================================

const int MODE_MANUAL_US = 1300;
const int MODE_AUTO_US   = 1700;

// CH6 LOW = 정상 주행, HIGH = E-STOP
const int ESTOP_THRESHOLD_US = 1500;

// ==================================================
// PPM 인터럽트 변수
// ==================================================

volatile uint16_t ppmCapture[CHANNELS];
volatile uint16_t ppmStable[CHANNELS];

volatile uint8_t channelIndex = 0;
volatile uint8_t detectedChannelCount = 0;

volatile uint32_t lastEdgeUs  = 0;
volatile uint32_t lastFrameMs = 0;

volatile bool frameOk  = false;
volatile bool ppmArmed = false;

volatile uint8_t warmupFrames = 2;

// ==================================================
// 일반 변수
// ==================================================

uint16_t ppm[CHANNELS];

float currentAngle    = CENTER_ANGLE;
float targetAngle     = CENTER_ANGLE;
float uartTargetAngle = CENTER_ANGLE;

// 실제 servo.write()에 마지막으로 전달한 명령 각도
int servoCommandAngle = CENTER_ANGLE;

unsigned long lastUartCmdTime = 0;
unsigned long lastSendTime    = 0;
unsigned long lastPrintMs     = 0;

bool lastModeAuto = false;

String jetsonRxBuffer;

// ==================================================
// PPM ISR
// ==================================================

void IRAM_ATTR ppmISR()
{
  uint32_t now = micros();
  uint32_t dt = now - lastEdgeUs;
  lastEdgeUs = now;

  // 긴 공백 = 이전 프레임 종료, 새 프레임 시작
  if (dt > PPM_SYNC_GAP_US)
  {
    detectedChannelCount = channelIndex;

    /*
      첫 6채널만 사용한다.

      실제 PPM 출력이 6채널보다 많더라도
      CH1~CH6가 정상이라면 프레임을 허용한다.
    */
    if (ppmArmed && channelIndex >= EXPECTED_CHANNELS)
    {
      bool validFrame = true;

      for (uint8_t i = 0; i < CHANNELS; i++)
      {
        if (
          ppmCapture[i] < PPM_MIN_VALID_US ||
          ppmCapture[i] > PPM_MAX_VALID_US
        )
        {
          validFrame = false;
          break;
        }
      }

      if (validFrame)
      {
        if (warmupFrames > 0)
        {
          warmupFrames--;
        }
        else
        {
          for (uint8_t i = 0; i < CHANNELS; i++)
          {
            ppmStable[i] = ppmCapture[i];
          }

          frameOk = true;
          lastFrameMs = millis();
        }
      }
    }

    ppmArmed = true;
    channelIndex = 0;
    return;
  }

  if (!ppmArmed)
  {
    return;
  }

  // 실제 사용하는 첫 6채널만 배열에 저장
  if (channelIndex < CHANNELS)
  {
    ppmCapture[channelIndex] = (uint16_t)dt;
  }

  // 실제 프레임 내 펄스 개수는 계속 측정
  if (channelIndex < 20)
  {
    channelIndex++;
  }
}

// ==================================================
// PPM 데이터 복사
// ==================================================

void readPpm(uint16_t out[CHANNELS])
{
  bool ok;

  noInterrupts();

  for (int i = 0; i < CHANNELS; i++)
  {
    out[i] = ppmStable[i];
  }

  ok = frameOk;

  interrupts();

  if (!ok)
  {
    for (int i = 0; i < CHANNELS; i++)
    {
      out[i] = 0;
    }
  }
}

// ==================================================
// 채널 값 검사
// ==================================================

bool validUs(int value)
{
  return (
    value >= PPM_MIN_VALID_US &&
    value <= PPM_MAX_VALID_US
  );
}

int readOne(int slot)
{
  if (slot < 0 || slot >= CHANNELS)
  {
    return 0;
  }

  int value = ppm[slot];

  return validUs(value) ? value : 0;
}

// ==================================================
// 값 변환 함수
// ==================================================

float mapFloat(
  float x,
  float inputMin,
  float inputMax,
  float outputMin,
  float outputMax
)
{
  return (
    (x - inputMin) *
    (outputMax - outputMin) /
    (inputMax - inputMin)
  ) + outputMin;
}

// ==================================================
// MANUAL / AUTO 판정
// ==================================================

bool isAuto(int ch5)
{
  // CH5가 비정상이면 기존 모드 유지
  if (ch5 <= 0)
  {
    return lastModeAuto;
  }

  // 정상 설정: CH5 LOW = MANUAL
  if (ch5 <= MODE_MANUAL_US)
  {
    lastModeAuto = false;
    return false;
  }

  // 정상 설정: CH5 HIGH = AUTO
  if (ch5 >= MODE_AUTO_US)
  {
    lastModeAuto = true;
    return true;
  }

  // 1300~1700 사이는 기존 상태 유지
  return lastModeAuto;
}

// ==================================================
// CH6 E-STOP 판정
// ==================================================

bool isEstop(int ch6)
{
  // 채널 손실/비정상 값은 안전하게 E-STOP 처리
  if (ch6 <= 0)
  {
    return true;
  }

  // 정상 설정: CH6 LOW = RUN, HIGH = E-STOP
  return ch6 >= ESTOP_THRESHOLD_US;
}

// ==================================================
// CH1 → 조향각
// ==================================================

float ch1ToAngle(int us)
{
  float angle = mapFloat(
    (float)us,
    1000.0f,
    2000.0f,
    (float)RIGHT_ANGLE,  // 기존 LEFT_ANGLE
    (float)LEFT_ANGLE    // 기존 RIGHT_ANGLE
  );

  return constrain(
    angle,
    (float)min(LEFT_ANGLE, RIGHT_ANGLE),
    (float)max(LEFT_ANGLE, RIGHT_ANGLE)
  );
}

// ==================================================
// Jetson -1~1 → 조향각
// ==================================================

float normToAngle(float steering)
{
  steering = constrain(steering, -1.0f, 1.0f);

  return mapFloat(
    steering,
    -1.0f,
    1.0f,
    (float)LEFT_ANGLE,
    (float)RIGHT_ANGLE
  );
}

// ==================================================
// Jetson 조향 명령 읽기
// ==================================================

void readJetsonSteer(bool autoMode)
{
  while (JetsonSerial.available())
  {
    char c = JetsonSerial.read();

    if (c == '\n')
    {
      String command = jetsonRxBuffer;
      jetsonRxBuffer = "";

      command.trim();

      // S: 형식이 아니거나 MANUAL이면 무시
      if (!command.startsWith("S:") || !autoMode)
      {
        continue;
      }

      float steering = command.substring(2).toFloat();

      uartTargetAngle = normToAngle(steering);
      lastUartCmdTime = millis();
    }
    else if (c != '\r')
    {
      jetsonRxBuffer += c;

      if (jetsonRxBuffer.length() > 30)
      {
        jetsonRxBuffer = "";
      }
    }
  }
}

// ==================================================
// 최종 서보 출력
// ==================================================

void writeServoCommand(float angleDeg)
{
  float limitedAngle = constrain(
    angleDeg,
    (float)min(LEFT_ANGLE, RIGHT_ANGLE),
    (float)max(LEFT_ANGLE, RIGHT_ANGLE)
  );

  // 기존 코드와 동일하게 가장 가까운 정수 각도로 반올림
  servoCommandAngle = (int)(limitedAngle + 0.5f);
  steeringServo.write(servoCommandAngle);
}

// ==================================================
// Jetson으로 RC 데이터 전송
// ==================================================

void sendTelemetry(int ch1, int ch2, int ch5, int ch6)
{
  if (millis() - lastSendTime < SEND_INTERVAL_MS)
  {
    return;
  }

  lastSendTime = millis();

  // 기존 RC 필드 5개는 그대로 유지하고 조향각 필드를 뒤에 추가한다.
  // RC,ch1,ch2,ch5,ch6,target_angle_deg,servo_command_deg
  JetsonSerial.print("RC,");
  JetsonSerial.print(ch1);
  JetsonSerial.print(",");
  JetsonSerial.print(ch2);
  JetsonSerial.print(",");
  JetsonSerial.print(ch5);
  JetsonSerial.print(",");
  JetsonSerial.print(ch6);
  JetsonSerial.print(",");
  JetsonSerial.print(targetAngle, 2);
  JetsonSerial.print(",");
  JetsonSerial.println(servoCommandAngle);
}

// ==================================================
// PPM Failsafe
// ==================================================

void applyPpmFailsafe()
{
  targetAngle     = CENTER_ANGLE;
  currentAngle    = CENTER_ANGLE;
  uartTargetAngle = CENTER_ANGLE;

  writeServoCommand(CENTER_ANGLE);
}

// ==================================================
// setup
// ==================================================

void setup()
{
  JetsonSerial.begin(JETSON_BAUD);
  delay(500);

  pinMode(PPM_PIN, INPUT);

  noInterrupts();

  lastEdgeUs = micros();
  channelIndex = 0;
  detectedChannelCount = 0;

  frameOk  = false;
  ppmArmed = false;

  warmupFrames = 2;

  for (int i = 0; i < CHANNELS; i++)
  {
    ppmCapture[i] = 0;
    ppmStable[i]  = 0;
  }

  interrupts();

  attachInterrupt(
    digitalPinToInterrupt(PPM_PIN),
    ppmISR,
    RISING
  );

  // 서보 하나만 사용하므로 타이머 하나만 할당
  ESP32PWM::allocateTimer(0);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2500);

  writeServoCommand(CENTER_ANGLE);

  lastUartCmdTime = millis();

  if (DEBUG_LOG)
  {
    JetsonSerial.println();
    JetsonSerial.println("ESP32 RC + Jetson USB-C");
    JetsonSerial.println("PPM channels: 6");
    JetsonSerial.println("CH1=PPM[0] CH2=PPM[1] CH5=PPM[4] CH6=PPM[5]");
    JetsonSerial.println("CH5 LOW=MANUAL, HIGH=AUTO");
    JetsonSerial.println("CH6 LOW=RUN, HIGH=E-STOP");
    JetsonSerial.println("MANUAL: CH1 -> servo");
    JetsonSerial.println("AUTO: Jetson S: -> servo");
    JetsonSerial.println("PPM timeout: CENTER");
    JetsonSerial.println("Jetson TX: RC,ch1,ch2,ch5,ch6,target_angle_deg,servo_command_deg");
  }
}

// ==================================================
// loop
// ==================================================

void loop()
{
  unsigned long nowMs = millis();

  readPpm(ppm);

  bool ppmFrameOk;
  uint32_t frameTime;
  uint8_t channelCount;

  noInterrupts();

  ppmFrameOk = frameOk;
  frameTime = lastFrameMs;
  channelCount = detectedChannelCount;

  interrupts();

  // ==================================================
  // PPM 손실 또는 동기 실패
  // ==================================================

  if (
    !ppmFrameOk ||
    (nowMs - frameTime) > PPM_TIMEOUT_MS
  )
  {
    applyPpmFailsafe();

    if (DEBUG_LOG && nowMs - lastPrintMs >= 200)
    {
      lastPrintMs = nowMs;

      JetsonSerial.print("PPM FAILSAFE -> CENTER");
      JetsonSerial.print(" count=");
      JetsonSerial.println(channelCount);
    }

    // PPM 손실 중에도 Jetson이 중앙 복귀 명령을 확인할 수 있게 전송한다.
    sendTelemetry(0, 0, 0, 0);

    delay(20);
    return;
  }

  // ==================================================
  // 정상 채널 읽기
  // ==================================================

  int ch1 = readOne(PPM_CH1);
  int ch2 = readOne(PPM_CH2);
  int ch5 = readOne(PPM_CH5);
  int ch6 = readOne(PPM_CH6);

  // 스위치 판정은 루프당 한 번만 수행
  bool autoMode = isAuto(ch5);
  bool estopActive = isEstop(ch6);

  // CH6 E-STOP이 활성화되면 즉시 중앙 복귀 후 명령 무시
  if (estopActive)
  {
    applyPpmFailsafe();
    sendTelemetry(ch1, ch2, ch5, ch6);

    if (DEBUG_LOG && nowMs - lastPrintMs >= 200)
    {
      lastPrintMs = nowMs;

      JetsonSerial.print("E-STOP -> CENTER");
      JetsonSerial.print(" CH5=");
      JetsonSerial.print(ch5);
      JetsonSerial.print(" CH6=");
      JetsonSerial.println(ch6);
    }

    delay(20);
    return;
  }

  readJetsonSteer(autoMode);

  // ==================================================
  // 목표 조향각 결정
  // ==================================================

  if (autoMode)
  {
    // Jetson 명령이 끊기면 중앙 복귀
    if (nowMs - lastUartCmdTime > UART_TIMEOUT_MS)
    {
      targetAngle = CENTER_ANGLE;
    }
    else
    {
      targetAngle = uartTargetAngle;
    }
  }
  else
  {
    // CH1이 정상이면 송신기 조향 사용
    if (ch1 > 0)
    {
      targetAngle = ch1ToAngle(ch1);
    }
    else
    {
      // 이전 각도 유지 금지
      targetAngle = CENTER_ANGLE;
    }
  }

  // ==================================================
  // 조향 스무딩
  // ==================================================

  currentAngle +=
    (targetAngle - currentAngle) *
    SMOOTH_FACTOR;

  if (fabsf(targetAngle - currentAngle) <= 0.5f)
  {
    currentAngle = targetAngle;
  }

  currentAngle = constrain(
    currentAngle,
    (float)min(LEFT_ANGLE, RIGHT_ANGLE),
    (float)max(LEFT_ANGLE, RIGHT_ANGLE)
  );

  // 스무딩 및 제한이 끝난 최종 각도를 저장한 뒤 서보에 출력한다.
  writeServoCommand(currentAngle);

  // ==================================================
  // Jetson 전송
  // ==================================================

  sendTelemetry(ch1, ch2, ch5, ch6);

  // ==================================================
  // 시리얼 출력
  // ==================================================

  if (DEBUG_LOG && nowMs - lastPrintMs >= 200)
  {
    lastPrintMs = nowMs;

    JetsonSerial.print("COUNT=");
    JetsonSerial.print(channelCount);

    JetsonSerial.print(" CH1=");
    JetsonSerial.print(ch1);

    JetsonSerial.print(" CH2=");
    JetsonSerial.print(ch2);

    JetsonSerial.print(" CH5=");
    JetsonSerial.print(ch5);

    JetsonSerial.print(" CH6=");
    JetsonSerial.print(ch6);

    JetsonSerial.print(" ESTOP=");
    JetsonSerial.print(estopActive ? "ON" : "OFF");

    JetsonSerial.print(" MODE=");
    JetsonSerial.print(autoMode ? "AUTO" : "MANUAL");

    JetsonSerial.print(" target=");
    JetsonSerial.print(targetAngle, 1);

    JetsonSerial.print(" filtered=");
    JetsonSerial.print(currentAngle, 1);

    JetsonSerial.print(" servo_cmd=");
    JetsonSerial.println(servoCommandAngle);
  }

  delay(20);
}
