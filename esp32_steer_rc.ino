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
    RC,ch1,ch2,ch5,ch6,target_angle_deg,servo_command_deg,pulses,frames,rejects

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

/*
  리셋 직후 PPM 캡처가 살아나지 못하는 경우가 실측 6회 중 2회 발생했다.
  한 번 실패하면 스스로 복구되지 않아서 조종이 통째로 먹통이 된다.
  (젯슨이 USB 포트를 열 때마다 CH340 자동 리셋 회로가 ESP 를 재부팅시키므로
   이 실패는 주행 중에도 계속 재현된다.)

  원인이 인터럽트 등록 실패든 프레임 동기 실패든, 일정 시간 유효 프레임이
  없으면 인터럽트를 떼었다 다시 붙이고 상태를 통째로 초기화한다.
*/
const unsigned long PPM_REARM_INTERVAL_MS = 500;

// ==================================================
// Jetson USB-C 설정
// ==================================================

const long JETSON_BAUD = 115200;

const unsigned long SEND_INTERVAL_MS = 20;
const unsigned long UART_TIMEOUT_MS  = 300;

// 제어 주기. 이전의 delay(20) 을 논블로킹으로 대체한다.
const unsigned long CONTROL_INTERVAL_MS = 20;

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

/*
  CH6 LOW = 정상 주행, HIGH = E-STOP

  임계값이 1500 이면 3단 스위치의 중립(정확히 1500)이나 경계 노이즈에서
  E-STOP 이 걸려버린다. 실측 정상값은 1000, E-STOP 위치는 2000 이므로
  둘 사이에서 충분히 떨어뜨리고 히스테리시스를 준다.
*/
const int ESTOP_THRESHOLD_US = 1700;
const int ESTOP_RELEASE_US   = 1400;

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

// 진단용: 유효 프레임 수와 범위 밖이라 버린 프레임 수
volatile uint16_t ppmFrameCount  = 0;
volatile uint16_t ppmRejectCount = 0;

// PPM 워치독이 인터럽트를 다시 붙인 횟수
uint16_t ppmRearmCount = 0;
unsigned long lastRearmMs = 0;

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
unsigned long lastControlMs   = 0;

// AUTO 이고 E-STOP/PPM 손실이 아닐 때만 젯슨 조향을 반영한다.
bool acceptJetsonSteer = false;

bool lastModeAuto = false;
bool estopLatched = false;

/*
  Arduino String 은 매 문자 += 마다 재할당이 일어나 장시간 주행에서 힙이 조각난다.
  고정 크기 char 버퍼로 바꿔서 할당 자체를 없앤다.
*/
const uint8_t JETSON_RX_MAX = 32;
char jetsonRxBuffer[JETSON_RX_MAX + 1];
uint8_t jetsonRxLen = 0;

// ==================================================
// PPM ISR
// ==================================================

static inline bool IRAM_ATTR inPpmRange(uint16_t value)
{
  return (value >= PPM_MIN_VALID_US && value <= PPM_MAX_VALID_US);
}

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
      /*
        실제로 쓰는 채널만 검사한다.

        이전 버전은 CH1~CH6 전부를 검사해서, 한 번도 안 쓰는 CH3/CH4 에 노이즈가
        끼면 프레임 전체를 버렸다. 그 상태가 100ms(5프레임) 이어지면 페일세이프로
        떨어져서 조종이 통째로 먹통이 됐다.
      */
      // IRAM ISR 안에서는 flash 접근을 피해야 하므로 배열 대신 직접 펼쳐 쓴다.
      bool validFrame =
        inPpmRange(ppmCapture[PPM_CH1]) &&
        inPpmRange(ppmCapture[PPM_CH2]) &&
        inPpmRange(ppmCapture[PPM_CH5]) &&
        inPpmRange(ppmCapture[PPM_CH6]);

      if (!validFrame)
      {
        ppmRejectCount++;
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
          ppmFrameCount++;
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
// PPM 캡처 (재)초기화
// ==================================================

void armPpmCapture()
{
  detachInterrupt(digitalPinToInterrupt(PPM_PIN));

  // 수신기 PPM 출력이 끊겨도 라인이 뜨지 않도록 고정한다.
  pinMode(PPM_PIN, INPUT_PULLDOWN);

  noInterrupts();

  lastEdgeUs   = micros();
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
    estopLatched = true;
    return true;
  }

  // 임계값 근처에서 떨렸을 때 E-STOP 이 깜빡이지 않도록 히스테리시스를 둔다.
  if (ch6 >= ESTOP_THRESHOLD_US)
  {
    estopLatched = true;
  }
  else if (ch6 <= ESTOP_RELEASE_US)
  {
    estopLatched = false;
  }

  return estopLatched;
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

/*
  젯슨 수신은 어떤 상태에서도 매 루프 호출해야 한다.

  이전 버전은 PPM 페일세이프와 E-STOP 구간에서 이 함수를 건너뛰었다.
  그 사이 젯슨이 계속 보내는 S: 명령이 ESP RX 버퍼(기본 256B)를 채워 넘치고,
  복구되는 순간 밀린 옛날 조향 명령이 한꺼번에 적용돼서 조향이 튀었다.
  그래서 파싱은 accept 여부와 무관하게 항상 끝까지 비운다.
*/
void readJetsonSteer(bool acceptCommands)
{
  while (JetsonSerial.available())
  {
    char c = (char)JetsonSerial.read();

    if (c == '\n' || c == '\r')
    {
      if (jetsonRxLen == 0)
      {
        continue;
      }

      jetsonRxBuffer[jetsonRxLen] = '\0';
      uint8_t len = jetsonRxLen;
      jetsonRxLen = 0;

      if (!acceptCommands)
      {
        continue;
      }

      if (len < 3 || jetsonRxBuffer[0] != 'S' || jetsonRxBuffer[1] != ':')
      {
        continue;
      }

      char *end = nullptr;
      float steering = strtof(&jetsonRxBuffer[2], &end);

      // 숫자가 하나도 안 읽혔거나 NaN/Inf 면 버린다.
      if (end == &jetsonRxBuffer[2] || isnan(steering) || isinf(steering))
      {
        continue;
      }

      uartTargetAngle = normToAngle(steering);
      lastUartCmdTime = millis();
      continue;
    }

    if (jetsonRxLen < JETSON_RX_MAX)
    {
      jetsonRxBuffer[jetsonRxLen++] = c;
    }
    else
    {
      // 줄이 비정상적으로 길면 개행까지 통째로 버린다.
      jetsonRxLen = 0;
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

  uint8_t channelCount;
  uint16_t frameCount;
  uint16_t rejectCount;

  noInterrupts();
  channelCount = detectedChannelCount;
  frameCount   = ppmFrameCount;
  rejectCount  = ppmRejectCount;
  interrupts();

  /*
    RC,ch1,ch2,ch5,ch6,target_angle_deg,servo_command_deg,pulses,frames,rejects,rearms

    뒤 4개는 진단용이다. 젯슨 파서는 앞 7개만 읽으므로 붙여도 안전하다.
      pulses  : 마지막 프레임에서 센 펄스 개수 (0 이면 PPM 선에 신호 자체가 없음)
      frames  : 누적 유효 프레임 수
      rejects : 채널값이 범위 밖이라 버린 프레임 수 (배선 노이즈 지표)
      rearms  : PPM 워치독이 인터럽트를 다시 붙인 횟수
  */
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
  JetsonSerial.print(servoCommandAngle);
  JetsonSerial.print(",");
  JetsonSerial.print(channelCount);
  JetsonSerial.print(",");
  JetsonSerial.print(frameCount);
  JetsonSerial.print(",");
  JetsonSerial.print(rejectCount);
  JetsonSerial.print(",");
  JetsonSerial.println(ppmRearmCount);
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
  // 젯슨이 20ms 마다 S: 를 보낸다. 기본 256B 로는 한 번 밀리면 바로 넘친다.
  JetsonSerial.setRxBufferSize(1024);
  JetsonSerial.begin(JETSON_BAUD);
  delay(500);

  // 부팅 직후 남아 있는 젯슨의 옛날 명령을 버린다.
  while (JetsonSerial.available())
  {
    JetsonSerial.read();
  }

  jetsonRxLen = 0;
  acceptJetsonSteer = false;
  lastControlMs = millis();

  armPpmCapture();
  lastRearmMs = millis();

  // 서보 하나만 사용하므로 타이머 하나만 할당
  ESP32PWM::allocateTimer(0);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2500);

  writeServoCommand(CENTER_ANGLE);

  lastUartCmdTime = millis();

  /*
    젯슨이 재부팅을 확실히 알아채도록 항상 한 줄 남긴다.
    CH340 자동 리셋 회로 때문에 젯슨이 포트를 열 때마다 여기를 지나간다.
    control_node 는 이 문구를 보고 RC 를 손실로 떨어뜨린 뒤 재동기를 기다린다.
  */
  JetsonSerial.println();
  JetsonSerial.println("ESP32 RC + Jetson USB-C BOOT");

  if (DEBUG_LOG)
  {
    JetsonSerial.println();
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

  /*
    젯슨 RX 는 제어 주기와 무관하게 매 루프 비운다.
    수락 여부는 직전 제어 주기의 판정을 쓰므로 최대 20ms 만 늦는다.
  */
  readJetsonSteer(acceptJetsonSteer);

  /*
    이전 버전은 분기마다 delay(20) 으로 루프를 막았다.
    그 20ms 동안 RX 를 못 읽어서 젯슨의 S: 명령이 쌓이다 넘쳤다.
    이제는 논블로킹으로 주기만 맞춘다.
  */
  if (nowMs - lastControlMs < CONTROL_INTERVAL_MS)
  {
    return;
  }

  lastControlMs = nowMs;

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

    // PPM 을 잃은 동안 젯슨 조향 명령을 받아두면 복구 순간 옛날 값이 튄다.
    acceptJetsonSteer = false;
    lastModeAuto = false;

    // 캡처가 죽은 채로 멈춰 있지 않도록 주기적으로 다시 붙인다.
    if (nowMs - lastRearmMs >= PPM_REARM_INTERVAL_MS)
    {
      lastRearmMs = nowMs;
      ppmRearmCount++;
      armPpmCapture();
    }

    if (DEBUG_LOG && nowMs - lastPrintMs >= 200)
    {
      lastPrintMs = nowMs;

      JetsonSerial.print("PPM FAILSAFE -> CENTER");
      JetsonSerial.print(" count=");
      JetsonSerial.println(channelCount);
    }

    // PPM 손실 중에도 Jetson이 중앙 복귀 명령을 확인할 수 있게 전송한다.
    sendTelemetry(0, 0, 0, 0);
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

    // E-STOP 해제 순간 밀린 조향이 튀지 않도록 수락을 끊는다.
    acceptJetsonSteer = false;

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

    return;
  }

  /*
    AUTO 로 갓 진입한 순간에는 직전 AUTO 구간에서 남은 uartTargetAngle 이 그대로
    쓰여서 조향이 튄다. 젯슨의 새 명령이 올 때까지 중앙을 목표로 둔다.
  */
  if (autoMode && !acceptJetsonSteer)
  {
    uartTargetAngle = CENTER_ANGLE;
    lastUartCmdTime = nowMs - UART_TIMEOUT_MS - 1;
  }

  acceptJetsonSteer = autoMode;

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
}
