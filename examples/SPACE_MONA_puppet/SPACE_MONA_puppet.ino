#include <WiFi.h>
#include <WiFiUdp.h>
#include "Mona_ESP_lib.h"
#include <math.h>
#include <strings.h> // strncasecmp를 위한 헤더 추가 (Core 3.x 대응)

// ===================== WiFi / UDP =====================
const char* ssid     = "InMOLab";
const char* password = "dlsahfoq104!";
WiFiUDP udp;
const int localPort = 8080;
char packetBuffer[255];

// ===================== 상태 정의 =====================
enum RobotState {
  STATE_IDLE,
  STATE_TURNING,
  STATE_MOVING,
  STATE_AVOID,
  STATE_ESCAPING,
  STATE_EMERGENCY
};
volatile RobotState state = STATE_IDLE;

// 펄스/제어 상수
static const float PULSES_PER_MM     = 18.0f;
static const float PULSES_PER_DEGREE = 12.8f;
static const int   FWD_SPD  = 100;
static const int   TURN_SPD = 100;

// 900펄스마다 새로운 명령을 받음
static const long  UPDATE_THRESHOLD_PULSES = 900;   
static const float MIN_DIST_MM               = 40.0f; 

// IR/회피
static const int TH        = 80;
static const int TH_OUTER  = 100;
static const int DELTA     = 15;

// 핀 번호 정의
static const int PIN_ENCODER_LEFT  = 35;
static const int PIN_ENCODER_RIGHT = 39;

// 제어 관련 임계값
static const float ROTATION_DEADBAND_DEG = 5.0f;    // 미세 회전 무시 각도
static const int   MIN_MOTOR_PWM         = 60;      // 모터 구동 최소 출력

// ===================== PI 제어 게인 (개선) =====================
static const float K_P = 0.5f;   // 비례 게인 감소 (0.95 → 0.5)
static const float K_I = 0.002f; // 적분 게인 대폭 감소 (0.01 → 0.002)

// 적분 제한 추가
static const float INTEGRAL_LIMIT = 500.0f;

// 오차 데드밴드
static const int ERROR_DEADBAND = 5;

// 비상 동작 속도
static const int EMERGENCY_SPIN_SPD = 200;

static const uint16_t ESCAPING_MS = 1000;
unsigned long escaping_until_ms = 0;

static const unsigned long EMERGENCY_SPIN_MS = 1200;
static const unsigned long BACK_MS           = 120;
static const unsigned long OSCILLATION_WINDOW_MS = 1200;
static const int   OSCILLATION_COUNT_THRESHOLD = 4;

int last_turn_direction = 0;         
int turn_change_count   = 0;
unsigned long oscillation_timer_start = 0;

unsigned long emergency_back_until = 0;
unsigned long emergency_spin_until = 0;

// ===================== 엔코더 =====================
volatile long left_encoder_count  = 0;
volatile long right_encoder_count = 0;

// Core 3.x에서는 인터럽트 핸들러에 IRAM_ATTR이 필수입니다.
void IRAM_ATTR isr_left_encoder()  { left_encoder_count++; }
void IRAM_ATTR isr_right_encoder() { right_encoder_count++; }

static long  start_left_count  = 0;
static long  start_right_count = 0;
static long  target_turn_pulses = 0;
static long  target_move_pulses = 0;
static float integral_error = 0.0f;

// ===================== IR 센서 캐시 (멀티코어용) =====================
volatile int cached_ir[5] = {0, 0, 0, 0, 0};
volatile bool ir_data_ready = false;
static const unsigned long IR_READ_INTERVAL_MS = 20;

// ===================== 제어 주기 관리 =====================
static const unsigned long CONTROL_INTERVAL_MS = 5;  // 5ms 제어 주기
unsigned long last_control_time = 0;

// ===================== LED 상태 표시 =====================
static const unsigned long LED_UPDATE_INTERVAL_MS = 100;
unsigned long last_led_update = 0;

// ===================== 유틸리티 =====================
static inline void clear_motion_targets() {
  target_turn_pulses = 0;
  target_move_pulses = 0;
  start_left_count  = left_encoder_count;
  start_right_count = right_encoder_count;
  integral_error = 0.0f;
}

static inline void enter_escaping(uint16_t ms = ESCAPING_MS) {
  escaping_until_ms = millis() + ms;
  Motors_forward(FWD_SPD);
  state = STATE_ESCAPING;
}

static inline void start_emergency_left_spin() {
  unsigned long now = millis();
  emergency_back_until = now + BACK_MS;
  emergency_spin_until = emergency_back_until + EMERGENCY_SPIN_MS;

  Motors_backward(FWD_SPD);
  state = STATE_EMERGENCY;

  turn_change_count = 0;
  last_turn_direction = -1;
  oscillation_timer_start = now;

  Serial.println("[EMERGENCY] back -> spin_left");
}

static inline void check_oscillation_and_escape(int current_direction) {
  if (last_turn_direction != 0 && current_direction != last_turn_direction) {
    unsigned long now = millis();
    if (now - oscillation_timer_start > OSCILLATION_WINDOW_MS) {
      turn_change_count = 1;
      oscillation_timer_start = now;
    } else {
      turn_change_count++;
    }

    if (turn_change_count >= OSCILLATION_COUNT_THRESHOLD) {
      start_emergency_left_spin();
      return;
    }
  }
  last_turn_direction = current_direction;
}

// ===================== 개선된 PI 제어 =====================
void pi_control(long l_now, long r_now, int* left_pwm, int* right_pwm) {
  long err = l_now - r_now;
  
  // 데드밴드 적용 - 작은 오차는 무시
  if (abs(err) < ERROR_DEADBAND) {
    err = 0;
  }
  
  // 적분 오차 누적 (제한 적용)
  integral_error += (float)err;
  if (integral_error > INTEGRAL_LIMIT) integral_error = INTEGRAL_LIMIT;
  if (integral_error < -INTEGRAL_LIMIT) integral_error = -INTEGRAL_LIMIT;
  
  float u = (K_P * (float)err) + (K_I * integral_error);
  
  *left_pwm  = constrain((int)lroundf(FWD_SPD - u), MIN_MOTOR_PWM, 255);
  *right_pwm = constrain((int)lroundf(FWD_SPD + u), MIN_MOTOR_PWM, 255);
}

// ===================== LED 상태 표시 =====================
void update_status_led() {
  switch (state) {
    case STATE_IDLE:
      Set_LED(1, 0, 30, 0);   // 녹색: 대기
      Set_LED(2, 0, 30, 0);
      break;
    case STATE_TURNING:
      Set_LED(1, 0, 0, 50);   // 파란색: 회전
      Set_LED(2, 0, 0, 50);
      break;
    case STATE_MOVING:
      Set_LED(1, 0, 50, 0);   // 밝은 녹색: 이동
      Set_LED(2, 0, 50, 0);
      break;
    case STATE_AVOID:
      Set_LED(1, 50, 50, 0);  // 노란색: 회피
      Set_LED(2, 50, 50, 0);
      break;
    case STATE_ESCAPING:
      Set_LED(1, 50, 25, 0);  // 주황색: 탈출
      Set_LED(2, 50, 25, 0);
      break;
    case STATE_EMERGENCY:
      Set_LED(1, 50, 0, 0);   // 빨간색: 비상
      Set_LED(2, 50, 0, 0);
      break;
  }
}

// ===================== Core 0: IR 센서 읽기 태스크 =====================
TaskHandle_t irTaskHandle = NULL;

void ir_sensor_task(void* parameter) {
  while (true) {
    int new_ir[5];
    for (int i = 0; i < 5; i++) {
      new_ir[i] = Get_IR(i + 1);
    }
    
    // 캐시 업데이트 (atomic하게)
    noInterrupts();
    for (int i = 0; i < 5; i++) {
      cached_ir[i] = new_ir[i];
    }
    ir_data_ready = true;
    interrupts();
    
    vTaskDelay(pdMS_TO_TICKS(IR_READ_INTERVAL_MS));
  }
}

// ===================== IR 값 가져오기 =====================
void get_cached_ir(int* r1, int* r2, int* r3, int* r4, int* r5) {
  noInterrupts();
  *r1 = cached_ir[0];
  *r2 = cached_ir[1];
  *r3 = cached_ir[2];
  *r4 = cached_ir[3];
  *r5 = cached_ir[4];
  interrupts();
}

// ===================== 함수 선언 =====================
void start_motion(float angle_deg, float dist_mm);
void handle_udp_packet();
void control_loop(int r1, int r2, int r3, int r4, int r5);

void setup() {
  Serial.begin(115200);
  
  Mona_ESP_init();

  // Core 3.x 대응: 인터럽트 설정
  pinMode(PIN_ENCODER_LEFT, INPUT);
  pinMode(PIN_ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), isr_left_encoder,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isr_right_encoder, RISING);

  // WiFi 연결
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  
  // Core 0에서 IR 센서 읽기 태스크 시작
  xTaskCreatePinnedToCore(
    ir_sensor_task,    // 태스크 함수
    "IR_Task",         // 태스크 이름
    4096,              // 스택 크기
    NULL,              // 파라미터
    1,                 // 우선순위
    &irTaskHandle,     // 태스크 핸들
    0                  // Core 0에서 실행
  );
  
  // 초기 LED 상태
  Set_LED(1, 0, 30, 0);
  Set_LED(2, 0, 30, 0);
  
  Serial.println("MONA ready with dual-core optimization!");
}

void loop() {
  unsigned long now = millis();
  
  // UDP 패킷 처리
  handle_udp_packet();

  // 제어 주기 확인 (5ms 간격)
  if (now - last_control_time >= CONTROL_INTERVAL_MS) {
    last_control_time = now;
    
    // 캐시된 IR 값 사용
    int r1, r2, r3, r4, r5;
    get_cached_ir(&r1, &r2, &r3, &r4, &r5);
    
    control_loop(r1, r2, r3, r4, r5);
  }
  
  // LED 업데이트 (100ms 간격)
  if (now - last_led_update >= LED_UPDATE_INTERVAL_MS) {
    last_led_update = now;
    update_status_led();
  }
}

void start_motion(float angle_deg, float dist_mm) {
  if (state == STATE_AVOID || state == STATE_ESCAPING || state == STATE_EMERGENCY) {
    return;
  }

  start_left_count  = left_encoder_count;
  start_right_count = right_encoder_count;
  integral_error = 0.0f;

  target_turn_pulses = (long)lroundf(fabsf(angle_deg) * PULSES_PER_DEGREE);
  target_move_pulses = (long)lroundf(fabsf(dist_mm)  * PULSES_PER_MM);

  if (target_turn_pulses > 0 && fabsf(angle_deg) > (ROTATION_DEADBAND_DEG)) {
    state = STATE_TURNING;
    if (angle_deg > 0) Motors_spin_right(TURN_SPD);
    else                Motors_spin_left(TURN_SPD);
  } else if (target_move_pulses > 0) {
    state = STATE_MOVING;
  } else {
    state = STATE_IDLE;
    Motors_stop();
  }
}

void handle_udp_packet() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  // 버퍼 플러시
  while (packetSize) {
     int len = udp.read(packetBuffer, 255);
     if (len > 0) packetBuffer[len] = 0;
     packetSize = udp.parsePacket();
  }

  if (strncasecmp(packetBuffer, "STOP", 4) == 0) {
    Motors_stop();
    clear_motion_targets();
    state = STATE_IDLE;
    return;
  }

  if (packetBuffer[0] == 'G' || packetBuffer[0] == 'g') {
    float angle = 0, dist = 0;
    if (sscanf(packetBuffer + 1, "%f %f", &angle, &dist) == 2) {
      if (dist < MIN_DIST_MM) return; 

      if (state == STATE_MOVING) {
        long l_now = labs(left_encoder_count - start_left_count);
        long r_now = labs(right_encoder_count - start_right_count);
        if (((l_now + r_now) / 2) < UPDATE_THRESHOLD_PULSES) return; 
      }
      start_motion(angle, dist);
    }
  }
}

void control_loop(int r1, int r2, int r3, int r4, int r5) {
  bool obstacle = (r1 > TH_OUTER) || (r2 > TH) || (r3 > TH) || (r4 > TH) || (r5 > TH_OUTER);

  if (obstacle) {
    if (state != STATE_AVOID && state != STATE_ESCAPING && state != STATE_EMERGENCY) {
      Motors_stop();
      clear_motion_targets();
      state = STATE_AVOID;
    }
  }

  if (state == STATE_EMERGENCY) {
    unsigned long now = millis();
    if (now < emergency_back_until) {
      Motors_backward(FWD_SPD);
      return;
    }
    if (now < emergency_spin_until) {
      Motors_spin_left(EMERGENCY_SPIN_SPD);
      return;
    }
    Motors_stop();
    state = STATE_IDLE;
    return;
  }

  long l_now = labs(left_encoder_count - start_left_count);
  long r_now = labs(right_encoder_count - start_right_count);
  long avg   = (l_now + r_now) / 2;

  switch (state) {
    case STATE_TURNING:
      if (avg >= target_turn_pulses) {
        Motors_stop();
        if (target_move_pulses > 0) {
          start_left_count  = left_encoder_count;
          start_right_count = right_encoder_count;
          integral_error = 0.0f;
          state = STATE_MOVING;
        } else {
          state = STATE_IDLE;
        }
      }
      break;

    case STATE_MOVING:
      if (avg >= target_move_pulses) {
        Motors_stop();
        state = STATE_IDLE;
      } else {
        // 개선된 PI 제어 사용
        int left_pwm, right_pwm;
        pi_control(l_now, r_now, &left_pwm, &right_pwm);
        Left_mot_forward(left_pwm);
        Right_mot_forward(right_pwm);
      }
      break;

    case STATE_AVOID:
      if ((r1 <= TH_OUTER) && (r2 <= TH) && (r3 <= TH) && (r4 <= TH) && (r5 <= TH_OUTER)) {
        enter_escaping(ESCAPING_MS);
        break;
      }
      if (r3 >= TH) {
        if (abs(r2 - r4) <= DELTA || r2 < r4) {
          Motors_spin_left(TURN_SPD);
          check_oscillation_and_escape(-1);
        } else {
          Motors_spin_right(TURN_SPD);
          check_oscillation_and_escape(+1);
        }
      } else if (r1 >= TH_OUTER || r2 >= TH) {
        Motors_spin_right(TURN_SPD);
        check_oscillation_and_escape(+1);
      } else if (r4 >= TH || r5 >= TH_OUTER) {
        Motors_spin_left(TURN_SPD);
        check_oscillation_and_escape(-1);
      }
      break;

    case STATE_ESCAPING:
      if (obstacle) {
        Motors_stop();
        state = STATE_AVOID;
        break;
      }
      if ((int32_t)(millis() - escaping_until_ms) >= 0) {
        Motors_stop();
        state = STATE_IDLE;
      }
      break;

    default: break;
  }
}
