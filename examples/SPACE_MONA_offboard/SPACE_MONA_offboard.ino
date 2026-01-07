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
RobotState state = STATE_IDLE;

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

// PI 제어 게인 (주행 보정)
static const float K_P = 0.95f;  // 비례 게인
static const float K_I = 0.01f;  // 적분 게인

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

// ===================== 엔코더 (volatile 유지) =====================
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

void start_motion(float angle_deg, float dist_mm);
void handle_udp_packet();
void control_loop(int r1, int r2, int r3, int r4, int r5);

void setup() {
  Serial.begin(115200);
  
  // Mona_ESP_init() 내부에 구형 LEDC 코드가 있다면 여기서 충돌이 날 수 있음
  Mona_ESP_init();

  // Core 3.x 대응: 인터럽트 설정
  pinMode(PIN_ENCODER_LEFT, INPUT);
  pinMode(PIN_ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), isr_left_encoder,  RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isr_right_encoder, RISING);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
}

void loop() {
  handle_udp_packet();

  int r1 = Get_IR(1);
  int r2 = Get_IR(2);
  int r3 = Get_IR(3);
  int r4 = Get_IR(4);
  int r5 = Get_IR(5);

  control_loop(r1, r2, r3, r4, r5);

  delay(2);
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
        long err = l_now - r_now;
        integral_error += err;
        float u = (K_P * (float)err) + (K_I * (float)integral_error);
        int left_pwm  = constrain((int)lroundf(FWD_SPD - u), MIN_MOTOR_PWM, 255);
        int right_pwm = constrain((int)lroundf(FWD_SPD + u), MIN_MOTOR_PWM, 255);
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

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ====== USER CONFIG ======
const char* SSID       = "InMOLab";
const char* PASSWORD   = "dlsahfoq104!";
const String SELF_ID   = "11";
const uint16_t SERVER_PORT = 8080;

// JSON 버퍼는 넉넉하게(단, 실제 ESPNOW payload는 1470 제한)
const size_t JSON_SIZE = 2048;
const int TOTAL_ROBOTS = 12;
const uint32_t Min_Broadcast_MS = 50;
const uint32_t Max_Broadcast_MS = 100;
const uint32_t Peer_LinkDrop_MS = 900;
const uint32_t WIFI_RECONNECT_INTERVAL_MS = 300;
const uint32_t WIFI_TIMEOUT_MS = 10000;
const uint32_t WIFI_RETRY_DELAY_MS = 200;
const uint32_t MONITORING_SEND_INTERVAL_MS = 50;
const uint32_t INITIAL_BROADCAST_INTERVAL_MS = 40;
const uint32_t SERIAL_STABILIZE_DELAY_MS = 1000;
// =========================

// ESPNOW v2 payload 최대
static const int ESPNOW_MAX_PAYLOAD = 1470;

WiFiServer server(SERVER_PORT);
std::vector<WiFiClient> clients;

DynamicJsonDocument selfMessageDoc(JSON_SIZE);
std::map<String, DynamicJsonDocument*> receivedJSON_MAP;
std::map<String, unsigned long> CommRecvTime_MAP;

SemaphoreHandle_t mapLock;

unsigned long lastBroadcast = 0;

// 큰 버퍼는 스택이 아니라 전역(static)로 (스택오버플로 방지)
static char g_jsonBuf[JSON_SIZE];

// ESPNOW 송신 패킷 고정 버퍼
static uint8_t g_pkt[ESPNOW_MAX_PAYLOAD];

// -------------------------
// Update map (safe & fast)
// -------------------------
bool update_Broadcast_recv_JSON_MAP(const String& senderID, const char* jsonBuf, size_t jsonLen) {
  DynamicJsonDocument* doc = new DynamicJsonDocument(JSON_SIZE);
  DeserializationError err = deserializeJson(*doc, jsonBuf, jsonLen);
  if (err) {
    delete doc;
    return false;
  }

  // 수신 콜백에서 오래 잠그지 않기: 즉시 락 실패 시 드랍
  if (xSemaphoreTake(mapLock, 0) != pdTRUE) {
    delete doc;
    return false;
  }

  auto it = receivedJSON_MAP.find(senderID);
  if (it != receivedJSON_MAP.end()) {
    delete it->second;
    it->second = doc;
  } else {
    receivedJSON_MAP[senderID] = doc;
  }
  CommRecvTime_MAP[senderID] = millis();

  xSemaphoreGive(mapLock);
  return true;
}

// =====================================================
// Core 3.x 변경: recv callback 시그니처
// =====================================================
void onEspNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incoming, int len) {
  (void)recv_info;

  if (len <= 1) return;

  uint8_t idLen = incoming[0];
  if (len < 1 + idLen) return;

  String senderID = String((const char*)(&incoming[1]), idLen);
  if (senderID == SELF_ID) return;

  int jsonLen = len - (1 + idLen);
  if (jsonLen <= 0) return;

  update_Broadcast_recv_JSON_MAP(senderID,
                                (const char*)(&incoming[1 + idLen]),
                                (size_t)jsonLen);
}

void ensureBroadcastPeer() {
  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  if (esp_now_is_peer_exist(bcast)) return;

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, bcast, 6);

  // Core 3.x에서 권장: 인터페이스/채널 명시
  p.ifidx = WIFI_IF_STA;
  p.channel = WiFi.channel();   // AP 연결된 채널과 동일해야 함
  p.encrypt = false;

  esp_err_t rc = esp_now_add_peer(&p);
  if (rc != ESP_OK && rc != ESP_ERR_ESPNOW_EXIST) {
    //Serial.printf("[ESPNOW] add_peer(bcast) failed: %d\n", (int)rc);
  }
}

void broadcastSelfMessageIfDue() {
  static uint32_t nextInterval = (INITIAL_BROADCAST_INTERVAL_MS);
  if (millis() - lastBroadcast < nextInterval) return;

  lastBroadcast = millis();
  nextInterval = random(Min_Broadcast_MS, Max_Broadcast_MS);

  if (selfMessageDoc.isNull()) return;

  ensureBroadcastPeer();

  // JSON 직렬화
  size_t jsonLen = serializeJson(selfMessageDoc, g_jsonBuf, sizeof(g_jsonBuf));
  if (jsonLen == 0) return;

  const uint8_t idLen = (uint8_t)SELF_ID.length();
  const size_t total = 1 + (size_t)idLen + jsonLen;

  // v2 payload 한도(1470) 기준으로 체크
  if ((int)total > ESPNOW_MAX_PAYLOAD) {
    return;
  }

  // 고정 버퍼에 패킷 구성 [idLen][id][json]
  g_pkt[0] = idLen;
  memcpy(&g_pkt[1], SELF_ID.c_str(), idLen);
  memcpy(&g_pkt[1 + idLen], g_jsonBuf, jsonLen);

  uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_err_t rc = esp_now_send(bcast, g_pkt, total);
  if (rc != ESP_OK) {
    //Serial.printf("[ESP-NOW TX] send failed: %d\n", (int)rc);
  }
}

void setupNetwork() {
  WiFi.mode(WIFI_STA);

  // 절전/모뎀슬립 비활성화 + 자동 재연결
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  WiFi.begin(SSID, PASSWORD);

  Serial.print("[WiFi] Connecting to AP");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(WIFI_RECONNECT_INTERVAL_MS);
    Serial.print(".");
    if (millis() - t0 > (WIFI_TIMEOUT_MS)) {
      Serial.println("\n[WiFi] connect timeout -> retry");
      WiFi.disconnect(true, true);
      delay(WIFI_RETRY_DELAY_MS);
      WiFi.begin(SSID, PASSWORD);
      t0 = millis();
      Serial.print("[WiFi] Connecting to AP");
    }
  }

  Serial.println("\n[WiFi] Connected!");
  Serial.printf("[WiFi] IP Address: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("[WiFi] Board ID (SELF_ID): %s\n", SELF_ID.c_str());

  // 실제 채널 출력 (ESPNOW는 이 채널과 동일해야 함)
  uint8_t primary = 0;
  wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("[WiFi] Channel: %d\n", primary);

  server.begin();

  // ESPNOW init (레거시 C API)
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] init failed -> restart");
    ESP.restart();
  }

  // Core 3.x 콜백 등록
  esp_now_register_recv_cb(onEspNowRecv);

  ensureBroadcastPeer();

  Serial.printf("[ESPNOW] Ready. Using max payload=%d (v2 confirmed in your environment)\n", ESPNOW_MAX_PAYLOAD);
}

void setup() {
  Serial.begin(115200);
  delay(SERIAL_STABILIZE_DELAY_MS);

  randomSeed(esp_random());
  mapLock = xSemaphoreCreateMutex();

  setupNetwork();
}

void loop() {
  // Wi-Fi 끊김 복구
  if (WiFi.status() != WL_CONNECTED) {
    for (auto &c : clients) c.stop();
    clients.clear();
    WiFi.reconnect();
    delay(10);
    return;
  }

  // 새 클라이언트 수락 (끊긴 슬롯 재사용)
  WiFiClient newcomer = server.available();
  if (newcomer) {
    newcomer.setTimeout(10);
    newcomer.setNoDelay(true);

    bool placed = false;
    for (auto &c : clients) {
      if (!c || !c.connected()) {
        c.stop();
        c = newcomer;
        placed = true;
        break;
      }
    }
    if (!placed) clients.push_back(newcomer);
  }

  // TCP 데이터 수신 (PC -> ESP32)
  for (auto &c : clients) {
    if (c && c.connected() && c.available()) {
      String line = c.readStringUntil('\n');
      (void)deserializeJson(selfMessageDoc, line); // 실패해도 조용히 무시(필요하면 로그 추가)
    }
  }

  // ESPNOW 브로드캐스트
  broadcastSelfMessageIfDue();

  // TCP 모니터 송신
  static unsigned long lastTcpSend = 0;
  if (millis() - lastTcpSend > (MONITORING_SEND_INTERVAL_MS)) {
    DynamicJsonDocument monitor(JSON_SIZE * TOTAL_ROBOTS);
    monitor["agent_id"] = SELF_ID;
    JsonObject rx = monitor.createNestedObject("received_messages");

    unsigned long now = millis();
    xSemaphoreTake(mapLock, portMAX_DELAY);
    for (auto const &kv : receivedJSON_MAP) {
      if (now - CommRecvTime_MAP[kv.first] <= Peer_LinkDrop_MS) {
        rx[kv.first] = kv.second->as<JsonObject>();
      }
    }
    xSemaphoreGive(mapLock);

    String out;
    serializeJson(monitor, out);
    out += "\n";

    for (auto &c : clients) {
      if (c && c.connected()) {
        c.print(out);
      }
    }

    lastTcpSend = millis();
  }

  // 끊긴 클라이언트 정리
  for (auto &c : clients) {
    if (c && !c.connected()) c.stop();
  }
  clients.erase(std::remove_if(clients.begin(), clients.end(),
    [](WiFiClient& c){ return !c.connected(); }), clients.end());

  delay(1);
}
