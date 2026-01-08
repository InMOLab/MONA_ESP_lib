#include <WiFi.h>
#include <WiFiUdp.h>
#include "Mona_ESP_lib.h"
#include <math.h>
#include <strings.h>
#include <Arduino.h>
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
#include <freertos/task.h>

// ====== USER CONFIG ======
const char* SSID       = "Your SSID";
const char* PASSWORD   = "Your Password";
const String SELF_ID   = "11";           // Change this for each robot (0-11)
const uint16_t TCP_PORT = 8080;
const uint16_t UDP_PORT = 8080;

// JSON buffer size
const size_t JSON_SIZE = 2048;

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

// ESP-NOW settings
const int TOTAL_ROBOTS = 12;
const uint32_t MIN_BROADCAST_MS = 50;
const uint32_t MAX_BROADCAST_MS = 100;
const uint32_t PEER_LINK_DROP_MS = 900;
const uint32_t WIFI_RECONNECT_INTERVAL_MS = 300;
const uint32_t WIFI_TIMEOUT_MS = 10000;
const uint32_t WIFI_RETRY_DELAY_MS = 200;
const uint32_t MONITORING_SEND_INTERVAL_MS = 50;
const uint32_t INITIAL_BROADCAST_INTERVAL_MS = 40;
const uint32_t SERIAL_STABILIZE_DELAY_MS = 1000;

static const int ESPNOW_MAX_PAYLOAD = 1470;


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

// ===== Network (Core 0) =====
WiFiServer tcpServer(TCP_PORT);
WiFiUDP udp;
std::vector<WiFiClient> tcpClients;

// ===== CBBA Communication (Core 0) =====
DynamicJsonDocument selfMessageDoc(JSON_SIZE);
std::map<String, String> receivedJSON_MAP;
std::map<String, unsigned long> CommRecvTime_MAP;
SemaphoreHandle_t mapLock;
SemaphoreHandle_t selfMsgLock;  // NEW: selfMessageDoc 보호

unsigned long lastBroadcast = 0;
volatile bool dirtySelf = false;
volatile bool dirtyNeighbors = false;

// ===== ESP-NOW counters =====
volatile uint32_t espnow_rx_bytes = 0;
volatile uint32_t espnow_tx_bytes = 0;

// Global JSON buffer
static char g_jsonBuf[JSON_SIZE];
static uint8_t g_pkt[ESPNOW_MAX_PAYLOAD];

// ===== Motion Control (Core 1) - Atomic variables for cross-core sharing =====
volatile float targetAngleDeg = 0;
volatile float targetDistMm = 0;
volatile bool newMotionCommand = false;  // Core 0 → Core 1 signal
volatile bool stopRequested = false;

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

TaskHandle_t commTaskHandle = NULL;
TaskHandle_t motionTaskHandle = NULL;

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

// =============================================================================
// ESP-NOW COMMUNICATION (Core 0)
// =============================================================================
bool update_Broadcast_recv_JSON_MAP(const String& senderID, const char* jsonBuf, size_t jsonLen) {
  if (jsonLen < 2) return false;

  // 콜백에서 풀 파싱은 부담 큼 -> 아주 가벼운 형태 체크만
  const char first = jsonBuf[0];
  const char last  = jsonBuf[jsonLen - 1];
  if (!((first == '{' && last == '}') || (first == '[' && last == ']'))) {
    return false;
  }

  // 수신 콜백에서 오래 잠그지 않기: 즉시 락 실패 시 드랍
  if (xSemaphoreTake(mapLock, 0) != pdTRUE) {
    return false;
  }

  // 기존 String capacity 재사용(힙 단편화 완화)
  String& dst = receivedJSON_MAP[senderID];
  dst.remove(0);
  dst.reserve(jsonLen + 1);
  dst.concat(jsonBuf, jsonLen);

  CommRecvTime_MAP[senderID] = millis();
  xSemaphoreGive(mapLock);
  return true;
}

void onEspNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incoming, int len) {
  (void)recv_info;
  if (len <= 1) return;

  uint8_t idLen = incoming[0];
  if (len < 1 + idLen) return;

  String senderID = String((const char*)(&incoming[1]), idLen);
  if (senderID == SELF_ID) return;

  int jsonLen = len - (1 + idLen);
  if (jsonLen <= 0) return;

  if (update_Broadcast_recv_JSON_MAP(senderID, (const char*)(&incoming[1 + idLen]), (size_t)jsonLen)) {
    espnow_rx_bytes += (uint32_t)len;
    dirtyNeighbors = true;
  }
}

void ensureBroadcastPeer() {
  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (esp_now_is_peer_exist(bcast)) return;

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, bcast, 6);
  p.ifidx = WIFI_IF_STA;
  p.channel = WiFi.channel();
  p.encrypt = false;

  esp_now_add_peer(&p);
}

void broadcastSelfMessageIfDue() {
  static uint32_t nextInterval = 40;
  if (millis() - lastBroadcast < nextInterval) return;

  lastBroadcast = millis();
  nextInterval = random(MIN_BROADCAST_MS, MAX_BROADCAST_MS);

  // Lock selfMessageDoc for reading
  if (xSemaphoreTake(selfMsgLock, pdMS_TO_TICKS(5)) != pdTRUE) return;
  
  if (selfMessageDoc.isNull() || selfMessageDoc.size() == 0) {
    xSemaphoreGive(selfMsgLock);
    return;
  }

  ensureBroadcastPeer();

  size_t jsonLen = serializeJson(selfMessageDoc, g_jsonBuf, sizeof(g_jsonBuf));
  xSemaphoreGive(selfMsgLock);
  
  if (jsonLen == 0) return;

  uint8_t idLen = (uint8_t)SELF_ID.length();
  size_t total = 1 + (size_t)idLen + jsonLen;

  if ((int)total > ESPNOW_MAX_PAYLOAD) return;

  g_pkt[0] = idLen;
  memcpy(&g_pkt[1], SELF_ID.c_str(), idLen);
  memcpy(&g_pkt[1 + idLen], g_jsonBuf, jsonLen);

  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(bcast, g_pkt, total);
  espnow_tx_bytes += (uint32_t)total;
}

// =============================================================================
// TCP COMMUNICATION (Core 0)
// =============================================================================
void handleTcpClients() {
  WiFiClient newcomer = tcpServer.available();
  if (newcomer) {
    newcomer.setTimeout(10);
    newcomer.setNoDelay(true);

    bool placed = false;
    for (auto &c : tcpClients) {
      if (!c || !c.connected()) {
        c.stop();
        c = newcomer;
        placed = true;
        break;
      }
    }
    if (!placed) tcpClients.push_back(newcomer);
  }

  for (auto &c : tcpClients) {
    if (c && c.connected() && c.available()) {
      String line = c.readStringUntil('\n');
      
      // Lock selfMessageDoc for writing
      if (xSemaphoreTake(selfMsgLock, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (deserializeJson(selfMessageDoc, line) == DeserializationError::Ok) {
          dirtySelf = true;
        }
        xSemaphoreGive(selfMsgLock);
      }
    }
  }
}

void sendMonitorToTcpClients() {
  static unsigned long lastTcpSend = 0;
  if (millis() - lastTcpSend < 50) return;

  // [FIX #1] 큰 DynamicJsonDocument 제거:
  // mapLock을 짧게 잡고 스냅샷만 뜬 뒤, 락 풀고 String으로 한 줄 JSON 구성
  std::vector<std::pair<String, String>> snapshot;
  snapshot.reserve(16);

  const unsigned long now = millis();

  if (xSemaphoreTake(mapLock, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (auto const &kv : receivedJSON_MAP) {
      auto itT = CommRecvTime_MAP.find(kv.first);
      if (itT != CommRecvTime_MAP.end() && (now - itT->second <= PEER_LINK_DROP_MS)) {
        snapshot.push_back({kv.first, kv.second}); // (senderID, raw json)
      }
    }
    xSemaphoreGive(mapLock);
  }

  // 한 줄 JSON: {"agent_id":"11","received_messages":{"03":{...},"04":{...}}}
  size_t est = 64;
  for (auto &kv : snapshot) est += 6 + kv.first.length() + kv.second.length();

  String out;
  out.reserve(est);
  out += "{\"agent_id\":\"";
  out += SELF_ID;
  out += "\",\"received_messages\":{";

  bool first = true;
  for (auto &kv : snapshot) {
    if (!first) out += ",";
    first = false;

    out += "\"";
    out += kv.first;
    out += "\":";
    out += kv.second; // raw JSON object/array
  }

  out += "}}\n";

  for (auto &c : tcpClients) {
    if (c && c.connected()) {
      c.print(out);
    }
  }

  dirtySelf = dirtyNeighbors = false;
  lastTcpSend = millis();

  // Cleanup disconnected clients
  for (auto &c : tcpClients) {
    if (c && !c.connected()) c.stop();
  }
  tcpClients.erase(std::remove_if(tcpClients.begin(), tcpClients.end(),
    [](WiFiClient& c) { return !c.connected(); }), tcpClients.end());
}

// =============================================================================
// UDP MOTION COMMANDS (Core 0 receives, Core 1 executes)
// =============================================================================
void handleUdpPacket() {
  static char udpBuffer[256];
  
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  // Keep only latest packet
  while (packetSize) {
    int len = udp.read(udpBuffer, 255);
    if (len > 0) udpBuffer[len] = 0;
    packetSize = udp.parsePacket();
  }

  // STOP command
  if (strncasecmp(udpBuffer, "STOP", 4) == 0) {
    stopRequested = true;
    return;
  }

  // G command: "G <angle> <distance>"
  if (udpBuffer[0] == 'G' || udpBuffer[0] == 'g') {
    float angle = 0, dist = 0;
    if (sscanf(udpBuffer + 1, "%f %f", &angle, &dist) == 2) {
      if (dist >= MIN_DIST_MM) {
        targetAngleDeg = angle;
        targetDistMm = dist;
        newMotionCommand = true;  // Signal to Core 1
      }
    }
  }
}

// =============================================================================
// COMMUNICATION TASK (Core 0) - HIGH PRIORITY
// =============================================================================
void commTask(void* parameter) {
  Serial.println("[Core 0] Communication task started");
  
  for (;;) {
    // WiFi reconnection
    if (WiFi.status() != WL_CONNECTED) {
      for (auto &c : tcpClients) c.stop();
      tcpClients.clear();
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ===== All communication handling =====
    handleTcpClients();           // TCP: PC <-> Robot
    broadcastSelfMessageIfDue();  // ESP-NOW: Robot <-> Robot
    sendMonitorToTcpClients();    // Send monitor to PC
    handleUdpPacket();            // UDP: Motion commands

    // Short delay to prevent watchdog trigger
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =============================================================================
// MOTION CONTROL TASK (Core 1) - INDEPENDENT
// =============================================================================
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

void motionTask(void* parameter) {
  Serial.println("[Core 1] Motion control task started");
  
  for (;;) {
    // Check for STOP command from Core 0
    if (stopRequested) {
      Motors_stop();
      clear_motion_targets();
      state = STATE_IDLE;
      stopRequested = false;
    }

    // Check for new motion command from Core 0
    if (newMotionCommand) {
      // Only accept if not in obstacle avoidance
      if (state == STATE_MOVING) {
        long l_now = labs(left_encoder_count - start_left_count);
        long r_now = labs(right_encoder_count - start_right_count);
        if (((l_now + r_now) / 2) >= UPDATE_THRESHOLD_PULSES) {
          start_motion(targetAngleDeg, targetDistMm);
        }
      } else if (state == STATE_IDLE || state == STATE_TURNING) {
        start_motion(targetAngleDeg, targetDistMm);
      }
      newMotionCommand = false;
    }

    // Read IR sensors (this is slow, but doesn't block Core 0 now!)
    int r1 = Get_IR(1);
    int r2 = Get_IR(2);
    int r3 = Get_IR(3);
    int r4 = Get_IR(4);
    int r5 = Get_IR(5);

    // Run motion control
    control_loop(r1, r2, r3, r4, r5);

    // Motion task runs at ~50Hz (20ms period)
    vTaskDelay(pdMS_TO_TICKS(50));
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

  // Start TCP server
  tcpServer.begin();
  Serial.printf("[TCP] Server on port %d\n", TCP_PORT);

  // Start UDP
  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Server on port %d\n", UDP_PORT);

  // Initialize ESP-NOW
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
  selfMsgLock = xSemaphoreCreateMutex();

  // Initialize Mona robot hardware
  Mona_ESP_init();

  // Setup encoders
  pinMode(PIN_ENCODER_LEFT, INPUT);
  pinMode(PIN_ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), isr_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isr_right_encoder, RISING);

  setupNetwork();

  Serial.println("=================================");
  Serial.println("MONA Firmware v2 - Dual Core");
  Serial.println("- Core 0: Communication (TCP/ESP-NOW/UDP)");
  Serial.println("- Core 1: Motion Control (IR/Motors)");
  Serial.println("=================================");

  // Create Communication Task on Core 0 (PRO_CPU)
  // Higher priority (2) than default Arduino loop
  xTaskCreatePinnedToCore(
    commTask,           // Task function
    "CommTask",         // Name
    8192,               // Stack size
    NULL,               // Parameters
    2,                  // Priority (higher = more important)
    &commTaskHandle,    // Task handle
    0                   // Core 0
  );

  // Create Motion Task on Core 1 (APP_CPU)
  // Lower priority (1), independent from communication
  xTaskCreatePinnedToCore(
    motionTask,         // Task function
    "MotionTask",       // Name
    4096,               // Stack size
    NULL,               // Parameters
    1,                  // Priority
    &motionTaskHandle,  // Task handle
    1                   // Core 1
  );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}