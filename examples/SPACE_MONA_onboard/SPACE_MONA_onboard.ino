#include <cmath>
#include <set>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Mona_ESP_lib.h"
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <algorithm>

// Behavior Tree library
#include "bt_nodes.h"

// CBBA library (ported from cbba.py)
#include "cbba.h"

// ====== USER CONFIG ======
const char* SSID       = "Your SSID";
const char* PASSWORD   = "Your Password";
const int   SELF_ID    = 0;           // Change this for each robot (0-11)
const uint16_t UDP_PORT = 9000;

// PC Status Report Configuration
const char* PC_HOST = "192.168.0.17";  // PC IP address
const int PC_STATUS_PORT = 9001;       // Port for MONA → PC status reports
const unsigned long STATUS_REPORT_INTERVAL_MS = 200;  // Report every 200ms

// CBBA Configuration (matching cbba.py config)
CBBAConfig cbba_config;

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
const uint32_t MIN_BROADCAST_MS = 100; //통신 안정화를 위해 변경
const uint32_t MAX_BROADCAST_MS = 200;
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
WiFiUDP udp;

// CBBA Message Codec constants
const float BID_SCALE_FACTOR = 100000.0f;
const unsigned long TIMESTAMP_OFFSET = 0UL;

// Received ESP-NOW messages storage
struct ReceivedMessage {
  StaticJsonDocument<1024> msg;
  unsigned long timestamp;
};
std::map<int, ReceivedMessage> receivedMessages_MAP;

// ===== CBBA Communication (Core 0) =====
StaticJsonDocument<JSON_SIZE> selfMessageDoc;
StaticJsonDocument<JSON_SIZE> messageToShare;
bool hasMessageToShare = false;

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
volatile float targetX = 0;
volatile float targetY = 0;
volatile bool hasTargetPosition = false;

// Position from PC
volatile float positionX = 0;
volatile float positionY = 0;
volatile float heading = 0;

volatile long left_encoder_count  = 0;
volatile long right_encoder_count = 0;

// Core 3.x에서는 인터럽트 핸들러에 IRAM_ATTR이 필수입니다.
void IRAM_ATTR isr_left_encoder()  { left_encoder_count++; }
void IRAM_ATTR isr_right_encoder() { right_encoder_count++; }

static long  start_left_count  = 0;
static long  start_right_count = 0;
static long  target_turn_pulses = 0;
static long  target_move_pulses = 0;

TaskHandle_t motionTaskHandle = NULL;

static inline void clear_motion_targets() {
  target_turn_pulses = 0;
  target_move_pulses = 0;
  start_left_count  = left_encoder_count;
  start_right_count = right_encoder_count;
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
// TASK MANAGER - Manages tasks received from PC
// =============================================================================
class TaskManager {
public:
    TaskManager() : _last_update_ms(0) {}

    void update_from_message(const std::vector<std::vector<float>>& task_list) {
        std::set<int> received_ids;

        for (const auto& task_data : task_list) {
            if (task_data.size() < 4) continue;

            int task_id = (int)task_data[0];
            float x = task_data[1];
            float y = task_data[2];
            float amount = task_data[3];

            received_ids.insert(task_id);

            auto it = _tasks.find(task_id);
            if (it != _tasks.end()) {
                it->second.x = x;
                it->second.y = y;
                it->second.amount = amount;
                it->second.completed = (amount <= 0);
            } else {
                _tasks[task_id] = CBBATask(task_id, x, y, amount);
            }
        }

        for (auto& pair : _tasks) {
            if (received_ids.find(pair.first) == received_ids.end()) {
                pair.second.completed = true;
            }
        }

        auto it = _tasks.begin();
        while (it != _tasks.end()) {
            if (it->second.completed) {
                it = _tasks.erase(it);
            } else {
                ++it;
            }
        }

        _last_update_ms = millis();
    }

    CBBATask* get_task(int task_id) {
        auto it = _tasks.find(task_id);
        return (it != _tasks.end()) ? &(it->second) : nullptr;
    }

    std::vector<CBBATask> get_all_tasks() {
        std::vector<CBBATask> result;
        for (auto& pair : _tasks) {
            if (!pair.second.completed) {
                result.push_back(pair.second);
            }
        }
        return result;
    }
    
    std::vector<Task*> get_all_tasks_ptr() {
        _task_ptrs.clear();
        for (auto& pair : _tasks) {
            if (!pair.second.completed) {
                _task_ptrs.push_back(reinterpret_cast<Task*>(&pair.second));
            }
        }
        return _task_ptrs;
    }

    int get_task_count() const {
        int count = 0;
        for (const auto& pair : _tasks) {
            if (!pair.second.completed) count++;
        }
        return count;
    }

    bool is_task_valid(int task_id) const {
        auto it = _tasks.find(task_id);
        return (it != _tasks.end() && !it->second.completed);
    }

    void clear() { _tasks.clear(); }

private:
    std::map<int, CBBATask> _tasks;
    std::vector<Task*> _task_ptrs;
    unsigned long _last_update_ms;
};

TaskManager* taskManager = nullptr;

// =============================================================================
// MESSAGE CODEC - Encodes/Decodes CBBA messages for ESP-NOW
// =============================================================================
class MessageCodec {
public:
    static bool encode(int agent_id, const JsonDocument& message, JsonDocument& output) {
        output.clear();
        if (message.isNull()) return false;
        output["id"] = agent_id;
        JsonObject y_out = output.createNestedObject("y");
        if (message.containsKey("winning_bids")) {
            JsonObjectConst bids = message["winning_bids"].as<JsonObjectConst>();
            for (JsonPairConst kv : bids) {
                float bid = kv.value().as<float>();
                int task_id = atoi(kv.key().c_str());
                y_out[String(task_id)] = (long)(bid * BID_SCALE_FACTOR);
            }
        }
        JsonObject z_out = output.createNestedObject("z");
        if (message.containsKey("winning_agents")) {
            JsonObjectConst agents = message["winning_agents"].as<JsonObjectConst>();
            for (JsonPairConst kv : agents) {
                int winner = kv.value().as<int>();
                int task_id = atoi(kv.key().c_str());
                z_out[String(task_id)] = winner;
            }
        }
        JsonObject s_out = output.createNestedObject("s");
        if (message.containsKey("message_received_time_stamp")) {
            JsonObjectConst timestamps = message["message_received_time_stamp"].as<JsonObjectConst>();
            for (JsonPairConst kv : timestamps) {
                unsigned long ts = kv.value().as<unsigned long>();
                int ag_id = atoi(kv.key().c_str());
                s_out[String(ag_id)] = (long)(ts - TIMESTAMP_OFFSET);
            }
        }
        return true;
    }

    static bool decode(const JsonDocument& payload, JsonDocument& output) {
        output.clear();
        if (!payload.containsKey("y")) return false;
        output["agent_id"] = payload["id"].as<int>();
        JsonObject bids_out = output.createNestedObject("winning_bids");
        if (payload.containsKey("y")) {
            JsonObjectConst y = payload["y"].as<JsonObjectConst>();
            for (JsonPairConst kv : y) {
                long val = kv.value().as<long>();
                bids_out[kv.key()] = (float)val / BID_SCALE_FACTOR;
            }
        }
        JsonObject agents_out = output.createNestedObject("winning_agents");
        if (payload.containsKey("z")) {
            JsonObjectConst z = payload["z"].as<JsonObjectConst>();
            for (JsonPairConst kv : z) {
                int winner = kv.value().as<int>();
                agents_out[kv.key()] = winner;
            }
        }
        JsonObject ts_out = output.createNestedObject("message_received_time_stamp");
        if (payload.containsKey("s")) {
            JsonObjectConst s = payload["s"].as<JsonObjectConst>();
            for (JsonPairConst kv : s) {
                long ts = kv.value().as<long>();
                ts_out[kv.key()] = (unsigned long)(ts + TIMESTAMP_OFFSET);
            }
        }
        return true;
    }
};

// =============================================================================
// ESP-NOW COMMUNICATION (Core 0)
// =============================================================================
void handleEspNowMessage(const uint8_t* data, int len) {
  if (len <= 1) return;
  uint8_t idLen = data[0];
  if (len < 1 + idLen) return;

  char sender_id_str[16];
  memcpy(sender_id_str, data + 1, idLen);
  sender_id_str[idLen] = '\0';
  int senderID = atoi(sender_id_str);
  if (senderID == SELF_ID) return;

  const uint8_t* jsonData = data + 1 + idLen;
  size_t jsonLen = len - 1 - idLen;

  StaticJsonDocument<1024> payload;
  DeserializationError error = deserializeJson(payload, jsonData, jsonLen);
  if (error) return;

  StaticJsonDocument<1024> decoded;
  if (MessageCodec::decode(payload, decoded)) {
    ReceivedMessage& msg = receivedMessages_MAP[senderID];
    msg.msg = decoded;
    msg.timestamp = millis();
    espnow_rx_bytes += (uint32_t)len;
    dirtyNeighbors = true;
  }
}

void onEspNowRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incoming, int len) {
  (void)recv_info;
  handleEspNowMessage(incoming, len);
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
  if (!hasMessageToShare) return;
  
  unsigned long now = millis();
  unsigned long interval = random(MIN_BROADCAST_MS, MAX_BROADCAST_MS + 1);
  if (now - lastBroadcast < interval) return;

  lastBroadcast = now;

  ensureBroadcastPeer();

  // Encode message using MessageCodec
  StaticJsonDocument<1024> encoded;
  if (!MessageCodec::encode(SELF_ID, messageToShare, encoded)) return;

  size_t jsonLen = serializeJson(encoded, g_jsonBuf, sizeof(g_jsonBuf));
  if (jsonLen == 0) return;

  char idStr[16];
  snprintf(idStr, sizeof(idStr), "%d", SELF_ID);
  uint8_t idLen = strlen(idStr);
  size_t total = 1 + (size_t)idLen + jsonLen;

  if ((int)total > ESPNOW_MAX_PAYLOAD) return;

  g_pkt[0] = idLen;
  memcpy(&g_pkt[1], idStr, idLen);
  memcpy(&g_pkt[1 + idLen], g_jsonBuf, jsonLen);

  uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_send(bcast, g_pkt, total);
  espnow_tx_bytes += (uint32_t)total;
}

// =============================================================================
// Get nearby agents (from ESP-NOW messages)
// =============================================================================
std::vector<int> getNearbyAgents() {
  std::vector<int> nearby;
  unsigned long now = millis();
  for (const auto& pair : receivedMessages_MAP) {
    if (now - pair.second.timestamp <= PEER_LINK_DROP_MS) {
      nearby.push_back(pair.first);
    }
  }
  return nearby;
}

// =============================================================================
// UDP COMMUNICATION (Core 0) - Position & Tasks from PC
// =============================================================================
std::vector<std::vector<float>> tasksRaw;

void handleUdpPacket() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  char buffer[2048];
  int len = udp.read(buffer, sizeof(buffer) - 1);
  if (len <= 0) return;
  buffer[len] = '\0';

  StaticJsonDocument<2048> doc;
  DeserializationError error = deserializeJson(doc, buffer, len);
  if (error) return;

  // Check if message is for this robot
  if (doc.containsKey("id")) {
    int msg_id = doc["id"];
    if (msg_id != SELF_ID) return;
  }

  // Update position
  if (doc.containsKey("x") && doc.containsKey("y")) {
    positionX = doc["x"];
    positionY = doc["y"];
    heading = doc.containsKey("yaw") ? doc["yaw"].as<float>() : 0.0f;
  }

  // Update tasks
  if (doc.containsKey("t")) {
    tasksRaw.clear();
    JsonArray tasks = doc["t"];
    for (JsonArray task : tasks) {
      if (task.size() >= 4) {
        std::vector<float> task_data;
        task_data.push_back(task[0]);
        task_data.push_back(task[1]);
        task_data.push_back(task[2]);
        task_data.push_back(task[3]);
        tasksRaw.push_back(task_data);
      }
    }
  }
}

// =============================================================================
// Send CBBA Status to PC
// =============================================================================
unsigned long lastStatusReportMs = 0;

void sendStatusToPC(CBBA* cbba) {
  unsigned long now = millis();
  if (now - lastStatusReportMs < STATUS_REPORT_INTERVAL_MS) return;
  lastStatusReportMs = now;

  StaticJsonDocument<512> doc;
  doc["type"] = "status";
  doc["id"] = SELF_ID;
  doc["assigned"] = cbba->get_assigned_task_id();

  // Bundle (planned tasks)
  JsonArray bundle_arr = doc.createNestedArray("bundle");
  const std::vector<int>& bundle = cbba->get_bundle();
  for (int task_id : bundle) {
    bundle_arr.add(task_id);
  }

  // Nearby agents (ESP-NOW connections)
  JsonArray nearby_arr = doc.createNestedArray("nearby");
  std::vector<int> nearby = getNearbyAgents();
  for (int agent_id : nearby) {
    nearby_arr.add(agent_id);
  }

  // Serialize and send
  char json_buffer[512];
  size_t len = serializeJson(doc, json_buffer, sizeof(json_buffer));
  if (len > 0) {
    udp.beginPacket(PC_HOST, PC_STATUS_PORT);
    udp.write((const uint8_t*)json_buffer, len);
    udp.endPacket();
  }
}

// =============================================================================
// MOTION CONTROL TASK (Core 1) - INDEPENDENT
// =============================================================================
static float normalize_angle(float angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

void compute_motion(float& angle_deg, float& dist_mm) {
  if (!hasTargetPosition) {
    angle_deg = 0;
    dist_mm = 0;
    return;
  }
  float dx = targetX - positionX;
  float dy = targetY - positionY;
  dist_mm = sqrtf(dx * dx + dy * dy);
  float desired_heading = atan2f(dy, dx);
  float angle_diff = normalize_angle(desired_heading - heading);
  angle_deg = angle_diff * 180.0f / M_PI;
}

void start_motion(float angle_deg, float dist_mm) {
  if (state == STATE_AVOID || state == STATE_ESCAPING || state == STATE_EMERGENCY) {
    return;
  }

  start_left_count  = left_encoder_count;
  start_right_count = right_encoder_count;

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
        // Use target-based motion update instead of fixed distance
        if (hasTargetPosition && avg >= UPDATE_THRESHOLD_PULSES) {
          float angle_deg, dist_mm;
          compute_motion(angle_deg, dist_mm);
          if (dist_mm >= MIN_DIST_MM) {
            start_motion(angle_deg, dist_mm);
            return;
          }
        }
        Motors_forward(FWD_SPD);
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

// LED Status indicator
void update_led(RobotState state, bool has_task) {
  if (state == STATE_IDLE) {
    if (has_task) {
      Set_LED(1, 0, 50, 0);
      Set_LED(2, 0, 50, 0);
    } else {
      Set_LED(1, 0, 0, 30);
      Set_LED(2, 0, 0, 30);
    }
  } else if (state == STATE_TURNING) {
    Set_LED(1, 0, 0, 50);
    Set_LED(2, 0, 0, 50);
  } else if (state == STATE_MOVING) {
    Set_LED(1, 0, 50, 0);
    Set_LED(2, 0, 50, 0);
  } else if (state == STATE_AVOID) {
    Set_LED(1, 50, 50, 0);
    Set_LED(2, 50, 50, 0);
  } else if (state == STATE_ESCAPING) {
    Set_LED(1, 50, 25, 0);
    Set_LED(2, 50, 25, 0);
  } else if (state == STATE_EMERGENCY) {
    Set_LED(1, 50, 0, 0);
    Set_LED(2, 50, 0, 0);
  }
}

void motionTask(void* parameter) {
  Serial.println("[Core 1] Motion control task started");
  int led_counter = 0;
  
  for (;;) {
    // Check if target is cleared
    if (!hasTargetPosition) {
      if (state != STATE_AVOID && state != STATE_ESCAPING && state != STATE_EMERGENCY) {
        Motors_stop();
        clear_motion_targets();
        state = STATE_IDLE;
      }
    }

    // Handle new target position
    if (hasTargetPosition) {
      if (state == STATE_IDLE) {
        float angle_deg, dist_mm;
        compute_motion(angle_deg, dist_mm);
        if (dist_mm >= MIN_DIST_MM) {
          start_motion(angle_deg, dist_mm);
        }
      }
    }

    // Read IR sensors (this is slow, but doesn't block Core 0 now!)
    int r1 = Get_IR(1);
    int r2 = Get_IR(2);
    int r3 = Get_IR(3);
    int r4 = Get_IR(4);
    int r5 = Get_IR(5);

    // Run motion control
    control_loop(r1, r2, r3, r4, r5);

    // Update LED status
    led_counter++;
    if (led_counter >= 50) {
      update_led(state, hasTargetPosition);
      led_counter = 0;
    }

    // Motion task runs at ~50Hz (20ms period)
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// =============================================================================
// BEHAVIOR TREE BUILDER
// =============================================================================
BTNode* build_default_tree_with_explore() {
    return TreeBuilder::ReactiveSequence("Root", {
        new GatherLocalInfo(),
        TreeBuilder::ReactiveSequence("MainSequence", {
            TreeBuilder::ReactiveFallback("TaskAssignmentFallback", {
                new AssignTask(),
                new Explore()
            }),
            TreeBuilder::ReactiveFallback("TaskExecutionFallback", {
                new IsTaskCompleted(),
                TreeBuilder::ReactiveSequence("ExecutionSequence", {
                    TreeBuilder::ReactiveFallback("MovementFallback", {
                        new IsArrivedAtTarget(),
                        new MoveToTarget()
                    }),
                    new ExecuteTask()
                })
            })
        })
    });
}

BTNode* build_default_tree_no_explore() {
    return TreeBuilder::ReactiveSequence("Root", {
        new GatherLocalInfo(),
        TreeBuilder::ReactiveSequence("MainSequence", {
            new AssignTask(),
            TreeBuilder::ReactiveFallback("TaskExecutionFallback", {
                new IsTaskCompleted(),
                TreeBuilder::ReactiveSequence("ExecutionSequence", {
                    TreeBuilder::ReactiveFallback("MovementFallback", {
                        new IsArrivedAtTarget(),
                        new MoveToTarget()
                    }),
                    new ExecuteTask()
                })
            })
        })
    });
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
  Serial.printf("[WiFi] Board ID (SELF_ID): %d\n", SELF_ID);

  // 실제 채널 출력 (ESPNOW는 이 채널과 동일해야 함)
  uint8_t primary = 0;
  wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("[WiFi] Channel: %d\n", primary);

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

  // Initialize Mona robot hardware
  Mona_ESP_init();

  // Setup encoders
  pinMode(PIN_ENCODER_LEFT, INPUT);
  pinMode(PIN_ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), isr_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isr_right_encoder, RISING);

  setupNetwork();

  Serial.println("==================================================");
  Serial.printf("MONA Onboard Firmware - Agent %d\n", SELF_ID);
  Serial.println("ESP-NOW v2 + PC Status Report");
  Serial.println("CBBA + Behavior Tree | Onboard Processing");
  Serial.printf("PC Status: %s:%d\n", PC_HOST, PC_STATUS_PORT);
  Serial.println("==================================================");

  // Initialize CBBA configuration
  cbba_config.max_tasks_per_agent = 4;
  cbba_config.lambda = 0.999f;
  cbba_config.winning_bid_cancel = true;
  cbba_config.no_bundle_duration_limit = 5.0f;
  cbba_config.keep_moving_during_convergence = false;
  cbba_config.work_rate = 1.0f;
  cbba_config.max_speed = 0.25f;

  // Initialize Task Manager
  taskManager = new TaskManager();

  // Initialize CBBA
  CBBA* cbba = new CBBA(SELF_ID, cbba_config);

  // Initialize Behavior Tree
  BTContext* bt_ctx = new BTContext(SELF_ID);
  bt_ctx->cbba = cbba;
  BTNode* tree_root = build_default_tree_no_explore();
  BehaviorTree* bt = new BehaviorTree(tree_root);

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

  Serial.println("==================================================");
  Serial.println("System ready!");
  Serial.println("==================================================");

  // Main loop timing
  unsigned long last_bt_tick_ms = 0;
  const unsigned long bt_interval_ms = 100;
  unsigned long last_debug_ms = 0;
  const unsigned long debug_interval_ms = 2000;

  // Main loop runs on Core 0
  for (;;) {
    unsigned long now = millis();

    // WiFi reconnection
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Reconnecting...");
      WiFi.reconnect();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ===== UDP: Receive position & tasks from PC =====
    handleUdpPacket();

    // Update task manager with received tasks
    if (!tasksRaw.empty()) {
      taskManager->update_from_message(tasksRaw);
      bt_ctx->tasks = taskManager->get_all_tasks_ptr();
      
      // Clear completed tasks from CBBA
      int current_task = cbba->get_assigned_task_id();
      if (current_task >= 0 && !taskManager->is_task_valid(current_task)) {
        cbba->clear_task(current_task);
      }
    }

    // Update positions for CBBA and BT context
    bt_ctx->position_x = positionX;
    bt_ctx->position_y = positionY;
    bt_ctx->yaw = heading;
    cbba->set_position(positionX, positionY);

    // ===== ESP-NOW: Process received messages =====
    unsigned long msg_now = millis();
    std::vector<StaticJsonDocument<1024>> received_msgs;
    auto it = receivedMessages_MAP.begin();
    while (it != receivedMessages_MAP.end()) {
      if (msg_now - it->second.timestamp <= PEER_LINK_DROP_MS) {
        received_msgs.push_back(it->second.msg);
        ++it;
      } else {
        it = receivedMessages_MAP.erase(it);
      }
    }
    for (auto& msg : received_msgs) {
      cbba->receive_message_json(msg);
    }

    // ===== ESP-NOW: Broadcast CBBA message =====
    cbba->get_message_to_share_json(messageToShare);
    hasMessageToShare = true;
    broadcastSelfMessageIfDue();

    // ===== UDP: Send status to PC =====
    sendStatusToPC(cbba);

    // ===== Behavior Tree: Tick =====
    if (now - last_bt_tick_ms >= bt_interval_ms) {
      last_bt_tick_ms = now;
      std::vector<CBBATask> cbba_tasks = taskManager->get_all_tasks();
      int assigned_id = cbba->decide(cbba_tasks);
      bt_ctx->assigned_task_id = assigned_id;
      bt->tick(*bt_ctx);

      // Update motion target from BT
      if (bt_ctx->has_target_position) {
        targetX = bt_ctx->target_position_x;
        targetY = bt_ctx->target_position_y;
        hasTargetPosition = true;
      } else {
        hasTargetPosition = false;
      }
    }

    // ===== Debug output =====
    if (now - last_debug_ms >= debug_interval_ms) {
      last_debug_ms = now;
      cbba->print_state();
    }

    // Short delay to prevent watchdog trigger
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}