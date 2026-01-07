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
