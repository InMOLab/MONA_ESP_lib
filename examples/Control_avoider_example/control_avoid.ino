#include <WiFi.h>
#include <Wire.h>
#include "Mona_ESP_lib.h"

// ===== WiFi 설정 =====
const char* ssid = "SSID";
const char* password = "PW";

// ===== 소켓 서버 =====
WiFiServer server(80);
WiFiClient client;

// ===== IR 센서 =====
bool IR_values[5] = {false, false, false, false, false};
int threshold = 60;

// ===== 상태 변수 =====
bool avoidanceMode = false;
char currentCommand = '\0';
unsigned long lastKeyTime = 0;
unsigned long commandTimeout = 300;

void sendAvoidanceStatus(bool state) {
  if (client && client.connected()) {
    if (state) client.println("AVOID_ON");
    else       client.println("AVOID_OFF");
  }
}

void readIRSensors() {
  for (int i = 0; i < 5; i++) {
    IR_values[i] = Detect_object(i + 1, threshold);
  }
}

bool isSafe(char cmd) {
  if (cmd == 'F') return !(IR_values[1] || IR_values[2] || IR_values[3]);
  if (cmd == 'B') return true;
  if (cmd == 'L') return !IR_values[0];
  if (cmd == 'R') return !IR_values[4];
  return true;
}

bool isFrontBlocked() {
  return (IR_values[1] || IR_values[2] || IR_values[3]);
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'F': Motors_forward(150); break;
    case 'B': Motors_backward(150); break;
    case 'L': Motors_spin_left(150); break;
    case 'R': Motors_spin_right(150); break;
    default: Motors_stop(); break;
  }
}

void performAvoidanceLoop() {
  avoidanceMode = true;
  sendAvoidanceStatus(true);

  const int maxAvoidanceAttempts = 10;
  int attempts = 0;

  while (true) {
    readIRSensors();

    // 실시간 키 입력 체크
    if (client.available()) {
      char c = client.read();

      // r 키 → 강제 회피 종료
      if (c == 'r' || c == 'R') {
        Serial.println(" Avoidance canceled by 'r' key inside loop");
        avoidanceMode = false;
        sendAvoidanceStatus(false);
        Motors_stop();
        currentCommand = '\0';        // 키 입력 리셋
        lastKeyTime = 0;              // 정지 상태 전환
        while (client.available()) client.read();
        return;
      }

      // 안전한 명령 → 회피 종료 후 해당 명령 수행
      if (isSafe(c)) {
        Serial.print(" Safe command received during avoidance: ");
        Serial.println(c);
        avoidanceMode = false;
        sendAvoidanceStatus(false);
        executeCommand(c);
        lastKeyTime = millis();
        currentCommand = c;
        while (client.available()) client.read();
        return;
      }

      // 위험한 명령은 무시
    }

    if (!isFrontBlocked()) break;

    if (++attempts > maxAvoidanceAttempts) {
      Serial.println(" Avoidance timeout reached");
      break;
    }

    // 회피 동작
    if (IR_values[4]) {
      Motors_spin_left(100);
    } else if (IR_values[0]) {
      Motors_spin_right(100);
    } else {
      Motors_spin_left(100);
    }

    delay(300);
    Motors_stop();
    delay(100);
  }

  avoidanceMode = false;
  sendAvoidanceStatus(false);
  Serial.println(" Avoidance ended");
}

void setup() {
  Serial.begin(115200);
  Mona_ESP_init();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    return;
  }

  // === 1. 키 입력 수신 ===
  if (client.available()) {
    char cmd = client.read();

    if (avoidanceMode) {
      while (client.available()) client.read(); // 회피 중 loop에서는 무시
      return;
    }

    readIRSensors();
    lastKeyTime = millis();
    currentCommand = cmd;

    if (!isSafe(cmd)) {
      performAvoidanceLoop();

      if (!avoidanceMode) return; // r 키 등으로 정상 종료됐으면 그대로 종료

      currentCommand = '\0';
      lastKeyTime = 0;
      Motors_stop();
      return;
    }

    executeCommand(cmd);
  }

  // === 2. 회피 중이면 loop 중단
  if (avoidanceMode) return;

  // === 3. 입력 없으면 정지
  if (millis() - lastKeyTime > commandTimeout) {
    Motors_stop();
    currentCommand = '\0';
    return;
  }

  // === 4. 기존 명령 반복 수행
  if (currentCommand != '\0') {
    readIRSensors();

    if (isSafe(currentCommand)) {
      executeCommand(currentCommand);
    } else {
      performAvoidanceLoop();

      currentCommand = '\0';
      lastKeyTime = 0;
      Motors_stop();
      return;
    }
  }
}
