/*
  Socket_control.ino -
  Control the Mona ESP through the network using sockets.
  Created by Bart Garcia, January 2021.
  bart.garcia.nathan@gmail.com
  Released into the public domain.
===========================================================
	To use:
	-In this code modify  line 26 and 27, set the ssid and password of
	the network that will be used to control Mona_ESP.
	-Compile and upload the code to Mona_ESP.
	-Open a serial terminal (For example the arduino serial Monitor)
	and check what is the IP given to Mona_ESP
	-Modify in the file 'Control_client.py' the host value, enter
	the IP read from the terminal in the previous step. Save the file
	-Run with python the file 'Control_client.py'
	-Enjoy controlling Mona_ESP through the network.
*/
//Include the Mona_ESP library
#include "Mona_ESP_lib.h"
#include <Wire.h>
#include <WiFi.h>

//Variables
bool IR_values[5] = {false, false, false, false, false};

// IR 센서 측정값 => r1–r3 비교 및 회피 / 회피연속시 긴급회피에 사용
int IR_raw[5] = {0, 0, 0, 0, 0};

//Threshold value used to determine a detection on the IR sensors.
//Reduce the value for a earlier detection, increase it if there
//false detections.
int threshold[5] = {70, 70, 50, 70, 70};
//State Machine Variable
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
int state=0, old_state=0;

//Enter the SSID and password of the WiFi you are going
//to use to communicate through
const char* ssid = "NetworkForMonaESP";
const char* password =  "WeLoveMONA123";
//A server is started using port 80
WiFiServer wifiServer(80);

//Socket 통신 유지 관련 전역 변수
WiFiClient activeClient;         
char pendingCmd = 0;             // 읽어둔 마지막 명령 저장
unsigned long lastClientRX = 0; 

// NonBlocking 명령 실행 상태 
char active_cmd = 0;                 // 현재 실행 중 명령('F','B','L','R')
unsigned long cmd_deadline_ms = 0;   // 명령 종료 시각

// 사용자 지정 파라미터
const unsigned long CMD_FWD_MS  = 1000;  // 필요 시 조정
const unsigned long CMD_BACK_MS = 1000;
const unsigned long CMD_TURN_MS = 500;

const int  CMD_SPEED = 150;   // Motor Speed 조절

const int SPEED_AVOID_LEFT  = 100; // 회피 Motor speed 조절
const int SPEED_AVOID_RIGHT = 100;

// r1–r3 비교 회피의 차이값 DELTA & 회피 연속시 비상 회피 파라미터
const int DELTA = 15;                               // r1–r3 차이값 파라미터
const unsigned long OSCILLATION_WINDOW_MS = 1200;   // 좌↔우 전환 감지 기준: 1.2초
const int OSCILLATION_COUNT_THRESHOLD = 4;          // 전환 횟수 임계값
const unsigned long EMERGENCY_SPIN_MS = 400;        // 비상 좌회전 MS값
const unsigned long BACK_MS = 120;                  // 비상 후진 MS값

// 회피 연속시 비상 회피 상태 변수
bool emergency_active = false;
unsigned long emergency_until = 0;
int last_turn_direction = 0;        // -1=좌, +1=우, 0=없음
int turn_change_count = 0;
unsigned long oscillation_timer_start = 0;

// 함수 선언
void read_IR_sensor();
void update_state();
void avoid_moving();
void socket_read_reconnection();
void socket_control(char c);        
void safe_stop();
void check_oscillation_and_escape(int current_direction);
void start_emergency_left_spin();

void setup() {
  //Initialize the MonaV2 robot
	Mona_ESP_init();
	//Turn LEDs to show that the Wifi connection is not ready
	Set_LED(1,20,0,0);
	Set_LED(2,20,0,0);
  //Initialize serial port
  Serial.begin(115200);
  //Connect to the WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("Connecting to WiFi ");
    Serial.print(ssid);
    Serial.println("....");
  }
  Serial.println("Connected to the WiFi network");
  //Print the IP of the Mona_ESP, which is information
  //needed communicate throught the sockets.
  Serial.println(WiFi.localIP());
  //Start the server as a host
  wifiServer.begin();
  wifiServer.setNoDelay(true);   // 작은 패킷 즉시 전송

	//Blink Leds in green to show end of booting/connecting
	Set_LED(1,0,20,0);
	Set_LED(2,0,20,0);
	delay(500);
	Set_LED(1,0,0,0);
	Set_LED(2,0,0,0);
	delay(500);
	Set_LED(1,0,20,0);
	Set_LED(2,0,20,0);
	delay(500);
	Set_LED(1,0,0,0);
	Set_LED(2,0,0,0);

  //회피 초기설정
  //Initialize variables
  state=0;
  old_state=0;

  pendingCmd = 0;
  lastClientRX = millis();

  // 초기 명령 상태 리셋
  active_cmd = 0;
  cmd_deadline_ms = 0;
}

void loop() {
  // TCP 세션 유지 위해 keyboard 명령어 항상 읽기 수행 (회피 중에도 수행함)
  socket_read_reconnection();

  // 센서/상태 갱신
  read_IR_sensor();
  update_state();

  bool obs = (state != 0);
  static bool was_obs = false;

  if (obs) {
    // 회피 우선: 명령 수행 중이면 즉시 중단
    if (active_cmd) {
      safe_stop();
    }
    // 장애물 O: 회피만 수행 (키 실행은 보류)
    avoid_moving();
  } else {
    // 회피 이후 1회 정지
    if (was_obs) {
      Motors_stop();
      // 회피 종료 시 회피 연속 감지를 위한 횟수 초기화
      last_turn_direction = 0;
      turn_change_count = 0;
      oscillation_timer_start = 0;
    }
    // 장애물 X : 저장된 키를 실행 (socket_read_reconnection에서 저장된 값)
    if (pendingCmd) {
      socket_control(pendingCmd);  // 시작만 수행(지속/종료는 틱에서)
      pendingCmd = 0;   // 소진
    }
  }

  // 입력 시간이 지났을 경우의 안전정지(네트워크 끊김 대비)
  if (millis() - lastClientRX > 2000) {
    safe_stop();
  }

  was_obs = obs;
  delay(10);
}

void socket_control(char c) {
  if(c=='F'){
    active_cmd = 'F';
    cmd_deadline_ms = millis() + CMD_FWD_MS;
    Motors_forward(CMD_SPEED);
  }
  else if(c=='B'){
    active_cmd = 'B';
    cmd_deadline_ms = millis() + CMD_BACK_MS;
    Motors_backward(CMD_SPEED);
  }
  else if(c=='R'){
    active_cmd = 'R';
    cmd_deadline_ms = millis() + CMD_TURN_MS;
    Motors_spin_right(CMD_SPEED);
  }
  else if(c=='L'){
    active_cmd = 'L';
    cmd_deadline_ms = millis() + CMD_TURN_MS;
    Motors_spin_left(CMD_SPEED);
  }
  else {
    safe_stop();
  }
}

// MONA의 상태 리셋 포함
void safe_stop() {
  Motors_stop();
  active_cmd = 0;
  cmd_deadline_ms = 0;
}

// 소켓 읽기만 수행 -> 통신 연결 유지
void socket_read_reconnection() {
  // 연결 미존재 or 끊겼으면 통신 accept
  if (!activeClient or !activeClient.connected()) {
    activeClient = wifiServer.available();
    if (activeClient) {
      activeClient.setNoDelay(true);
      lastClientRX = millis();  
    }
  }

  // 연결 존재 시 수신 버퍼 비우기(마지막 명령만 저장)
  if (activeClient && activeClient.connected()) {
    while (activeClient.available() > 0) {
      char c = activeClient.read();
      if (c=='F' or c=='B' or c=='L' or c=='R' or c=='S') {
        lastClientRX = millis(); 
        if (c=='S') {
          safe_stop();           // 즉시정지 명령 추가
          pendingCmd = 0;
        } else {
          pendingCmd = c;
        }
      }
    }
  }
}

void avoid_moving(){
  //--------------Motors------------------------
  //Set motors movement based on the state machine value.
  static unsigned long turn_until = 0;
  unsigned long now = millis();

  //회피 연속시 비상 좌회전 로직 : 시간 끝날 때까지 강제 좌회전
  if (emergency_active) {
    if (now < emergency_until) {
      Motors_spin_left(SPEED_AVOID_LEFT);
      return;
    } else {
      emergency_active = false; // 커밋 종료 → 일반 회피 로직으로
    }
  }

  if (now >= turn_until) {
    if(state == 0){
      // Start moving Forward -> 원래 정지였지만, 정면 감지시 좌회전 유지로 바꾸었습니다.
      Motors_spin_left(SPEED_AVOID_LEFT);
      turn_until = now + 0;   // 짧게 유지(비블로킹)
      return;
    }
    else if(state == 1){
      // 전방 장애물 존재시 회전 방향 결정: r1(센서2)·r3(센서4) 비교로 좌/우 선택 + 과회피 감지
      int r1 = IR_raw[1];  // 좌전방
      int r3 = IR_raw[3];  // 우전방
      int diff = abs(r1 - r3);

      if (diff <= DELTA) {
        Motors_spin_left(SPEED_AVOID_LEFT);
        check_oscillation_and_escape(-1);
      } else if (r1 < r3) {
        Motors_spin_left(SPEED_AVOID_LEFT);     // 왼쪽이 더 여유 → 좌
        check_oscillation_and_escape(-1);
      } else {
        Motors_spin_right(SPEED_AVOID_RIGHT);   // 오른쪽이 더 여유 → 우
        check_oscillation_and_escape(+1);
      }
      turn_until = now + 0;  
    }
    else if(state == 2){
      //Spin to the left (우측 근접 회피)
      Motors_spin_left(SPEED_AVOID_LEFT);
      check_oscillation_and_escape(-1);         // 과회피 감지 호출
      turn_until = now + 0;
    }
    else if(state == 3){
      //Spin to the right (좌측 근접 회피)
      Motors_spin_right(SPEED_AVOID_RIGHT);
      check_oscillation_and_escape(+1);         // 과회피 진동 감지 호출
      turn_until = now + 0;
    }
  }
}

void read_IR_sensor(){
  //--------------IR sensors------------------------
  //Decide future state:
  //Read IR values to determine maze walls

  // IR센서 감지값 위한 각각 센서값 저장
  IR_raw[0] = Get_IR(1);
  IR_raw[1] = Get_IR(2);
  IR_raw[2] = Get_IR(3);
  IR_raw[3] = Get_IR(4);
  IR_raw[4] = Get_IR(5);

  IR_values[0] = Detect_object(1,threshold[0]);
  IR_values[1] = Detect_object(2,threshold[1]);
  IR_values[2] = Detect_object(3,threshold[2]);
  IR_values[3] = Detect_object(4,threshold[3]);
  IR_values[4] = Detect_object(5,threshold[4]);
}

void update_state(){
	//--------------State Machine------------------------
	//Use the retrieved IR values to set state
	//Check for frontal wall, which has priority
	if(IR_values[2] or IR_values[3] or IR_values[4]){
		state=1;
	}
	else if(IR_values[0]){ //Check for left proximity
		state=3;
	}
	else if(IR_values[4]){// Check for right proximity
		state=2;
	}
	else{ //If there are no proximities, move forward
		state=0;
	}

  //delay(5);
}

// 과회피 감지: 빠른 방향 전환 + 누적 시 비상 좌회전 로직 진입
void check_oscillation_and_escape(int current_direction) {
  if (current_direction == 0) return;

  unsigned long now = millis();
  if (last_turn_direction != 0 && current_direction != last_turn_direction) {
    if (now - oscillation_timer_start > OSCILLATION_WINDOW_MS) {
      turn_change_count = 1;
      oscillation_timer_start = now;
    } else {
      turn_change_count++;
    }
    if (turn_change_count >= OSCILLATION_COUNT_THRESHOLD) {
      Serial.println("\n*** OSCILLATION DETECTED! -> EMERGENCY ESCAPE (LEFT) ***\n");
      start_emergency_left_spin();
      return;
    }
  } else if (last_turn_direction == 0) {
    oscillation_timer_start = now;
    turn_change_count = 0;
  }
  last_turn_direction = current_direction;
}

// 비상 좌회전 로직: 즉시 정지→후진→좌회전 일정 시간 유지
void start_emergency_left_spin() {
  safe_stop();
  Motors_backward(CMD_SPEED);
  delay(BACK_MS);
  Motors_spin_left(CMD_SPEED);
  emergency_active = true;
  emergency_until = millis() + EMERGENCY_SPIN_MS;
  turn_change_count = 0;
  last_turn_direction = -1;
}
