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

//Enter the SSID and password of the WiFi you are going
//to use to communicate through
const char* ssid = "NetworkForMonaESP";
const char* password =  "WeLoveMONA123";
//A server is started using port 80
WiFiServer wifiServer(80);

// User-defined parameters
const int CMD_SPEED = 150;    // Motor speed for manual commands
const int CMD_TIMEOUT = 2000; // Command timeout in milliseconds

const int SPEED_AVOID_LEFT  = 100; // Motor speed during left-side obstacle avoidance
const int SPEED_AVOID_RIGHT = 100; // Motor speed during right-side obstacle avoidance

// Threshold for determining a neutral forward obstacle condition.
// If the difference between left and right IR sensor readings is smaller than this value,
// the robot treats it as a head-on obstacle (no strong preference for turning left or right).
const int DELTA = 15;



//Variables
bool IR_detected[5] = {false, false, false, false, false};
int IR_value[5] = {0, 0, 0, 0, 0};

//Threshold value used to determine a detection on the IR sensors.
//Reduce the value for a earlier detection, increase it if there
//false detections.
int threshold[5] = {70, 70, 50, 70, 70};
//State Machine Variable
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
int state, old_state;

// Global variables for maintaining the socket connection
WiFiClient client;                // Active client connection
char pending_cmd = 0;             // Last received command waiting to be executed
unsigned long cmd_received_time;  // Timestamp of the most recent command received

// Function declarations
void read_IR_sensor();            // Read IR sensor values and update detection states
void update_state();              // Determine robot state based on IR sensor readings
void avoid_moving();              // Perform obstacle avoidance maneuvers
void read_socket_command();       // Maintain socket connection and read incoming commands
void socket_control(char c);      // Execute motor actions based on received command 

void setup() {
  //Initialize the MonaV2 robot
	Mona_ESP_init();
  //Initialize variables
  state=0;
  old_state=0;    
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
}


void loop() {
  // TCP 세션 유지 위해 keyboard 명령어 항상 읽기 수행 (회피 중에도 수행함)
  read_socket_command();

  // 센서/상태 갱신
  read_IR_sensor();
  update_state();

  bool obs = (state != 0);
  if (obs) {
    collision_avoidance_loop();
  } 
  else if (pending_cmd) {
    socket_control(pending_cmd);  // 시작만 수행(지속/종료는 틱에서)
    pending_cmd = 0;   // 소진
  }

  // 입력 시간이 지났을 경우의 안전정지(네트워크 끊김 대비)
  if (millis() - cmd_received_time > CMD_TIMEOUT) {
    Motors_stop();
  }
  delay(10);
}


void socket_control(char c) {
  if(c=='F'){
    Motors_forward(CMD_SPEED);
  }
  else if(c=='B'){
    Motors_backward(CMD_SPEED);
  }
  else if(c=='R'){
    Motors_spin_right(CMD_SPEED);
  }
  else if(c=='L'){
    Motors_spin_left(CMD_SPEED);
  }
  else {
    Motors_stop();
  }
}

// 소켓 읽기만 수행 -> 통신 연결 유지
void read_socket_command() {
  // 연결 미존재 or 끊겼으면 통신 accept
  if (!client or !client.connected()) {
    //Create a client object
    client = wifiServer.available();
    if (client) {
      client.setNoDelay(true);
    }
  }

  // 연결 존재 시 수신 버퍼 비우기(마지막 명령만 저장)
  if (client && client.connected()) {
    //Read data sent by the client
    while (client.available() > 0) {      
      char c = client.read();
      if (c=='F' or c=='B' or c=='L' or c=='R' or c=='S') {
        cmd_received_time = millis(); 
        pending_cmd = c;
      }
    }
  }
}

void collision_avoidance_loop(){
  //--------------Motors------------------------
  //Set motors movement based on the state machine value.
  if(state == 0){
    // Start moving Forward
    return;
  }
  if(state == 1){
    // 전방 장애물 존재시 회전 방향 결정: r1(센서2)·r3(센서4) 비교로 좌/우 선택 + 과회피 감지
    int r1 = IR_value[1];  // 좌전방
    int r3 = IR_value[3];  // 우전방
    int diff = abs(r1 - r3);

    if (diff <= DELTA) {
      //Spin to the left
      Motors_spin_left(SPEED_AVOID_LEFT);
    } else if (r1 < r3) {
      Motors_spin_left(SPEED_AVOID_LEFT);     // 왼쪽이 더 여유 → 좌
    } else {
      Motors_spin_right(SPEED_AVOID_RIGHT);   // 오른쪽이 더 여유 → 우
    }
  }
  if(state == 2){
    //Spin to the left
    Motors_spin_left(SPEED_AVOID_LEFT);
  }
  if(state == 3){
    //Spin to the right
    Motors_spin_right(SPEED_AVOID_RIGHT);
  }
}

void read_IR_sensor(){
  //--------------IR sensors------------------------
  //Decide future state:
  //Read IR values to determine maze walls

  // IR센서 감지값 위한 각각 센서값 저장
  IR_value[0] = Get_IR(1);
  IR_value[1] = Get_IR(2);
  IR_value[2] = Get_IR(3);
  IR_value[3] = Get_IR(4);
  IR_value[4] = Get_IR(5);

  IR_detected[0] = Detect_object(1,threshold[0]);
  IR_detected[1] = Detect_object(2,threshold[1]);
  IR_detected[2] = Detect_object(3,threshold[2]);
  IR_detected[3] = Detect_object(4,threshold[3]);
  IR_detected[4] = Detect_object(5,threshold[4]);
}

void update_state(){
	//--------------State Machine------------------------
	//Use the retrieved IR values to set state
	//Check for frontal wall, which has priority
	if(IR_detected[1] or IR_detected[2] or IR_detected[3]){
		state=1;
	}
	else if(IR_detected[0]){ //Check for left proximity
		state=3;
	}
	else if(IR_detected[4]){// Check for right proximity
		state=2;
	}
	else{ //If there are no proximities, move forward
		state=0;
	}

  //delay(5);
}
