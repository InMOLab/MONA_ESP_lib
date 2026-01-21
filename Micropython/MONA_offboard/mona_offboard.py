import network
import socket
import time
import json
import random
import _thread
from machine import Pin, reset
import espnow
import gc

from mona_esp_lib import (
    mona_esp_init,
    motors_forward,
    motors_backward,
    motors_spin_left,
    motors_spin_right,
    motors_stop,
    left_mot_forward,
    right_mot_forward,
    get_ir,
    set_led,
)

# ====== USER CONFIG ======
SSID = "Your SSID"
PASSWORD = "Your Password"
SELF_ID = "0"  # 각 로봇마다 변경 (0-11)
TCP_PORT = 8080
UDP_PORT = 8080

# ===================== 상태 정의 =====================
STATE_IDLE = 0
STATE_TURNING = 1
STATE_MOVING = 2
STATE_AVOID = 3
STATE_ESCAPING = 4
STATE_EMERGENCY = 5

# ===================== ESP-NOW 설정 =====================
TOTAL_ROBOTS = 12
MIN_BROADCAST_MS = 50
MAX_BROADCAST_MS = 100
PEER_LINK_DROP_MS = 900
ESPNOW_MAX_PAYLOAD = 250  # MicroPython ESP-NOW 제한

# ===================== 제어 상수 =====================
PULSES_PER_MM = 18.0
PULSES_PER_DEGREE = 12.8
FWD_SPD = 100
TURN_SPD = 100

UPDATE_THRESHOLD_PULSES = 900
MIN_DIST_MM = 40.0

# IR/회피 임계값
TH = 80
TH_OUTER = 100
DELTA = 15

# 엔코더 핀
PIN_ENCODER_LEFT = 35
PIN_ENCODER_RIGHT = 39

# 제어 관련 임계값
ROTATION_DEADBAND_DEG = 5.0

# 비상 동작
EMERGENCY_SPIN_SPD = 200
ESCAPING_MS = 1000
EMERGENCY_SPIN_MS = 1200
BACK_MS = 120
OSCILLATION_WINDOW_MS = 1200
OSCILLATION_COUNT_THRESHOLD = 4

# ===================== 전역 변수 (Core 간 공유) =====================
# 상태
state = STATE_IDLE

# 엔코더 카운트
left_encoder_count = 0
right_encoder_count = 0

# 모션 제어 변수
start_left_count = 0
start_right_count = 0
target_turn_pulses = 0
target_move_pulses = 0
turn_direction = 0

# Core 0 → Core 1 통신
target_angle_deg = 0.0
target_dist_mm = 0.0
new_motion_command = False
stop_requested = False

# 탈출/비상 타이머
escaping_until_ms = 0
emergency_back_until = 0
emergency_spin_until = 0

# 진동 감지
last_turn_direction = 0
turn_change_count = 0
oscillation_timer_start = 0

# ESP-NOW 통신
received_json_map = {}  # {sender_id: json_string}
comm_recv_time_map = {}  # {sender_id: timestamp}
self_message_doc = {}  # 자신의 CBBA 메시지
last_broadcast = 0
dirty_self = False
dirty_neighbors = False

# 네트워크 객체
wlan = None
esp = None
tcp_server = None
tcp_clients = []
udp_socket = None

# Lock for thread safety
map_lock = None
self_msg_lock = None

# 엔코더 핀 객체
encoder_left_pin = None
encoder_right_pin = None


# ===================== 엔코더 인터럽트 =====================
def isr_left_encoder(pin):
    global left_encoder_count
    left_encoder_count += 1


def isr_right_encoder(pin):
    global right_encoder_count
    right_encoder_count += 1


# ===================== 유틸리티 함수 =====================
def clear_motion_targets():
    global target_turn_pulses, target_move_pulses
    global start_left_count, start_right_count
    
    target_turn_pulses = 0
    target_move_pulses = 0
    start_left_count = left_encoder_count
    start_right_count = right_encoder_count


def enter_escaping(ms=ESCAPING_MS):
    global escaping_until_ms, state
    escaping_until_ms = time.ticks_ms() + ms
    motors_forward(FWD_SPD)
    state = STATE_ESCAPING


def start_emergency_left_spin():
    global emergency_back_until, emergency_spin_until, state
    global turn_change_count, last_turn_direction, oscillation_timer_start
    
    now = time.ticks_ms()
    emergency_back_until = now + BACK_MS
    emergency_spin_until = emergency_back_until + EMERGENCY_SPIN_MS
    
    motors_backward(FWD_SPD)
    state = STATE_EMERGENCY
    
    turn_change_count = 0
    last_turn_direction = -1
    oscillation_timer_start = now
    
    print("[EMERGENCY] back -> spin_left")


def check_oscillation_and_escape(current_direction):
    global last_turn_direction, turn_change_count, oscillation_timer_start
    
    if last_turn_direction != 0 and current_direction != last_turn_direction:
        now = time.ticks_ms()
        if time.ticks_diff(now, oscillation_timer_start) > OSCILLATION_WINDOW_MS:
            turn_change_count = 1
            oscillation_timer_start = now
        else:
            turn_change_count += 1
        
        if turn_change_count >= OSCILLATION_COUNT_THRESHOLD:
            start_emergency_left_spin()
            return
    
    last_turn_direction = current_direction


# ===================== 네트워크 초기화 =====================
def reset_wifi():
    """WiFi 완전 리셋"""
    global wlan
    
    # 기존 연결 정리
    try:
        if wlan is not None:
            wlan.disconnect()
            wlan.active(False)
            time.sleep_ms(100)
    except:
        pass
    
    wlan = None
    gc.collect()
    time.sleep_ms(200)


def connect_wifi():
    global wlan
    
    print("[WiFi] Initializing...")
    
    # WiFi 완전 리셋
    reset_wifi()
    
    # 새로 초기화
    wlan = network.WLAN(network.STA_IF)
    
    # 먼저 비활성화 후 활성화
    wlan.active(False)
    time.sleep_ms(100)
    wlan.active(True)
    time.sleep_ms(100)
    
    # 절전 모드 비활성화
    try:
        wlan.config(pm=0)  # Power management off
    except:
        pass  # 일부 펌웨어에서 지원 안 할 수 있음
    
    print(f"[WiFi] Connecting to {SSID}...")
    
    # 연결 시도
    try:
        wlan.connect(SSID, PASSWORD)
    except OSError as e:
        print(f"[WiFi] Connect error: {e}")
        return False
    
    # 연결 대기
    timeout = 40  # 20초
    while not wlan.isconnected() and timeout > 0:
        time.sleep(0.5)
        print(".", end="")
        timeout -= 1
    
    if wlan.isconnected():
        print(f"\n[WiFi] Connected!")
        print(f"[WiFi] IP: {wlan.ifconfig()[0]}")
        print(f"[WiFi] Board ID: {SELF_ID}")
        return True
    else:
        print("\n[WiFi] Connection failed!")
        return False


def setup_espnow():
    global esp, wlan
    
    # ESP-NOW는 WiFi가 활성화된 상태에서 초기화해야 함
    if wlan is None or not wlan.active():
        print("[ESP-NOW] Error: WiFi not active")
        return False
    
    try:
        esp = espnow.ESPNow()
        esp.active(True)
        
        # 브로드캐스트 피어 추가
        broadcast_mac = b'\xff\xff\xff\xff\xff\xff'
        try:
            esp.add_peer(broadcast_mac)
        except OSError:
            pass  # 이미 추가됨
        
        print("[ESP-NOW] Initialized")
        return True
    except Exception as e:
        print(f"[ESP-NOW] Init error: {e}")
        return False


def setup_tcp_server():
    global tcp_server
    
    try:
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind(('0.0.0.0', TCP_PORT))
        tcp_server.listen(5)
        tcp_server.setblocking(False)
        
        print(f"[TCP] Server on port {TCP_PORT}")
        return True
    except Exception as e:
        print(f"[TCP] Setup error: {e}")
        return False


def setup_udp():
    global udp_socket
    
    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(('0.0.0.0', UDP_PORT))
        udp_socket.setblocking(False)
        
        print(f"[UDP] Server on port {UDP_PORT}")
        return True
    except Exception as e:
        print(f"[UDP] Setup error: {e}")
        return False


# ===================== ESP-NOW 통신 =====================
def update_broadcast_recv_json_map(sender_id, json_buf):
    global dirty_neighbors, map_lock
    
    if len(json_buf) < 2:
        return False
    
    # JSON 형식 체크
    if not ((json_buf[0] == '{' and json_buf[-1] == '}') or 
            (json_buf[0] == '[' and json_buf[-1] == ']')):
        return False
    
    if map_lock is None:
        return False
    
    if map_lock.acquire(False):  # Non-blocking
        try:
            received_json_map[sender_id] = json_buf
            comm_recv_time_map[sender_id] = time.ticks_ms()
            dirty_neighbors = True
        finally:
            map_lock.release()
        return True
    
    return False


def handle_espnow_recv():
    """ESP-NOW 메시지 수신 처리"""
    global esp
    
    if esp is None:
        return
    
    try:
        # Non-blocking receive
        host, msg = esp.recv(0)  # timeout=0 for non-blocking
        if msg is None:
            return
        
        if len(msg) <= 1:
            return
        
        # 패킷 파싱: [id_len][id][json...]
        id_len = msg[0]
        if len(msg) < 1 + id_len:
            return
        
        sender_id = msg[1:1+id_len].decode('utf-8')
        if sender_id == SELF_ID:
            return
        
        json_data = msg[1+id_len:].decode('utf-8')
        if len(json_data) > 0:
            update_broadcast_recv_json_map(sender_id, json_data)
    
    except OSError:
        pass  # 데이터 없음
    except Exception as e:
        print(f"[ESP-NOW] Recv error: {e}")


def broadcast_self_message_if_due():
    """자신의 CBBA 메시지 브로드캐스트"""
    global last_broadcast, esp, self_msg_lock
    
    if esp is None:
        return
    
    now = time.ticks_ms()
    interval = random.randint(MIN_BROADCAST_MS, MAX_BROADCAST_MS)
    
    if time.ticks_diff(now, last_broadcast) < interval:
        return
    
    last_broadcast = now
    
    if self_msg_lock is None:
        return
    
    json_str = None
    if self_msg_lock.acquire(False):
        try:
            if self_message_doc:
                json_str = json.dumps(self_message_doc)
        finally:
            self_msg_lock.release()
    
    if json_str is None or len(json_str) == 0:
        return
    
    # 패킷 구성: [id_len][id][json]
    id_bytes = SELF_ID.encode('utf-8')
    id_len = len(id_bytes)
    
    packet = bytes([id_len]) + id_bytes + json_str.encode('utf-8')
    
    if len(packet) > ESPNOW_MAX_PAYLOAD:
        return
    
    try:
        broadcast_mac = b'\xff\xff\xff\xff\xff\xff'
        esp.send(broadcast_mac, packet)
    except Exception as e:
        print(f"[ESP-NOW] Send error: {e}")


# ===================== TCP 통신 =====================
def handle_tcp_clients():
    """TCP 클라이언트 연결 및 데이터 수신"""
    global tcp_server, tcp_clients, self_message_doc, dirty_self, self_msg_lock
    
    if tcp_server is None:
        return
    
    # 새 연결 수락
    try:
        client, addr = tcp_server.accept()
        client.setblocking(False)
        tcp_clients.append(client)
        print(f"[TCP] New client: {addr}")
    except OSError:
        pass  # 새 연결 없음
    
    # 기존 클라이언트 데이터 처리
    for client in tcp_clients[:]:  # 복사본으로 순회
        try:
            data = client.recv(2048)
            if data:
                line = data.decode('utf-8').strip()
                if line and self_msg_lock is not None:
                    if self_msg_lock.acquire(True, 0.01):
                        try:
                            self_message_doc = json.loads(line)
                            dirty_self = True
                        except (json.JSONDecodeError, ValueError):
                            pass
                        finally:
                            self_msg_lock.release()
            else:
                # 연결 종료
                client.close()
                if client in tcp_clients:
                    tcp_clients.remove(client)
        except OSError:
            pass  # 데이터 없음
        except Exception as e:
            print(f"[TCP] Client error: {e}")
            try:
                client.close()
            except:
                pass
            if client in tcp_clients:
                tcp_clients.remove(client)


def send_monitor_to_tcp_clients():
    """모니터링 데이터를 TCP 클라이언트에 전송"""
    global dirty_self, dirty_neighbors, map_lock
    
    if not tcp_clients:
        return
    
    if map_lock is None:
        return
    
    now = time.ticks_ms()
    
    # 스냅샷 생성
    snapshot = {}
    if map_lock.acquire(True, 0.005):
        try:
            for sender_id, json_str in received_json_map.items():
                recv_time = comm_recv_time_map.get(sender_id, 0)
                if time.ticks_diff(now, recv_time) <= PEER_LINK_DROP_MS:
                    snapshot[sender_id] = json_str
        finally:
            map_lock.release()
    
    # JSON 구성
    output = {
        "agent_id": SELF_ID,
        "received_messages": {}
    }
    
    for sender_id, json_str in snapshot.items():
        try:
            output["received_messages"][sender_id] = json.loads(json_str)
        except:
            pass
    
    out_str = json.dumps(output) + "\n"
    out_bytes = out_str.encode('utf-8')
    
    # 모든 클라이언트에 전송
    for client in tcp_clients[:]:
        try:
            client.send(out_bytes)
        except Exception as e:
            print(f"[TCP] Send error: {e}")
            try:
                client.close()
            except:
                pass
            if client in tcp_clients:
                tcp_clients.remove(client)
    
    dirty_self = False
    dirty_neighbors = False


# ===================== UDP 명령 처리 =====================
def handle_udp_packet():
    """UDP 모션 명령 수신"""
    global target_angle_deg, target_dist_mm, new_motion_command, stop_requested
    global udp_socket
    
    if udp_socket is None:
        return
    
    try:
        data, addr = udp_socket.recvfrom(256)
    except OSError:
        return  # 데이터 없음
    
    if not data:
        return
    
    packet = data.decode('utf-8').strip()
    
    # STOP 명령
    if packet.upper().startswith("STOP"):
        stop_requested = True
        return
    
    # G 명령: "G <angle> <distance>"
    if packet.upper().startswith("G"):
        try:
            parts = packet[1:].strip().split()
            if len(parts) >= 2:
                angle = float(parts[0])
                dist = float(parts[1])
                
                if dist >= MIN_DIST_MM:
                    target_angle_deg = angle
                    target_dist_mm = dist
                    new_motion_command = True
        except ValueError:
            pass


# ===================== 모션 제어 =====================
def start_motion(angle_deg, dist_mm):
    """모션 시작"""
    global state, target_turn_pulses, target_move_pulses
    global start_left_count, start_right_count, turn_direction
    
    if state in (STATE_AVOID, STATE_ESCAPING, STATE_EMERGENCY):
        return
    
    start_left_count = left_encoder_count
    start_right_count = right_encoder_count
    
    target_turn_pulses = int(round(abs(angle_deg) * PULSES_PER_DEGREE))
    target_move_pulses = int(round(abs(dist_mm) * PULSES_PER_MM))
    
    if target_turn_pulses > 0 and abs(angle_deg) > ROTATION_DEADBAND_DEG:
        state = STATE_TURNING
        if angle_deg > 0:
            motors_spin_right(TURN_SPD)
            turn_direction = 1
        else:
            motors_spin_left(TURN_SPD)
            turn_direction = -1
    elif target_move_pulses > 0:
        state = STATE_MOVING
        motors_forward(FWD_SPD)  # 즉시 모터 시작
    else:
        state = STATE_IDLE
        motors_stop()


def control_loop(r1, r2, r3, r4, r5):
    """모션 제어 루프"""
    global state, start_left_count, start_right_count
    
    # 장애물 감지
    obstacle = (r1 > TH_OUTER) or (r2 > TH) or (r3 > TH) or (r4 > TH) or (r5 > TH_OUTER)
    
    if obstacle:
        if state not in (STATE_AVOID, STATE_ESCAPING, STATE_EMERGENCY):
            motors_stop()
            clear_motion_targets()
            state = STATE_AVOID
    
    # 비상 상태 처리
    if state == STATE_EMERGENCY:
        now = time.ticks_ms()
        if time.ticks_diff(now, emergency_back_until) < 0:
            motors_backward(FWD_SPD)
            return
        if time.ticks_diff(now, emergency_spin_until) < 0:
            motors_spin_left(EMERGENCY_SPIN_SPD)
            return
        motors_stop()
        state = STATE_IDLE
        return
    
    # 엔코더 값 계산
    l_now = abs(left_encoder_count - start_left_count)
    r_now = abs(right_encoder_count - start_right_count)
    avg = (l_now + r_now) // 2
    
    # 상태별 처리
    if state == STATE_TURNING:
        if avg >= target_turn_pulses:
            motors_stop()
            if target_move_pulses > 0:
                start_left_count = left_encoder_count
                start_right_count = right_encoder_count
                state = STATE_MOVING
                motors_forward(FWD_SPD)  # 즉시 직진 시작!
            else:
                state = STATE_IDLE
    
    elif state == STATE_MOVING:
        if avg >= target_move_pulses:
            motors_stop()
            state = STATE_IDLE
        else:
            # 단순 직진 (PI 제어 제거)
            motors_forward(FWD_SPD)
    
    elif state == STATE_AVOID:
        # 장애물 없으면 탈출
        if (r1 <= TH_OUTER) and (r2 <= TH) and (r3 <= TH) and (r4 <= TH) and (r5 <= TH_OUTER):
            enter_escaping(ESCAPING_MS)
            return
        
        # 회피 동작
        if r3 >= TH:
            if abs(r2 - r4) <= DELTA or r2 < r4:
                motors_spin_left(TURN_SPD)
                check_oscillation_and_escape(-1)
            else:
                motors_spin_right(TURN_SPD)
                check_oscillation_and_escape(1)
        elif r1 >= TH_OUTER or r2 >= TH:
            motors_spin_right(TURN_SPD)
            check_oscillation_and_escape(1)
        elif r4 >= TH or r5 >= TH_OUTER:
            motors_spin_left(TURN_SPD)
            check_oscillation_and_escape(-1)
    
    elif state == STATE_ESCAPING:
        if obstacle:
            motors_stop()
            state = STATE_AVOID
            return
        if time.ticks_diff(time.ticks_ms(), escaping_until_ms) >= 0:
            motors_stop()
            state = STATE_IDLE


# ===================== Core 1: 모션 태스크 =====================
def motion_task():
    """Core 1에서 실행 - 모션 제어"""
    global state, new_motion_command, stop_requested
    global encoder_left_pin, encoder_right_pin
    
    print("[Core 1] Motion control task started")
    
    # 엔코더 인터럽트 설정 (Core 1에서)
    encoder_left_pin = Pin(PIN_ENCODER_LEFT, Pin.IN)
    encoder_right_pin = Pin(PIN_ENCODER_RIGHT, Pin.IN)
    encoder_left_pin.irq(trigger=Pin.IRQ_RISING, handler=isr_left_encoder)
    encoder_right_pin.irq(trigger=Pin.IRQ_RISING, handler=isr_right_encoder)
    
    last_control_time = time.ticks_ms()
    
    while True:
        try:
            # STOP 명령 처리
            if stop_requested:
                motors_stop()
                clear_motion_targets()
                state = STATE_IDLE
                stop_requested = False
            
            # 새 모션 명령 처리
            if new_motion_command:
                if state == STATE_MOVING:
                    l_now = abs(left_encoder_count - start_left_count)
                    r_now = abs(right_encoder_count - start_right_count)
                    if ((l_now + r_now) // 2) >= UPDATE_THRESHOLD_PULSES:
                        start_motion(target_angle_deg, target_dist_mm)
                elif state == STATE_IDLE:
                    start_motion(target_angle_deg, target_dist_mm)
                new_motion_command = False
            
            # IR 센서 읽기
            r1 = get_ir(1)
            r2 = get_ir(2)
            r3 = get_ir(3)
            r4 = get_ir(4)
            r5 = get_ir(5)
            
            # 모션 제어
            control_loop(r1, r2, r3, r4, r5)
            
            # 제어 주기 유지 (약 5ms 목표) - 기존 20ms에서 단축
            elapsed = time.ticks_diff(time.ticks_ms(), last_control_time)
            if elapsed < 5:
                time.sleep_ms(5 - elapsed)
            last_control_time = time.ticks_ms()
        
        except Exception as e:
            print(f"[Core 1] Error: {e}")
            time.sleep_ms(100)


# ===================== LED 상태 표시 =====================
def update_status_led():
    if state == STATE_IDLE:
        set_led(1, 0, 30, 0)
        set_led(2, 0, 30, 0)
    elif state == STATE_TURNING:
        set_led(1, 0, 0, 50)
        set_led(2, 0, 0, 50)
    elif state == STATE_MOVING:
        set_led(1, 0, 50, 0)
        set_led(2, 0, 50, 0)
    elif state == STATE_AVOID:
        set_led(1, 50, 50, 0)
        set_led(2, 50, 50, 0)
    elif state == STATE_ESCAPING:
        set_led(1, 50, 25, 0)
        set_led(2, 50, 25, 0)
    elif state == STATE_EMERGENCY:
        set_led(1, 50, 0, 0)
        set_led(2, 50, 0, 0)


# ===================== 메인 =====================
def main():
    global map_lock, self_msg_lock, wlan
    
    print("=" * 40)
    print("MONA Firmware v2 - MicroPython Dual Core")
    print("(PI Control Removed)")
    print("=" * 40)
    
    # 락 초기화
    map_lock = _thread.allocate_lock()
    self_msg_lock = _thread.allocate_lock()
    
    # WiFi 먼저 초기화 (하드웨어 전에)
    retry_count = 0
    max_retries = 3
    
    while retry_count < max_retries:
        if connect_wifi():
            break
        retry_count += 1
        print(f"[WiFi] Retry {retry_count}/{max_retries}...")
        time.sleep(2)
    
    if not wlan or not wlan.isconnected():
        print("[WiFi] Failed after retries. Restarting...")
        time.sleep(3)
        reset()
    
    # 하드웨어 초기화 (WiFi 연결 후)
    print("Initializing MONA ESP...")
    mona_esp_init()
    
    # ESP-NOW 초기화
    setup_espnow()
    
    # TCP 서버 시작
    setup_tcp_server()
    
    # UDP 서버 시작
    setup_udp()
    
    print("=" * 40)
    print("- Core 0: Communication (TCP/ESP-NOW/UDP)")
    print("- Core 1: Motion Control (IR/Motors)")
    print("- Control Loop: 5ms (200Hz)")
    print("=" * 40)
    
    # Core 1에서 모션 태스크 시작
    _thread.start_new_thread(motion_task, ())
    
    # Core 0에서 통신 태스크 실행 (메인 스레드)
    led_counter = 0
    last_monitor_send = 0
    
    try:
        while True:
            try:
                # WiFi 재연결
                if wlan and not wlan.isconnected():
                    print("[WiFi] Reconnecting...")
                    try:
                        wlan.connect(SSID, PASSWORD)
                    except:
                        pass
                    time.sleep_ms(100)
                    continue
                
                # TCP 클라이언트 처리
                handle_tcp_clients()
                
                # ESP-NOW 수신
                handle_espnow_recv()
                
                # ESP-NOW 브로드캐스트
                broadcast_self_message_if_due()
                
                # TCP 모니터링 전송 (50ms마다)
                now = time.ticks_ms()
                if time.ticks_diff(now, last_monitor_send) >= 50:
                    send_monitor_to_tcp_clients()
                    last_monitor_send = now
                
                # UDP 명령 처리
                handle_udp_packet()
                
                # LED 업데이트 (100회마다)
                led_counter += 1
                if led_counter >= 100:
                    update_status_led()
                    led_counter = 0
                
                time.sleep_ms(1)
            
            except Exception as e:
                print(f"[Main] Error: {e}")
                time.sleep_ms(100)
    
    except KeyboardInterrupt:
        print("\nStopping...")
        motors_stop()
        set_led(1, 0, 0, 0)
        set_led(2, 0, 0, 0)
        if tcp_server:
            tcp_server.close()
        if udp_socket:
            udp_socket.close()
        print("MONA stopped.")


if __name__ == "__main__":
    main()


