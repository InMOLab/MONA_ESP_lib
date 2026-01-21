import network
import socket
import time
from machine import Pin
import _thread

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

# ===================== WiFi / UDP 설정 =====================
SSID = "InMOLab"
PASSWORD = "dlsahfoq104!"
LOCAL_PORT = 8080

# ===================== 상태 정의 =====================
STATE_IDLE = 0
STATE_TURNING = 1
STATE_MOVING = 2
STATE_AVOID = 3
STATE_ESCAPING = 4
STATE_EMERGENCY = 5

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
MIN_MOTOR_PWM = 60

# PI 제어 게인
K_P = 0.5
K_I = 0.002

# 적분 제한 추가
INTEGRAL_LIMIT = 500  # 적분 오차 상한

# 비상 동작
EMERGENCY_SPIN_SPD = 200
ESCAPING_MS = 1000
EMERGENCY_SPIN_MS = 1200
BACK_MS = 120
OSCILLATION_WINDOW_MS = 1200
OSCILLATION_COUNT_THRESHOLD = 4

# ===================== 전역 변수 =====================
state = STATE_IDLE

# 엔코더 카운트
left_encoder_count = 0
right_encoder_count = 0
_encoder_lock = _thread.allocate_lock()  # 스레드 안전성

# IR 센서 값
ir_values = [0, 0, 0, 0, 0]
_ir_lock = _thread.allocate_lock()

# 모션 제어 변수
start_left_count = 0
start_right_count = 0
target_turn_pulses = 0
target_move_pulses = 0
integral_error = 0.0
turn_direction = 0

# 탈출/비상 타이머
escaping_until_ms = 0
emergency_back_until = 0
emergency_spin_until = 0

# 진동 감지
last_turn_direction = 0
turn_change_count = 0
oscillation_timer_start = 0

# UDP 소켓
udp_socket = None

# 엔코더 핀 객체
encoder_left_pin = None
encoder_right_pin = None

# 제어 루프 실행 플래그
running = True


# ===================== 엔코더 인터럽트 (최적화) =====================
def isr_left_encoder(pin):
    global left_encoder_count
    left_encoder_count += 1


def isr_right_encoder(pin):
    global right_encoder_count
    right_encoder_count += 1


def get_encoder_counts():
    """스레드 안전하게 엔코더 값 읽기"""
    return left_encoder_count, right_encoder_count


# ===================== 유틸리티 함수 =====================
def clear_motion_targets():
    global target_turn_pulses, target_move_pulses
    global start_left_count, start_right_count, integral_error
    
    target_turn_pulses = 0
    target_move_pulses = 0
    l, r = get_encoder_counts()
    start_left_count = l
    start_right_count = r
    integral_error = 0.0


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


def constrain(val, min_val, max_val):
    return max(min_val, min(max_val, val))


# ===================== WiFi 연결 =====================
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print(f"Connecting to {SSID}...")
        wlan.connect(SSID, PASSWORD)
        
        timeout = 20
        while not wlan.isconnected() and timeout > 0:
            time.sleep(0.5)
            print(".", end="")
            timeout -= 1
        
        if wlan.isconnected():
            print(f"\nWiFi connected!")
            print(f"IP: {wlan.ifconfig()[0]}")
        else:
            print("\nWiFi connection failed!")
            return False
    
    return True


def setup_udp():
    global udp_socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', LOCAL_PORT))
    udp_socket.setblocking(False)
    print(f"UDP listening on port {LOCAL_PORT}")


# ===================== 모션 제어 =====================
def start_motion(angle_deg, dist_mm):
    global state, target_turn_pulses, target_move_pulses
    global start_left_count, start_right_count, integral_error, turn_direction
    
    if state in (STATE_AVOID, STATE_ESCAPING, STATE_EMERGENCY):
        return
    
    l, r = get_encoder_counts()
    start_left_count = l
    start_right_count = r
    integral_error = 0.0
    
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
    else:
        state = STATE_IDLE
        motors_stop()


# ===================== UDP 패킷 처리 =====================
def handle_udp_packet():
    global state
    
    last_packet = None
    while True:
        try:
            data, addr = udp_socket.recvfrom(255)
            if data:
                last_packet = data
        except OSError:
            break
    
    if last_packet is None:
        return
    
    packet = last_packet.decode('utf-8').strip()
    print(f"Received: {packet}")
    
    # STOP 명령
    if packet.upper().startswith("STOP"):
        motors_stop()
        clear_motion_targets()
        state = STATE_IDLE
        return
    
    # G<angle> <distance> 명령
    if packet.upper().startswith("G"):
        try:
            parts = packet[1:].strip().split()
            if len(parts) >= 2:
                angle = float(parts[0])
                dist = float(parts[1])
                
                if dist < MIN_DIST_MM:
                    return
                
                # 이동 중이면 일정 거리 이상 이동 후에만 새 명령 수락
                if state == STATE_MOVING:
                    l, r = get_encoder_counts()
                    l_now = abs(l - start_left_count)
                    r_now = abs(r - start_right_count)
                    if ((l_now + r_now) // 2) < UPDATE_THRESHOLD_PULSES:
                        return
                
                start_motion(angle, dist)
        except ValueError:
            print("Invalid G command format")


# ===================== IR 센서 백그라운드 읽기 (Core 1) =====================
def ir_sensor_thread():
    """별도 스레드에서 IR 센서를 읽어 메인 루프 속도 유지"""
    global ir_values, running
    
    while running:
        new_values = [0, 0, 0, 0, 0]
        for i in range(5):
            new_values[i] = get_ir(i + 1)
        
        with _ir_lock:
            for i in range(5):
                ir_values[i] = new_values[i]
        
        time.sleep_ms(20)  # IR 읽기 주기


def get_ir_values():
    """스레드 안전하게 IR 값 읽기"""
    with _ir_lock:
        return ir_values.copy()


# ===================== PI 제어 (개선) =====================
def pi_control(l_now, r_now):
    """개선된 PI 제어 - 적분 제한 및 데드밴드 적용"""
    global integral_error
    
    err = l_now - r_now
    
    # 적분 오차 누적 (제한 적용)
    integral_error += err
    integral_error = constrain(integral_error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
    
    # 작은 오차는 무시 (데드밴드)
    if abs(err) < 5:
        err = 0
    
    u = (K_P * err) + (K_I * integral_error)
    
    left_pwm = constrain(int(round(FWD_SPD - u)), MIN_MOTOR_PWM, 255)
    right_pwm = constrain(int(round(FWD_SPD + u)), MIN_MOTOR_PWM, 255)
    
    return left_pwm, right_pwm


# ===================== 제어 루프 =====================
def control_loop(r1, r2, r3, r4, r5):
    global state, start_left_count, start_right_count, integral_error
    
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
    
    # 엔코더 값 읽기
    l, r = get_encoder_counts()
    l_now = abs(l - start_left_count)
    r_now = abs(r - start_right_count)
    avg = (l_now + r_now) // 2
    
    # 상태별 처리
    if state == STATE_TURNING:
        if avg >= target_turn_pulses:
            motors_stop()
            if target_move_pulses > 0:
                start_left_count = l
                start_right_count = r
                integral_error = 0.0
                state = STATE_MOVING
            else:
                state = STATE_IDLE
    
    elif state == STATE_MOVING:
        if avg >= target_move_pulses:
            motors_stop()
            state = STATE_IDLE
        else:
            # 개선된 PI 제어 사용
            left_pwm, right_pwm = pi_control(l_now, r_now)
            left_mot_forward(left_pwm)
            right_mot_forward(right_pwm)
    
    elif state == STATE_AVOID:
        if (r1 <= TH_OUTER) and (r2 <= TH) and (r3 <= TH) and (r4 <= TH) and (r5 <= TH_OUTER):
            enter_escaping(ESCAPING_MS)
            return
        
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
    global encoder_left_pin, encoder_right_pin, running
    
    print("Initializing MONA ESP...")
    mona_esp_init()
    
    # 엔코더 인터럽트 설정
    encoder_left_pin = Pin(PIN_ENCODER_LEFT, Pin.IN)
    encoder_right_pin = Pin(PIN_ENCODER_RIGHT, Pin.IN)
    encoder_left_pin.irq(trigger=Pin.IRQ_RISING, handler=isr_left_encoder)
    encoder_right_pin.irq(trigger=Pin.IRQ_RISING, handler=isr_right_encoder)
    
    # WiFi 연결
    if not connect_wifi():
        print("Failed to connect WiFi. Restarting...")
        time.sleep(3)
        import machine
        machine.reset()
    
    # UDP 설정
    setup_udp()
    
    # IR 센서 스레드 시작 (Core 1에서 실행)
    _thread.start_new_thread(ir_sensor_thread, ())
    
    print("MONA UDP Controller ready!")
    set_led(1, 0, 30, 0)
    set_led(2, 0, 30, 0)
    
    led_update_counter = 0
    last_control_time = time.ticks_ms()
    
    try:
        while True:
            # UDP 패킷 처리
            handle_udp_packet()
            
            # IR 센서 값 가져오기
            ir = get_ir_values()
            r1, r2, r3, r4, r5 = ir[0], ir[1], ir[2], ir[3], ir[4]
            
            # 제어 루프
            control_loop(r1, r2, r3, r4, r5)
            
            # LED 상태 업데이트
            led_update_counter += 1
            if led_update_counter >= 100:
                update_status_led()
                led_update_counter = 0
            
            # 제어 주기 유지 (약 5ms 목표)
            elapsed = time.ticks_diff(time.ticks_ms(), last_control_time)
            if elapsed < 5:
                time.sleep_ms(5 - elapsed)
            last_control_time = time.ticks_ms()
    
    except KeyboardInterrupt:
        print("\nStopping...")
        running = False
        motors_stop()
        set_led(1, 0, 0, 0)
        set_led(2, 0, 0, 0)
        if udp_socket:
            udp_socket.close()
        print("MONA stopped.")


if __name__ == "__main__":
    main()
