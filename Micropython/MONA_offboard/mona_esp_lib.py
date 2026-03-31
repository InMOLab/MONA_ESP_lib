"""
Mona_ESP_lib.py - MicroPython library for MONA ESP robot
Ported from Arduino C++ library by Bart Garcia
Created for MicroPython on ESP32
"""

from machine import Pin, PWM, I2C, ADC
import time

from ads7830 import ADS7830, SDMODE_SINGLE, PDIROFF_ADON
from mcp23008 import MCP23008, INPUT, OUTPUT, HIGH, LOW
from neopixel_led import NeoPixelLED

# ============== Pin Definitions ==============

# Motor Control Pins
MOT_RIGHT_FORWARD = 19
MOT_RIGHT_BACKWARD = 21
MOT_LEFT_FORWARD = 4
MOT_LEFT_BACKWARD = 18

# Motor Feedback (Encoder) Pins
MOT_RIGHT_FEEDBACK = 39
MOT_RIGHT_FEEDBACK_2 = 23
MOT_LEFT_FEEDBACK = 35
MOT_LEFT_FEEDBACK_2 = 34

# I2C Pins
SDA_PIN = 32
SCL_PIN = 33

# LED Pins
LED_RGB1_PIN = 22
LED_RGB2_PIN = 15

# Battery Voltage Pin
BATT_VOL_PIN = 36

# IO Expander Pin Definitions
EXP_0 = 0
EXP_1 = 1
EXP_2 = 2
EXP_3 = 3
EXP_4 = 4
EXP_5 = 5
EXP_6 = 6
EXP_7 = 7

# IR Enable pins (on IO Expander)
IR_ENABLE_1 = EXP_4
IR_ENABLE_2 = EXP_3
IR_ENABLE_3 = EXP_2
IR_ENABLE_4 = EXP_1
IR_ENABLE_5 = EXP_0

# IR Sensor ADC channels
IR1_SENSOR = 7  # Left IR
IR2_SENSOR = 6  # Left diagonal IR
IR3_SENSOR = 5  # Front IR
IR4_SENSOR = 4  # Right diagonal IR
IR5_SENSOR = 0  # Right IR

# Motor PWM Settings
MOT_FREQ = 5000
MOT_DUTY_MAX = 255

# I2C Device Addresses
IO_EXP_ADDRESS = 0x20
ADC_ADDRESS = 0x48

# ============== Global Objects ==============

# I2C bus
_i2c = None

# IO Expander
_io_expander = None

# ADC
_adc = None

# RGB LEDs
_rgb1 = None
_rgb2 = None

# Motor PWM objects
_mot_rf = None  # Right forward
_mot_rb = None  # Right backward
_mot_lf = None  # Left forward
_mot_lb = None  # Left backward

# Battery ADC
_batt_adc = None


def mona_esp_init():
    """Initialize the MONA ESP robot hardware"""
    global _i2c, _io_expander, _adc, _rgb1, _rgb2
    global _mot_rf, _mot_rb, _mot_lf, _mot_lb, _batt_adc
    
    # Initialize Motor PWM
    _mot_rf = PWM(Pin(MOT_RIGHT_FORWARD), freq=MOT_FREQ, duty=0)
    _mot_rb = PWM(Pin(MOT_RIGHT_BACKWARD), freq=MOT_FREQ, duty=0)
    _mot_lf = PWM(Pin(MOT_LEFT_FORWARD), freq=MOT_FREQ, duty=0)
    _mot_lb = PWM(Pin(MOT_LEFT_BACKWARD), freq=MOT_FREQ, duty=0)
    
    # Initialize I2C
    _i2c = I2C(0, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=400000)
    
    # Initialize IO Expander
    _io_expander = MCP23008(_i2c, IO_EXP_ADDRESS)
    
    # Set IO Expander pin modes
    _io_expander.pin_mode(IR_ENABLE_5, OUTPUT)
    _io_expander.pin_mode(IR_ENABLE_4, OUTPUT)
    _io_expander.pin_mode(IR_ENABLE_3, OUTPUT)
    _io_expander.pin_mode(IR_ENABLE_2, OUTPUT)
    _io_expander.pin_mode(IR_ENABLE_1, OUTPUT)
    _io_expander.pin_mode(EXP_5, INPUT)
    _io_expander.pin_mode(EXP_6, INPUT)
    _io_expander.pin_mode(EXP_7, INPUT)
    
    # Turn off all IR LEDs
    _io_expander.digital_write(IR_ENABLE_5, LOW)
    _io_expander.digital_write(IR_ENABLE_4, LOW)
    _io_expander.digital_write(IR_ENABLE_3, LOW)
    _io_expander.digital_write(IR_ENABLE_2, LOW)
    _io_expander.digital_write(IR_ENABLE_1, LOW)
    
    # Initialize ADC (ADS7830)
    _adc = ADS7830(_i2c, ADC_ADDRESS)
    _adc.sd_mode = SDMODE_SINGLE
    _adc.pd_mode = PDIROFF_ADON
    
    # Initialize RGB LEDs
    _rgb1 = NeoPixelLED(LED_RGB1_PIN, 1)
    _rgb2 = NeoPixelLED(LED_RGB2_PIN, 1)
    
    # Initialize Battery Voltage ADC
    _batt_adc = ADC(Pin(BATT_VOL_PIN))
    _batt_adc.atten(ADC.ATTN_0DB)  # 0dB attenuation (up to ~800mV)
    
    print("MONA ESP initialized!")


# ============== Motor Control Functions ==============

def _set_motor_speed(pwm_obj: PWM, speed: int):
    """Set motor PWM duty cycle (0-255)"""
    speed = min(max(speed, 0), 255)
    # Convert 8-bit (0-255) to 10-bit duty cycle (0-1023)
    duty = int(speed * 1023 / 255)
    pwm_obj.duty(duty)


def right_mot_forward(speed: int):
    """Move right motor forward"""
    _set_motor_speed(_mot_rf, speed)
    _set_motor_speed(_mot_rb, 0)


def right_mot_backward(speed: int):
    """Move right motor backward"""
    _set_motor_speed(_mot_rf, 0)
    _set_motor_speed(_mot_rb, speed)


def right_mot_stop():
    """Stop right motor"""
    _set_motor_speed(_mot_rf, 0)
    _set_motor_speed(_mot_rb, 0)


def left_mot_forward(speed: int):
    """Move left motor forward"""
    _set_motor_speed(_mot_lf, speed)
    _set_motor_speed(_mot_lb, 0)


def left_mot_backward(speed: int):
    """Move left motor backward"""
    _set_motor_speed(_mot_lf, 0)
    _set_motor_speed(_mot_lb, speed)


def left_mot_stop():
    """Stop left motor"""
    _set_motor_speed(_mot_lf, 0)
    _set_motor_speed(_mot_lb, 0)


def motors_forward(speed: int):
    """Move both motors forward"""
    right_mot_forward(speed)
    left_mot_forward(speed)


def motors_backward(speed: int):
    """Move both motors backward"""
    right_mot_backward(speed)
    left_mot_backward(speed)


def motors_spin_left(speed: int):
    """Spin robot to the left (right forward, left backward)"""
    right_mot_forward(speed)
    left_mot_backward(speed)


def motors_spin_right(speed: int):
    """Spin robot to the right (right backward, left forward)"""
    right_mot_backward(speed)
    left_mot_forward(speed)


def motors_stop():
    """Stop both motors"""
    right_mot_stop()
    left_mot_stop()


# ============== IR Sensor Functions ==============

# Mapping from IR number to enable pin on IO expander
_IR_ENABLE_MAP = {
    1: IR_ENABLE_1,
    2: IR_ENABLE_2,
    3: IR_ENABLE_3,
    4: IR_ENABLE_4,
    5: IR_ENABLE_5,
}

# Mapping from IR number to ADC channel
_IR_SENSOR_MAP = {
    1: IR1_SENSOR,
    2: IR2_SENSOR,
    3: IR3_SENSOR,
    4: IR4_SENSOR,
    5: IR5_SENSOR,
}


def enable_ir(ir_number: int):
    """Enable IR LED for specified sensor (1-5)"""
    if ir_number in _IR_ENABLE_MAP:
        _io_expander.digital_write(_IR_ENABLE_MAP[ir_number], HIGH)


def disable_ir(ir_number: int):
    """Disable IR LED for specified sensor (1-5)"""
    if ir_number in _IR_ENABLE_MAP:
        _io_expander.digital_write(_IR_ENABLE_MAP[ir_number], LOW)


def read_ir(ir_number: int) -> int:
    """
    Read raw ADC value from IR sensor (1-5)
    
    Returns:
        8-bit ADC value (0-255), or 0 if invalid sensor number
    """
    if ir_number in _IR_SENSOR_MAP:
        return _adc.measure_single_ended(_IR_SENSOR_MAP[ir_number])
    return 0


def get_ir(ir_number: int) -> int:
    """
    Get IR sensor reading (difference between dark and light values)
    
    This function reads the ambient light, turns on the IR LED,
    reads again, then turns off the IR LED and returns the difference.
    
    Args:
        ir_number: IR sensor number (1-5)
        
    Returns:
        Difference value (0-255), or 0 if invalid sensor number
    """
    if ir_number not in _IR_SENSOR_MAP:
        return 0
    
    # Read dark value (ambient)
    dark_val = read_ir(ir_number)
    
    # Enable IR LED
    enable_ir(ir_number)
    time.sleep_ms(1)  # Wait for IR LED to reach full brightness
    
    # Read light value
    light_val = read_ir(ir_number)
    
    # Disable IR LED
    disable_ir(ir_number)
    
    return abs(dark_val - light_val)


def detect_object(ir_number: int, threshold: int) -> bool:
    """
    Detect if an object is present near the specified IR sensor
    
    Args:
        ir_number: IR sensor number (1-5)
        threshold: Detection threshold value
        
    Returns:
        True if object detected, False otherwise
    """
    if ir_number not in _IR_SENSOR_MAP:
        return False
    
    ir_val = get_ir(ir_number)
    return ir_val > threshold


# ============== Battery Voltage ==============

def batt_vol() -> int:
    """
    Get battery percentage (0-100)
    
    MONA's battery voltage ranges from 4.2V (full) to 3.3V (minimum working).
    On-board resistors convert this to 0.869-0.630V.
    Using 0dB attenuation, ADC range is approximately 3550-2750.
    
    Note: When USB is connected, this reads USB voltage instead.
    
    Returns:
        Battery percentage (0-100)
    """
    adc_val = _batt_adc.read()
    
    # Convert to percentage
    # Offset 2750 (3.3V), range 800 (3.3V to 4.2V maps to 2750-3550)
    bat_percentage = (adc_val - 2750) // 8
    
    # Clamp to 0-100
    return max(0, min(100, bat_percentage))


# ============== LED Control ==============

def set_led(led_number: int, red: int, green: int, blue: int):
    """
    Set RGB LED color
    
    Args:
        led_number: LED number (1 or 2)
        red: Red component (0-255)
        green: Green component (0-255)
        blue: Blue component (0-255)
    """
    if led_number == 1:
        _rgb1.fill(red, green, blue)
    elif led_number == 2:
        _rgb2.fill(red, green, blue)
