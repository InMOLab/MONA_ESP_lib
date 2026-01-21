import time
from mona_esp_lib import (
    mona_esp_init,
    motors_forward,
    motors_spin_left,
    motors_spin_right,
    motors_stop,
    detect_object,
    set_led,
)

THRESHOLD = 80

# Motor speeds
FORWARD_SPEED = 150
TURN_SPEED = 100

# State machine states
STATE_FORWARD = 0
STATE_OBSTACLE_FRONT = 1
STATE_OBSTACLE_RIGHT = 2
STATE_OBSTACLE_LEFT = 3

# Loop delay (ms)
LOOP_DELAY_MS = 5


def main():
    # Initialize the MONA robot
    print("Initializing MONA ESP...")
    mona_esp_init()
    print("MONA ESP ready!")
    
    # Set initial LED color (green = running)
    set_led(1, 0, 50, 0)
    set_led(2, 0, 50, 0)
    
    # State machine variables
    state = STATE_FORWARD
    old_state = STATE_FORWARD
    
    # IR sensor values (1-5: left, left-diagonal, front, right-diagonal, right)
    ir_values = [False, False, False, False, False]
    
    print("Starting collision avoidance...")
    
    try:
        while True:
            if state == STATE_FORWARD:
                motors_forward(FORWARD_SPEED)
                # LED: Green when moving forward
                if state != old_state:
                    set_led(1, 0, 50, 0)
                    set_led(2, 0, 50, 0)
                    
            elif state == STATE_OBSTACLE_FRONT:
                # Obstacle in front: spin left
                motors_spin_left(TURN_SPEED)
                # LED: Red when obstacle detected
                if state != old_state:
                    set_led(1, 50, 0, 0)
                    set_led(2, 50, 0, 0)
                    
            elif state == STATE_OBSTACLE_RIGHT:
                # Obstacle on right: spin left
                motors_spin_left(TURN_SPEED)
                # LED: Yellow (right side obstacle)
                if state != old_state:
                    set_led(1, 50, 25, 0)
                    set_led(2, 50, 25, 0)
                    
            elif state == STATE_OBSTACLE_LEFT:
                # Obstacle on left: spin right
                motors_spin_right(TURN_SPEED)
                # LED: Blue (left side obstacle)
                if state != old_state:
                    set_led(1, 0, 0, 50)
                    set_led(2, 0, 0, 50)
            
            old_state = state
            
            # ============== IR Sensor Reading ==============
            ir_values[0] = detect_object(1, THRESHOLD)  # Left
            ir_values[1] = detect_object(2, THRESHOLD)  # Left diagonal
            ir_values[2] = detect_object(3, THRESHOLD)  # Front
            ir_values[3] = detect_object(4, THRESHOLD)  # Right diagonal
            ir_values[4] = detect_object(5, THRESHOLD)  # Right
            
            # ============== State Machine ==============
            # Determine next state based on IR readings
            
            # Priority: Front obstacles first
            if ir_values[2] or ir_values[3] or ir_values[4]:
                # Front, right-diagonal, or right obstacle detected
                state = STATE_OBSTACLE_FRONT
            elif ir_values[0]:
                # Left obstacle: turn right
                state = STATE_OBSTACLE_LEFT
            elif ir_values[4]:
                # Right obstacle: turn left
                state = STATE_OBSTACLE_RIGHT
            else:
                # No obstacles: move forward
                state = STATE_FORWARD
            
            # Small delay
            time.sleep_ms(LOOP_DELAY_MS)
            
    except KeyboardInterrupt:
        # Clean shutdown on Ctrl+C
        print("\nStopping...")
        motors_stop()
        set_led(1, 0, 0, 0)
        set_led(2, 0, 0, 0)
        print("MONA stopped.")


# Run main when executed directly
if __name__ == "__main__":
    main()