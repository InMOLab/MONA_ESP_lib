import time
import gc
from machine import Pin

# ====== CONFIGURATION ======
SELF_ID = "2"  # Change this for each robot!
SSID = "InMOLab"
PASSWORD = "dlsahfoq104!"
TCP_PORT = 8080

# Optional: LED for status indication
LED_PIN = 2  # Built-in LED on most ESP32 boards

# ====== Global Objects ======
comm = None
led = None


def setup():
    """Initialize hardware and communication."""
    global comm, led
    
    print("\n" + "="*50)
    print(f"MONA ESP-NOW Test - Robot ID: {SELF_ID}")
    print("="*50 + "\n")
    
    # Setup status LED
    try:
        led = Pin(LED_PIN, Pin.OUT)
        led.off()
    except:
        led = None
        print("[Setup] No LED available")
    
    # Initialize communication
    from mona_comm import MonaComm
    comm = MonaComm(SELF_ID, SSID, PASSWORD, TCP_PORT)
    
    if not comm.start():
        print("[Setup] Communication init failed!")
        blink_error()
        return False
        
    print("[Setup] Complete!\n")
    return True


def blink_error():
    """Blink LED rapidly to indicate error."""
    if led is None:
        return
    for _ in range(10):
        led.on()
        time.sleep_ms(100)
        led.off()
        time.sleep_ms(100)


def blink_status(count: int = 1):
    """Blink LED to indicate activity."""
    if led is None:
        return
    for _ in range(count):
        led.on()
        time.sleep_ms(50)
        led.off()
        time.sleep_ms(50)


def main_loop():
    """Main communication loop."""
    global comm
    
    last_stats_ms = time.ticks_ms()
    stats_interval_ms = 5000  # Print stats every 5 seconds
    
    # Test message (simulating CBBA data)
    test_message = {
        "id": int(SELF_ID),
        "y": {},  # Winning bids (will be populated by simulator)
        "z": {},  # Winning agents
        "s": {}   # Timestamps
    }
    
    print("[Loop] Starting main loop...")
    print("[Loop] Waiting for TCP connection from simulator...\n")
    
    while True:
        try:
            # Update communication (handles all send/receive)
            comm.update()
            
            # Get TCP message from simulator (if any)
            tcp_msg = comm.get_tcp_message()
            if tcp_msg:
                # Forward to ESP-NOW broadcast
                comm.set_message(tcp_msg)
                blink_status(1)
            
            # Get received ESP-NOW messages
            received = comm.get_received_messages()
            
            # Print stats periodically
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_stats_ms) > stats_interval_ms:
                last_stats_ms = now_ms
                stats = comm.get_stats()
                
                print(f"[Stats] TX:{stats['tx_count']} RX:{stats['rx_count']} "
                      f"Fail:{stats['tx_fail_count']} Peers:{stats['active_peers']} "
                      f"TCP:{stats['tcp_clients']}")
                
                if received:
                    print(f"[Peers] Active: {list(received.keys())}")
                    
                # Memory info
                gc.collect()
                free_mem = gc.mem_free()
                print(f"[Mem] Free: {free_mem} bytes")
                print()
                
            # Small delay to prevent tight loop
            time.sleep_ms(1)
            
        except KeyboardInterrupt:
            print("\n[Loop] Interrupted by user")
            break
            
        except Exception as e:
            print(f"[Loop] Error: {e}")
            time.sleep_ms(100)


def main():
    """Main entry point."""
    if setup():
        try:
            main_loop()
        finally:
            if comm:
                comm.stop()
            print("[Main] Shutdown complete")
    else:
        print("[Main] Setup failed, halting")


# Auto-run on import
if __name__ == "__main__":
    main()

