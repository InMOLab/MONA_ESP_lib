import network
import time


class WiFiManager:
    
    def __init__(self, ssid: str, password: str):
        self._ssid = ssid
        self._password = password
        self._wlan = None
        
        # Timing constants (ms)
        self._connect_timeout_ms = 10000
        self._retry_delay_ms = 500
        self._check_interval_ms = 1000
        self._last_check_ms = 0
        
    def connect(self) -> bool:
        self._wlan = network.WLAN(network.STA_IF)
        self._wlan.active(True)
        
        # Disable power save for better performance
        try:
            self._wlan.config(pm=0)  # Disable power management
        except:
            pass  # Not all firmwares support this
        
        if self._wlan.isconnected():
            print(f"[WiFi] Already connected: {self._wlan.ifconfig()[0]}")
            return True
            
        print(f"[WiFi] Connecting to {self._ssid}...")
        self._wlan.connect(self._ssid, self._password)
        
        start_ms = time.ticks_ms()
        while not self._wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), start_ms) > self._connect_timeout_ms:
                print("[WiFi] Connection timeout")
                return False
                
            time.sleep_ms(100)
            
        ip = self._wlan.ifconfig()[0]
        channel = self._get_channel()
        print(f"[WiFi] Connected! IP: {ip}, Channel: {channel}")
        
        return True
    
    def _get_channel(self) -> int:
        """Get current WiFi channel."""
        try:
            return self._wlan.config('channel')
        except:
            return 0
            
    def is_connected(self) -> bool:
        """Check if WiFi is connected."""
        return self._wlan is not None and self._wlan.isconnected()
    
    def get_ip(self) -> str:
        """Get current IP address."""
        if self.is_connected():
            return self._wlan.ifconfig()[0]
        return "0.0.0.0"
    
    def get_wlan(self) -> network.WLAN:
        """Get WLAN object for ESP-NOW initialization."""
        return self._wlan
    
    def check_connection(self) -> bool:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self._last_check_ms) < self._check_interval_ms:
            return self.is_connected()
            
        self._last_check_ms = now_ms
        
        if not self.is_connected():
            print("[WiFi] Connection lost, reconnecting...")
            self._wlan.disconnect()
            time.sleep_ms(self._retry_delay_ms)
            return self.connect()
            
        return True
    
    def disconnect(self):
        """Disconnect from WiFi."""
        if self._wlan:
            self._wlan.disconnect()
            self._wlan.active(False)
            print("[WiFi] Disconnected")

