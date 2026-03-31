from wifi_manager import WiFiManager
from espnow_comm import ESPNowComm
from tcp_server import TCPServer
import time
import gc


class MonaComm:
    
    def __init__(self, self_id: str, ssid: str, password: str, tcp_port: int = 8080):
        self._self_id = self_id
        
        # Communication modules
        self._wifi = WiFiManager(ssid, password)
        self._espnow = ESPNowComm(self_id)
        self._tcp = TCPServer(port=tcp_port)
        
        # State
        self._initialized = False
        self._last_gc_ms = 0
        self._gc_interval_ms = 5000  # Garbage collect every 5 seconds
        
    def start(self) -> bool:
        # Connect WiFi first
        if not self._wifi.connect():
            print("[MonaComm] WiFi connection failed")
            return False
            
        # Initialize ESP-NOW
        self._espnow.init(self._wifi.get_wlan())
        
        # Start TCP server
        self._tcp.start()
        
        self._initialized = True
        print(f"[MonaComm] Started. ID={self._self_id}, IP={self._wifi.get_ip()}")
        
        return True
    
    def update(self):
        if not self._initialized:
            return
            
        # Check WiFi connection
        if not self._wifi.check_connection():
            return
            
        # Process ESP-NOW
        self._espnow.poll()
        
        # Process TCP with monitor data
        monitor_data = self.get_monitor_data()
        self._tcp.poll(monitor_data)
        
        # Update self message from TCP if received
        tcp_msg = self._tcp.get_received_message()
        if tcp_msg:
            self._espnow.set_message(tcp_msg)
            self._tcp.clear_received_message()
            
        # Periodic garbage collection
        self._periodic_gc()
        
    def set_message(self, message: dict):
        self._espnow.set_message(message)
        
    def get_received_messages(self) -> dict:
        return self._espnow.get_received_messages()
    
    def get_monitor_data(self) -> dict:
        return self._espnow.build_monitor_json()
    
    def get_tcp_message(self) -> dict:
        return self._tcp.get_received_message()
    
    def is_connected(self) -> bool:
        return self._wifi.is_connected()
    
    def is_tcp_client_connected(self) -> bool:
        return self._tcp.is_client_connected()
    
    def get_ip(self) -> str:
        return self._wifi.get_ip()
    
    def get_stats(self) -> dict:
        espnow_stats = self._espnow.get_stats()
        espnow_stats["wifi_connected"] = self._wifi.is_connected()
        espnow_stats["tcp_clients"] = self._tcp.get_client_count()
        espnow_stats["ip"] = self._wifi.get_ip()
        return espnow_stats
        
    def _periodic_gc(self):
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self._last_gc_ms) > self._gc_interval_ms:
            self._last_gc_ms = now_ms
            gc.collect()
            
    def stop(self):
        self._tcp.close()
        self._espnow.close()
        self._wifi.disconnect()
        self._initialized = False
        print("[MonaComm] Stopped")


# ====== Convenience function for quick setup ======

def create_mona_comm(self_id: str, ssid: str = "InMOLab", 
                      password: str = "dlsahfoq104!", 
                      tcp_port: int = 8080) -> MonaComm:
    comm = MonaComm(self_id, ssid, password, tcp_port)
    if not comm.start():
        raise RuntimeError("Failed to start MonaComm")
    return comm


