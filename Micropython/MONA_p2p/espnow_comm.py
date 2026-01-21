import network
import espnow
import json
import time
from machine import Timer

# ====== Configuration ======
ESPNOW_MAX_PAYLOAD = 250  # MicroPython ESP-NOW limit
MIN_BROADCAST_MS = 50
MAX_BROADCAST_MS = 100
PEER_LINK_DROP_MS = 900
BROADCAST_ADDR = b'\xff\xff\xff\xff\xff\xff'


class ESPNowComm:
    
    def __init__(self, self_id: str):
        self._self_id = self_id
        self._self_id_bytes = self_id.encode('utf-8')
        
        # Message storage
        self._self_message = {}  # Message to broadcast
        self._received_messages = {}  # {sender_id: message_dict}
        self._recv_timestamps = {}  # {sender_id: timestamp_ms}
        
        # ESP-NOW objects
        self._wlan = None
        self._espnow = None
        
        # Timing
        self._last_broadcast_ms = 0
        self._next_interval_ms = MIN_BROADCAST_MS
        
        # Statistics
        self._tx_count = 0
        self._rx_count = 0
        self._tx_fail_count = 0
        
    def init(self, wlan: network.WLAN = None):
        if wlan is None:
            self._wlan = network.WLAN(network.STA_IF)
            self._wlan.active(True)
        else:
            self._wlan = wlan
            
        # Initialize ESP-NOW
        self._espnow = espnow.ESPNow()
        self._espnow.active(True)
        
        # Add broadcast peer
        try:
            self._espnow.add_peer(BROADCAST_ADDR)
        except OSError as e:
            # Peer might already exist
            pass
        
        print(f"[ESP-NOW] Initialized. Self ID: {self._self_id}")
        
    def set_message(self, message: dict):
        self._self_message = message
        
    def get_message(self) -> dict:
        return self._self_message
    
    def get_received_messages(self) -> dict:
        now_ms = time.ticks_ms()
        active_messages = {}
        
        for sender_id, msg in self._received_messages.items():
            recv_time = self._recv_timestamps.get(sender_id, 0)
            if time.ticks_diff(now_ms, recv_time) <= PEER_LINK_DROP_MS:
                active_messages[sender_id] = msg
                
        return active_messages
    
    def poll(self):
        self._receive_all()
        self._broadcast_if_due()
        
    def _receive_all(self):
        if self._espnow is None:
            return
            
        while True:
            try:
                host, msg = self._espnow.recv(0)  # Non-blocking
                if msg is None:
                    break
                    
                self._process_received_packet(msg)
                
            except OSError:
                break
                
    def _process_received_packet(self, packet: bytes):
        if len(packet) < 2:
            return
            
        id_len = packet[0]
        if len(packet) < 1 + id_len:
            return
            
        sender_id = packet[1:1+id_len].decode('utf-8', 'ignore')
        
        # Ignore self messages
        if sender_id == self._self_id:
            return
            
        json_data = packet[1+id_len:]
        if len(json_data) == 0:
            return
            
        try:
            message = json.loads(json_data.decode('utf-8', 'ignore'))
            self._received_messages[sender_id] = message
            self._recv_timestamps[sender_id] = time.ticks_ms()
            self._rx_count += 1
            
        except (ValueError, UnicodeError) as e:
            # JSON parse error - ignore
            pass
            
    def _broadcast_if_due(self):
        if self._espnow is None:
            return
            
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self._last_broadcast_ms) < self._next_interval_ms:
            return
            
        self._last_broadcast_ms = now_ms
        
        # Randomize next interval
        import random
        self._next_interval_ms = random.randint(MIN_BROADCAST_MS, MAX_BROADCAST_MS)
        
        if not self._self_message:
            return
            
        # Build packet: [id_len][id][json]
        try:
            json_bytes = json.dumps(self._self_message, separators=(',', ':')).encode('utf-8')
        except (TypeError, ValueError):
            return
            
        id_len = len(self._self_id_bytes)
        total_len = 1 + id_len + len(json_bytes)
        
        if total_len > ESPNOW_MAX_PAYLOAD:
            print(f"[ESP-NOW] Message too large: {total_len} > {ESPNOW_MAX_PAYLOAD}")
            self._tx_fail_count += 1
            return
            
        packet = bytes([id_len]) + self._self_id_bytes + json_bytes
        
        try:
            self._espnow.send(BROADCAST_ADDR, packet)
            self._tx_count += 1
        except OSError as e:
            self._tx_fail_count += 1
            
    def build_monitor_json(self) -> dict:
        return {
            "agent_id": self._self_id,
            "received_messages": self.get_received_messages()
        }
        
    def get_stats(self) -> dict:
        return {
            "tx_count": self._tx_count,
            "rx_count": self._rx_count,
            "tx_fail_count": self._tx_fail_count,
            "active_peers": len(self.get_received_messages())
        }
        
    def close(self):
        """Cleanup ESP-NOW resources."""
        if self._espnow:
            self._espnow.active(False)
            self._espnow = None

