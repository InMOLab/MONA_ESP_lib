import socket
import select
import json
import time


class TCPServer:
    
    def __init__(self, port: int = 8080, max_clients: int = 4):
        self._port = port
        self._max_clients = max_clients
        
        self._server_socket = None
        self._clients = []  # List of (socket, buffer)
        
        # Received message from PC
        self._received_message = {}
        
        # Timing
        self._last_send_ms = 0
        self._send_interval_ms = 50  # Same as C++ MONITORING_SEND_INTERVAL_MS
        
    def start(self):
        """Start the TCP server."""
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.setblocking(False)
        
        self._server_socket.bind(('0.0.0.0', self._port))
        self._server_socket.listen(self._max_clients)
        
        print(f"[TCP] Server listening on port {self._port}")
        
    def poll(self, monitor_data: dict = None):
        self._accept_new_clients()
        self._receive_from_clients()
        
        if monitor_data is not None:
            self._send_to_clients_if_due(monitor_data)
            
    def _accept_new_clients(self):
        if self._server_socket is None:
            return
            
        try:
            client_socket, addr = self._server_socket.accept()
            client_socket.setblocking(False)
            
            # Check if we have room
            if len(self._clients) >= self._max_clients:
                # Remove disconnected clients first
                self._cleanup_clients()
                
            if len(self._clients) < self._max_clients:
                self._clients.append({
                    'socket': client_socket,
                    'buffer': b'',
                    'addr': addr
                })
                print(f"[TCP] Client connected: {addr}")
            else:
                client_socket.close()
                print(f"[TCP] Rejected client (max reached): {addr}")
                
        except OSError:
            # No pending connections
            pass
            
    def _receive_from_clients(self):
        for client in self._clients[:]:  # Copy list for safe iteration
            try:
                data = client['socket'].recv(1024)
                if data:
                    client['buffer'] += data
                    self._process_client_buffer(client)
                else:
                    # Client disconnected
                    self._remove_client(client)
                    
            except OSError as e:
                # Would block or other error
                if e.args[0] not in (11, 110):  # EAGAIN, ETIMEDOUT
                    self._remove_client(client)
                    
    def _process_client_buffer(self, client: dict):
        while b'\n' in client['buffer']:
            line, client['buffer'] = client['buffer'].split(b'\n', 1)
            
            if line:
                try:
                    message = json.loads(line.decode('utf-8', 'ignore'))
                    self._received_message = message
                except (ValueError, UnicodeError):
                    pass
                    
    def _send_to_clients_if_due(self, data: dict):
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self._last_send_ms) < self._send_interval_ms:
            return
            
        self._last_send_ms = now_ms
        
        try:
            json_line = json.dumps(data, separators=(',', ':')) + '\n'
            json_bytes = json_line.encode('utf-8')
        except (TypeError, ValueError):
            return
            
        for client in self._clients[:]:
            try:
                client['socket'].send(json_bytes)
            except OSError:
                self._remove_client(client)
                
    def _remove_client(self, client: dict):
        try:
            client['socket'].close()
        except:
            pass
            
        if client in self._clients:
            self._clients.remove(client)
            print(f"[TCP] Client disconnected: {client.get('addr', 'unknown')}")
            
    def _cleanup_clients(self):
        for client in self._clients[:]:
            try:
                # Check if socket is still valid
                client['socket'].getpeername()
            except OSError:
                self._remove_client(client)
                
    def get_received_message(self) -> dict:
        return self._received_message
    
    def clear_received_message(self):
        self._received_message = {}
        
    def is_client_connected(self) -> bool:
        return len(self._clients) > 0
    
    def get_client_count(self) -> int:
        return len(self._clients)
    
    def close(self):
        for client in self._clients:
            try:
                client['socket'].close()
            except:
                pass
                
        self._clients.clear()
        
        if self._server_socket:
            try:
                self._server_socket.close()
            except:
                pass
            self._server_socket = None
            
        print("[TCP] Server closed")

