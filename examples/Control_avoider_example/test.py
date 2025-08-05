import socket
import curses
import threading

# MONA ESP IP & Port
host = "192.168.0.6"   # MONA ESP IP
port = 80              # Server Port

sock = socket.socket()
sock.connect((host, port))

# Avoidance mode flag
avoidance_mode = False

def listen_feedback():
    global avoidance_mode
    while True:
        try:
            data = sock.recv(1024).decode().strip()
            if not data:
                break

            if data == "AVOID_ON":
                avoidance_mode = True
                print("\n Avoidance maneuver in progress...")
            elif data == "AVOID_OFF":
                avoidance_mode = False
                print("\n Avoidance maneuver complete.")
        except:
            break

threading.Thread(target=listen_feedback, daemon=True).start()

# Curses setup
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)
screen.addstr(0, 0, 'MONA ESP CONTROL')
screen.addstr(1, 0, 'Press "q" to exit the program && Press "r" to exit the avoidance mode')
screen.addstr(2, 0, 'Use the arrow keys to move')

try:
    while True:
        status_msg = f"🚧 Avoidance Mode: {'ON' if avoidance_mode else 'OFF'}"
        screen.addstr(4, 0, status_msg + " " * 10)

        char = screen.getch()

        if char == ord('q'):
            break

        # r키: 항상 수신 → 회피 강제 해제 요청
        if char == ord('r') or char == ord('R'):
            sock.sendall(b"r")
            screen.addstr(5, 0, " Sent: Cancel Avoidance ('r')        ")
            continue

        # 회피 중에는 키 입력 무시 (단 r키는 위에서 예외 처리)
        if avoidance_mode:
            screen.addstr(5, 0, " In avoidance mode. Command ignored. ")
            continue

        if char == curses.KEY_RIGHT:
            screen.addstr(5, 0, "▶ Turning Right           ")
            sock.sendall(b"R")

        elif char == curses.KEY_LEFT:
            screen.addstr(5, 0, "◀ Turning Left            ")
            sock.sendall(b"L")

        elif char == curses.KEY_UP:
            screen.addstr(5, 0, "▲ Moving Forward          ")
            sock.sendall(b"F")

        elif char == curses.KEY_DOWN:
            screen.addstr(5, 0, "▼ Moving Backward         ")
            sock.sendall(b"B")

finally:
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()
    sock.close()
