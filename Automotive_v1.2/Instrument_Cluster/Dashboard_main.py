import sys
import socket
import threading
import time
import resources_rc

# --- CHUYỂN ĐỔI SANG PYQT5 ---
from PyQt5.QtCore import QObject, pyqtSignal as Signal, pyqtSlot as Slot
from PyQt5.QtGui import QGuiApplication
from PyQt5.QtQml import QQmlApplicationEngine

class DashboardBackend(QObject):
    # Khai báo tín hiệu
    dataReceived = Signal(str)

    def __init__(self):
        super().__init__()
        self.running = True

        # Tạo luồng lắng nghe UDP chạy ngầm
        self.thread = threading.Thread(target=self.udp_listener)
        self.thread.daemon = True # Tự tắt khi chương trình tắt
        self.thread.start()

    def udp_listener(self):
        UDP_IP = "127.0.0.1"
        UDP_PORT = 5005
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            sock.bind((UDP_IP, UDP_PORT))
            print(f"Dashboard đang đợi tin hiệu từ AI tại port {UDP_PORT}...")
        except Exception as e:
            print(f"Lỗi mở port 5005: {e}")
            return

        while self.running:
            try:
                # Chờ nhận dữ liệu
                data, addr = sock.recvfrom(1024)
                message = data.decode("utf-8").strip()

                if message.startswith("DATA:"):
                    self.parse_esp32_data(message)

                elif message == "GATEWAY_STARTED":
                    self.dataReceived.emit("Car:Started!")

                elif message.startswith("AI:"):
                    self.dataReceived.emit(message)

                elif message.startswith("GEAR:"):
                    self.dataReceived.emit(message)

                else:
                    self.dataReceived.emit(message)

            except Exception as e:
                print(f"Lỗi UDP: {e}")


    def parse_esp32_data(self, raw_data):
        try:
            content = raw_data.split(":")[1]
            params = content.split(",")

            if len(params) < 10: return

            speed_val = float(params[0])
            self.dataReceived.emit(f"REAL_SPEED:{speed_val}")

            rpm_val = int(params[1])

            dist_val = int(params[2])
            self.dataReceived.emit(f"DISTANCE:{dist_val}")

            pha_state = int(params[3])
            cos_state = int(params[4])
            left_state = int(params[5])
            right_state = int(params[6])

            cruise_state = int(params[7])
            self.dataReceived.emit(f"CRUISE:{cruise_state}")

            target_speed_val = int(params[8])
            self.dataReceived.emit(f"SET_SPEED:{target_speed_val}")

            gap_val = int(params[9])
            self.dataReceived.emit(f"GAP_LEVEL:{gap_val}")

            MAX_DISPLAY_SPEED = 232.0
            mapped_val = int((speed_val / MAX_DISPLAY_SPEED) * 4095)
            if mapped_val > 4095: mapped_val = 4095
            self.dataReceived.emit(f"POT_VAL:{mapped_val}")

            msg_pha = "DEN_PHA:ON" if pha_state == 1 else "DEN_PHA:OFF"
            self.dataReceived.emit(msg_pha)

            msg_cos = "DEN_COS:ON" if cos_state == 1 else "DEN_COS:OFF"
            self.dataReceived.emit(msg_cos)

            if left_state == 1 and right_state == 1:
                 self.dataReceived.emit("HAZARD:ON")
            else:
                 self.dataReceived.emit("HAZARD:OFF")

                 if left_state == 1: self.dataReceived.emit("TURN_LEFT:ON")
                 else: self.dataReceived.emit("TURN_LEFT:OFF")

                 if right_state == 1: self.dataReceived.emit("TURN_RIGHT:ON")
                 else: self.dataReceived.emit("TURN_RIGHT:OFF")

        except Exception as e:
            print(f"Lỗi parse dữ liệu: {e}")

if __name__ == "__main__":
    app = QGuiApplication(sys.argv)
    engine = QQmlApplicationEngine()

    backend = DashboardBackend()

    # Nhúng biến 'myBackend' vào QML
    engine.rootContext().setContextProperty("myBackend", backend)

    # Load file giao diện
    engine.load("main.qml")

    if not engine.rootObjects():
        sys.exit(-1)

    # PyQt5 dùng exec_() có dấu gạch dưới
    sys.exit(app.exec_())
