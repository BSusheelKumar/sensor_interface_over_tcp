import socket
import struct
import threading
import time

class SensorSimulator:
    def __init__(self, host='127.0.0.1', port=2000):
        self.host = host
        self.port = port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket = None
        self.is_running = True
        self.status_interval = None  # Interval in milliseconds
        self.thread = None

    def start_server(self):
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        print(f"[INFO] Sensor server started on {self.host}:{self.port}. Waiting for connections...")
        self.client_socket, addr = self.server.accept()
        print(f"[INFO] Connection established with {addr}")
        
        # Start a thread to handle client communication
        self.thread = threading.Thread(target=self.handle_client)
        self.thread.start()

    def handle_client(self):
        while self.is_running:
            try:
                # Receive data from the client
                data = self.client_socket.recv(1024)
                if not data:
                    break
                self.process_command(data)
            except Exception as e:
                print(f"[ERROR] {e}")
                break

        self.client_socket.close()
        print("[INFO] Client disconnected.")

    def process_command(self, data):
        try:
            message = data.decode().strip()
            print(f"[RECEIVED] {message}")

            if message.startswith('#03'):  # Start Command
                # Extract the payload (interval in Little-Endian)
                payload = bytes.fromhex(message[3:-4])  # Skip '#03' and '<CR><LF>'
                self.status_interval = struct.unpack('<H', payload)[0]  # Decode Little-Endian
                print(f"[INFO] Start Command Received. Interval: {self.status_interval}ms")
                self.start_sending_status()

            elif message.startswith('#09'):  # Stop Command
                print("[INFO] Stop Command Received.")
                self.status_interval = None

        except Exception as e:
            print(f"[ERROR] Error processing command: {e}")

    def start_sending_status(self):
        if self.status_interval:
            threading.Thread(target=self.send_status_messages).start()

    def send_status_messages(self):
        while self.status_interval and self.is_running:
            # Construct the status message
            supply_voltage = 5000  # Example: 5000 mV
            env_temp = 250  # Example: 25.0°C (in deci-Celsius)
            yaw = -120  # Example: -12.0° (in deci-degrees)
            pitch = 30  # Example: 3.0° (in deci-degrees)
            roll = 10  # Example: 1.0° (in deci-degrees)

            # Pack payload in Little-Endian
            payload = struct.pack('<Hhhhh', supply_voltage, env_temp, yaw, pitch, roll)
            response = f"$11{payload.hex().upper()}\r\n"

            # Send to the client
            self.client_socket.sendall(response.encode())
            print(f"[SENT] {response.strip()}")

            # Wait for the interval duration
            time.sleep(self.status_interval / 1000.0)

    def stop_server(self):
        self.is_running = False
        self.server.close()
        print("[INFO] Sensor server stopped.")

if __name__ == "__main__":
    sensor_sim = SensorSimulator()
    try:
        sensor_sim.start_server()
    except KeyboardInterrupt:
        sensor_sim.stop_server()
