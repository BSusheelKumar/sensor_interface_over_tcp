import socket
import rclpy
from rclpy.node import Node
import struct
import threading 




class TcpServer(Node):
    def __init__(self):
        super().__init__("tcp_server_node")


        self.host = '127.0.0.1'
        self.port = 2000

        self.server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)


        self.timer = None

        self.client_socket = None


        # self.start_server()

        server_thread = threading.Thread(target=self.start_server,daemon=True)

        server_thread.start()





    def start_server(self):
        self.server_socket.bind((self.host,self.port))
        self.server_socket.listen(1)
        self.get_logger().info(f"server is listening on {self.host,self.port}, waiting for connection....")

        while True:
            self.client_socket, addr = self.server_socket.accept()
            self.get_logger().info(f"connection established with {addr}")

            try:
                data = self.client_socket.recv(1024)


                if not data:
                    self.get_logger().info("Client Disconnected")

                self.get_logger().info(f"Recieved data {data.decode()}")
                self.le_hexadecimal_to_decimal(data)
                

            except Exception as e:
                self.get_logger().info(f"Error occured {e}")


    def le_hexadecimal_to_decimal(self,data):


        
        command_id = data.decode()[1:3]


        if command_id == '03':
            hex_part = data[3:7]
            self.get_logger().info(f"extracted hex part is {hex_part} and command Id is {command_id}")

            numeric_bytes = bytes.fromhex(hex_part.decode())
            self.get_logger().info(f"Numeric bytes is {numeric_bytes}")

            payload = struct.unpack('<H',numeric_bytes)[0]
            self.get_logger().info(f"unpacked data is {payload}")

            self.start_sending_data(payload)


        else:
            self.get_logger().info("no valid data")


    def start_sending_data(self,interval):

        self.payload_interval = interval/1000.0

        self.timer = self.create_timer(self.payload_interval,self.send_response_to_client)
        self.get_logger().info(f"starting to send messages at {self.payload_interval}")

    def send_response_to_client(self):



        supply_voltage = 12000     
        env_temp = 250             
        yaw = 100                   
        pitch = -50                 
        roll = 30                  
        command_id = 11

        packed_data = struct.pack('<BhhhhH', command_id, supply_voltage, env_temp, yaw, pitch, roll)

        hex_message = ''.join(f'{byte:02X}' for byte in packed_data)

        formatted_message = '$' + hex_message + '0D0A'


        self.get_logger().info(f"Sent: {formatted_message}")
        self.client_socket.sendall(formatted_message.encode())



    def shutdown_server(self):

        self.get_logger().info("shutting down server")

        self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = TcpServer()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.shutdown_server()

    finally:
            node.shutdown_server()
            node.destroy_node()
            rclpy.shutdown()
if __name__=="__main__":
    main()