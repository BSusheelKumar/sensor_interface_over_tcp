import socket
import rclpy
from rclpy.node import Node
import struct
import threading 
import select



class TcpServer(Node):
    def __init__(self):
        super().__init__("tcp_server_node")

        ## server host and port
        self.host = '127.0.0.1'
        self.port = 2000
        ##

        ## creating server with that host and port
        self.server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        #


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

            rlist, _, _ = select.select([self.server_socket], [], [], 1)

            if rlist:
                self.client_socket, addr = self.server_socket.accept()
                self.get_logger().info(f"connection established with {addr}")
                self.client_socket.setblocking(False) 

            if self.client_socket:
                rlist, _, _ = select.select([self.client_socket], [], [], 0.1)
                if rlist:
                    data = self.client_socket.recv(1024)

                    if data:
                        # self.get_logger().info(f"")
                        self.get_logger().info(f"Recieved data {data.decode()}")
                        self.le_hexadecimal_to_decimal(data)

                    else:
                        self.get_logger().info("Client Disconnected")
                        self.client_socket.close()
                        self.client_socket = None
            # try:


                # if not data:


                

            # except Exception as e:
            #     self.get_logger().info(f"Error occured {e}")


    def le_hexadecimal_to_decimal(self,data):



        
        command_id = data.decode()[1:3]
        self.get_logger().info(f"Command ID received: {command_id}")

        if command_id == '03':
            self.get_logger().info(f"Recieved start command, starting sensor data transmission")
            hex_part = data[3:7]
            # self.get_logger().info(f"extracted hex part is {hex_part} and command Id is {command_id}")

            numeric_bytes = bytes.fromhex(hex_part.decode())
            # self.get_logger().info(f"Numeric bytes is {numeric_bytes}")

            payload = struct.unpack('<H',numeric_bytes)[0]
            # self.get_logger().info(f"unpacked data is {payload}")

            self.start_sending_data(payload)


        elif command_id == '09':
            self.get_logger().info("Recieved stop command, stopping sensor data transmission")
            self.stop_sending_data()

    def start_sending_data(self,interval):

        self.payload_interval = interval/1000.0

        self.timer = self.create_timer(self.payload_interval,self.send_response_to_client)
        self.get_logger().info(f"starting to send responses at {self.payload_interval} seconds")



    def send_response_to_client(self):

        supply_voltage = 12000     
        env_temp = 250             
        yaw = 100                   
        pitch = -50                 
        roll = 30                  
        command_id = 11

        packed_data = struct.pack('<BhhhhH', command_id, supply_voltage, env_temp, yaw, pitch, roll)  # using struct library for packing the values and into little endian format

        hex_message = ''.join(f'{byte:02X}' for byte in packed_data)

        formatted_message = '$' + hex_message + '0D0A'


        self.get_logger().info(f"Sent: {formatted_message}")
        self.client_socket.sendall(formatted_message.encode())




    def stop_sending_data(self):
        self.timer.cancel()
        self.get_logger().info(f"shutting down data transmission")

    def shutdown_server(self):

        self.get_logger().info("shutting down server")

        self.server_socket.close()
        self.client_socket.close()

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