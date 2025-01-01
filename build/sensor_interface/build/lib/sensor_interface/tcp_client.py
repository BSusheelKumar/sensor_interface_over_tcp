import socket
import rclpy
from rclpy.node import Node
import struct

class TcpClient(Node):
    def __init__(self):
        super().__init__("tcp_client_node")


        self.start_command = '#03'
        self.stop_command = '#09'
        cr_lf = '0D0A'

        self.declare_parameter('interval',1000)
        self.interval = self.get_parameter('interval').value

        self.get_logger().info(f"the parameter value is {self.interval}")

        self.host = '127.0.0.1'
        self.port = 2000
        self.client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.connect_to_server()

        self.send_command(f"{self.start_command}{self.decimal_to_hexadecimal(self.interval)}{cr_lf}")

        # self.send_command(f"{self.stop_command}")

        self.listen_for_responses()

    def connect_to_server(self):
        try:
            self.client_socket.connect((self.host,self.port))
            self.get_logger().info("Connected to server")
        except ConnectionRefusedError:
            self.get_logger().info("Unable to connect to server, check if the server is running")
        except socket.timeout:
            self.get_logger().info("server timeout")
        except Exception as e:
            self.get_logger().info(f"Error occured{e}")
        


    def decimal_to_hexadecimal(self,decimal_number):
        self.get_logger().info(f"the data to be sent is {struct.pack('<H',decimal_number).hex().upper()}")
        return struct.pack('<H',decimal_number).hex().upper()




    def send_command(self,command):
        try:
            self.get_logger().info(f"data sending is {command}")

            self.client_socket.sendall(command.encode())

        except Exception as e:
            print("error occured",e)


    def listen_for_responses(self):

        try:
            while rclpy.ok():
                response = self.client_socket.recv(1024)
                if response:
                    self.get_logger().info(f"Response from server {response.decode()}")

        except Exception as e:
            self.get_logger().info(f"error occured {e}")

        finally:
            self.shutdown_client()

    def shutdown_client(self):

        self.get_logger().info("shutting down client")

        self.client_socket.close()








def main(args=None):
    rclpy.init(args=args)
    node = TcpClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:

        node.get_logger().info("Shutting down TCP Client Node.")

    finally:
        node.shutdown_client()
        node.destroy_node()
        rclpy.shutdown()

if __name__ =="__main__":
    main()