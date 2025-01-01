import socket
import rclpy
from rclpy.node import Node
import struct
from std_msgs.msg import UInt16, Int16
from std_srvs.srv import Trigger
import threading



class TcpClient(Node):
    def __init__(self):
        super().__init__("tcp_client_node")
        ## creating publishers for publishing the respective values in the topics
        self.pub_voltage = self.create_publisher(UInt16,'sensor/supply_voltage',10)
        self.pub_temp = self.create_publisher(Int16,'sensor/environment_temperature',10)
        self.pub_yaw = self.create_publisher(Int16,'sensor/yaw',10)
        self.pub_pitch = self.create_publisher(Int16,'sensor/pitch',10)
        self.pub_roll = self.create_publisher(Int16,'sensor/roll',10)
        ##



        ## creating services 
        self.srv_stop = self.create_service(Trigger,'stop_command',self.stop_command_callback)  # start command service
        self.srv_start = self.create_service(Trigger,'start_command',self.start_command_callback) # stop command service
        self.get_logger().info('Service ready to be triggered!') 
        ##


        ##commands
        self.start_command = '#03' # for start
        self.stop_command = '#09' # for stop
        self.cr_lf = '0D0A' # for carriage return/ line feed
        ##


        ## declaring parameter 
        self.declare_parameter('interval',1000)
        self.interval = self.get_parameter('interval').value
        self.get_logger().info(f"the interval is {self.interval}")
        ##

        ## server host, port declaration and connection
        self.host = '127.0.0.1'
        self.port = 2000
        self.client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.connect_to_server()
        ##

        
        # starting automatically with default interval of 1000 milliseconds
        self.send_command(f"{self.start_command}{self.decimal_to_hexadecimal(self.interval)}{self.cr_lf}")

        # self.send_command(f"{self.stop_command}")


        # listening for sensor response
        self.listen_for_responses()




    

    def listen_for_responses(self):
        response_thread = threading.Thread(target=self._listen_for_responses)
        response_thread.start()



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
        # self.get_logger().info(f"the data to be sent is {struct.pack('<H',decimal_number).hex().upper()}")
        return struct.pack('<H',decimal_number).hex().upper()  # making payload into hexadecimal and little endian format for transferring data




    def send_command(self,command):
        try:
            self.get_logger().info(f"data sent {command}")

            self.client_socket.sendall(command.encode())

        except Exception as e:
            print("error occured",e)


    def _listen_for_responses(self):

        try:
            while rclpy.ok():
                response = self.client_socket.recv(1024)
                if response:
                    # self.get_logger().info(f"Response from server {response.decode()}")
                    self.unpack_response(response)

        except Exception as e:
            self.get_logger().info(f"error occured {e}")

        finally:
            self.shutdown_client()

    def unpack_response(self, response):


        command_id = response.decode()[1:3]
        # self.get_logger().info(f"Command ID: {command_id}")

        hex_part = response[1:-4]  
        # self.get_logger().info(f"Hex part: {hex_part}")

        numeric_bytes = bytes.fromhex(hex_part.decode())
        # self.get_logger().info(f"Numeric bytes: {numeric_bytes}")

        command_id, supply_voltage, env_temp, yaw, pitch, roll = struct.unpack('<BhhhhH', numeric_bytes) # unpacking the values which were recieved 
        

        self.pub_voltage.publish(UInt16(data=supply_voltage))
        self.pub_temp.publish(Int16(data=env_temp))
        self.pub_yaw.publish(Int16(data=yaw))
        self.pub_pitch.publish(Int16(data=pitch))
        self.pub_roll.publish(Int16(data=roll))


        # self.get_logger().info(f"command_id: {command_id}")
        # self.get_logger().info(f"supply_voltage: {supply_voltage}")
        # self.get_logger().info(f"env_temp: {env_temp}")
        # self.get_logger().info(f"yaw: {yaw}")
        # self.get_logger().info(f"pitch: {pitch}")
        # self.get_logger().info(f"roll: {roll}")

    def shutdown_client(self):

        self.get_logger().info("shutting down client")

        self.client_socket.close()


    def start_command_callback(self,request,response):

        self.get_logger().info(f"Start command is triggered")
        self.interval = self.get_parameter('interval').value
        self.get_logger().info(f"{self.interval}")
        self.send_command(f"{self.start_command}{self.decimal_to_hexadecimal(self.interval)}{self.cr_lf}")
        response.success = True
        response.message = 'start command sent'
        return response


    def stop_command_callback(self,request,response):
        self.get_logger().info(f"Stop service is triggered")
        self.send_command(f"{self.stop_command}{self.cr_lf}")
        response.success = True
        response.message = "stop command sent"
        return response




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