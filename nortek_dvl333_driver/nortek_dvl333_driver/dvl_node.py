#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import socket

# from dvl_parser import DvlParser

from std_msgs.msg import String

from lolo_msgs.msg import BottomTrack

from lolo_msgs.msg import Topics as LoloTopics
# from lolo_msgs.msg import Links as LoloLinks

# Allow for imports from both ROS2 and direct exectution
try:
    from .dvl_parser import DvlParser
except ImportError:
    from dvl_parser import DvlParser
    


class DvlNode(Node):
    """
    Class to handle the data coming from a nortek dvl via tcp (default) or udp.
    Defaults can be set for the individual protocols.
    """

    # Default namespace
    defualt_namespace = "lolo"
    
    # Default parameter values and names
    default_tcp_mode = True

    default_sonar_ip_tcp = "192.168.1.95"  
    default_sonar_port_tcp = 8112

    default_sonar_ip_udp = "0.0.0.0"
    default_sonar_port_udp = 8112

    defualt_buffer_size_bytes = 512000

    param_tcp_mode = 'tcp_mode'
    param_sonar_ip = 'sonar_ip'
    param_sonar_port = 'sonar_port'
    param_buffer_size_bytes = 'buffer_size_bytes'

    # Class attributes
    sonar_ip: str  # Set by parameter
    sonar_port: int  # Set by parameter
    buffer_size_bytes: int  # Set by parameter
    tcp_mode: bool  # Set by parameter, default: True
    tcp_sock: socket.socket
    udp_sock: socket.socket
    buffer: bytes
    p: DvlParser
    publish_qos_depth: int = 128  # Used by publisher
    parser_timer_period: float = 0.1  # Controls the rate that the dvl buffer is parsed and published at 

    def __init__(self, namespace=None):
        # Check if namespace is set, if not use default_namespace
        if namespace is None:
            namespace = self.defualt_namespace

        super().__init__('dvl_node', namespace=namespace)

        # Declare and read parameters parameters.
        self.declare_and_read_node_parameters()
        
        if self.tcp_mode:
            self.get_logger().info(f"[{self.get_namespace()}/{self.get_name()}] Connecting to DVL via TCP")
        else:
            self.get_logger().info(f"[{self.get_namespace()}/{self.get_name()}] Connecting to DVL via UDP")
        self.get_logger().info(f"[{self.get_namespace()}/{self.get_name()}] Connecting to DVL at {self.sonar_ip}:{self.sonar_port}...")

        # Create the TCP socket.
        self.connect_socket()

        # Build our parser.
        self.p = DvlParser()

        # Create the buffer where we'll store the data being streamed in.
        self.buffer = b''

        # Setup our raw array publisher and the image publisher.
        # self.bt_pub = rospy.Publisher("dvl/bottomtrack",BottomTrack, queue_size=128)
        odom_qos_profile = QoSProfile(depth=self.publish_qos_depth)
        self.bt_pub = self.create_publisher(msg_type=BottomTrack,
                                            topic=LoloTopics.DVL_TOPIC,
                                            qos_profile=odom_qos_profile)
        
        # Create a timer to run the parse and publish function
        self.timer = self.create_timer(0.1, self.parse_and_publish)

    def declare_and_read_node_parameters(self):
        """
        Responsible setting attributes via ROS2 parameters.
        The first step is to declare the parameters, and set defaults, then to read them.
        The default values are determined by the mode specified.
        """
        # Declare and read tcp_mode
        self.declare_parameter(self.param_tcp_mode, self.default_tcp_mode)
        self.tcp_mode = self.get_parameter(self.param_tcp_mode).value

        # Tcp mode determines the default values used when declaring parameters
        if self.tcp_mode:
            self.declare_parameter(self.param_sonar_ip, self.default_sonar_ip_tcp)
            self.declare_parameter(self.param_sonar_port, self.default_sonar_port_tcp)
            self.declare_parameter(self.param_buffer_size_bytes, self.defualt_buffer_size_bytes)
        else:
            self.declare_parameter(self.param_sonar_ip, self.default_sonar_ip_udp)
            self.declare_parameter(self.param_sonar_port, self.default_sonar_port_udp)
            self.declare_parameter(self.param_buffer_size_bytes, self.defualt_buffer_size_bytes)

        # Read the parameters
        self.sonar_ip = self.get_parameter(self.param_sonar_ip).value
        self.sonar_port = self.get_parameter(self.param_sonar_port).value
        self.buffer_size_bytes = self.get_parameter(self.param_buffer_size_bytes).value

    def connect_socket(self):
        """Try to connect the socket. TCP or UDP determined by tcp_mode"""
    
        if self.tcp_mode:
            # Attempt to make tcp connection
            while rclpy.ok():
                try:
                    self.sock.connect((self.sonar_ip, self.sonar_port))
                    # Set a timout for the socket.
                    self.sock.settimeout(2)
                    self.get_logger().info(f"TCP socket connected to: {self.sonar_ip}:{self.sonar_port}")
                    return
                except Exception as e:
                    self.get_logger().error(f"Failed to set up TCP socket: {e}. Retrying...")
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
        else:
            # Attempt to make udp conection
            while rclpy.ok():
                try:
                    self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    self.udp_socket.bind((self.sonar_ip, self.sonar_port))

                    # Set a timeout 
                    self.udp_socket.settimeout(2)

                    self.get_logger().info(f"UDP socket bound to: {self.sonar_ip}:{self.sonar_port}")
                    return

                except socket.error as e:
                    self.get_logger().error(f"Failed to set up UDP socket: {e}. Retrying...")
                    self.udp_socket.close()
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
        
    def publish_BT(self, beam_data):
        #print("-----")
        #print("velocity: " + str((beam_data[0])['velocity']) + ", " + str((beam_data[1])['velocity']) + ", " + str((beam_data[2])['velocity']) + ", " + str((beam_data[3])['velocity']))
        #print("sigma: " + str((beam_data[0])['sigma']) + ", " + str((beam_data[1])['sigma']) + ", " + str((beam_data[2])['sigma']) + ", " + str((beam_data[3])['sigma']))
        #print("distance: " + str((beam_data[0])['distance']) + ", " + str((beam_data[1])['distance']) + ", " + str((beam_data[2])['distance']) + ", " + str((beam_data[3])['distance']))

        msg = BottomTrack()
        msg.velBeam0 = (beam_data[0])['velocity']
        msg.distBeam0 = (beam_data[0])['distance']
        msg.fomBeam0 = (beam_data[0])['sigma']

        msg.velBeam1 = (beam_data[1])['velocity']
        msg.distBeam1 = (beam_data[1])['distance']
        msg.fomBeam1 = (beam_data[1])['sigma']

        msg.velBeam2 = (beam_data[2])['velocity']
        msg.distBeam2 = (beam_data[2])['distance']
        msg.fomBeam2 = (beam_data[2])['sigma']

        msg.velBeam3 = (beam_data[3])['velocity']
        msg.distBeam3 = (beam_data[3])['distance']
        msg.fomBeam3 = (beam_data[3])['sigma']

        self.bt_pub.publish(msg)

    def parse_and_publish(self):
        """Parse and publish the data received from the TCP or UDP socket."""
        try:
            if self.tcp_mode:
                data = self.tcp_sock.recv(self.buffer_size_bytes)
            else:
                # Fetch the initial chunk.
                data, _ = self.udp_socket.recvfrom(self.buffer_size_bytes)

            for b in data:
                self.buffer += bytes([b])
                failed = False

                if self.p.parse_sync_bit(self.buffer) == 0xA5:
                    if len(self.buffer) < 12:
                        continue
                    
                    header_size = self.p.parse_header_size(self.buffer)
                    if header_size in [10, 12]:
                        expected_package_size = self.p.parse_full_size(self.buffer)
                        if len(self.buffer) == expected_package_size:
                            if self.p.parse_data_type(self.buffer) == 0x1B:
                                beam_data, _ = self.p.parse_bt_payload(self.buffer, header_size)
                                self.publish_BT(beam_data)

                            self.buffer = self.buffer[expected_package_size:]
                    else:
                        self.get_logger().error("Header size wrong: " + str(header_size))
                        failed = True
                else:
                    failed = True

                if failed:
                    self.buffer = self.buffer[1:]
        except socket.timeout as e:
            if self.tcp_mode:
                self.get_logger().error(f"TCP socket timed out: {e}")
                self.connect_socket()
            else:
                self.get_logger().error(f"UDP socket timed out: {e}")
                self.connect_socket()


def main():
    rclpy.init()
    dvl_node = DvlNode()
    rclpy.spin(dvl_node)
    dvl_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
