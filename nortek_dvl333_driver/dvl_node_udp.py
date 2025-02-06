#!/usr/bin/env python3
import rospy
import socket   
from dvl_parser import DvlParser
from nortek_dvl333_driver.msg import BottomTrack


class DvlNode:
    """
    Class to handle the data coming from a nortek dvl via tcp.
    """
    BUFFER_SIZE_BYTES = 512000

    def __init__(self):
        
        host='0.0.0.0'
        port=8112
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_socket.bind((host, port))

        # Set a timout for the socket.
        self.udp_socket.settimeout(2)

        # Build our parser.
        self.p = DvlParser()

        # Create the buffer where we'll store the data being streamed in.
        self.buffer = b''

        # Setup our raw array publisher and the image publisher.
        self.bt_pub = rospy.Publisher("dvl/bottomtrack",BottomTrack, queue_size=128)

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
        """
        Parse and publish the data recived from the ICP socket.
        """
        # Get data from udp socket
        try:
            # Fetch the initial chunk.
            data, addr = self.udp_socket.recvfrom(self.BUFFER_SIZE_BYTES)
            
            for b in data:
                #Put the data into the buffer and run the parser
                failed = False
                self.buffer+=bytes([b])

                #Check sync byte
                if self.p.parse_sync_bit(self.buffer) == 0xA5:
                    # make sure there is enough data in the buffer to parse header
                    if len(self.buffer) < 12: 
                        continue
                    # Check header
                    header_size = self.p.parse_header_size(self.buffer)
                    if header_size in [10, 12] :
                        #check data size
                        expected_package_size = self.p.parse_full_size(self.buffer)
                        if(len(self.buffer) == expected_package_size):

                            if self.p.parse_data_type(self.buffer) == 0x1B:  # Bottom track data.
                                (beam_data, offset_data) = self.p.parse_bt_payload(self.buffer, header_size)
                                self.publish_BT(beam_data)

                            #remove old data from self.buffer
                            self.buffer = self.buffer[expected_package_size:]
                    else:
                        print("Header size wrong : " + str(self.p.parse_header_size(self.buffer)))
                        failed = True
                else:
                    failed = True
                
                if(failed):
                    self.buffer = self.buffer[1:]
                #TODO check for max buffer size
        except socket.timeout as e:
            print("Error: " + str(e))
            rospy.logerr("UDP socket timed out, verify connection.")
            return

def main():
    """
    Main method for the ROS node.
    """
    rospy.init_node('wbms_parser')
    rospy.loginfo("Starting the WBMS sonar parsing node...")
    dvl = DvlNode()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        dvl.parse_and_publish()
        rate.sleep()

if __name__ == "__main__":
    main()
