# nortek_dvl333_driver

This is the ROS2 driver for the Nortek dvl333 used on Lolo.
This is adapted from the ROS1 version, https://gits-15.sys.kth.se/MaritimeRoboticsLaboratory/nortek_dvl333_driver

# Message Definition
The mesage definition can be found in the ROS2 package nortek_dvl333_msgs.

# DVL parser
Parsing is performed by dvl_parser.py and can be tested with dvl_parser_test.py with the data found in the data folder. 

# DVL nodes
dvl_node.py provides a node for connectiong to DVL via TCP or UDP, parsing data and publishing 

# Parameters
- tcp_mode (bool): Defines whether connection is TCP (True) or UDP (False)
- socket_ip (str): The IP address of the DVL
- socket_port (int): The port number of the DVL
- buffer_size_bytes (int): The buffer size us for streaming date

# TODO
- [X] Port dvl_node_udp.py, see below
- [X] Port dvl_node.py, combined UDP functionality
- [X] Confirm that the above nodes have been properly setup within ROS2
- [ ] Testing with actual hardware

