# nortek_dvl333_driver

This is the ROS2 driver for the Nortek dvl333 used on Lolo.

This is adapted from the ROS1 version, https://gits-15.sys.kth.se/MaritimeRoboticsLaboratory/nortek_dvl333_driver

# Message Definition
The mesage definition is included here for completeness but is not used.
Please see the lolo_msg packaged for the definition of the utilized msg.

# DVL parser
Parsing is performed by dvl_parser.py and can be tested with dvl_parser_test.py with the data found in the data folder. 

# DVL nodes

# TODO
- [ ] Port dvl_node_udp.py
- [ ] Port dvl_node.py
- [ ] Confirm that the above nodes have been properly setup within ROS2

