#!/usr/bin/env python3

import time
from dvl_parser import DvlParser

parser = DvlParser()

f = open("../data/dvl.bin",'rb')
buffer = b''

for b in f.read():
    failed = False
    buffer+=bytes([b])

    #Check sync byte
    if parser.parse_sync_bit(buffer) == 0xA5:
        #print("Sync bit OK")
        # make sure there is enough data in the buffer to parse header
        if len(buffer) < 12: 
            continue
        # Check header
        header_size = parser.parse_header_size(buffer)
        if header_size in [10, 12] :
            #print("\tHeader size OK")
            #check data size
            expected_package_size = parser.parse_full_size(buffer)
            #print("\tExpected package size: " + str(expected_package_size) + " | " + str(len(buffer)))
            if(len(buffer) == expected_package_size):
                #print("\t\t*****************")
                #print("\t\tAll data received")
                #print("\t\t*****************")
                
                #parser.print_full_header(buffer)
                #print("" + parser.data_type_str(buffer))

                if parser.parse_data_type(buffer) == 0x1B:  # Bottom track data.
                    (beam_data, offset_data) = parser.parse_bt_payload(buffer, header_size)
                    print("" + str((beam_data[0])['velocity']) + ", " + str((beam_data[1])['velocity']) + ", " + str((beam_data[2])['velocity']) + ", " + str((beam_data[3])['velocity']))
                    #print("" + str((beam_data[0])['sigma']) + ", " + str((beam_data[1])['sigma']) + ", " + str((beam_data[2])['sigma']) + ", " + str((beam_data[3])['sigma']))
                    #print("" + str((beam_data[0])['distance']) + ", " + str((beam_data[1])['distance']) + ", " + str((beam_data[2])['distance']) + ", " + str((beam_data[3])['distance']))
                    #print("" + str(parser.parse_bt_temperature(buffer, header_size)))
                    #print("----------")
                    #print(offset_data)

                #time.sleep(1)
                # Fetch the "timestamp" and the payload.
                #t = parser.parse_bt_timestamp(buffer, header_size)
                #payload = parser.parse_bt_payload(buffer, header_size)
                #print(payload)
                #removed old data from buffer
                buffer = buffer[expected_package_size:]
        else:
            print("Header size wrong : " + str(parser.parse_header_size(buffer)))
            failed = True
    else:
        failed = True
    
    if(failed):
        buffer = buffer[1:]