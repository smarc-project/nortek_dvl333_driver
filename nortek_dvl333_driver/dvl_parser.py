#!/usr/bin/env python3
"""
data parser for the Nortek DVL333.
"""
import struct
import numpy as np


__author__ = "Aldo Teran"
__author_email = "aldot@kth.se"
__license__ = "MIT"
__status__ = "Development"


class DvlParser:
    """
    Class for handling the logic for parsing a raw DVL data packet.
    """
    def __init__(self):
        self.data_type_map = {
            0x15: "Burst Data Record.",
            0x16: "Average Data Record.",
            0x17: "Bottom Track Data Record.",
            0x18: "Interleaved Burst Data Record (beam 5).",
            0x21: "Altimeter Data Record.",
            0x1B: "DVL Bottom Track Record.",
            0x1D: "DVL Water Track Record.",
            0xA0: "String Data Record, eg. GPS NMEA data, etc.",
        }

    # Header data fields.
    def parse_sync_bit(self, msg):
        '''The `sync_bit` should always be 0xA5 (or 165 in decimal).'''
        return struct.unpack('<B', msg[0:1])[0]

    def parse_header_size(self, msg):
        '''Returns the size of the header in bytes.'''
        return struct.unpack('<B', msg[1:2])[0]

    def parse_data_type(self, msg):
        '''Returns the type of the data record that follows.

        The mapping from key to data type is as follows:
            0x15 - Burst Data Record.
            0x16 - Average Data Record.
            0x17 - Bottom Track Data Record.
            0x18 - Interleaved Burst Data Record (beam 5).
            0x21 - Altimeter Data Record.
            0x1B - DVL Bottom Track Record.
            0x1D - DVL Water Track Record.
            0xA0 - String Data Record, eg. GPS NMEA data, etc.
        '''
        return struct.unpack('<B', msg[2:3])[0]

    def data_type_str(self, msg):
        '''Returns a human readable representation of the data type.'''
        data_type = self.parse_data_type(msg)
        if data_type not in self.data_type_map:
            return "Invalid data type! Key not in map!"
        return self.data_type_map[data_type]

    def parse_data_size(self, msg):
        '''Returns the number of bytes in the following Data Record.'''
        header_size = self.parse_header_size(msg)
        if header_size == 10:
            return struct.unpack('<H', msg[4:6])[0]
        else:
            return struct.unpack('<I', msg[4:8])[0]

    def parse_data_checksum(self, msg):
        '''Checksum of the following Data Record.'''
        header_size = self.parse_header_size(msg)
        if header_size == 10:
            return struct.unpack('<H', msg[6:8])[0]
        else:
            return struct.unpack('<H', msg[8:10])[0]

    def parse_header_checksum(self, msg):
        '''Checksum of the following Data Record.'''
        header_size = self.parse_header_size(msg)
        if header_size == 10:
            return struct.unpack('<H', msg[8:10])[0]
        else:
            return struct.unpack('<H', msg[10:12])[0]

    def parse_full_size(self, msg):
        '''Total size in bytes of this message.'''
        header_size = self.parse_header_size(msg)
        data_size = self.parse_data_size(msg)
        return header_size + data_size

    def print_full_header(self, msg):
        '''Pretty prints the entire header information.'''
        header_size = self.parse_header_size(msg)
        print("")
        print("----------------------")
        print("  DVL Header Info")
        print("----------------------")
        print(" - sync bit       : ", hex(self.parse_sync_bit(msg)))
        print(" - header size    : ", header_size)
        print(" - data           : ", self.data_type_str(msg))
        print(" - data size      : ", self.parse_data_size(msg))
        print(" - full size      : ", self.parse_full_size(msg))
        print(" - data checksum  : ", self.parse_data_checksum(msg))
        print(" - header checksum: ", self.parse_header_checksum(msg))

        if self.parse_data_type(msg) == 0x1B:  # Bottom track data.
            print(self.parse_bt_payload(msg, header_size))

        print("----------------------")
        print("")


    # Common data fields.
    def parse_version(self, msg, header_size):
        '''Version number of the Data Record Definition (should be 3).'''
        return struct.unpack('<B', msg[header_size:header_size+1])[0]

    # Bottom track (bt) data fields for Data Record.
    def parse_bt_version(self, msg, header_size):
        '''Version number of the Data Record Definition (should be 3).'''
        return struct.unpack('<B', msg[header_size:header_size+1])[0]

    def parse_bt_offset_of_data(self, msg, header_size):
        '''Number of bytes from start of record to start of actual data.'''
        pos = header_size + 1
        return struct.unpack('<B', msg[pos:pos+1])[0]

    def parse_bt_serial_number(self, msg, header_size):
        '''Instrument serial number from factory.'''
        pos = header_size + 2
        return struct.unpack('<I', msg[pos:pos+4])[0]

    def parse_bt_date_time(self, msg, header_size):
        '''The date and time of the data record.

        Year: Is given as years from 1900.
        Month: January is 0.
        Milli seconds: Are given by the hundreds. That is as
        desi seconds.
        '''
        pos = header_size + 6
        year = 1900 + struct.unpack('<B', msg[pos:pos+1])[0]
        month = 1 + struct.unpack('<B', msg[pos+1:pos+2])[0]
        secs = struct.unpack('<I', msg[pos+2:pos+6])[0] / 10.0
        return {'year': year, 'month': month, 'secs': secs}

    def parse_bt_microsecs(self, msg, header_size):
        '''Remaining micro seconds (object has millisec resolution).'''
        pos = header_size + 12
        return struct.unpack('<H', msg[pos:pos+2])[0]

    def parse_bt_timestamp(self, msg, header_size):
        '''Parsing the date time and the microsec timestamp.'''
        datetime = self.parse_bt_date_time(msg, header_size)
        datetime['secs'] += self.parse_bt_microsecs(msg, header_size) * 1e-6
        return datetime

    def parse_bt_sound_speed(self, msg, header_size):
        '''Sound speed in meters / second.'''
        pos = header_size + 24
        return struct.unpack('<f', msg[pos:pos+4])[0]

    def parse_bt_temperature(self, msg, header_size):
        '''Temperature in Celsius.'''
        pos = header_size + 28
        return struct.unpack('<f', msg[pos:pos+4])[0]

    def parse_bt_pressure(self, msg, header_size):
        '''Pressure in Bar.'''
        pos = header_size + 32
        return struct.unpack('<f', msg[pos:pos+4])[0]

    def parse_bt_payload(self, msg, header_size):
        '''Actual bottom track payload.

        We have four beams (1, 2, 3, 4). For each beam, we have:
            - Velocity [m/s]
            - Distance [m]
            - Stddev of the velocity uncertainty [m/s]
            - Dt1 [s]
            - Dt2 [s]
            - Time velocity [s]

        We have four dimensions (X, Y, Z1, Z2). For each dimension, we have:
            - Velocity [m/s]
            - Stddev of the velocity uncertainty in said dimension [m/s]
            - Dt1 [s]
            - Dt2 [s]
            - Time velocity (duration of velocity) [s]
        '''
        # Parse the per-beam data.
        beam_data = dict()
        for beam_num in range(4):
            vel_offset = header_size + 36 + 4 * beam_num
            vel = struct.unpack('<f', msg[vel_offset:vel_offset+4])[0]

            dist_offset = header_size + 52 + 4 * beam_num
            dist = struct.unpack('<f', msg[dist_offset:dist_offset+4])[0]

            sigma_offset = header_size + 68 + 4 * beam_num
            sigma = struct.unpack('<f', msg[sigma_offset:sigma_offset+4])[0]

            dt1_offset = header_size + 84 + 4 * beam_num
            dt1 = struct.unpack('<f', msg[dt1_offset:dt1_offset+4])[0]

            dt2_offset = header_size + 100 + 4 * beam_num
            dt2 = struct.unpack('<f', msg[dt2_offset:dt2_offset+4])[0]

            tvel_offset = header_size + 116 + 4 * beam_num
            tvel = struct.unpack('<f', msg[tvel_offset:tvel_offset+4])[0]

            beam_data[beam_num] = {
                'velocity': vel,
                'distance': dist,
                'sigma': sigma,
                'dt1': dt1,
                'dt2': dt2,
                'tvel': tvel
            }

        # Parse the per-dimension data.
        dim_data = dict()
        for dim_num in range(4):
            vel_offset = header_size + 132 + 4 * beam_num
            vel = struct.unpack('<f', msg[vel_offset:vel_offset+4])[0]

            sigma_offset = header_size + 148 + 4 * beam_num
            sigma = struct.unpack('<f', msg[sigma_offset:sigma_offset+4])[0]

            dt1_offset = header_size + 164 + 4 * beam_num
            dt1 = struct.unpack('<f', msg[dt1_offset:dt1_offset+4])[0]

            dt2_offset = header_size + 180 + 4 * beam_num
            dt2 = struct.unpack('<f', msg[dt2_offset:dt2_offset+4])[0]

            tvel_offset = header_size + 196 + 4 * beam_num
            tvel = struct.unpack('<f', msg[tvel_offset:tvel_offset+4])[0]

            dim_data[dim_num] = {
                'velocity': vel,
                'sigma': sigma,
                'dt1': dt1,
                'dt2': dt2,
                'tvel': tvel
            }

        return (beam_data, dim_data)
