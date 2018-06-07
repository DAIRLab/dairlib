#!/usr/bin/env python3
import serial.tools.list_ports
from struct import unpack
import sys
from math import log
from ctypes import c_ushort


class BMS(object):

    def __init__(self):
        # BMS Registers
        self.CHANNELS = 0x03
        self.AUX0_OFST = 0xD4
        self.AUX1_OFST = 0xD6
        self.AUX2_OFST = 0xD8
        self.AUX3_OFST = 0xDA
        self.AUX4_OFST = 0xDC
        self.MAGIC1 = 0x82
        self.MAGIC2 = 0xFC
        self.CTO = 0x28
        self.SERIAL_NUMBER = 0xC8  # 2 Bytes for serial number
        self.EE_BURN = 0xFA  # Tracks EEPROM Writes
        self.DEV_CTRL = 0x0C  # used for writing EEPROM

        # Thermister Constants
        self.R25 = 10
        self.B = 3435
        self.T0 = 273.15
        self.T25 = 298.15

        # Voltage/state of charge vectors for interpolation
        self.SOC_volts = [4.18415, 4.1335, 4.12371, 4.11416, 4.09691, 4.07317, 4.04483, 4.00714, 3.96639,
                          3.93094, 3.90218, 3.87735, 3.8488, 3.81696, 3.78343, 3.75219, 3.72205, 3.69351,
                          3.67999, 3.64979, 3.63051, 3.59738, 3.56685, 3.53505, 3.50685, 3.4725, 3.41747,
                          3.33443, 3.23209, 3.12087, 2.96037, 2.50725, 0]

        self.SOC_perc = [1, 0.967742, 0.935484, 0.903226, 0.870968, 0.83871, 0.806452, 0.774194, 0.741935,
                         0.709677, 0.677419, 0.645161, 0.612903, 0.580645, 0.548387, 0.516129, 0.483871,
                         0.451613, 0.419355, 0.387097, 0.354839, 0.322581, 0.290323, 0.258065, 0.225806,
                         0.193548, 0.16129, 0.129032, 0.0967742, 0.0645161, 0.0322581, 0, 0]

        ports = serial.tools.list_ports.comports()
        usb_ports = []
        i = 0
        for port in ports:
            try:
                serTest = serial.Serial(port.device, 250000, timeout=1)
                serTest.close()
                serTest.open()
                serTest.close()
                usb_ports.append(port)
            except:
                pass

        if not usb_ports:
            print('ERROR: No compatible batteries connected')
            while True:
                pass

        if len(usb_ports) == 1:
            port_num = 0
        else:
            print('\nSelect BMS Port from Available USB-Serial Ports:')
            for port in ports:
                print('[' + str(i) + '] ' + port.description)
                i += 1

            ask = True
            while ask:
                port_num = int(input('Choose Port: '))
                if (port_num >= i) | (port_num < 0):
                    print('Invalid Entry\n\n', file=sys.stderr)
                else:
                    ask = False

        self.ser = serial.Serial(usb_ports[port_num].device, 250000, timeout=1)
        self.ser.close()
        self.ser.open()

    def read_from_register(self, reg, num_bytes):
        # Command frame (see page 52 of datasheet)
        cf = bytearray.fromhex('81 00')

        # Register and data size frame
        df = reg.to_bytes(1, byteorder='big') + (num_bytes - 1).to_bytes(1, byteorder='big')

        # Completed command (includes crc)
        frm = cf + df + self.crc_calc(cf + df)

        self.ser.write(frm)

        # first byte is number of bytes in packet minus 1
        size_in = self.ser.read(1)
        size = unpack('>B', size_in)[0] + 1
        data = self.ser.read(size)

        # last two bytes are CRC
        crc_in = self.ser.read(2)
        check = self.crc_calc(size_in + data + crc_in)
        if unpack('>H', check)[0]:
            print('**************************', file=sys.stderr)
            print('CRC Error: invalid packet!', file=sys.stderr)
            print('**************************', file=sys.stderr)

        return data

    # Passive Read reads BMS data packets that are on the bus without sending command frames
    def passive_read(self):
        # Flush input buffer to only get new data
        self.ser.reset_input_buffer()

        # First byte is number of bytes in packet minus 1
        size_in = self.ser.read(1)
        # If nothing is returned, exit function
        if size_in == b'':
            return None

        # Process the rest of the Data
        size = unpack('>B', size_in)[0] + 1
        data = self.ser.read(size)

        # Last two bytes are CRC
        crc_in = self.ser.read(2)
        check = self.crc_calc(size_in + data + crc_in)
        if unpack('>H', check)[0]:
            print('**************************', file=sys.stderr)
            print('CRC Error: invalid packet!', file=sys.stderr)
            print('**************************', file=sys.stderr)

        return data

    def write_to_register(self, reg, data, size):
        # Command frame (see page 52 of datasheet)
        # 144 is 0x90 in decimal, 00 is device id
        cf = (size + 144).to_bytes(1, byteorder='big') + bytearray.fromhex('00')

        # register and data size frame
        if (type(data).__name__ == 'bytes') or (type(data).__name__ == 'bytearray'):
            df = reg.to_bytes(1, byteorder='big') + data
        else:
            df = reg.to_bytes(1, byteorder='big') + data.to_bytes(size, byteorder='big')

        frm = cf + df + self.crc_calc(cf + df)

        self.ser.write(frm)

    def read_aux_offsets(self):
        aux_ofst = [0]*5
        for k in range(5):
            # read from registers in reverse order
            ofst_in = self.read_from_register(self.AUX0_OFST + 2*(4-k), 2)

            # need to pad 1's for proper sign conversion: 10-bit signed number in 16-bit register
            if ofst_in[0] >= 2:
                ofst_bytes = bytes([ofst_in[0] | 252, ofst_in[1]])
            else:
                ofst_bytes = ofst_in
            aux_ofst[k] = self.bytearray_to_intarray(ofst_bytes, 2, '>h')[0] * 5/65536

        return aux_ofst

    # Uses the thermister formula to convert voltage measurements to temperature
    def volts_to_temp(self, therm_V):
        therm_R = 1/(therm_V/(5.3*10)) - 10
        therm_C = (-1/(log(therm_R/self.R25)/(-self.B) - 1/self.T25)) - self.T0
        return therm_C

    def crc_calc(self, data_packet):
        crc = self.uint16(0)
        for i in range(len(data_packet)):
            crc ^= data_packet[i]
            for j in range(8):
                crc = (crc >> 1) ^ (int('0xa001', 16) if (crc & int('0x0001', 16)) else int('0x0000', 16))
            crc = self.uint16(crc)
        crc = crc.to_bytes(2, byteorder='little')
        return crc

    # Estimates state of charged based on current voltage. Assumes no current
    # Uses interpolation of data vectors from Mikhail's data
    def soc_estimate(self, voltage):
        for idx, v in enumerate(self.SOC_volts):
            if v < voltage:
                break
        if idx == 0:
            soc = 1
        elif idx == len(self.SOC_volts)-1:
            soc = 0
        else:
            soc = self.SOC_perc[idx] + (voltage - self.SOC_volts[idx])*(self.SOC_perc[idx-1] - self.SOC_perc[idx])/(self.SOC_volts[idx-1] - self.SOC_volts[idx])

        return soc*100

    @staticmethod
    def uint16(data):
        '''outputs an unsigned 16 bit int'''
        return c_ushort(data).value

    # Displays an array as a row in a table
    @staticmethod
    def display_array(title, array):
        print('{:>22}'.format(title) + ':  | ', end='')
        for k in range(len(array)):
            print('{:^7.6}'.format(str(array[k])) + ' | ', end='')
        print('')

    # -----------------------------------------------------
    # restructures a byte array into an int array given word size and format string (used in struct.unpack)
    # format strings:   > : big-endian (correct byte order)
    # Size modifiers (lowercase for signed, uppercase for unsigned):
    #       b/B : 8-bit
    #       h/H : 16-bit
    #       i/I : 32-bit
    #       q/Q : 64-bit
    # -----------------------------------------------------
    @staticmethod
    def bytearray_to_intarray(array, word_size, format_str):
        # check if input is correct type
        array_type = type(array).__name__
        if (array_type != 'bytearray') and (array_type != 'bytes'):
            return 0

        length = len(array) // word_size
        if (length * word_size - len(array)) != 0:
            print('*********************************', file=sys.stderr)
            print('Warning: Byte Array size mismatch', file=sys.stderr)
            print('*********************************', file=sys.stderr)

        output = [0] * length  # initialize output array
        for i in range(length):
            output[i] = unpack(format_str, array[word_size * i: word_size * (i + 1)])[0]

        return output

    # takes input in mV and outputs a number that can be loaded into the BMS registers
    @staticmethod
    def offset_mV_to_bits(ofst):
        ofst_int = int((ofst * 65536) // 5)
        if ofst_int > 511:
            ofst_int = 511
        elif ofst_int < -512:
            ofst_int = -512

        ofst_bits = ofst_int.to_bytes(2, byteorder='big', signed=True)
        # clear the first 6 bits of the register to output to chip (see page 96 of datasheet)
        ofst_out = bytes([ofst_bits[0] & 3, ofst_bits[1]])
        return ofst_out