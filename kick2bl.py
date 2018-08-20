#!/usr/bin/env python
# import serial, sys
# serialPort = sys.argv[1]
# ser = serial.Serial(
#     port=serialPort,
#     baudrate=1200,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     bytesize=serial.EIGHTBITS
# )
# ser.isOpen()
# ser.close() # always close port

import argparse
import serial
import os.path
from time import sleep

parser = argparse.ArgumentParser(description='Reset an Arduino')
parser.add_argument('port', nargs=1, help='Serial device e.g. /dev/ttyACM0')
args = parser.parse_args()


# ser.setBaudrate (1200)
ser = serial.Serial(args.port[0], 1200)

ser.setRTS(True)  # RTS line needs to be held high and DTR low
ser.setDTR(False) # (see Arduino IDE source code)
ser.close()
sleep(1)

while not os.path.exists(args.port[0]):
    print('Waiting for %s to come back' % args.port[0])
    sleep(0.1)
