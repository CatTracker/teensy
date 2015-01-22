#!/usr/bin/env python
# -*- coding: ms949 -*-
#
#     python_example.py 
#     2014.07.05 ('c')void 
#
#

import sys
import time 
import serial 
import traceback

def read_example(serial_device):
    print 'START TEST(%s)'%(serial_device)
    
    try:
        serial_port = serial.Serial(serial_device, 38400, timeout=1.0)
    except serial.serialutil.SerialException:
        print 'Can not open serial port(%s)'%(serial_device)
        traceback.print_exc()
        return 
    
    
    while True:
      line = serial_port.readline().strip()
      print line

    serial_port.close()    
    
    print 'END OF TEST'

if __name__ == '__main__': 
    if(len(sys.argv) < 2):
        serial_device = '/dev/ttyACM0'
    else : 
        serial_device = sys.argv[1]
                        
    read_example(serial_device)
