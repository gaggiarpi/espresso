#!/usr/bin/python

import time, signal, sys, math, thread
from Adafruit_ADS1x15 import ADS1x15
from collections import deque

def signal_handler(signal, frame):
        #print 'You pressed Ctrl+C!'
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
#print 'Press Ctrl+C to exit'

ADS1015 = 0x00	# 12-bit ADC
ADS1115 = 0x01	# 16-bit ADC

# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
adc = ADS1x15(ic=ADS1115)

sps = 16
voltsdiff=[0]*8

for j in range(0,9):
    for i in range(0, len(voltsdiff)):
        voltsdiff[i] = -adc.readADCDifferential01(2048, sps)/1000.0
        time.sleep(1/sps+.01) # E.g. if sps = 250: take a measurement every 0.004 sec
    print "%.10f" % (math.fsum(voltsdiff)/len(voltsdiff))
