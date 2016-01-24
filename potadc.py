import time, signal, sys, math, thread, pygame, os, json
import RPi.GPIO as GPIO
from Adafruit_ADS1x15 import ADS1x15
from collections import deque

os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
pygame.font.init()
pygame.mouse.set_visible(False)
lcd = pygame.display.set_mode((320, 240))

def signal_handler(signal, frame):
        #print 'You pressed Ctrl+C!'
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
#print 'Press Ctrl+C to exit'

ADS1115 = 0x01	# 16-bit ADC

# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
adc = ADS1x15(ic=ADS1115)

sps = 16
# sps can be 8, 16, 32, 64, 128, 250, 475, 860

#voltsdiff=deque([-adc.readADCSingleEnded(2,256, sps)/1000.0]*int(sps/sensitivity_factor), int(sps/sensitivity_factor))
pga = 4096
channel = 3

while True:
    print "Percentage = %0.1f " %(adc.readADCSingleEnded(channel, pga, sps)/3313.875*100)
    time.sleep(.2)
