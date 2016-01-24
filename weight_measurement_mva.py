#!/usr/bin/python

# White wire: output+
# Green/blue wire: output-
# Red: VCC excitation voltage +
# Black: GND excitation voltage -

# Wiring between INA125 and load cell
# White goes to 6
# Green/blue goes to 7
# Red goes to 4
# Black goes to GND


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

ADS1015 = 0x00	# 12-bit ADC
ADS1115 = 0x01	# 16-bit ADC

# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
adc = ADS1x15(ic=ADS1115)
adc0 = ADS1x15(ic=ADS1115)

sps = 16
# sps can be 8, 16, 32, 64, 128, 250, 475, 860

sensitivity_factor = 2 # The sensitivity factor determines the length of the voltsdiff list, on which the moving average is computed: the longer the list, the less sensitive the measurement. A sensitivity factor of 4 means that the moving average is run on the voltages measured in the last one fourth of a second.
#voltsdiff=deque([-adc.readADCSingleEnded(2,256, sps)/1000.0]*int(sps/sensitivity_factor), int(sps/sensitivity_factor))
adc_resolution = 2048
# adc_resolution = 256
scaling = [-287.35410, 430.74201]
# scaling = [constant, slope]
# grams = constant + slope*vdiff.
# e.g. 1 volt translates into [slope] 445 grams. You need about 2 mV for 1 g on the scale. Or 0.2 mV for 0.1 g. 

print "Resolution = %0.4f grams" %(round(adc_resolution,0)/(2**15)*scaling[1]/1000.0)
# scaling = [-278.4809081840, 3244.5206395731]

voltsdiff=deque([-adc.readADCDifferential01(adc_resolution, sps)/1000.0]*int(sps/sensitivity_factor), int(sps/sensitivity_factor))
tare_weight = 0.0
prev_weight = [0.0]*4
mva_voltsdiff = 0.0

print "V0 = %s " %(adc.readADCSingleEnded(0, 4096, sps)/1000.0)
print "V1 = %s " %(adc.readADCSingleEnded(1, 4096, sps)/1000.0)
print "V1 - V0 = %s " %(-adc.readADCDifferential01(adc_resolution, sps)/1000.0)

# adc.startContinuousDifferentialConversion(0, 1, adc_resolution, sps)/1000.0

def read_voltage():
    global voltsdiff
    # voltsdiff.append(-adc.getLastConversionResults()/1000.0)
    voltsdiff.append(-adc.readADCDifferential01(adc_resolution, sps)/1000.0)

def mean(x):
    return math.fsum(x)/len(x)

def from_voltages_to_weight():
    global current_weight, prev_weight, mva_voltsdiff, tare_weight
    mva_voltsdiff = mean(voltsdiff)
    # mva_voltsdiff = sorted(voltsdiff)[int((len(voltsdiff)-1)/2)] # If sps = 250, take the (smoothing_factor*12.5)th element of the sorted list of voltages: that's the median.
    # trimmed_voltsdiff = sorted(voltsdiff)[int(round(.05*len(voltsdiff), 0)):int(round(.95*len(voltsdiff), 0))]
    # mva_voltsdiff = mean(trimmed_voltsdiff)
    current_weight = scaling[0] + mva_voltsdiff*scaling[1] - tare_weight
    if abs(current_weight) <= 0.3:
        prev_tare_weight = tare_weight
        tare_weight += current_weight
        current_weight = current_weight + prev_tare_weight - tare_weight
    # if (abs(current_weight - prev_weight) <= 0.15) | (prev_weight - 0.2 <= current_weight < prev_weight):
    #         current_weight = prev_weight # Consider variations of +/- 0.1g as noise; consider decreases of up to 0.2g as noise.
    
    if (abs(round(current_weight,1)) < 0.1) & (current_weight < 0):
        current_weight = -current_weight # This is just to get rid of these annoying "-0.0" weight measurements (why does Python need to put a negative sign???).
    # if ((abs(current_weight - prev_weight[0])<.1) |
    #    ((abs(current_weight - prev_weight[0])<.2) & (prev_weight[0] == prev_weight[1])) |
    #    ((abs(current_weight - prev_weight[0])<.3) & (prev_weight[0] == prev_weight[2]))):
    #     current_weight = prev_weight[0]
        # QUICK AND SUPER DIRTY WAY TO ENSURE STABLE READINGS: TARE WHEN READING LOOKS STABLE. THIS IS VERY AGRESSIVE
        # prev_tare_weight = tare_weight
        # tare_weight += current_weight - prev_weight[0]
        # current_weight = current_weight + prev_tare_weight - tare_weight
    prev_weight = [current_weight, prev_weight[0], prev_weight[1], prev_weight[2]]


# Zero tracking: any new weight that falls within +/- 0.2g (or 0.3g) of zero weight instantaneously becomes the new zero weight.

def tare(channel):
    global tare_weight, prev_weight
    tare_weight += current_weight
    prev_weight = [0.0]*4
    print "Tare"

def thread_read_voltage():
    global log_values, log_start_time
    # t = time.time()
    while True:
        read_voltage()
        # if time.time()-t >= 0.1:
        #     display_voltages_to_weight()
        #     t = time.time()
        if log_mode == 1:
            log_values.append(scaling[0] + voltsdiff[-1]*scaling[1] - tare_weight)
            log_time.append(time.time()-log_start_time)
            if len(log_values) % 20 == 0:
                print "Logging: %s" %(len(log_values))
        time.sleep(1/round(sps, 0) + .0001) # E.g. if sps = 250: take a measurement every 0.0041 sec

def display_text(string, coordinates, size, color):
    font = pygame.font.Font(None, size)
    text_surface = font.render(string, True, color)  
    lcd.blit(text_surface, coordinates)

def sd(x):
    sqdev = [0]*len(x)
    for i in range(0, len(x)-1):
        sqdev[i] = (x[i] - mean(x))**2
    return ((math.fsum(sqdev))/(len(x)-1))**.5

def mean_abs_dev(x):
    gap = [0]*len(x)
    for i in range(0, len(x)):
        gap[i] = abs(x[i] - mean(x))
    return mean(gap)

weight_hist = deque([0], 30)

def display_voltages_to_weight():
    global log_mva, log_time_mva
    from_voltages_to_weight()
    if log_mode == 1:
        log_mva.append(current_weight)
        log_time_mva.append(time.time()-log_start_time)
    lcd.fill((0,0,0))
    display_text("%s g." % round(current_weight, 1), (10,10), 60, (255,255,255))
    display_text("Zero weight = %0.8f g." % tare_weight, (10,70), 20, (255,255,255))
    display_text("VDiff = %0.12fmV" %(mva_voltsdiff*1000), (10,100), 20, (255,255,255))
    display_text("%s sps; MVA on last %s samples" %(sps, len(voltsdiff)), (10, 130), 20, (255,255,255))
    sd_g = sd(voltsdiff)*scaling[1]
    display_text("SD of sampled weights in g. = %0.5f g." %(sd_g), (10,190), 20, (255,255,255))
    display_text("SD of the mean weight in g. = %0.5f g." %(sd_g/(len(voltsdiff)**0.5)), (10,210), 20, (255,255,255))
    pygame.display.update()
    # print "SD of the mean weight in g. = %0.5f g." %(sd_g/(len(voltsdiff)**0.5))
    

def thread_voltages_to_weight():
    while True:
        display_voltages_to_weight()
        time.sleep(.1)

log_mode = 0

def log_weight(channel):
    global log_mode, log_values, log_mva, log_time, log_time_mva, log_start_time
    if log_mode == 0:
        log_values = []
        log_mva = []
        log_time = []
        log_time_mva = []
        log_start_time = time.time()
        print "Log started"
        log_mode = 1
    elif log_mode == 1:
        print "Ending log"
        log_mode = 0
        json.dump({"sps": sps, 
                   "sensitivity_factor" :sensitivity_factor, 
                   "log_values": log_values, 
                   "log_time": log_time, 
                   "log_mva": log_mva, 
                   "log_time_mva": log_time_mva}, open("log.json", "w"))
        # csvfile = open('log.csv', 'w')
        # writer_object = csv.writer(csvfile)
        # writer_object.writerows([log_values, log_time])
        # csvfile.close()
        # os.system('scp log.csv pkremp@pc23.home:/Users/pkremp/Files/Misc/python/logdata')
        print "SD of current_weight %0.4f" %(sd(log_mva)) 
        print "log.csv saved and copy with:"
        print ""
        print "scp log.json pkremp@pc23.home:/Users/pkremp/Files/Misc/python/logdata"
        print ""

thread.start_new_thread(thread_read_voltage, ())
thread.start_new_thread(thread_voltages_to_weight, ())

button1_pin = 17
button4_pin = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(button1_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button4_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Pull-up vs pull-down resistors explained: http://www.bit-101.com/blog/?p=3813
# The switch on GPIO 27 is apparently pulled up. This sucks, as this may add EMF noise.
# Why prefer pull-down resistors: http://raspberrypi.stackexchange.com/questions/9481/gpio-why-wire-button-to-ground-rather-than-3-3v
# Seem to get much better results (lower SD) with capacitors across pin (22, 27, 17, 23) and ground. Not sure why...
GPIO.add_event_detect(button1_pin, GPIO.BOTH, callback=log_weight, bouncetime = 500)
GPIO.add_event_detect(button4_pin, GPIO.BOTH, callback=tare, bouncetime = 500)

start = time.time()
tare_flag = False

try:
    while True:
        pass
        time.sleep(.01)
except KeyboardInterrupt:
    print "KeyboardInterrupt"
    print "Exiting after cleaning up GPIO"
    GPIO.cleanup()


# adc.startContinuousDifferentialConversion(2, 3, 256, 860)
#
# while True:
#     print -adc.getLastConversionResults()/1000.0
#     time.sleep(.5)
