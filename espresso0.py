#!/usr/bin/python
# -*- coding: utf-8 -*-

import pygame
import random
import time
import math
import os
import glob
import subprocess
import Adafruit_GPIO.SPI as SPI
# import Adafruit_MAX31855.MAX31855 as MAX31855
import RPi.GPIO as GPIO
import thread
import signal
import json
import sys
import uptime
import numpy as np
import itertools
import traceback

from Adafruit_ADS1x15 import ADS1x15
from collections import deque

# Remember that the PiTFT display uses the following GPIO pins: all the hardware SPI pins (SCK, MOSI, MISO, CE0, CE1), and GPIO 24 and 25.
# The 4 microswitches on the side of the display use: GPIO 17, 22, 23, 27 (from top to bottom)

# sys.stdout = open("/home/pi/logs/stdout-" + time.strftime('%Y-%m-%d-%H%M') + ".txt", 'w')

# sys.stdout = open("/home/pi/logs/stdout-" + time.strftime('%Y-%m-%d-%H%M') + ".txt", 'w', 0)
# sys.stderr = open("/home/pi/logs/stderr-" + time.strftime('%Y-%m-%d-%H%M') + ".txt", 'w', 0)

# This redirects stdout to a file. 0 means no buffering. Each new line is redirected immediately.

GPIO.setmode(GPIO.BCM)

button1_pin = 17
button2_pin = 22
button3_pin = 23
button4_pin = 27

pump_pin = 12
heat_pin = 16

# Will also need to add: heat_pin and threewayvalve_pin 
#

# Reading temperature from thermocouple, using software SPI connection

# DO  = 26
# CS  = 19
# CLK = 13
#
# sensor = MAX31855.MAX31855(CLK, CS, DO)

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
 
base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'
 
def read_sensor_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines

# More reliable by a bit slower
# def read_sensor_raw():
#     catdata = subprocess.Popen(['cat',device_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#     out,err = catdata.communicate()
#     out_decode = out.decode('utf-8')
#     lines = out_decode.split('\n')
#     return lines

 
def read_sensor():
    lines = read_sensor_raw()
    while lines[0].strip()[-3:] != 'YES':
        # time.sleep(0.2)
        lines = read_sensor_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c



# Setting up input pins for buttons:

GPIO.setup(button1_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button2_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button3_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button4_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# Setting the output pins:

GPIO.setup(pump_pin, GPIO.OUT)
# GPIO.setup(threewayvalve_pin, GPIO.OUT)
GPIO.setup(heat_pin, GPIO.OUT)

GPIO.output(pump_pin, 0)
GPIO.output(heat_pin, 0)

# I2C connection to ADC
# ADC connected to potentiometer and load cell.

ADS1115 = 0x01	# 16-bit ADC
adc = ADS1x15(ic=ADS1115)

# ADC for scale voltage readings

sps = 8 # sps can be 8, 16, 32, 64, 128, 250, 475, 860
len_voltsdiff = 3 
adc_resolution = 2048
scaling = [-287.35410, 430.74201]

# print "Scale resolution = %0.4f grams" %(round(adc_resolution,0)/(2**15)*scaling[1]/1000.0)

voltsdiff=deque([-adc.readADCDifferential01(adc_resolution, sps)/1000.0]*len_voltsdiff, len_voltsdiff)
tare_weight = 0.0
prev_weight = [0.0]*4
mva_voltsdiff = 0.0

# Open-loop temperature control

power_when_pulling_shot = 28

time_preheat = 3

#####################################################################
# Global variables used to share state across functions and threads #
#####################################################################

post_shot = False

pot_value = 0
keep_reading_pot = False

last_timer = 0
last_weight = 0

shot_pouring = False 
old_shot_pouring = False
seconds_elapsed = 0.0

end_shot_time = time.time()-10
keep_reading_scale = False

menu = 0 # Other options: 0 = main menu; 1 = settings menu
old_menu = 1
steaming_on = False
old_steaming_on = False
backflush_on = False
old_backflush_on = False
flush_on = False
old_flush_on = False

def load_settings():
    global set_temp, target_weight, ramp_up_time, pp0, pp1, pp2, kP, kI, kD, k0
    try:
        settings = json.load(open("/home/pi/settings.txt","r"))
        set_temp = settings["set_temp"]
        target_weight = settings["target_weight"]
        ramp_up_time = settings["ramp_up_time"]
        pp0 = settings["pp0"]
        pp1 = settings["pp1"]
        pp2 = settings["pp2"]
        kP = settings["kP"]
        kI = settings["kI"]
        kD = settings["kD"]
        k0 = settings["k0"]
    except:
        reset_settings()

def reset_settings():
    global set_temp, target_weight, ramp_up_time, pp0, pp1, pp2, kP, kI, kD, k0
    set_temp = 90
    target_weight = 36
    ramp_up_time = 10
    pp0 = 1
    pp1 = 7
    pp2 = 7
    kP = 0.07
    kI = 0.12
    kD = 2.50
    k0 = 0.035
    # Or these values seem quite good: {"kI": 0.12, "pp2": 7.0, "pp1": 7.0, "target_weight": 36, "k0": 0.035, "kD": 2.499999999999999, "ramp_up_time": 10, "pp0": 3.0, "set_temp": 90, "kP": 0.07}

load_settings()

shot_temp = set_temp
steam_temp = 90.0

old_set_temp = 0
power = 0
pump_power = 0 

submenu = 0
m_item_selected = 0

# These values will be logged and saved in a json file
# y, heat_series, shot_series are directly used for display

temp0 = read_sensor()
y = deque([temp0, temp0, temp0], 3600) 
y_time = deque([0,0,0], 3600)

heat_series = deque([0,0,0], 3600)
shot_series = deque([False, False, False], 3600)

weight_series = []
weight_series_time = []
filtered_weight_series = []
filtered_flow_series = []
filtered_time = []


# With a refresh speed of 0.5 seconds, 3600 data points = 30 minutes of temperature logging. Perhaps overkill.
# Making sure that y starts with a length of 3, so that the PID function can be run right after read_temp.

trigger_refresh_temp_display = False
trigger_heat = False
trigger_refresh_timer = False
filter_on = False

####################################
# UI area positions and dimensions #
####################################

# Screen resolution is 320 * 240.
# Careful here: pygame.Rect uses these coordinates for rectangles: (left, top, width, height) [not (left, top, right, bottom)]
area_graph = ((0,65),(290,155))
# This is the reduced-size graph window, used when pulling a shot or entering the settings menu.
# area_graph = ((0,100),(140,100)) # Note that we need to change npixels_per_tempreading
area_text_temp = ((0,0),(160,60)) # same here, for the refresh_temp_display function.
area_timer = ((160,0),(120,60))   # same for the refresh_timer_display
area_icons =((295,0),(25,240))
area_belowgraph = ((0,220), (280, 20))
area_menu = ((150, 65), (130, 155))

min_range = 5
graph_top = area_graph[0][1]
graph_bottom = area_graph[0][1] + area_graph[1][1] 
graph_left  = area_graph[0][0]
graph_right = area_graph[0][0] + area_graph[1][0] 

npixels_per_tempreading = 2

#################################################################################################
# Functions defining actions triggered by buttons (e.g. pour_shot) or automatically (e.g. heat) #
#################################################################################################

length_history = len(y)

def read_temp():
    global length_history
    current_temp_reading = read_sensor()
    y.append(current_temp_reading)
    y_time.append(time.time()-start_script_time)
    if length_history < 60:
        # This variable will be used to adjust how far back the integral term should go in the PID.
        length_history += 1
    
def pid(y, set_temp):
    # P: how far from the set_temp is y?
    P = set_temp - y[-1]
    # I: Mean error over the last 60 measurements (60*.80 = 48 second)
    I = 1/length_history * sum(set_temp - y[-i] for i in xrange(1, length_history))
    # Avoid integral windup:
    if I > 1:
        I = 1
    elif I < -1:
        I = -1
    # D: how has y evolved over the last 4 readings?
    if len(y) < 4:
        D = 0
    else:
        D = y[-2] - y[-1]
        # D = ((y[-4] + y[-3])/2 - (y[-2] + y[-1])/2)/2
    power = (k0 + kP*P + kI*I + kD*D)*100
    # Could also try adding a 2% intercept = the expected amount of power needed to keep temperature from dropping once steady-state is reached
    # We can't go above 100% and below 0% heat.
    if power > 100:
        power = 100
    elif power < 0:
        power = 0
    return(power)




def adjust_heating_power():
    global power, post_shot, length_history
    if heat_cycles < 3.5*60/0.79 and start_temp < 50:
        early_boost = 100 - set_temp # For the first 3.5 minutes, if the machine starts cold, set_temp will be boosted up to 100 degrees. The machine will warm up faster to set_temp.
    else:
        early_boost = 0
    if shot_pouring == False:
        if (steaming_on == False) and (post_shot == False): 
            if (y[-1] > .90 * (set_temp + early_boost)) and (y[-1] < 105):
            #     if abs(y[-1] - set_temp) <= .5 and y[-1] <= y[-4] + .25  :  # Temp is close to set point and either dropping or increasing very slowly.
            #         power = max(2, pid(y, set_temp + early_boost)) # Power on will always be at least 2%.
            #     else:
                power = pid(y, set_temp + early_boost)
            elif (y[-1] <= .90 * (set_temp + early_boost)):
                power = 100
            elif (y[-1] >= 105):
                power = 0
        elif (steaming_on == False) and (post_shot == True):
            if (y[-1] > set_temp) and (time.time() - end_shot_time < 40) and (y[-1] < 98): # Wait until temperature goes back down and stabilizes somewhat before handing temp control back to the PID
                if y[-1] < y[-2]:
                    power = 16
                else:
                    power = 0
            else:
                length_history = 2 # Resetting length_history to avoid confusing the PID (integral term) with high post shot temperature values
                power = pid(y, set_temp + early_boost)
                post_shot = False 
        elif steaming_on == True:
            if y[-1] < set_temp:
                power = 100
            else:
                power = 0
    elif shot_pouring == True:
        if (y[-1] >= 105):
                power = 0
        else:
            if seconds_elapsed < 25:
                power = power_when_pulling_shot
            else:
                power = 0
    # Now that everything is calculated: append heat power data, along with whether a shot is being pulled or not, to lists.
    heat_series.append(power)
    shot_series.append(shot_pouring)
    
heat_cycles = 0

def output_heat():
    global heat_cycles
    if power > 0:
        GPIO.output(heat_pin, True)
        time.sleep(.79 * power/100) # remember that the conversion speed of the temperature sensor is always > .82 second. read_temp() and output_heat() work in 2 different threads; they shouldn't get out of sync. output_heat() is called each time a new temperature reading has been made; and the last output_heat() should always end before a new reading becomes available...
    if power < 100:
        GPIO.output(heat_pin, False)
        time.sleep(.79 * (100-power)/100)
    GPIO.output(heat_pin, False)
    if heat_cycles <= 3.5*60/0.79 + 1: 
        heat_cycles += 1

# def output_pump(pump_power):
#     if pump_power < 0 or pump_power > 10:
#         print "Error, choose pump_power between 0 and 10"
#     else:
#         pulses = pulse_sequence[pump_power] # Is this necessary? Should only reset pulses if pump_power changes
#         for i in range(0, 10):
#             GPIO.output(pump_pin, pulses[i])
#             time.sleep(.02) # wait 20 ms between pulses; could increase this to .04 to make it easier on the relay and the pump...

# Another solution: start a new thread:
#               thread.start_new_thread(output_pump, ()) 
# inside pour_shot() and flush(). 
# Make sure that there's a time.sleep(.125) in pour_shot() and in flush().
#

def output_pump():
    # Setting the pump pulse sequences corresponding to power levels in 10% increments.
    pulse_sequence = [[0,0,0,0,0,0,0,0,0,0], # 0% power
                      [0,0,0,0,1,0,0,0,0,0], # 10%
                      [0,0,0,0,1,0,0,0,0,1], # 20%
                      [0,0,0,1,0,0,1,0,0,1], # 30%
                      [1,0,0,1,0,1,0,0,1,0], # 40%
                      [1,0,1,0,1,0,1,0,1,0], # 50%
                      [0,1,1,0,1,0,1,1,0,1], # 60%
                      [1,1,1,0,1,1,0,1,1,0], # 70%
                      [1,1,1,1,0,1,1,1,1,0], # 80%
                      [1,1,1,1,0,1,1,1,1,1], # 90%
                      [1,1,1,1,1,1,1,1,1,1]] # 100% power
    p = pump_pin
    cycle = itertools.cycle(range(0,10))
    while pump_power >= 0 and pump_power <= 10 and (flush_on == True or shot_pouring == True):
        GPIO.output(p, pulse_sequence[int(pump_power)][cycle.next()])
        time.sleep(.02)
    if pump_power < 0 or pump_power > 10 or flush_on == False or shot_pouring == False:
        GPIO.output(p, 0)
        if pump_power < 0 or pump_power > 10:
            print "Error, choose pump_power between 0 and 10"
        return

adc_singl3 = 0
adc_diff01 = 0

def read_adc():
    # ADC measurements (scale and pot) have to be read sequentially; can't be done in 2 separate threads running concurrently, or output values will be wrong.
    global adc_diff01, adc_singl3
    while keep_reading_pot == True or keep_reading_scale == True:
        if keep_reading_scale == True:
            try:
                adc_diff01 = adc.readADCDifferential01(adc_resolution, sps)
            except Exception as e:
                print "Exception in read_adc(): adc.readADCDifferential01() " + str(e)
            read_scale(adc_diff01)
        if keep_reading_pot == True:
            try:
                adc_singl3 = adc.readADCSingleEnded(3, 4096, 860)
            except Exception as e:
                print "Exception in read_adc(): adc.readADCSingleEnded() " + str(e)
            read_pot(adc_singl3)
            if keep_reading_scale == False: # Give the loop a rest if we're only measuring the pot voltage (at a super high sampling rate already); but if we also need to measure the voltage differential from the scale (at a lower sampling rate, for more precision), skip this step...
                time.sleep(.1)
    if keep_reading_pot == False and keep_reading_scale == False:
        return

trigger_volts_convert = False

def read_scale(adc_diff01):
    # Running in its own thread
    global t1, voltsdiff, trigger_volts_convert
    # Voltages are read 16 times per second
    voltsdiff.append(-adc_diff01/1000.0)
    t1 = time.time()
    trigger_volts_convert = True
    # Let the volts-gram conversion and all the filtering happen in another thread.

def read_pot(adc_singl3):
    global pot_value
    old_pot_value = pot_value
    pot_value = adc_singl3/3313.0 * 100
    if abs(pot_value - old_pot_value) < 2: # To prevent oscillation, any change in pot value that is less than 2% will be discarded.
        pot_value = old_pot_value

def flush():
    global keep_reading_pot, pump_power, trigger_refresh_display_pump
    thread.start_new_thread(output_pump, ())
    keep_reading_pot = True
    trigger_refresh_display_pump = True
    previous_pump_power = -1
    while flush_on == True:
        pump_power = clip(int(round(pot_value / 10, 0)), 0, 10)
        time.sleep(.1)
        if pump_power != previous_pump_power:
            trigger_refresh_display_pump = True
            previous_pump_power = pump_power
    if flush_on == False:
        keep_reading_pot = False
        GPIO.output(pump_pin, 0)
        print "flush thread exited"
        return

def mva(x, n1, n2):
    # Moving average of last [n1, n2] values of x
    return(mean(list(x[i] for i in range(len(list(x))-n1-1,len(list(x))-n2-2, -1))))


def interpolate(x, point1, point2):
    # Find y so that (x, y) falls on the line between (x1, y1) and (x2, y2)
    # If x < x1, y = y1
    # If x > x2, y = y2
    x1, y1 = float(point1[0]), float(point1[1])
    x2, y2 = float(point2[0]), float(point2[1])
    if x1 <= x <= x2: 
        y_value = y1 + (y2 - y1)/(x2 - x1)*(x - x1)
    elif x > x2:
        y_value = y2
    elif x < x1:
        y_value = y1
    return y_value

def cut(x, breakpoints):
    n = len(breakpoints)
    if x < breakpoints[0]:
        return 0
    for i in range(1, n):
        if breakpoints[i-1] <= x < breakpoints[i]:
            return i
    if x >= breakpoints[n-1]:
        return n

def auto_adjust_pump(predicted_end_time, change_predicted_end_time, seconds_elapsed):
    deriv_gain = 3.0
    deriv_gain0 = min(max(33.0 - seconds_elapsed, 0), deriv_gain)
    breakpoints_now = [interpolate(seconds_elapsed, (10,35), (25,27)), 
                       interpolate(seconds_elapsed, (10,40), (25,30)), 
                       interpolate(seconds_elapsed, (10,70), (25,33))]
    breakpoints_future =  [interpolate(seconds_elapsed + deriv_gain0, (10,35), (25,27)), 
                           interpolate(seconds_elapsed + deriv_gain0, (10,40), (25,30)), 
                           interpolate(seconds_elapsed + deriv_gain0, (10,70), (25,33))]
    a = 3 - cut(predicted_end_time, breakpoints_now)
    b = 3 - cut(predicted_end_time + deriv_gain0*change_predicted_end_time/dt, breakpoints_future)
    # Look up table of pump actions: pump_action[row][column] = pump_action[a][b]
    #    b is: Too long, OK, Too short, Way too short 
    #                                                   a is:
    pump_action = [[  2,  1,        1,            0], # Too long
                   [  1,  0,       -1,           -1], # OK
                   [  0, -1,       -1,           -2], # Too short
                   [ -1, -2,       -2,           -3]] # Way too short
    return pump_action[a][b]

def clip(x, low, high):
    if x >= low and x <= high:
        return x
    elif x < low:
        return low
    elif x > high:
        return high

flow_mode = "Auto"


def pour_shot():
    # This function will be run in its own thread
    print "pour_shot thread started"
    # Remember to open the 3-way valve here.
    global pump_power, shot_pouring, end_shot_time, trigger_update_log_shot, flow_per_second, predicted_end_time, current_time, dt, keep_reading_pot, trigger_refresh_display_pump
    if flow_mode == "Manual":
        keep_reading_pot = True
        thread.start_new_thread(read_pot, ())
    time.sleep(time_preheat+.5)
    last_adjust = 0.0
    pump_power = 0
    previous_pump_power = -1
    dt = 1.5 
    thread.start_new_thread(output_pump, ()) 
    while shot_pouring == True:
        current_time = time.time() - start_script_time
        
        if flow_mode == "Auto":    
            
            # Simple case: no ramp_up_time. Just apply full power 
            
            if (ramp_up_time == 0) and (seconds_elapsed < 11) and (filtered_weight <= 2):
                # print "Case 0 - no ramp_up_time"
                pump_power = clip(int(pp2), 1, 10)
                if (seconds_elapsed - last_adjust >= 1):
                    last_adjust = seconds_elapsed
                    trigger_update_log_shot = True
                    
            # First stage of the shot: pump power ramps up from pp0 to pp1 over ramp up time, then pressure goes up to pp2 and stays there.
            # But only as long as weight <= 2 grams and time < 11 seconds. Otherwise, pump_power is set according to the rules for Stage 2 of the shot
            
            elif (seconds_elapsed <= ramp_up_time) and (filtered_weight <= 2):
                # print "Case 1 - ramping up from pp0 to pp1 until 2g"
                pump_power = int(clip(seconds_elapsed/ramp_up_time * (pp1 - pp0) + pp0, 1, 10)) # increase pump power continuously during ramp_up_time.
                if (pump_power != previous_pump_power):
                    trigger_update_log_shot = True
                    last_adjust = seconds_elapsed
            elif (seconds_elapsed > ramp_up_time) and (seconds_elapsed < 11) and (filtered_weight <= 2) :
                # print "Case 2 - after ramp_up_time, but before 11 seconds, and weight < 2"
                pump_power = int(clip(pp2, 1, 10))
                if (pump_power != previous_pump_power) or (time.time()-last_log_time >= 1):
                    trigger_update_log_shot = True
            
            # Second stage of the shot: after 11 seconds, or if weight reaches 2 grams (whichever comes first), adjust pump power every second by evaluating the flow:
            
            else:
                if (seconds_elapsed - last_adjust >= dt):
                    flow_per_second = flow_smooth
                    if (flow_per_second > 0):
                        predicted_end_time = seconds_elapsed + (target_weight - filtered_weight) / flow_per_second - 1.15
                        try: 
                            old_predicted_end_time 
                        except NameError:
                            old_predicted_end_time = predicted_end_time
                        change_predicted_end_time = predicted_end_time - old_predicted_end_time
                    else: # Should never be in this situation: weight was reported as > 2 grams, or we're after 11 seconds, but coffee is not flowing. 
                        predicted_end_time = 100  # The solution: force a pump power increase by reporting a long predicted time.
                        old_predicted_end_time = 100
                        change_predicted_end_time = 0
                    pump_change = auto_adjust_pump(predicted_end_time, change_predicted_end_time, seconds_elapsed)
                    pump_power = clip(int(pump_power + pump_change), 1, 10)
                    old_predicted_end_time = predicted_end_time
                    last_adjust = seconds_elapsed
                    trigger_update_log_shot = True
                    
                
            
        elif flow_mode == "Manual":
            pump_power = clip(int(round(pot_value / 10, 0)), 0, 10)
            if (seconds_elapsed - last_adjust >= 1) or (pump_power != previous_pump_power):
                last_adjust = seconds_elapsed
                trigger_update_log_shot = True
        
        if filtered_weight > target_weight - 1.15*flow_smooth: 
            end_shot()
            pump_power = 0
            flow_per_second = 0
            predicted_end_time = 0
            last_adjust = 0
            trigger_update_log_shot = True
        
        if pump_power != previous_pump_power:
            trigger_refresh_display_pump = True
            previous_pump_power = pump_power
        time.sleep(.125)
    
    if shot_pouring == False:
        GPIO.output(pump_pin, 0)
        keep_reading_pot = False
        return

flow_per_second = 0
predicted_end_time = 0
time_too_long = 0
time_too_short = 0
time_way_too_short = 0
current_time = 0


def update_log_shot():
    global last_log_time, trigger_update_log_shot, weight_series, weight_series_time, filtered_weight_series, filtered_flow_series, filtered_time
    log_current_time = []
    log_current_weight = []
    log_filtered_weight = []
    log_flow_per_second = []
    log_predicted_end_time = []
    log_pump_power = []
    log_time_too_long = []
    log_time_too_short = []
    log_time_way_too_short = []
    log_start_shot_time = []
    
    # Idea: record more parameters here: target_weight, ramp_up_time, max_power, temperature at the beginning of the shot, temperature at the end, power_when_pulling_shot, etc. These could be added to the text of the e-mail.
    
    last_log_time = 0
    trigger_update_log_shot = False
    while log_shot == True:
        time_too_long      = interpolate(seconds_elapsed, (10,70), (25,33))
        time_too_short     = interpolate(seconds_elapsed, (10,40), (25,30)) 
        time_way_too_short = interpolate(seconds_elapsed, (10,35), (25,27))
        if (trigger_update_log_shot == True):
            log_current_time.append(current_time)
            log_current_weight.append(current_weight)
            log_filtered_weight.append(filtered_weight)
            log_flow_per_second.append(flow_per_second)
            log_predicted_end_time.append(predicted_end_time) 
            log_pump_power.append(pump_power)
            log_time_too_long.append(time_too_long)
            log_time_too_short.append(time_too_short)
            log_time_way_too_short.append(time_way_too_short)
            last_log_time = time.time()
            trigger_update_log_shot = False
        time.sleep(.02)
    if log_shot == False:
        start = start_shot_time - start_script_time
        end   = end_shot_time   - start_script_time
        if end - start > 15: # Only save logs of shots >= 15 seconds; don't bother otherwise.
            filename = "/home/pi/logs/log_shot" + time.strftime('%Y-%m-%d-%H%M') + ".json"
            json.dump({"time": list(log_current_time),
                       "weight": list(log_current_weight),
                       "weight_filtered": list(log_filtered_weight),
                       "flow_per_second": list(log_flow_per_second),
                       "predicted_end_time": list(log_predicted_end_time),
                       "pump_power": list(log_pump_power),
                       "t0": list(log_time_too_long),
                       "t1": list(log_time_too_short),
                       "t2": list(log_time_way_too_short),
                       "full_weight_series": list(weight_series),
                       "full_weight_series_time": list(weight_series_time),
                       "filtered_time": list(filtered_time),
                       "filtered_weight_series": list(filtered_weight_series),
                       "filtered_flow_series": list(filtered_flow_series),
                       "start": start,
                       "target_weight": target_weight,
                       "end": end}, open(filename, "w"))
            os.system(str("sudo R --silent --vanilla -f /home/pi/flow_analysis.R --args " + filename))
            # flow_analysis.R will generate graphs with ggplot, save them as pdf, and run send_email.py to send the pdf & json files as attachment
        # To clean up (but once everything has been dumped into the json file), empty the _series lists so that old values don't get dumped again for the next shot. 
        weight_series = []
        weight_series_time = []
        filtered_weight_series = []
        filtered_flow_series = []
        filtered_time = []
        return

def time_shot():
    # This function will be run in its own thread
    # print "time_shot thread started"
    global seconds_elapsed, trigger_refresh_timer, log_shot, start_shot_time
    time.sleep(time_preheat+.5)
    log_shot = True
    start_shot_time = time.time()
    thread.start_new_thread(update_log_shot, ())
    while shot_pouring == True:
        seconds_elapsed = math.floor((time.time() - start_shot_time)*10)/10
        trigger_refresh_timer = True
        time.sleep(.1)
    # Make sure that the thread can exit
    if shot_pouring == False:
        # print "time_shot thread exited"
        return

def end_shot():
    global end_shot_time, shot_pouring, pump_power, post_shot
    end_shot_time = time.time()
    shot_pouring = False
    pump_power = 0
    GPIO.output(pump_pin, 0)
    post_shot = True
    # Remember to close the 3-way valve here.
    thread.start_new_thread(wait_after_shot_and_refresh, ())

last_log_time = time.time()

def wait_after_shot_and_refresh():
    global trigger_refresh_temp_display, keep_reading_scale, last_weight, last_timer, log_shot, trigger_update_log_shot, current_time, filter_on
    while time.time() - end_shot_time < 6:
        if time.time() - last_log_time >= .5:
            current_time = time.time() - start_script_time
            trigger_update_log_shot = True
        time.sleep(.02)
    if time.time() - end_shot_time >= 6:
        reset_graph_area(menu, shot_pouring)
        lcd.fill(col_background, area_timer)
        pygame.display.update(area_timer)
        refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
        trigger_refresh_temp_display = True
        keep_reading_scale = False
        last_weight = current_weight
        last_timer = seconds_elapsed
        log_shot = False
        filter_on = False
        return

# def try_it(f):
#     # Quick and dirty way to make sure that we get a clean exit if something goes wrong.
#     # Functions that run in their own thread are unsafe: if they crash, the whole script could crash with the GPIO output pins stuck in On position.
#     def safe_function():
#         try:
#             f()
#         except BaseException as e:
#             clean_exit()
#             print "Exiting: something went very wrong in " + str(f) + ". Exception: " + e
#             traceback.print_exc()
#             return
#     return safe_function

# def start_safe_thread(f, *args):
#     # print "Starting safe thread: " + str(f)
#     thread.start_new_thread(try_it(f), ())

######################################
# DISPLAY INFO / MENUS ON THE SCREEN #
######################################

# Setting up the screen 


os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.display.init()
pygame.font.init()
pygame.mouse.set_visible(False)
lcd = pygame.display.set_mode((320, 240))
# lcd = pygame.display.set_mode((320, 240),0, 24)

col_lightblue = (0,191,255)
col_orange = (255,165,0)
col_lightgreen = (124,252,0)
col_red = (255,0,0)
col_white = (255,255,255)
col_verydarkgrey = (64,64,64)
col_darkgrey = (128,128,128)
col_medgrey = (192,192,192)
col_lightgrey = (224,224,224)
col_black = (0,0,0)

col_background = col_black
col_text = col_lightblue
col_graph = col_white
col_templine = col_orange

def display_brightness(value):
    if value > 100 or value < 0:
        print "Error: pick a value between 0 and 100"
        return
    else:
        global brightness
        brightness = value
        v = str(int(value * 1023.0 / 100.0))
        os.system(str("gpio -g pwm 18 " + v))
        return

def BestTick(largest):
    if largest > 150:
        tick = 50
    elif largest > 100:
        tick = 25
    elif largest > 50:
        tick = 20
    elif largest > 20:
        tick = 10
    elif largest > 10:
        tick = 5
    else:
        tick = 2
    return tick

def coordinates(max_values, value, graph_top, graph_bottom):
    # graph_top and graph_bottom are the location of the top and bottom of the graph area on the screen in pixels (0 is the very top of the screen, 240 is the very bottom)
    # example: graph_top = 80, and graph_bottom = 220.
    return (graph_top + (max_values[1] - value) * (graph_bottom - graph_top)/(max_values[1] - max_values[0]))

def axis_min_max(subset_y):
    if max(subset_y) - min(subset_y) < min_range:
        y_range = (min(subset_y) - min_range/2 + (max(subset_y) - min(subset_y))/2, max(subset_y) + min_range/2 - (max(subset_y) - min(subset_y))/2)
    else:
        y_range = (min(subset_y), max(subset_y))
    padding_y_axis = 0.05 * (y_range[1] - y_range[0])
    # These are the most extreme values that could be plotted on the y axis, on the top/bottom edges of area_graph
    y_axis = (int(math.floor(y_range[0] - padding_y_axis)), int(math.ceil(y_range[1] + padding_y_axis)))
    return y_axis


def display_text(string, coordinates, size, color):
    font = pygame.font.Font(None, size)
    text_surface = font.render(string, True, color)  
    lcd.blit(text_surface, coordinates)


def draw_axes(y_axis, tick, graph_top, graph_bottom):
    for val in xrange(0, y_axis[1], tick) :
        coord_val = coordinates(y_axis, val, graph_top, graph_bottom)
        if (coord_val < graph_bottom) and (coord_val > graph_top): # remember that y coordinates start at 0 from the top of the display
            pygame.draw.line(lcd, col_text, (graph_left, coord_val), (graph_right-30, coord_val)) # change this 300 value, based on graph_right
            display_text(str(val), (graph_right-25, coord_val-10), 20, col_text) # change this 305 value, based on graph_right, leave space for icons

def draw_lines(y_coord, y_axis, graph_top, graph_bottom, color_series):
    coord_set_temp = coordinates(y_axis, set_temp, graph_top, graph_bottom)
    if (coord_set_temp < graph_bottom) and (coord_set_temp > graph_top): # remember that y coordinates start at 0 from the top of the display
        pygame.draw.line(lcd, col_templine, (graph_left, coord_set_temp), (graph_right-30, coord_set_temp)) # change this 300 value, based on graph_right
    # pointlist = [[graph_left + npixels_per_tempreading*j for j in range(0, len(y_coord))],
    #              [y_coord[j] for j in range(0, len(y_coord))]]
    # pointlist = [[pointlist[0][i],pointlist[1][i]] for i in range(0, len(y_coord))]
    # pygame.draw.aalines(lcd, col_white, False, pointlist) # does anti-aliasing still work?.
    for j in xrange(1, len(y_coord)):
        pygame.draw.line(lcd, color_series[j-1], (graph_left + npixels_per_tempreading*(j-1), y_coord[j-1]),(graph_left + npixels_per_tempreading*j, y_coord[j]))

# def draw_lines(subset_y, y_axis, tick, graph_top, graph_bottom, color_series):
#     coord_set_temp = coordinates(y_axis, set_temp, graph_top, graph_bottom)
#     if (coord_set_temp < graph_bottom) and (coord_set_temp > graph_top): # remember that y coordinates start at 0 from the top of the display
#         pygame.draw.line(lcd, col_templine, (graph_left, coord_set_temp), (graph_right-30, coord_set_temp)) # change this 300 value, based on graph_right
#     y_coord  = [coordinates(y_axis, subset_y[j], graph_top, graph_bottom) for j in xrange(0, len(subset_y))]
#     # pointlist = [[[graph_left + npixels_per_tempreading*(j-1), y_coord[j-1]], [graph_left + npixels_per_tempreading*j, y_coord[j]]] for j in range(1, len(y_coord))]
#     # pointlist = list(itertools.chain.from_iterable(pointlist))
#     # pygame.draw.lines(lcd, col_white, False, pointlist, 1)
#     graph_surface.lock()
#     for j in xrange(1, len(y_coord)):
#         pygame.draw.aaline(graph_surface, color_series[j-1], (npixels_per_tempreading*(j-1), y_coord[j-1] - graph_top),(npixels_per_tempreading*j, y_coord[j] - graph_top)) # 2 is the line thickness here.
#     #   pygame.draw.line(lcd, color_series[j-1], (graph_left + npixels_per_tempreading*(j-1), y_coord[j-1]),(graph_left + npixels_per_tempreading*j, y_coord[j]), 1) # 2 is the line thickness here.
#     graph_surface.unlock()
#     lcd.blit(graph_surface, (graph_left, graph_top))


def draw_power(power_data):
    for j in xrange(0, len(power_data)):
        pygame.draw.line(lcd, col_orange, (graph_left+npixels_per_tempreading*j, 220), (graph_left+npixels_per_tempreading*j, int(220-power_data[j]/4)),1)

j = 0

def draw_belowgraph():
    global j
    string1 = "T: " + str(set_temp) + u"\u2103" + "  -  H: " 
    display_text(string1, area_belowgraph[0], 25, col_text)
    string2 = str(int(power)) + "%  -"
    surf = pygame.font.Font(None, 25).render(string2, True, col_white)
    width = surf.get_width()
    display_text(string2, (100+59-width, area_belowgraph[0][1]), 25, col_text)
    if steaming_on == True:
        string3 = "  -  Steam"
    if steaming_on == False:
        # Update the rest of the display only every 2 refreshes
        if (j % 12 < 2): # 0-1
            if time.time() - last_input_time < 2700: # If time.time() - last_input_time >= 2700, the system should either shutdown OR should be detecting that NTPD has reset time (in which case "On time" will be off for less than 5 seconds -- until thread_auto_dim_or_shutdown() reajusts start_script_time and last_input_time)
                minutes = int((time.time() - start_script_time)/60)
                string3 = "On: %s min." %(minutes)
            else:
                string3 = "Target: %s g." %(target_weight)
        elif (j % 12 < 4): # 2-3
            string3 = "Target: %s g." %(target_weight)
        else:
            if flow_mode == "Auto":
                if (j % 12 < 6): # 4-5
                    string3 = "Flow: Auto"
                elif (j % 12 < 8): # 6-7
                    string3 = "Preinf: %ss." %(ramp_up_time)
                elif (j % 12 < 10): # 8-9
                    start_preinf = int(pp0*10)
                    end_preinf   = int(pp1*10)
                    string3 = "Preinf: %s-%s%%" %(start_preinf, end_preinf)
                elif (j % 12 < 12): # 10-11
                    peak_pump = int(pp2*10)
                    string3 = "Peak: %s%%" %(peak_pump)
            elif flow_mode == "Manual": # 4 or 8
                string3 = "Flow: Manual"
                j += 3
        j += 1
    display_text(string3, (165, area_belowgraph[0][1]), 25, col_text)


def refresh_timer_display(seconds_elapsed, area_timer):
    lcd.fill(col_background, rect = area_timer)
    display_text("%4s" % '{0:.1f}'.format(seconds_elapsed) + "s", (175, 5), 60, col_text)
    # display_text('{:>8}'.format(str(seconds_elapsed) + "s."), (180, 5), 60, col_text)
    #display_text(str(seconds_elapsed)+" s.", (180, 5), 70, col_text)
    pygame.display.update(area_timer)

# prev_y_axis = (0, 0)
# y_minmax = (65, 215)
# prev_y_minmax = (0, 0)

def refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp):
    global prev_y_axis, prev_y_minmax, prev_x_range
    # Transform the series of 150 most recent ys into coordinates on the screen
    n_datapoints = int(math.floor((graph_right-30-graph_left)/npixels_per_tempreading))
    subset_y = [y[k] for k in xrange(max(0, len(y)-n_datapoints), len(y))]
    subset_heat_series = [heat_series[k] for k in xrange(max(0, len(heat_series)-n_datapoints), len(heat_series))]
    subset_shot_series = [shot_series[k] for k in xrange(max(0, len(shot_series)-n_datapoints), len(shot_series))]
    color_series = [col_red if whatshappening else col_white for whatshappening in subset_shot_series]
    # Find the range of y to be plotted, the tick marks, and draw.
    y_axis = axis_min_max(subset_y)
    y_coord  = [coordinates(y_axis, subset_y[j], graph_top, graph_bottom-25) for j in xrange(0, len(subset_y))]
    # y_minmax = (int(min(y_coord)), int(max(y_coord)))
    tick = BestTick(y_axis[1]-y_axis[0])
    # Erase the areas to be updated
    # if y_axis == prev_y_axis and area_graph[1][0] == prev_x_range:
    #     # Redraw only a subset of area_graph if y_axis hasn't changed.
    #     area_graph_reduced = ((0, min(y_minmax[0], prev_y_minmax[0]) - 2 ),
    #                           (area_graph[1][0], 5 + max(y_minmax[1], prev_y_minmax[1]) - min(y_minmax[0], prev_y_minmax[0])))
    # else:
    #     area_graph_reduced = area_graph
    lcd.fill(col_background, rect = area_graph)
    # lcd.fill(col_background, rect = area_graph_reduced)
    lcd.fill(col_background, rect = area_timer)
    lcd.fill(col_background, rect = area_text_temp)
    lcd.fill(col_background, rect = area_belowgraph)
    # lcd.fill(col_background, rect = ((0,195),(area_graph[1][0], 25)))
    draw_power(subset_heat_series)
    draw_axes(y_axis, tick, graph_top, graph_bottom-25)
    draw_lines(y_coord, y_axis, graph_top, graph_bottom-25, color_series)
    if keep_reading_scale == False:
        draw_belowgraph()
        if subset_y[-1] >= 100:
            display_text(str(int(round(subset_y[-1]))) + u"\u2103", (5, 5), 60, col_text)
        elif subset_y[-1] < 100:
            display_text('{0:.1f}'.format(subset_y[-1]) + u"\u2103", (5, 5), 60, col_text) # u"\u2103" is the unicode degrees celsisus sign.
        if (last_weight != 0) and (last_timer != 0):
            display_text("Last shot", (180, 8), 25, col_text)
            display_text("%0.0f s. / %0.0f g." %(last_timer, last_weight), (180, 26), 25, col_text)
        pygame.display.update([area_graph, area_belowgraph, area_text_temp, area_timer])
    else:
        pygame.display.update([area_graph, area_belowgraph])
    # prev_y_axis = y_axis
    # prev_y_minmax = y_minmax
    # prev_x_range = area_graph_reduced[1][0]
    

icon_start = pygame.image.load(os.path.join('/home/pi/icons', 'start.png'))
icon_plus = pygame.image.load(os.path.join('/home/pi/icons', 'plus.png'))
icon_minus = pygame.image.load(os.path.join('/home/pi/icons', 'minus.png'))
icon_up = pygame.image.load(os.path.join('/home/pi/icons', 'up.png'))
icon_down = pygame.image.load(os.path.join('/home/pi/icons', 'down.png'))
icon_more = pygame.image.load(os.path.join('/home/pi/icons', 'more.png'))
icon_settings = pygame.image.load(os.path.join('/home/pi/icons', 'settings.png'))
icon_check = pygame.image.load(os.path.join('/home/pi/icons', 'check.png'))
icon_back = pygame.image.load(os.path.join('/home/pi/icons', 'back.png'))
icon_stop = pygame.image.load(os.path.join('/home/pi/icons', 'stop.png'))

def draw_buttons(menu, shot_pouring, steaming_on, backflush_on, flush_on):
    # Buttons will look different depending on menu and whether steam_mode is selected, or steaming is started.
    if menu == 0:
        if shot_pouring == True:
            lcd.blit(icon_stop, (295, 216))
        elif shot_pouring == False:
            lcd.blit(icon_more, (295, 36))
            lcd.blit(icon_up, (295, 96))
            lcd.blit(icon_down, (295, 156))
            lcd.blit(icon_start, (295, 216))
    elif menu == 1:
        if (steaming_on == True) or (backflush_on == True) or (flush_on == True):
            lcd.blit(icon_stop, (295, 216))
        else:
            lcd.blit(icon_back, (295, 36))
            lcd.blit(icon_up, (295, 96))
            lcd.blit(icon_down, (295, 156))
            lcd.blit(icon_check, (295, 216))

def refresh_buttons(menu, shot_pouring, steaming_on, backflush_on, flush_on):
    # Same as time: only refresh if something changes.
    global old_menu, old_set_temp, old_shot_pouring, old_steaming_on, old_backflush_on, old_flush_on
    if (menu != old_menu) or (shot_pouring != old_shot_pouring) or (steaming_on != old_steaming_on) or (backflush_on != old_backflush_on) or (flush_on != old_flush_on):
        # print "Refreshing button icons"
        lcd.fill(col_background, rect = area_icons)
        draw_buttons(menu, shot_pouring, steaming_on, backflush_on, flush_on)
        pygame.display.update(area_icons)
        old_menu = menu
        old_shot_pouring = shot_pouring
        old_steaming_on = steaming_on
        old_backflush_on = backflush_on
        old_flush_on = flush_on

def reset_graph_area(menu, shot_pouring):
    global area_graph, graph_top, graph_bottom, graph_left, graph_right, npixels_per_tempreading
    if (menu == 0) and (shot_pouring == False):
        # print "Big graph area"
        lcd.fill(col_background, rect = area_graph)
        pygame.display.update(area_graph)
        area_graph = ((0,65),(290,155))
        npixels_per_tempreading = 2
    elif (menu == 1) or (shot_pouring == True):
        # print "Small graph area"
        lcd.fill(col_background, rect = area_graph)
        pygame.display.update(area_graph)
        area_graph = ((0,65),(150,155))
        npixels_per_tempreading = 1
    graph_top = area_graph[0][1]
    graph_bottom = area_graph[0][1] + area_graph[1][1] 
    graph_left  = area_graph[0][0]
    graph_right = area_graph[0][0] + area_graph[1][0]

trigger_refresh_display_pump = False

def display_pump_power():
    global trigger_refresh_display_pump
    trigger_refresh_display_pump = True
    while (keep_reading_scale == True) or (flush_on == True):
        if trigger_refresh_display_pump:
            print "Pump power: %s" %(pump_power)
            # print "display pump power update"
            lcd.fill(col_background, rect = (150, 65, 130, 155))
            display_text("Pump", (200, 180), 25, col_text)
            display_text(str(pump_power*10) + "%", (202 if pump_power == 10 else 206, 200), 25, col_text)
            for box in range(1, 11):
                lcd.fill(col_verydarkgrey, rect= ((215, 170-(box-1)*10), (20, 8)))
            for box in range(1, pump_power + 1):
                lcd.fill(col_lightblue, rect= ((215, 170-(box-1)*10), (20, 8)))
            # pygame.display.update((200, 65, 70, 155))
            pygame.display.update((150, 65, 130, 155))
            trigger_refresh_display_pump = False
        time.sleep(.05)
    if (keep_reading_scale == False) and (flush_on == False):
        # print "Exiting display pump power thread"
        return

area_menu = ((150, 65), (130, 155))

def backflush():
    global backflush_on
    backflush_on = True
    i = 1
    while (backflush_on == True) and (i <= 5):
        lcd.fill(col_background, area_menu)
        display_text("Backflush...", (160, 100), 25, col_white)
        display_text("%s/5" % i, (160, 125), 25, col_white)
        pygame.display.update(area_menu)
        # Display info
        GPIO.output(pump_pin, 1)
        time.sleep(5)
        GPIO.output(pump_pin, 0)
        time.sleep(5)
        i = i+1
    backflush_on = False
    display_main_menu()
    # print "Exiting backflush thread"
    return

def mean(x):
    return math.fsum(x)/len(x)

def sd(x):
    sqdev = [0]*len(x)
    for i in range(0, len(x)-1):
        sqdev[i] = (x[i] - mean(x))**2
    return ((math.fsum(sqdev))/(len(x)-1))**.5

def tare_and_preheat():
    global tare_weight, prev_weight, voltsdiff, filter_on, filtered_weight
    lcd.fill(col_background, rect = area_text_temp)
    text = "Preheat"
    for i in range(1, 4):
        display_text(text + "."*i, (175, 20), 30, col_text)
        pygame.display.update(area_timer)
        time.sleep(time_preheat/3)
    tare_weight += current_weight
    prev_weight = [0.0]*4
    filter_initialize()
    filter_on = True
    filtered_weight = 0.0


weighing_time = time.time()-1
t1 = time.time()

def convert_volts():
    global current_weight, prev_weight, mva_voltsdiff, tare_weight, displayed_weight, weighing_time, previous_time, filtered_weight
    mva_voltsdiff = mean(voltsdiff)
    current_weight = scaling[0] + mva_voltsdiff*scaling[1] - tare_weight
    previous_time = weighing_time
    weighing_time = t1
    # print "%s: converting V into g: last voltsdiff" %t1
    if filter_on == True:
        # print "filtering: previous_time %s; weighing_time %s" %(previous_time, weighing_time)
        filter(current_weight, prev_weight[0], weighing_time, previous_time, Q0, R0)
        displayed_weight = filtered_weight if filtered_weight >= 0 else 0.001
    elif filter_on == False:
        filtered_weight = current_weight
        displayed_weight = current_weight
    prev_weight = [current_weight, prev_weight[0], prev_weight[1], prev_weight[2]]

def voltages_to_weight():
    # Conversion in grams every 10th of a second
    global trigger_volts_convert
    while keep_reading_scale == True:
        if trigger_volts_convert == True:
            convert_volts()
            weight_series_time.append(weighing_time - start_script_time)
            weight_series.append(current_weight)
            # print "%s: current_weight = %s" %(weighing_time, current_weight)
            if filter_on == True:
                filtered_time.append(weighing_time - start_script_time)
                filtered_weight_series.append(filtered_weight)
                filtered_flow_series.append(flow_smooth)
                # print "%s: filtered_weight = %s" %(weighing_time, filtered_weight)
            lcd.fill(col_background, rect = area_text_temp)
            display_text('{0:.1f}'.format(displayed_weight) + " g.", (5, 5), 60, col_text)
            pygame.display.update(area_text_temp)
            trigger_volts_convert = False
        elif trigger_volts_convert == False:
            # print "Waiting for next trigger_volts_convert"
            time.sleep(.01)
    if keep_reading_scale == False:
        # print "voltage_to_weight thread exited"
        return

def filter_initialize():
    global Q0, R0, x, P
    Q0 =np.array([[0.04, 0.00], 
                  [0.00, 0.08]])
    R0 = np.array([[2.0, -1.0],
                  [-1.0, 2.0]])
    x = np.array([0.0, 0.0])
    P = np.array([[0.0, 0.0],
                  [0.0, 0.0]])

flow_list6 = deque([0], 6)
flow_smooth = 0

def filter(current_weight, prev_weight, time1, time0, Q0, R):
    global x, P, filtered_weight, filtered_flow, flow_list5, flow_smooth
    try:
        dt = time1 - time0
        z = [current_weight, (current_weight-prev_weight)/dt] 
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        if (seconds_elapsed) < 7: 
            Q = Q0 * (seconds_elapsed/7.0)**5
            # Filter very aggressively early: Q should be close to 0.
        else:
            Q = Q0
        x = np.dot(F, x)
        P = np.dot(np.dot(F, P), F.T) + Q
        y = z - x
        R = R0
        if shot_pouring == False: # At the end of shot: progressively reduce the amount of filtering to 0.
            seconds_after_end = time.time() - end_shot_time
            if  seconds_after_end < 1.5:
                R = R0 * (1.5 - seconds_after_end)/1.5
            else:
                R = R0 * 0.0
        K = np.dot(P, np.linalg.inv(P + R))
        x = x + np.dot(K, y)
        P = P - np.dot((np.eye(2) - K), P)
    except Exception as e:
        print e
        x = [current_weight, (current_weight-prev_weight)*sps]
    filtered_weight = x[0]
    filtered_flow = x[1]
    flow_list6.append(filtered_flow)
    flow_smooth = mean(flow_list6)


def save_temperature_logs():
    filename = "/home/pi/logs/log_temp" + time.strftime('%Y-%b-%d-%H%M') + ".json"  
    # Hidden feature: Cancel shutdown will save temperature logs.
    json.dump({"y": list(y),
               "time": list(y_time),
               "shot_series": list(shot_series),
               "heat_series": list(heat_series)}, open(filename, "w"))

def save_settings():
    json.dump({"set_temp": set_temp, 
               "target_weight": target_weight, 
               "ramp_up_time": ramp_up_time, 
               "pp0": pp0,
               "pp1": pp1,
               "pp2": pp2,
               "kP": kP,
               "kI": kI,
               "kD": kD,
               "k0": k0}, open("/home/pi/settings.txt", "w"))

def shutdown():
    # Save data in json file (y, heat_series,)
    # Save settings (temperature, preinfusion, etc.) in another json file to be loaded later on.
    # save_temperature_logs()
    GPIO.cleanup()
    save_settings()
    pygame.quit()
    display_brightness(10)
    # os._exit(1)
    # sys.exit()
    os.system("sudo shutdown now")
    

##################################################################
# BUTTON FUNCTIONS AND THREADS WAITING FOR BUTTONS TO BE PRESSED #
##################################################################

## Redefine buttons when state changes:
#
# def all_buttons(state):
#     if state == True:
#         def button1():
#             print "1"
#         def button2():
#             print "a"
#     elif state == False:
#         def button1():
#             print "2"
#         def button2():
#             print "b"
#     return({"button1": button1, "button2": button2})
#
# state = True
#
# all_buttons(state)["button1"]()
# all_buttons(state)["button2"]()
#
# state = False
#
# all_buttons(state)["button1"]()
# all_buttons(state)["button2"]()
#
## GPIO.add_event_detect(button1_pin, GPIO.BOTH, callback=all_buttons(state)["button1"], bouncetime = 500)
#
# e.g. in one state: button4 will start a shot, or stop a shot, or start steaming, or select.

old_number_of_choices = 0

def display_menu_items(items, n_item_selected, number_of_choices):
    global first_index_shown, last_index_shown, old_number_of_choices
    max_height = 240 - 60 - 25
    text_height = 25 * min(6, number_of_choices) # Displaying at most 6 choices (text rows) at a time, with 25 vertical pixels for each row.
    margin_height = (max_height - text_height)/2
    lcd.fill(col_background, area_menu)
    index_selected = n_item_selected % number_of_choices
    if old_number_of_choices != number_of_choices:
        # Redefine first/last index if the menu has changed
        first_index_shown = 0 
        last_index_shown = min(6, number_of_choices) - 1
    while index_selected > last_index_shown or index_selected < first_index_shown: 
        if index_selected > last_index_shown: 
            first_index_shown += 1
            last_index_shown += 1
        elif index_selected < first_index_shown:
            first_index_shown -= 1
            last_index_shown -= 1
    row = 0
    for i in range(first_index_shown, last_index_shown + 1): # i = 0...5, or i = 1...6, or i = 2...7
        font = pygame.font.Font('/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Bold.ttf', 17)
        if i == index_selected:
            # display_text(">", (160, 60 + margin_height - 2 + 25*row), 25, col_white)
            text_surface = font.render(">", True, col_white)  
            lcd.blit(text_surface, (150, 60 + margin_height + 25*row))
        # display_text(items[i], (170, 60 + margin_height + 25*row), 25, col_white)
        text_surface = font.render(items[i], True, col_white)  
        lcd.blit(text_surface, (160, 60 + margin_height + 25*row))
        row += 1
    pygame.display.update(area_menu)
    old_number_of_choices = number_of_choices

def display_main_menu():
    global items, n
    items = ["Flush", "Shut Down", "Target Weight", "Pump Power P0", "Pump Power P1", "Pump Power P2", "Preinfusion Time", "Backflush", "Flow Mode", "kP", "kI", "kD", "k0", "Reset Defaults", "Cancel Changes"]
    n = len(items)
    display_menu_items(items, n_item_selected, n)
    
def display_change_param(name, value, units):
    # This is function is used to display: max weight, preinfusion time, max pump, etc. as they are being changed.
    lcd.fill(col_background, area_menu)
    display_text(name, (170, 100), 25, col_white)
    display_text(str(value) + units, (170, 135), 25, col_white)
    pygame.display.update(area_menu)

def display_confirm_shutdown_menu():
    global items, m_item_selected, m
    items = ["Shut Down", "Cancel"]
    m = len(items)
    display_menu_items(items, m_item_selected, m)

def clean_exit():
    print "Exiting after cleaning up GPIO"
    GPIO.output(heat_pin, 0)
    GPIO.output(pump_pin, 0)
    GPIO.cleanup()
    pygame.quit()
    display_brightness(10)
    sys.exit()

n_item_selected = 0

def button1(channel):
    time.sleep(.01)
    if GPIO.input(button1_pin) != GPIO.LOW:
        print "Probably a false positive button1 press"
        return
    print "Button 1 pressed"
    global menu, n_item_selected, submenu, last_input_time
    last_input_time = time.time()
    display_brightness(100)
    # All this works as expected, but need to rewrite the (cryptic) logic behind all these if statements.
    if submenu == 0:
        menu = 1-menu
    elif submenu >= 1:
        menu = 1
        submenu = 0
    reset_graph_area(menu, shot_pouring)
    refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
    if menu == 1:
        # n_item_selected = 0
        display_main_menu()
    if menu == 0:
        n_item_selected = 0

    
def button2(channel):
    time.sleep(.01)
    if GPIO.input(button2_pin) != GPIO.LOW:
        print "Probably a false positive button2 press"
        return
    print "Button 2 pressed"
    global set_temp, shot_temp, n_item_selected, m_item_selected, target_weight, pp0, pp1, pp2, ramp_up_time, last_input_time, flow_mode, kP, kI, kD, k0
    last_input_time = time.time()
    display_brightness(100)

    if menu == 0:
        set_temp += 1
        shot_temp = set_temp
        refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
    if menu == 1:
        if submenu == 0:
            n_item_selected -= 1
            display_main_menu()
        elif submenu == 1:
            m_item_selected -= 1
            display_confirm_shutdown_menu()
        elif submenu == 2:
            if target_weight < 50:
                target_weight += 1
            display_change_param("Target Weight:", int(target_weight), "g.")
        elif submenu == 3:
            # if pp0 < 10 and pp0 < pp1:
            if pp0 < 10:
                pp0 += 1
            display_change_param("Pump Power P0:", int(pp0*10), "%")
        elif submenu == 4:
            # if pp1 < 10 and pp1 < pp2:
            if pp1 < 10:
                pp1 += 1
            display_change_param("Pump Power P1:", int(pp1*10), "%")
        elif submenu == 5:
            if pp2 < 10:
                pp2 += 1
            display_change_param("Pump Power P2:", int(pp2*10), "%")
        elif submenu == 6:
            if ramp_up_time < 10:
                ramp_up_time += 1
            display_change_param("Preinfusion:", int(ramp_up_time), "s.")
        elif submenu == 8:
            if flow_mode == "Manual":
                flow_mode = "Auto"
            else:
                flow_mode = "Manual"
            display_change_param("Flow Mode:", flow_mode, "")
        elif submenu == 9:
            if kP < 0.20:
                kP += .005
            display_change_param("PID: kP:", kP, "")
        elif submenu == 10:
            if kI < 0.30:
                kI += .01
            display_change_param("PID: kI:", kI, "")
        elif submenu == 11:
            if kD < 4:
                kD += .10
            display_change_param("PID: kD:", kD, "")
        elif submenu == 12:
            if k0 < .06:
                k0 += .005
            display_change_param("PID: k0:", k0, "")


def button3(channel):
    time.sleep(.01)
    if GPIO.input(button3_pin) != GPIO.LOW:
        print "Probably a false positive button3 press"
        return
    print "Button 3 pressed"
    global set_temp, shot_temp, n_item_selected, m_item_selected, target_weight, pp0, pp1, pp2, ramp_up_time, last_input_time, flow_mode, kP, kI, kD, k0
    last_input_time = time.time()
    display_brightness(100)

    if menu == 0:
        set_temp -= 1
        shot_temp = set_temp
        refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
    if menu == 1:
        if submenu == 0:
            n_item_selected += 1
            display_main_menu()
        elif submenu == 1:
            m_item_selected += 1
            display_confirm_shutdown_menu()
        elif submenu == 2:
            if target_weight > 24:
                target_weight -= 1
            display_change_param("Target Weight:", int(target_weight), "g.")
        elif submenu == 3:
            if pp0 > 1:
                pp0 -= 1
            display_change_param("Pump Power P0:", int(pp0*10), "%")
        elif submenu == 4:
            # if pp1 > 1 and pp1 > pp0:
            if pp1 > 1:
                pp1 -= 1
            display_change_param("Pump Power P1:", int(pp1*10), "%")
        elif submenu == 5:
            # if pp2 > 3 and pp2 > pp1:
            if pp2 > 3:
                pp2 -= 1
            display_change_param("Pump Power P2:", int(pp2*10), "%")
        elif submenu == 6:
            if ramp_up_time > 0:
                ramp_up_time -= 1
            display_change_param("Preinfusion:", int(ramp_up_time), "s.")
        elif submenu == 8:
            if flow_mode == "Manual":
                flow_mode = "Auto"
            else:
                flow_mode = "Manual"
            display_change_param("Flow Mode:", flow_mode, "")
        elif submenu == 9:
            if kP > 0.00:
                kP -= .005
            display_change_param("PID: kP:", kP, "")
        elif submenu == 10:
            if kI > 0:
                kI -= .01
            display_change_param("PID: kI:", kI, "")
        elif submenu == 11:
            if kD > 0:
                kD -= .10
            display_change_param("PID: kD:", kD, "")
        elif submenu == 12:
            if k0 > 0:
                k0 -= .005
            display_change_param("PID: k0:", k0, "")


def button4(channel):
    time.sleep(.01)
    if GPIO.input(button4_pin) != GPIO.LOW:
        print "Probably a false positive button4 press"
        return
    print "Button 4 pressed"
    global shot_pouring, pump_power, end_shot_time, keep_reading_scale, steaming_on, set_temp, button4_repress, m_item_selected, n_item_selected, menu, submenu, backflush_on, flush_on, last_input_time
    last_input_time = time.time()
    display_brightness(100)

    # Check if button 4 is called from within a submenu
    # This will change the behavior of button4: first button4 press: submenu is displayed; second button4 press: choice is selected.
    if submenu == 1:
        button4_repress = True
    elif submenu == 0:
        button4_repress = False
        m_item_selected = 0

    if menu == 0: # Main menu
        if shot_pouring == True:
            end_shot()
        elif shot_pouring == False :
            # here: tare the scale and thread.start_new_thread(update_scale, ())
            # here: preheat for 1-2 seconds if needed
            shot_pouring = True
            pump_power = 0
            reset_graph_area(menu, shot_pouring)
            refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
            keep_reading_scale = True
            thread.start_new_thread(read_adc, ())
            thread.start_new_thread(voltages_to_weight, ())
            thread.start_new_thread(tare_and_preheat, ())
            thread.start_new_thread(time_shot, ())
            thread.start_new_thread(pour_shot, ())
            thread.start_new_thread(display_pump_power, ())
            # Note: it's easy to start a thread from an outside function: button1 can start 2 threads: time_shot() and pour_shot()
            # but I don't know how to exit a thread remotely, from an outside function. thread.exit only works from the inside of the thread.
            # Make sure that each function called by thread.start_new_thread() has an if condition that ends with "return" when its job is done.
    elif menu == 1: # Settings menu
        if n_item_selected % n == 0:
            if flush_on == False:
                flush_on = True
                thread.start_new_thread(flush, ())
                thread.start_new_thread(read_adc, ())
                thread.start_new_thread(display_pump_power, ())
            elif flush_on == True:
                display_main_menu()
                flush_on = False
        # elif n_item_selected % n == 2:
        #     # Steam
        #     if (steaming_on == False):
        #         ####### STEAMING DISABLED ######## THE TEMPERATURE SENSOR IS ONLY RATED FOR 125 CELSIUS
        #         set_temp = shot_temp
        #         steaming_on = False
        #         # set_temp = steam_temp
        #         # steaming_on = True
        #     elif steaming_on == True:
        #         set_temp = shot_temp
        #         steaming_on = False
        elif n_item_selected % n == 1: 
            # This is the "Shut Down" choice.
            submenu = 1
            # Asking for confirmation
            display_confirm_shutdown_menu()
            if (button4_repress == True) and (m_item_selected % m == 0): 
                shutdown()
            elif (button4_repress == True) and (m_item_selected % m == 1):
                submenu = 0
                m_item_selected = 0
                button4_repress = False
                display_main_menu()
                save_settings()
                save_temperature_logs()
        elif n_item_selected % n == 2:
            submenu = 2
            display_change_param("Target Weight:", int(target_weight), "g.")
        elif n_item_selected % n == 3:
            submenu = 3
            display_change_param("Pump Power P0:", int(pp0*10), "%")
        elif n_item_selected % n == 4:
            submenu = 4
            display_change_param("Pump Power P1:", int(pp1*10), "%")
        elif n_item_selected % n == 5:
            submenu = 5
            display_change_param("Pump Power P2:", int(pp2*10), "%")
        elif n_item_selected % n == 6:
            submenu = 6
            display_change_param("Preinfusion:", int(ramp_up_time), "s.")
        elif n_item_selected % n == 7: 
            # Backflush
            if backflush_on == False:
                thread.start_new_thread(backflush, ())
            elif backflush_on == True:
                backflush_on = False
        elif n_item_selected % n == 8:
            # Manual mode
            submenu = 8
            display_change_param("Flow Mode:", flow_mode, "")
        elif n_item_selected % n == 9:
            # kP
            submenu = 9
            display_change_param("PID: kP:", kP, "")
        elif n_item_selected % n == 10:
            # kI
            submenu = 10
            display_change_param("PID: kI:", kI, "")
        elif n_item_selected % n == 11:
            # kD
            submenu = 11
            display_change_param("PID: kD:", kD, "")
        elif n_item_selected % n == 12:
            # k0
            submenu = 12
            display_change_param("PID: k0:", k0, "")
        elif n_item_selected % n == 13:
            # Reset
            reset_settings()
        elif n_item_selected % n == 14:
            # Reset
            load_settings()


##################################
# THREADS REFRESHING THE DISPLAY #
##################################

def thread_read_temp():
    global trigger_heat, trigger_refresh_temp_display
    while True:
        read_temp()
        adjust_heating_power()
        trigger_heat = True
        trigger_refresh_temp_display = True

def thread_heat():
    global trigger_heat
    while True:
        if trigger_heat:
            trigger_heat = False
            output_heat()
        else:
            time.sleep(.01)


def thread_refresh_temp_display():
    global trigger_refresh_temp_display
    while True:
        if trigger_refresh_temp_display:
            refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
            trigger_refresh_temp_display = False
        time.sleep(.02)

def thread_refresh_buttons():
    while True:
        refresh_buttons(menu, shot_pouring, steaming_on, backflush_on, flush_on)
        time.sleep(.02)

def thread_refresh_timer_display():
    global trigger_refresh_timer
    while True:
        if trigger_refresh_timer:
            refresh_timer_display(seconds_elapsed, area_timer)
            trigger_refresh_timer = False
        time.sleep(.02)

def thread_auto_dim_or_shutdown():
    while True:
        global start_script_time, last_input_time, old_time, old_start_script_time
        if time.time() - old_time > 100:
            # The raspberry pi does not have a clock running when it is powered off; it syncs its time with a server at boottime.
            # Problem: sometimes, it can take a few seconds to get the network connection; by then, the python script might already have started.
            # The script substracts start_script_time from time.time() to measure time since start.
            # If NTPD manages to sync time over Wifi only after the script started, things can get messy.
            # this is a hack to reset start_script_time when it appears that NTPD has finished syncing.
            print "Resetting start script time at" + time.strftime('%Y-%m-%d-%H:%M:%S')
            old_start_script_time = start_script_time
            start_script_time = time.time() - (old_time - old_start_script_time + 5)
            last_input_time = last_input_time + start_script_time - old_start_script_time
        if time.time() - last_input_time >= 300 and brightness == 100:
            display_brightness(10)
        if time.time() - last_input_time >= 2700:
            shutdown()
        old_time = time.time()
        time.sleep(5)

uptime_s = uptime.uptime()
print "Uptime in seconds %s" %(uptime_s)

start_temp = read_sensor()
old_time = time.time()
start_script_time = time.time()
last_input_time = start_script_time
print "Start script time: " + time.strftime('%Y-%m-%d-%H:%M:%S')

thread.start_new_thread(thread_auto_dim_or_shutdown, ())
thread.start_new_thread(thread_read_temp, ())
thread.start_new_thread(thread_heat, ())
thread.start_new_thread(thread_refresh_timer_display, ())
thread.start_new_thread(thread_refresh_temp_display, ())
thread.start_new_thread(thread_refresh_buttons, ())

GPIO.add_event_detect(button1_pin, GPIO.FALLING, callback=button1, bouncetime = 100)
GPIO.add_event_detect(button2_pin, GPIO.FALLING, callback=button2, bouncetime = 100)
GPIO.add_event_detect(button3_pin, GPIO.FALLING, callback=button3, bouncetime = 100)
GPIO.add_event_detect(button4_pin, GPIO.FALLING, callback=button4, bouncetime = 100)

os.system("gpio -g mode 18 pwm")
os.system("gpio pwmc 1000")
display_brightness(100)

try:
    while True:
        pass
        time.sleep(1)
except:
    clean_exit()


# sys.stderr.close()
# sys.stderr = sys.__stderr__