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
# Note: could switch to gpiozero (instead of RPi.GPIO).
# https://gpiozero.readthedocs.io/en/v1.2.0/index.html
import thread
import signal
import json
import sys
import uptime
import numpy as np
import itertools
import traceback
if len(sys.argv) > 1 and sys.argv[1] == "test":
    pass
else:
    import plotly.plotly as py
import datetime


from Adafruit_ADS1x15 import ADS1x15
from collections import deque

# Remember that the PiTFT display uses the following GPIO pins: all the hardware SPI pins (SCK, MOSI, MISO, CE0, CE1), and GPIO 24 and 25.
# The 4 microswitches on the side of the display use: GPIO 17, 22, 23, 27 (from top to bottom)

# sys.stdout = open("/home/pi/logs/stdout-" + time.strftime('%Y-%m-%d-%H%M') + ".txt", 'w')

if len(sys.argv) > 1 and sys.argv[1] == "test":
    pass
else:
    sys.stdout = open("/home/pi/logs/stdout-" + time.strftime('%Y-%m-%d-%H%M') + ".txt", 'w', 1)
    sys.stderr = open("/home/pi/logs/stderr-" + time.strftime('%Y-%m-%d-%H%M') + ".txt", 'w', 1)

max_brightness = 30

GPIO.setmode(GPIO.BCM)

button1_pin = 17
button2_pin = 22
button3_pin = 23
button4_pin = 27

button_pins = [button1_pin, button2_pin, button3_pin, button4_pin]

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
len_raw_weight_values = 15
adc_resolution = 2048
scaling = [-287.35410, 430.74201]

# print "Scale resolution = %0.4f grams" %(round(adc_resolution,0)/(2**15)*scaling[1]/1000.0)

raw_weight_values=deque([scaling[0] - adc.readADCDifferential01(adc_resolution, sps)/1000.0 * scaling[1]]*len_raw_weight_values, len_raw_weight_values)
tare_weight = 0.0
prev_weight = 0.0

min_pp = 1
max_pp = 10

#####################################################################
# Global variables used to share state across functions and threads #
#####################################################################

steam_on = False
post_shot = False

pot_value = 0
keep_reading_pot = False

last_timer = 0
last_weight = 0

shot_pouring = False 
seconds_elapsed = 0.0

end_shot_time = 0.0
keep_reading_scale = False

menu = 0 # Other options: 0 = main menu; 1 = settings menu

backflush_on = False
flush_on = False


# Betas control the total amount of heat to distribute across shot_heat_duration

# Alpha controls how much of the shot total energy should be placed towards the preheat or towards the end of shot_heat_duration

settings_namelist = ["set_temp",
                     "target_weight","target_time",
                     "time_t1","time_t2","time_t3","time_t4","profile_01","profile_12","profile_23","profile_34","pp0","pp1","pp2","pp3","pp4",
                     "flow_mode", "time_auto_flow",
                     "kP","kI","kD","k0",
                     "time_preheat","time_shot_heat",
                     "beta_0","alpha","warmup_temp","warmup_minutes"]

def load_settings():
    try:
        settings = json.load(open("/home/pi/settings.txt","r"))
        for varname in settings_namelist:
            globals()[varname] = settings[varname]
    except:
        reset_settings()

def save_settings():
    settings = {}
    for varname in settings_namelist:
        settings[varname] = globals()[varname]
    json.dump(settings, open("/home/pi/settings.txt", "w"))


def reset_settings():
    global flow_mode, set_temp,target_weight, target_time, time_t1, time_t2, time_t3, time_t4, profile_01, profile_12, profile_23, profile_34, pp0, pp1, pp2, pp3, pp4, kP, kI, kD, k0, time_preheat, time_shot_heat, beta_0, alpha, warmup_temp, warmup_minutes
    set_temp = 89
    target_weight = 36
    target_time = 31
    flow_mode = "Auto"
    time_auto_flow = 12
    pp0, pp1, pp2, pp3, pp4                        = 1, 7, 7, 7, 7
    time_t1, time_t2, time_t3, time_t4             =    8, 8, 8, 8
    profile_01, profile_12, profile_23, profile_34 =  "Gradual", "Flat", "Flat", "Flat"
    kP = 0.07
    kI = 0.12
    kD = 2.50
    k0 = 0.035
    time_preheat = 3.0 # Preheating lasts 3 seconds
    time_shot_heat = 23.0 # No heat in the last 31-23 = 8 seconds
    beta_0 = 29.0  # means: 29% heat on average over entire time_preheat+time_shot_heat for a reference 36g shot, where time_preheat+time_shot_heat = 27 seconds.
    alpha = -25.0   # means: a 25% drop in heat from preheat to end of shot_heat_duration
    warmup_temp = 101
    warmup_minutes = 3



load_settings()

steam_temp = 90.0

time_auto_flow = 13

power = 0
pump_power = 0 

# These values will be logged and saved in a json file
# y, heat_series, shot_series are directly used for display

start_temp = read_sensor()
y = deque([start_temp]*3, 3600) 
y_time = deque([0]*3, 3600)

heat_series = deque([100]*3, 3600)
shot_series = deque([False]*3, 3600)

weight_series = []
weight_series_time = []
filtered_weight_series = []
filtered_flow_series = []
filtered_time = []
adc_values = []
raw_weights = []


# With a refresh speed of 0.5 seconds, 3600 data points = 30 minutes of temperature logging. Perhaps overkill.
# Making sure that y starts with a length of 3, so that the PID function can be run right after read_temp.

trigger_refresh_graph = False
trigger_heat = False
trigger_refresh_timer = False
filter_on = False
trigger_refresh_weight_graph = False

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
    I = clip(I, -1, 1)
    # D: how has y evolved over the last 4 readings?
    if len(y) < 4:
        D = 0
    else:
        D = y[-2] - y[-1]
        # D = ((y[-4] + y[-3])/2 - (y[-2] + y[-1])/2)/2
    if start_script_time > end_shot_time:
        minutes_on = (time.time()-start_script_time)/60
        baseline_power = interpolate(minutes_on, (13, k0), (20, k0/4))
    else:
        minutes_since_last_shot = (time.time()-end_shot_time)/60
        baseline_power = interpolate(minutes_since_last_shot, (6, k0), (10, k0/4))
    pid_power = (baseline_power + kP*P + kI*I + kD*D)*100
    power = clip(pid_power, 0, 100)
    return(power)

# Open-loop temperature control during the shot
# Using settings: beta_0, alpha, and:

beta_1 = ((set_temp-20.0)/340.0)/(time_preheat + time_shot_heat)*100.0    # means: average heat during shot increases by about 0.70% for every additional gram of water.

# About beta_1:
# Heating 1 calorie = heating 1 gram of water by 1 degree celsius.
# Heating 1 additional gram from 20 C to 90 C = 70 calories.
# The boiler is 1425 Watt; i.e. it can produce 1425 joules per second (1 watt = 1 joule per second by definition) = 340 calories per second (1 calorie = 4.19 joules).
# Heating 1 additional gram requires turning the boiler on for an additional 70/340 = 0.20 second during the total duration of the shot.
# Spread over the entire duration of heating: 0.20/(time_preheat + time_shot_heat)*100% heat needs to be applied for each heat cycle.
# beta_1 should be hard-coded and not adjustable from the settings menu.

def shot_heat_gain(target_weight, target_time, time_preheat, time_shot_heat, alpha, beta_0, beta_1):
    # this function is used to scale heat power during the shot
    shot_heat_duration = time_preheat + time_shot_heat
    t = -time_preheat
    unscaled_total_heat = 0.0
    while t < time_shot_heat:
        unscaled_total_heat += 1 + alpha/100 * (t+time_preheat)/shot_heat_duration # Could add other stuff here: predicted flow rate, pump power???
        t += .82 # This is approximately the time between heat cycles
    heat_gain = 27/.82 *(beta_0 + beta_1 * (target_weight - 36))/unscaled_total_heat
    # print "Heat gain = %s" %heat_gain
    return heat_gain


def adjust_heating_power():
    global power, post_shot, length_history
    if time.time() - start_script_time < warmup_minutes*60 and start_temp < 50:
    # if heat_cycles < warmup_minutes*60/0.79 and start_temp < 50:
        early_boost = warmup_temp - set_temp # For the first 3.5 minutes, if the machine starts cold, set_temp will be boosted up to 101 degrees. The machine will stabilize faster around set_temp.
    else:
        early_boost = 0
    if shot_pouring == False:
        if (steam_on == False) and (post_shot == False): 
            if (y[-1] > .90 * (set_temp + early_boost)) and (y[-1] < 110):
            #     if abs(y[-1] - set_temp) <= .5 and y[-1] <= y[-4] + .25  :  # Temp is close to set point and either dropping or increasing very slowly.
            #         power = max(2, pid(y, set_temp + early_boost)) # Power on will always be at least 2%.
            #     else:
                power = pid(y, set_temp + early_boost)
            elif (y[-1] <= .90 * (set_temp + early_boost)):
                power = 100
            elif (y[-1] >= 110):
                power = 0
        elif (steam_on == False) and (post_shot == True):
            if (y[-1] > set_temp) and (time.time() - end_shot_time < 40) and (y[-1] < 98): # Wait until temperature goes back down and stabilizes somewhat before handing temp control back to the PID
                if y[-1] < y[-2]:
                    power = 16
                else:
                    power = 0
            else:
                length_history = 2 # Resetting length_history to avoid confusing the PID (integral term) with high post shot temperature values
                power = pid(y, set_temp + early_boost)
                post_shot = False 
        elif steam_on == True:
            if y[-1] < steam_temp:
                power = 100
            else:
                power = 0
    elif shot_pouring == True:
        if (y[-1] >= 105):
                power = 0
        else:
            if seconds_elapsed < time_shot_heat:
                power = clip(heat_gain * (1 + alpha/100 * (time.time() - start_preheat_time)/(time_preheat + time_shot_heat)), 0, 100)
            else:
                power = 0
            # print round(power, 2)
    # Now that everything is calculated: append heat power data, along with whether a shot is being pulled or not, to lists.
    heat_series.append(power)
    shot_series.append(shot_pouring)
    
# heat_cycles = 0

def output_heat():
    # global heat_cycles
    if power > 0:
        GPIO.output(heat_pin, True)
        time.sleep(.79 * power/100) # remember that the conversion speed of the temperature sensor is always > .82 second. read_temp() and output_heat() work in 2 different threads; they shouldn't get out of sync. output_heat() is called each time a new temperature reading has been made; and the last output_heat() should always end before a new reading becomes available...
    if power < 100:
        GPIO.output(heat_pin, False)
        time.sleep(.79 * (100-power)/100)
    GPIO.output(heat_pin, False)
    # if heat_cycles <= warmup_minutes*60/0.79 + 1:
    #     heat_cycles += 1        # Note it's better to count heat cycles than to rely on time.time() as NTP may need to sync if the RPi just booted.

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
adc_diff10 = 0

read_adc_thread_already_started = False

def read_adc():
    # ADC measurements (scale and pot) have to be read sequentially; can't be done in 2 separate threads running concurrently for scale and pot, or output values will be wrong.
    global adc_diff10, adc_singl3, read_adc_thread_already_started
    # Make sure that only one read_adc thread is running at the same time.
    if read_adc_thread_already_started == True:
        print "Another read_adc() thread is already running, no need to open another one."
        return
    else:
        read_adc_thread_already_started = True
    print "Started read_adc() thread"
    while keep_reading_pot == True or keep_reading_scale == True:
        if keep_reading_scale == True:
            try:
                adc_diff10 = -adc.readADCDifferential01(adc_resolution, sps)
            except Exception as e:
                print "Exception in read_adc(): adc.readADCDifferential01() " + str(e)
            read_scale(adc_diff10)
        if keep_reading_pot == True:
            try:
                adc_singl3 = adc.readADCSingleEnded(3, 4096, 860)
            except Exception as e:
                print "Exception in read_adc(): adc.readADCSingleEnded() " + str(e)
            read_pot(adc_singl3)
            if keep_reading_scale == False: # Give the loop a rest if we're only measuring the pot voltage (at a super high sampling rate already); but if we also need to measure the voltage differential from the scale (at a lower sampling rate, for more precision), skip this step...
                time.sleep(.1)
    if keep_reading_pot == False and keep_reading_scale == False:
        read_adc_thread_already_started = False
        print "Ended read_adc() thread"
        return

trigger_process_weight_reading = False

def read_scale(adc_diff10):
    # Running in its own thread
    global t1, raw_weight_values, trigger_process_weight_reading
    if adc_diff10 > 0.1:
        raw_weight_values.append(scaling[0] + adc_diff10/1000.0*scaling[1])
    else:
        print "Problem with scale: Reading unlikely voltage differential between ADC1 and ADC0 = %sV." %(adc_diff10)
        print "Replacing implied raw_weight_value (%s g.) with previous value (%s g.)." %(scaling[0] + adc_diff10/1000.0*scaling[1] ,raw_weight_values[-1])
        raw_weight_values.append(raw_weight_values[-1])
    t1 = time.time()
    trigger_process_weight_reading = True
    # Let the volts-gram conversion and all the filtering happen in another thread.

def read_pot(adc_singl3):
    global pot_value
    old_pot_value = pot_value
    pot_value = adc_singl3/3313.0 * 100
    if abs(pot_value - old_pot_value) < 2: # To prevent oscillation, any change in pot value that is less than 2% will be discarded.
        pot_value = old_pot_value


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
    # For example: if target_time = 31.5:
    # breakpoints_now = [interpolate(seconds_elapsed, (10,35), (25,27)), 
    #                    interpolate(seconds_elapsed, (10,40), (25,30)),
    #                    interpolate(seconds_elapsed, (10,70), (25,33))]
    breakpoints_now = [interpolate(seconds_elapsed, (10, target_time +  3.5), (0.80 * target_time, target_time - 4.5)), 
                       interpolate(seconds_elapsed, (10, target_time +  8.5), (0.80 * target_time, target_time - 1.5)), 
                       interpolate(seconds_elapsed, (10, target_time + 38.5), (0.80 * target_time, target_time + 1.5))]
    breakpoints_future =  [interpolate(seconds_elapsed + deriv_gain0, (10, target_time +  3.5), (0.80 * target_time, target_time - 4.5)), 
                           interpolate(seconds_elapsed + deriv_gain0, (10, target_time +  8.5), (0.80 * target_time, target_time - 1.5)), 
                           interpolate(seconds_elapsed + deriv_gain0, (10, target_time + 38.5), (0.80 * target_time, target_time + 1.5))]
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

def apply_pump_profile():
    global pump_power
    if seconds_elapsed >= t_list[len(pp_list) - 1]:
        # If seconds elapsed >= t4
        pump_power = int(clip(pp_list[len(pp_list) - 1], min_pp, max_pp))
    else: 
        # Otherwise, follow the pump profile:
        for i in range(0, len(pp_list) - 1):
            # if seconds_elapsed in [t0, t1), [t1, t2), [t2, t3), [t3, t4)
            if seconds_elapsed >= t_list[i] and seconds_elapsed < t_list[i + 1]:
                if profile_list[i] == "Flat" or t_list[i] == t_list[i+1]:
                    pump_power = int(clip(pp_list[i], 0, max_pp))
                else:
                    pump_power = int(clip(interpolate(seconds_elapsed, (t_list[i], pp_list[i]), (t_list[i+1], pp_list[i+1])), 0, max_pp))
                break

def pour_shot():
    # This function will be run in its own thread
    print "pour_shot thread started"
    # Remember to open the 3-way valve here.
    global pump_power, shot_pouring, end_shot_time, trigger_update_log_shot, flow_per_second, predicted_end_time, current_time, dt, keep_reading_pot, trigger_refresh_display_pump, log_shot, pp_list, t_list, profile_list
    if flow_mode == "Manual":
        keep_reading_pot = True
        thread.start_new_thread(read_pot, ())
    else: 
        keep_reading_pot = False
    time.sleep(time_preheat+.5)
    log_shot = True
    trigger_update_log_shot = False
    thread.start_new_thread(update_log_shot, ())
    last_auto_adjust = 0.0
    predicted_end_time = 0
    pump_power = 0
    previous_pump_power = -1
    dt = 1.5
    thread.start_new_thread(output_pump, ())
    pp_list      = [pp0, pp1,     pp2,     pp3,     pp4]
    t_list       = [  0, time_t1, time_t2, time_t3, time_t4]
    profile_list = [profile_01, profile_12, profile_23, profile_34]
    # pp_list.append(pp_list[-1])
    # t_list.append(time_auto_flow)
    # profile_list.append("Flat")
    
    while shot_pouring == True:
        current_time = time.time() - start_script_time
        
        if flow_mode == "Auto":    
            if (seconds_elapsed < time_auto_flow) and (filtered_weight <= 1):
                apply_pump_profile()
            else:
                if (time.time() - last_auto_adjust >= dt):
                    flow_per_second = flow_smooth
                    if (flow_per_second > 0):
                        predicted_end_time = (time.time() - start_shot_time) + (target_weight - filtered_weight) / flow_per_second - 1.15
                        try: 
                            old_predicted_end_time 
                        except NameError:
                            old_predicted_end_time = predicted_end_time
                        change_predicted_end_time = predicted_end_time - old_predicted_end_time
                    else: # Should never be in this situation: weight was reported as > 2 grams, or we're after 11 seconds, but coffee is not flowing. 
                        predicted_end_time = 100  # The solution: force a pump power increase by reporting a long predicted time.
                        old_predicted_end_time = 100
                        change_predicted_end_time = 0
                    pump_change = auto_adjust_pump(predicted_end_time, change_predicted_end_time, time.time() - start_shot_time)
                    pump_power = int(clip(pump_power + pump_change, min_pp, max_pp))
                    old_predicted_end_time = predicted_end_time
                    last_auto_adjust = time.time()
                    
        elif flow_mode == "Manual":
            pump_power = clip(int(round(pot_value / 10, 0)), 0, 10)
            
        elif flow_mode == "Program":
            apply_pump_profile()
        
        if filtered_weight > target_weight - 1.15*flow_smooth: 
            end_shot()
        
        if (pump_power != previous_pump_power) or (time.time()-last_log_time >= dt) or (time.time() - last_auto_adjust < .125):
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
    global last_log_time, trigger_update_log_shot, weight_series, weight_series_time, filtered_weight_series, adc_values, raw_weights, filtered_flow_series, filtered_time
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
    log_raw_weight = []
    log_adc_diff10 = []
    # Idea: record more parameters here: target_weight, time_t1, max_power, temperature at the beginning of the shot, temperature at the end, power_when_pulling_shot, etc. These could be added to the text of the e-mail.
    
    last_log_time = 0
    while log_shot == True:
        time_too_long      =  interpolate(seconds_elapsed, (10, target_time + 38.5), (0.80 * target_time, target_time + 1.5))
        time_too_short     =  interpolate(seconds_elapsed, (10, target_time +  8.5), (0.80 * target_time, target_time - 1.5))  
        time_way_too_short =  interpolate(seconds_elapsed, (10, target_time +  3.5), (0.80 * target_time, target_time - 4.5)) 
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
                       "target_time": target_time,
                       "alpha": alpha,
                       "beta_0": beta_0,
                       "time_preheat": time_preheat,
                       "time_shot_heat": time_shot_heat,
                       "end": end,
                       "raw_weights": list(raw_weights),
                       "adc_values" : list(adc_values),
                       "tare_weight": tare_weight
                   }, open(filename, "w"))
            if len(sys.argv) > 1 and sys.argv[1] == "test":
                pass
            else:
                os.system(str("sudo R --silent --vanilla -f /home/pi/flow_analysis.R --args " + filename))
            # flow_analysis.R will generate graphs with ggplot, save them as pdf, and run send_email.py to send the pdf & json files as attachment
        # To clean up (but once everything has been dumped into the json file), empty the _series lists so that old values don't get dumped again for the next shot. 
        weight_series = []
        weight_series_time = []
        filtered_weight_series = []
        filtered_flow_series = []
        filtered_time = []
        adc_values = []
        raw_weights = []
        return

def time_shot():
    # This function will be run in its own thread
    # print "time_shot thread started"
    global seconds_elapsed, start_shot_time
    time.sleep(time_preheat+.5)
    start_shot_time = time.time()
    while shot_pouring == True:
        seconds_elapsed = math.floor((time.time() - start_shot_time)*10)/10
        refresh_timer_display()
        time.sleep(.1)
    # Make sure that the thread can exit
    if shot_pouring == False:
        # print "time_shot thread exited"
        return
        
# Several problems with time_shot():
# this function does too much. but it is not a good timer: "seconds_elapsed" is inaccurate: even if time.time() - start_shot_time = 12.199, seconds_elapsed = 12.1.
# Most of what the function does should be called directly by pour_shot. 
# seconds_elapsed can only be used for display. for filtering calculations, pump power, logs, each function needs to call time.time() when it needs to.


def end_shot():
    global wait, end_shot_time, shot_pouring, pump_power, post_shot, display_end_shot_line_now, flow_per_second, predicted_end_time
    end_shot_time = time.time()
    shot_pouring = False
    wait = True
    GPIO.output(pump_pin, 0)
    pump_power = 0
    flow_per_second = 0
    predicted_end_time = 0
    display_end_shot_line_now = True
    post_shot = True
    # Remember to close the 3-way valve here.
    thread.start_new_thread(wait_after_shot_and_refresh, ())

last_log_time = time.time()
wait = False

def wait_after_shot_and_refresh():
    global wait, trigger_refresh_graph, keep_reading_scale, last_weight, last_timer, log_shot, trigger_update_log_shot, current_time, filter_on, seconds_elapsed, filtered_weight
    wait = True
    while time.time() - end_shot_time < 6:
        if time.time() - last_log_time >= .5:
            current_time = time.time() - start_script_time
            trigger_update_log_shot = True
        time.sleep(.02)
    if time.time() - end_shot_time >= 6:
        temp_surface = lcd.subsurface(((27, 70), (253, 150)))
        pygame.image.save(temp_surface, "/home/pi/lastshot.png")
        reset_graph_area()
        lcd.fill(col_background, area_timer)
        pygame.display.update(area_timer)
        refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)
        trigger_refresh_graph = True
        keep_reading_scale = False
        last_weight = current_weight
        last_timer = seconds_elapsed
        log_shot = False
        filter_on = False
        seconds_elapsed = 0.0
        filtered_weight = 0.0
        wait = False
        refresh_buttons()
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

####################################
# UI area positions and dimensions #
####################################

# Screen resolution is 320 * 240.
# Careful here: pygame.Rect uses these coordinates for rectangles: (left, top, width, height) [not (left, top, right, bottom)]
area_graph = ((0,60),(290,160))
# This is the reduced-size graph window, used when pulling a shot or entering the settings menu.
# area_graph = ((0,65),(150,155))
area_text_temp = ((0,0),(160,60)) # same here, for the refresh_graph function.
area_timer = ((160,0),(120,60))   # same for the refresh_timer_display
area_icons =((290,0),(30,240))
area_belowgraph = ((0,220), (280, 20))
area_menu = ((150, 60), (130, 160))

min_range = 5
graph_top = area_graph[0][1]
graph_bottom = area_graph[0][1] + area_graph[1][1] 
graph_left  = area_graph[0][0]
graph_right = area_graph[0][0] + area_graph[1][0] 

npixels_per_tempreading = 2


# Setting up the screen 


os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.display.init()
pygame.font.init()
pygame.mouse.set_visible(False)
# lcd = pygame.display.set_mode((320, 240))
# 24 bit depth required to use anti-aliasing
lcd = pygame.display.set_mode((320, 240),0, 24)

col_lightblue = (0,191,255)
col_orange = (255,165,0)
col_lightgreen = (124,252,0)
col_red = (255,0,0)
col_white = (255,255,255)
col_veryverydarkgrey = (48,48,48)
col_verydarkgrey = (64,64,64)
col_darkgrey = (128,128,128)
col_medgrey = (192,192,192)
col_lightgrey = (224,224,224)
col_black = (0,0,0)
col_lightred = (153,0,76)
col_verylightred = (218,62,140)
col_yellow = (255,255,0)
col_darkred = (204,0,0)
col_blue = (44, 103, 210)

col_background                  = col_black
col_text                        = col_lightblue
col_axis_lines                  = col_lightblue
col_axis_labels                 = col_lightblue
col_pump_box_full               = col_lightblue
col_pump_box_empty              = col_verydarkgrey
col_pump_text                   = col_lightblue
col_temp_line                   = col_white
col_temp_line_during_shot       = col_red
col_heat                        = col_orange
col_weight_text                 = col_white
col_weight_line                 = col_white
col_lastshot                    = col_darkgrey
col_settemp_line                = col_orange
col_menu_text                   = col_white
col_menu_text_select            = col_black
col_menu_text_select_background = col_lightgrey
col_menu_value                   = col_white
col_flow_polygon                = col_lightred
col_flow_line                   = col_verylightred
col_weight_target               = col_orange
col_pump_profile_line           = col_white
col_pump_profile_grid           = col_verydarkgrey
col_pump_profile_param             = col_red
col_intrashot_grid              = col_verydarkgrey
col_intrashot_label             = col_white
color_scenarios                 = [col_darkred, col_orange, col_yellow, col_white]
col_lights_outline                = col_verydarkgrey



# col_background                  = col_white
# col_text                        = col_blue
# col_axis_lines                  = col_blue
# col_axis_labels                 = col_blue
# col_pump_box_full               = col_blue
# col_pump_box_empty              = col_lightgrey
# col_pump_text                   = col_blue
# col_temp_line                   = col_verydarkgrey
# col_temp_line_during_shot       = col_orange
# col_heat                        = col_red
# col_weight_text                 = col_verydarkgrey
# col_weight_line                 = col_black
# col_lastshot                    = col_lightgrey
# col_settemp_line                = col_red
# col_menu_text                   = col_black
# col_menu_text_select            = col_white
# col_menu_text_select_background = col_blue
# col_menu_value                   = col_black
# col_flow_polygon                = col_lightred
# col_flow_line                   = col_lightred
# col_weight_target               = col_darkred
# col_pump_profile_line           = col_black
# col_pump_profile_grid           = col_lightgrey
# col_pump_profile_param             = col_red
# col_intrashot_grid              = col_lightgrey
# col_intrashot_label             = col_black
# color_scenarios                 = [col_darkred, col_orange, col_yellow, col_white]
# col_lights_outline                = col_verydarkgrey

lcd.fill(col_background, ((0,0), (320,240)))
pygame.display.update()

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
    elif largest > 75:
        tick = 25
    elif largest > 40:
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
    # if max(subset_y) - min(subset_y) < min_range:
    #     y_range = (min(subset_y) - min_range/2 + (max(subset_y) - min(subset_y))/2, max(subset_y) + min_range/2 - (max(subset_y) - min(subset_y))/2)
    # else:
    #     y_range = (min(subset_y), max(subset_y))
    y_range = (min(min(subset_y), set_temp), max(max(subset_y), set_temp))
    padding_y_axis = 0.05 * (y_range[1] - y_range[0])
    # These are the most extreme values that could be plotted on the y axis, on the top/bottom edges of area_graph
    y_axis = (int(min(math.floor(y_range[0] - padding_y_axis), set_temp - 3)), int(max(math.ceil(y_range[1] + padding_y_axis), set_temp + 3)))
    return y_axis


def display_text(string, coordinates, size, color, condensed = False):
    if condensed == False:
        font = pygame.font.Font(None, size)
    else:
        font = font = pygame.font.Font('/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Bold.ttf', size)
    text_surface = font.render(string, True, color)  
    lcd.blit(text_surface, coordinates)


def draw_axes(y_axis, tick, graph_top, graph_bottom):
    for val in xrange(0, y_axis[1], tick) :
        coord_val = coordinates(y_axis, val, graph_top, graph_bottom)
        if (coord_val < graph_bottom) and (coord_val > graph_top): # remember that y coordinates start at 0 from the top of the display
            pygame.draw.line(lcd, col_axis_lines, (graph_left, coord_val), (graph_right-30, coord_val)) # change this 300 value, based on graph_right
            display_text(str(val), (graph_right-25, coord_val-10), 20, col_axis_labels) # change this 305 value, based on graph_right, leave space for icons

def draw_lines(y_coord, y_axis, graph_top, graph_bottom, color_series):
    coord_set_temp = coordinates(y_axis, set_temp, graph_top, graph_bottom)
    if (coord_set_temp < graph_bottom) and (coord_set_temp > graph_top): # remember that y coordinates start at 0 from the top of the display
        pygame.draw.line(lcd, col_settemp_line, (graph_left, coord_set_temp), (graph_right-30, coord_set_temp)) # change this 300 value, based on graph_right
    # pointlist = [[graph_left + npixels_per_tempreading*j for j in range(0, len(y_coord))],
    #              [y_coord[j] for j in range(0, len(y_coord))]]
    # pointlist = [[pointlist[0][i],pointlist[1][i]] for i in range(0, len(y_coord))]
    for j in xrange(1, len(y_coord)):
        pygame.draw.aaline(lcd, color_series[j-1], (graph_left + npixels_per_tempreading*(j-1), y_coord[j-1]),(graph_left + npixels_per_tempreading*j, y_coord[j]), 0)

# def draw_lines(subset_y, y_axis, tick, graph_top, graph_bottom, color_series):
#     coord_set_temp = coordinates(y_axis, set_temp, graph_top, graph_bottom)
#     if (coord_set_temp < graph_bottom) and (coord_set_temp > graph_top): # remember that y coordinates start at 0 from the top of the display
#         pygame.draw.line(lcd, col_settemp_line, (graph_left, coord_set_temp), (graph_right-30, coord_set_temp)) # change this 300 value, based on graph_right
#     y_coord  = [coordinates(y_axis, subset_y[j], graph_top, graph_bottom) for j in xrange(0, len(subset_y))]
#     # pointlist = [[[graph_left + npixels_per_tempreading*(j-1), y_coord[j-1]], [graph_left + npixels_per_tempreading*j, y_coord[j]]] for j in range(1, len(y_coord))]
#     # pointlist = list(itertools.chain.from_iterable(pointlist))
#     # pygame.draw.lines(lcd, col_temp_line, False, pointlist, 1)
#     graph_surface.lock()
#     for j in xrange(1, len(y_coord)):
#         pygame.draw.aaline(graph_surface, color_series[j-1], (npixels_per_tempreading*(j-1), y_coord[j-1] - graph_top),(npixels_per_tempreading*j, y_coord[j] - graph_top)) # 2 is the line thickness here.
#     #   pygame.draw.line(lcd, color_series[j-1], (graph_left + npixels_per_tempreading*(j-1), y_coord[j-1]),(graph_left + npixels_per_tempreading*j, y_coord[j]), 1) # 2 is the line thickness here.
#     graph_surface.unlock()
#     lcd.blit(graph_surface, (graph_left, graph_top))


def draw_power(power_data):
    for j in xrange(0, len(power_data)):
        pygame.draw.line(lcd, col_heat, (graph_left+npixels_per_tempreading*j, 220), (graph_left+npixels_per_tempreading*j, int(220-power_data[j]/4)),1)



# prev_y_axis = (0, 0)
# y_minmax = (65, 215)
# prev_y_minmax = (0, 0)

def refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp):
    global prev_y_axis, prev_y_minmax, prev_x_range
    # Transform the series of 150 most recent ys into coordinates on the screen
    lcd.fill(col_background, rect = area_timer)
    lcd.fill(col_background, rect = area_text_temp)
    lcd.fill(col_background, rect = area_belowgraph)
    if (items_in_current_menu[selection] != "Shot Profile" and items_in_current_menu[selection] not in all_items["Shot Profile"].children 
    and items_in_current_menu[selection] != "Pump Profile" and items_in_current_menu[selection] not in all_items["Pump Profile"].children 
    and items_in_current_menu[selection] != "Preset Profiles" and items_in_current_menu[selection] not in all_items["Preset Profiles"].children
    and items_in_current_menu[selection] != "Heat Profile" and items_in_current_menu[selection] not in all_items["Heat Profile"].children 
    and items_in_current_menu[selection] != "Intrashot" and items_in_current_menu[selection] not in all_items["Intrashot"].children 
    and items_in_current_menu[selection] != "Time Auto Flow" 
    and display_lastshot == False and (keep_reading_scale == False or (keep_reading_scale == True and items_in_current_menu[selection] == "Test Scale"))):
        n_datapoints = int(math.floor((graph_right-30-graph_left)/npixels_per_tempreading))
        subset_y = [y[k] for k in xrange(max(0, len(y)-n_datapoints), len(y))]
        subset_heat_series = [heat_series[k] for k in xrange(max(0, len(heat_series)-n_datapoints), len(heat_series))]
        subset_shot_series = [shot_series[k] for k in xrange(max(0, len(shot_series)-n_datapoints), len(shot_series))]
        color_series = [col_temp_line_during_shot if whatshappening else col_temp_line for whatshappening in subset_shot_series]
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
        # lcd.fill(col_background, rect = ((0,195),(area_graph[1][0], 25)))
        draw_power(subset_heat_series)
        draw_axes(y_axis, tick, graph_top, graph_bottom-25)
        draw_lines(y_coord, y_axis, graph_top, graph_bottom-25, color_series)
    if keep_reading_scale == False:
        draw_belowgraph()
        if y[-1] >= 100:
            display_text(str(int(round(y[-1]))) + u"\u2103", (5, 5), 60, col_text)
        elif y[-1] < 100:
            display_text('{0:.1f}'.format(y[-1]) + u"\u2103", (5, 5), 60, col_text) # u"\u2103" is the unicode degrees celsisus sign.
        if (last_weight != 0) and (last_timer != 0):
            display_text("Last shot", (180, 8), 25, col_text)
            display_text("%0.0f s. / %0.0f g." %(last_timer, last_weight), (180, 26), 25, col_text)
        for rect in [area_graph, area_belowgraph, area_text_temp, area_timer]:
            pygame.display.update(rect)
    else:
        return
        # draw_belowgraph()
        # pygame.display.update(area_belowgraph)
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
icon_stop = pygame.image.load(os.path.join('/home/pi/icons', 'stop_circle.png'))
icon_next = pygame.image.load(os.path.join('/home/pi/icons', 'next.png'))
icon_forward = pygame.image.load(os.path.join('/home/pi/icons', 'forward.png'))
icon_steam = pygame.image.load(os.path.join('/home/pi/icons', 'steam.png'))
icon_flush = pygame.image.load(os.path.join('/home/pi/icons', 'flush.png'))
icon_coffee = pygame.image.load(os.path.join('/home/pi/icons', 'coffee.png'))
pressed_bg = pygame.image.load(os.path.join('/home/pi/icons', 'pressed_bg.png'))

def display_intrashot_graph():
    area_graph = ((0,60),(150,160))
    lcd.fill(col_background, rect = area_graph)
    x0 = 35.0
    x1 = 135.0
    y0 = 90.0
    y1 = 180.0
    a = (x1-x0)/(target_time + time_preheat)
    b = -(y1-y0)/100.0
    heat_gain = shot_heat_gain(target_weight, target_time, time_preheat, time_shot_heat, alpha, beta_0, beta_1)
    def intrashot_heat(t):
        # t is time since start_preheat_time
        if t <= time_shot_heat + time_preheat:
            value = clip(heat_gain * (1 + alpha/100 * t/(time_preheat + time_shot_heat)), 0, 100)
        else:
            value = 0
        return(value)
    def x_coord(t):
        return(a*t + x0)
    def t(x):
        return((float(x)-x0)/a)
    def y_coord(h):
        return(b*h + y1)
    for h_value in [25, 50, 75]:
        pygame.draw.line(lcd, col_intrashot_grid, (x0,y_coord(h_value)), (x1,y_coord(h_value)))
        display_text(str(h_value) + "%", (0, y_coord(h_value)-5), 20, col_intrashot_label)
    for time in [0, time_preheat, time_preheat+time_shot_heat, time_preheat + target_time]:
        pygame.draw.line(lcd, col_intrashot_grid, (x_coord(time), y1), (x_coord(time), y0))
    for x in range(int(x0), int(x1 + 2), 2):
        h_value = intrashot_heat(t(x))
        pygame.draw.line(lcd, col_heat, (x,y1), (x,y_coord(h_value)))
    for time in [0, time_preheat+time_shot_heat]:
        time_to_display = int(round(time - time_preheat, 0))
        string = "%ss." %(time_to_display)
        width = pygame.font.Font(None, 20).render(string, True, (0,0,0)).get_width()
        display_text(string, (x_coord(time) - width/2, y1 + 5), 20, col_intrashot_label)
    for h_value in [0, 100]:
        pygame.draw.line(lcd, col_intrashot_grid, (x0,y_coord(h_value)), (x1,y_coord(h_value)))
        display_text(str(h_value) + "%", (0, y_coord(h_value)-5), 20, col_intrashot_label)
    legend_string = str("%0.0f" %(intrashot_heat(0)))+"-"+str("%0.0f" %(intrashot_heat(time_preheat + time_shot_heat)))+"%"
    width = pygame.font.Font(None, 20).render(legend_string, True, (0,0,0)).get_width()
    display_text(legend_string, ((x0+x1)/2-width/2, y0 - 20) ,20, col_intrashot_label)
    pygame.display.update(area_graph)

def display_profile_graph():
    # print all_items[items_in_current_menu[selection]].title
    area_graph = ((0,60),(150,160))
    maxt = max(12, time_t4, time_auto_flow)
    x0 = 15
    x1 = 135
    alpha_x = (x1-x0)/maxt
    y0 = 80
    y1 = 180
    pp_list      = [pp0, pp1,     pp2,     pp3,     pp4]
    t_list       = [  0, time_t1, time_t2, time_t3, time_t4]
    profile_list = [profile_01, profile_12, profile_23, profile_34]
    # Create a list of tuples for points between which lines have to be drawn.
    points = [(0, pp0)]
    for i in range(0, len(profile_list)):
        if profile_list[i] == "Flat":
            # if profile is flat on an interval, the line will look like a step function
            points.append((t_list[i+1], pp_list[i]))
        points.append((t_list[i+1], pp_list[i+1]))
    points.append((maxt, pp4))
    vlines = range(0, maxt + 1)
    hlines = range(0, 11)
    coord_points = []
    lcd.fill(col_background, rect = area_graph)
    for i in range(0, len(vlines)):
        x = alpha_x * vlines[i] + x0
        pygame.draw.line(lcd, col_pump_profile_grid, (x, y0), (x, y1))
    for i in range(0, len(hlines)):
        y = 10 * hlines[i] + y0
        pygame.draw.line(lcd, col_pump_profile_grid, (x0 ,y), (x1 ,y))
    for i in range(0, len(points)):
        # linear transformation into coordinates
        x = alpha_x * points[i][0] + x0 
        y = 10 * (10 - points[i][1]) + y0
        coord_points.append((x, y))
    pygame.draw.lines(lcd, col_pump_profile_line, False, coord_points)
    pygame.draw.circle(lcd, col_pump_profile_line, (x0                    , y1 - 10 * pp0), 3)
    pygame.draw.circle(lcd, col_pump_profile_line, (x0 + alpha_x * time_t1, y1 - 10 * pp1), 3)
    pygame.draw.circle(lcd, col_pump_profile_line, (x0 + alpha_x * time_t2, y1 - 10 * pp2), 3)
    pygame.draw.circle(lcd, col_pump_profile_line, (x0 + alpha_x * time_t3, y1 - 10 * pp3), 3)
    pygame.draw.circle(lcd, col_pump_profile_line, (x0 + alpha_x * time_t4, y1 - 10 * pp4), 3)
    if items_in_current_menu[selection] == "Pump Power P0":
        # Pump P0
        pygame.draw.line(lcd, col_pump_profile_param, (x0, y1), (x0, y1 - 10 * pp0))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0, y1 - 10 * pp0), 3)
    if items_in_current_menu[selection] == "Pump Power P1":
        # Pump P1
        pygame.draw.line(lcd, col_pump_profile_param, (x0 + alpha_x * time_t1, y1), (x0 + alpha_x * time_t1, y1 - 10 * pp1))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t1, y1 - 10 * pp1), 3)
    if items_in_current_menu[selection] == "Pump Power P2":
        # Pump P2
        pygame.draw.line(lcd, col_pump_profile_param, (x0 + alpha_x * time_t2, y1), (x0 + alpha_x * time_t2, y1 - 10 * pp2))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t2, y1 - 10 * pp2), 3)
    if items_in_current_menu[selection] == "Pump Power P3":
        # Pump P3
        pygame.draw.line(lcd, col_pump_profile_param, (x0 + alpha_x * time_t3, y1), (x0 + alpha_x * time_t3, y1 - 10 * pp3))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t3, y1 - 10 * pp3), 3)
    if items_in_current_menu[selection] == "Pump Power P4":
        # Pump P4
        pygame.draw.line(lcd, col_pump_profile_param, (x0 + alpha_x * time_t4, y1), (x0 + alpha_x * time_t4, y1 - 10 * pp4))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t4, y1 - 10 * pp4), 3)
    if items_in_current_menu[selection] == "Time T1":
        # Time T1
        pygame.draw.line(lcd, col_pump_profile_param, (x0, y1), (x0 + alpha_x * time_t1, y1))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t1, y1), 3)
    if items_in_current_menu[selection] == "Time T2":
        # Time T2
        pygame.draw.line(lcd, col_pump_profile_param, (x0, y1), (x0 + alpha_x * time_t2, y1))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t2, y1), 3)
    if items_in_current_menu[selection] == "Time T3":
        # Time T3
        pygame.draw.line(lcd, col_pump_profile_param, (x0, y1), (x0 + alpha_x * time_t3, y1))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t3, y1), 3)
    if items_in_current_menu[selection] == "Time T4":
        # Time T4
        pygame.draw.line(lcd, col_pump_profile_param, (x0, y1), (x0 + alpha_x * time_t4, y1))
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t4, y1), 3)
    if items_in_current_menu[selection] == "Profile 0-T1":
        # Profile 0-T1
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t1, y1 - 10 * pp1), 3)
        pygame.draw.circle(lcd, col_pump_profile_param, (x0, y1 - 10 * pp0), 3)
    if items_in_current_menu[selection] == "Profile T1-T2":
        # Profile T1-T2
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t1, y1 - 10 * pp1), 3)
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t2, y1 - 10 * pp2), 3)
    if items_in_current_menu[selection] == "Profile T2-T3":
        # Profile T2-T3
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t2, y1 - 10 * pp2), 3)
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t3, y1 - 10 * pp3), 3)
    if items_in_current_menu[selection] == "Profile T3-T4":
        # Profile T3-T4
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t3, y1 - 10 * pp3), 3)
        pygame.draw.circle(lcd, col_pump_profile_param, (x0 + alpha_x * time_t4, y1 - 10 * pp4), 3)
    # Draw a vertical line for time_auto_flow if flow_mode == "Auto"
    if flow_mode == "Auto":
        pygame.draw.line(lcd, col_orange, (x0 + alpha_x * time_auto_flow, y0), (x0 + alpha_x * time_auto_flow, y1))
    pygame.display.update(area_graph)

def highlight_pressed_icon(button_num):
    i = button_num - 1
    icon_location = [(290, 30), (290, 90), (290, 150), (290, 210)]
    foreground = pygame.Surface((30,30))
    foreground.blit(lcd, (0,0), (icon_location[i], (30,30)))
    foreground.set_colorkey(col_background)
    lcd.fill(col_black, (icon_location[i], (30,30)))
    lcd.blit(pressed_bg, (icon_location[i], (30,30)))
    lcd.blit(foreground, (icon_location[i], (30,30)))
    pygame.display.update((icon_location[i], (30,30)))
    while GPIO.input(button_pins[i]) == GPIO.LOW: 
        time.sleep(.01)

def display_buttons(blist):
    x0 = 293
    y0 = [33, 93, 153, 213]
    for i in range(0, 4):
        if blist[i] == None:
            next
        else:
            lcd.blit(blist[i], (x0, y0[i]))
    pygame.display.update(area_icons)

def refresh_buttons():
    lcd.fill(col_background, rect = area_icons)
    if menu == 0:
        if flush_on == True:
            display_buttons([None, icon_stop, None, None])
            return
        if steam_on == True:
            display_buttons([None, None, icon_stop, None])
            return
        if shot_pouring == True:
            display_buttons([None, None, None, icon_stop])
            return
        if shot_pouring == False and wait == True:
            display_buttons([None, None, None, None])
            return
        if shot_pouring == False and flush_on == False and steam_on == False and wait == False:
            display_buttons([icon_more, icon_flush, icon_steam, icon_coffee])
            return
    elif menu == 1:
        if menu_stack != []:
            prev_stack = menu_stack[-1]
            items_in_prev_menu = prev_stack["items_in_current_menu"]
            prev_selection = prev_stack["selection"]
        if display_lastshot == True or (keep_reading_scale == True):
            display_buttons([None, None, None, icon_back])
        elif (isinstance(all_items[items_in_current_menu[selection]], Param) == False and 
                 isinstance(all_items[items_in_current_menu[selection]], Action) == False):
            display_buttons([icon_back, icon_up, icon_down, icon_forward])
        elif (isinstance(all_items[current_menu], Param) == True and
                 prev_selection < len(items_in_prev_menu) - 1 and
                 isinstance(all_items[items_in_prev_menu[prev_selection + 1]], Param) == True): 
            display_buttons([icon_back, icon_up, icon_down, icon_next])
        else:
            display_buttons([icon_back, icon_up, icon_down, icon_check])

def reset_graph_area():
    global area_graph, graph_top, graph_bottom, graph_left, graph_right, npixels_per_tempreading
    if (menu == 0) and (shot_pouring == False) and (flush_on == False):
        print "Big graph area"
        lcd.fill(col_background, rect = area_graph)
        pygame.display.update(area_graph)
        area_graph = ((0,60),(290,160))
        npixels_per_tempreading = 2
    elif (menu == 1) or (shot_pouring == True) or (flush_on == True):
        print "Small graph area"
        lcd.fill(col_background, rect = area_graph)
        pygame.display.update(area_graph)
        area_graph = ((0,60),(150,160))
        npixels_per_tempreading = 1
    graph_top = area_graph[0][1]
    graph_bottom = area_graph[0][1] + area_graph[1][1] 
    graph_left  = area_graph[0][0]
    graph_right = area_graph[0][0] + area_graph[1][0]

trigger_refresh_display_pump = False

def refresh_timer_display():
    lcd.fill(col_background, rect = area_timer)
    display_text("%4s" % '{0:.1f}'.format(seconds_elapsed) + "s", (175, 5), 60, col_text)
    pygame.display.update(area_timer)

def display_pump_power():
    global trigger_refresh_display_pump
    trigger_refresh_display_pump = True
    while (keep_reading_scale == True) or (flush_on == True): # "keeps_reading_scale, not shot_pouring, to make sure that pump power is also displayed during taring/preheat phase, when the scale is used, but the shot is not yet pouring"
        if trigger_refresh_display_pump:
            if shot_pouring == True:
                string = "Pump power: %s at %.1fs." %(pump_power, seconds_elapsed)
            else:
                string = "Pump power: %s" %(pump_power)
            print string
            if flush_on:
                area_pump_display = (150, 60, 130, 160)
                lcd.fill(col_background, rect = area_pump_display)
                display_text("Pump", (200, 180), 25, col_pump_text)
                display_text(str(pump_power*10) + "%", (202 if pump_power == 10 else 206, 200), 25, col_pump_text)
                for box in range(1, 11):
                    lcd.fill(col_pump_box_empty, rect= ((215, 170-(box-1)*10), (20, 8)))
                for box in range(1, pump_power + 1):
                    lcd.fill(col_pump_box_full, rect= ((215, 170-(box-1)*10), (20, 8)))
                pygame.display.update(area_pump_display)
                trigger_refresh_display_pump = False
            if (keep_reading_scale == True) and (flush_on == False):
                area_pump_display = (0, 65, 26, 155)
                lcd.fill(col_background, rect = area_pump_display)
                display_text("P", (9, 199), 20, col_pump_text)
                # display_text(str(pump_power*10) + "%", (282 if pump_power == 10 else 285, 187), 20, col_text)
                for box in range(1, 11):
                    lcd.fill(col_pump_box_empty, rect= ((5, 186-(box-1)*11), (17, 8)))
                for box in range(1, pump_power + 1):
                    lcd.fill(col_pump_box_full, rect= ((5, 186-(box-1)*11), (17, 8)))
                lcd.fill(col_background, rect = area_belowgraph)
                draw_belowgraph()
                pygame.display.update(area_pump_display)
                pygame.display.update(area_belowgraph)
                trigger_refresh_display_pump = False
        time.sleep(.05)
    if (keep_reading_scale == False) and (flush_on == False):
        # print "Exiting display pump power thread"
        return

display_end_shot_line_now = False

# Load all_values from a json file, and save into json file on shutdown
try: 
    all_values = json.load(open("/home/pi/lastshot.json","r"))["all_values"]
except:
    all_values = {"w_pointlist": [], "x_endshot": 0, "target_weight": target_weight, "f_pointlist": [], "max_t": 0} 

def display_weight_graph():
    global display_end_shot_line_now, all_values, trigger_refresh_weight_graph
    time.sleep(.10)
    lcd.fill(col_background, area_belowgraph)
    lcd.fill(col_background, area_graph)
    draw_belowgraph()
    pygame.display.update(area_belowgraph)
    pygame.display.update(area_graph)
    # Axes and text
    x0 = 35
    x1 = 280
    # Push x1 a bit to the left to add a pressure display?
    y0 = 70
    y1 = 194
    # lcd.fill(col_background, rect = ((26, y0), (280-26, 220 - y0)))
    max_w = target_weight*1.158887 # make sure that max_w line ends up at line 87, aligned with top of the upper rectangle of pump power.
    max_t = 40.0
    min_w = 0.0
    min_t = 0.0
    a = (x1 - x0)/(max_t - min_t)
    b = -(y1 - y0)/(max_w - min_w)
    target_y =  int(round(b * (target_weight - min_w) + y1))
    # HORIZONTAL LINE FOR TARGET WEIGHT, WITH CENTERED LABEL
    pygame.draw.line(lcd, col_weight_target, (x0, target_y), (x1, target_y))
    string_target_weight = str(target_weight) + "g"
    width = pygame.font.Font(None, 20).render(string_target_weight, True, (0,0,0)).get_width()
    display_text(string_target_weight, (int((x0 + x1)/2 - width/2), target_y - 19), 20, col_weight_target)
    # TICK MARKS FOR DESIRED END SHOT TIME WINDOW (30 - 33s)
    pygame.draw.line(lcd, col_weight_target, (int(round(a*(target_time - 1.5)) + x0), target_y - 3), (int(round(a*(target_time - 1.5)) + x0), target_y + 3))
    pygame.draw.line(lcd, col_weight_target, (int(round(a*(target_time + 1.5)) + x0), target_y - 3), (int(round(a*(target_time + 1.5)) + x0), target_y + 3))
    # DRAW PREVIOUS SHOT WEIGHT CURVE AND END SHOT VERTICAL LINE
    if all_values["w_pointlist"] != [] and all_values["target_weight"] == target_weight:
        lastshot_values = all_values
        lastshot_w = []
        for i in range(0, len(lastshot_values["w_pointlist"])):
            rescaled_x = int(round((lastshot_values["w_pointlist"][i][0] - x0)/max_t*lastshot_values["max_t"] + x0))
            if rescaled_x <= x1:
                lastshot_w.append([rescaled_x, lastshot_values["w_pointlist"][i][1]])
            else:
                break
        pygame.draw.lines(lcd, col_lastshot, False, lastshot_w)
        if lastshot_values["x_endshot"] != 0 and lastshot_values["max_t"] <= max_t:
            for i in range(y0, y1, 5):
                rescaled_x_endshot = int(round((lastshot_values["x_endshot"] - x0)/max_t*lastshot_values["max_t"] + x0))
                lcd.set_at((int(round(rescaled_x_endshot)), i), col_lastshot)
    # X AXIS, WITH TICK MARKS FOR TIME AND CENTERED LABELS 
    vlines = range(0, int(max_t + 1), 10)
    pygame.draw.line(lcd, col_axis_lines, (x0, y1), (x1, y1))
    for i in range(0, len(vlines)):
        x = int(a * (vlines[i] - min_t) + x0)
        pygame.draw.line(lcd, col_axis_lines, (x, y1 + 3), (x, y1))
        if vlines[i] != max_t:
            string = str(vlines[i]) + "s"
            width = pygame.font.Font(None, 20).render(string, True, (0,0,0)).get_width()
            display_text(str(vlines[i]) + "s", (x - width/2, y1 + 5), 20, col_axis_labels)
    pygame.display.update(((26, y0), (280-26, 220-y0)))
    last_x = x0
    last_y_weight = y1
    last_y_flow = y1
    last_update_weight_graph = time.time() - .125
    x_pred = 0
    all_values = {"target_weight": target_weight, "w_pointlist": [], "x_endshot": 0, "f_pointlist": [], "max_t": max_t}
    while keep_reading_scale:
        changed_areas = []
        if (trigger_refresh_weight_graph == True and seconds_elapsed > 0) or (display_end_shot_line_now == True):
            if trigger_refresh_weight_graph == True:
                time_now = time.time() - start_shot_time
                last_update_weight_graph = time.time()
                new_x = int(round(a * (time_now - min_t) + x0))
                new_y_weight = clip(int(round(b * (filtered_weight - min_w) + y1)), y0, y1)
                new_y_flow = clip(int(round(-(y1 - y0)/(3.5/.6)*flow_smooth) + y1), y0, y1) # Range is 0-3.5 g/s = 60% of the range of the y axis
                # if new_x >= last_x + 1:
                # Restore background surfaces where things just moved.
                if x_pred != 0:
                    lcd.blit(background_dotpred, (x_pred-2, y_pred-2), ((0,0),(5, 5)))
                    changed_areas.append(((x_pred-2, y_pred-2), (5, 5)))
                if last_x < 294 - 42 and last_y_flow < y1 - 10 and last_y_flow > target_y + 10 and abs(last_y_flow - last_y_weight) > 14:
                    lcd.blit(background_flow, (last_x + 2, last_y_flow - 7), ((0,0), (45, 15)))
                    # lcd.fill(col_background, ((last_x + 2, last_y_flow - 7), (45, 15)))
                    changed_areas.append(((last_x + 2, last_y_flow - 7), (45, 15)))
                if last_x < 294 - 42 and last_y_weight < y1 - 10 and last_y_weight > target_y + 10:
                    lcd.blit(background_weight, (last_x + 2, last_y_weight - 7), ((0,0), (45, 15)))
                    # lcd.fill(col_background, ((last_x + 2, last_y_weight - 7), (45, 15)))
                    changed_areas.append(((last_x + 2, last_y_weight - 7), (45, 15)))
                ## REDRAW EVERYTHING IF WE HIT THE END OF THE X-AXIS ##
                if time_now >= max_t:
                    lcd.fill(col_background, rect = ((26, y0), (280-26 + 4, 220 - y0)))
                    last_max_t = max_t
                    max_t = max_t + 10
                    a = (x1 - x0)/(max_t - min_t)
                    for i in range(0,len(all_values["w_pointlist"])):
                        all_values["w_pointlist"][i][0] = int(round((all_values["w_pointlist"][i][0]-x0) / max_t * last_max_t + x0))
                        all_values["f_pointlist"][i][0] = int(round((all_values["f_pointlist"][i][0]-x0) / max_t * last_max_t + x0))
                    ### Redrawing flow rate curve
                    flowpoints = [[x0,y1]]
                    flowpoints.extend(all_values["f_pointlist"])
                    flowpoints.append([all_values["f_pointlist"][len(all_values["f_pointlist"]) - 1][0], y1])
                    pygame.draw.polygon(lcd, col_flow_polygon, flowpoints, 0)
                    pygame.draw.lines(lcd, col_flow_line, False, all_values["f_pointlist"])
                    ### Redrawing weight curve
                    pygame.draw.lines(lcd, col_weight_line, False, all_values["w_pointlist"])
                    ### Redrawing x-axis
                    pygame.draw.line(lcd, col_axis_lines, (x0, y1), (x1, y1))
                    vlines = range(0, int(max_t + 1), 10)
                    for i in range(0, len(vlines)):
                        x = int(a * (vlines[i] - min_t) + x0)
                        pygame.draw.line(lcd, col_axis_lines, (x, y1 + 3), (x, y1))
                        if vlines[i] != max_t:
                            string = str(vlines[i]) + "s"
                            width = pygame.font.Font(None, 20).render(string, True, (0,0,0)).get_width()
                            display_text(str(vlines[i]) + "s", (x - width/2, y1 + 5), 20, col_axis_labels)
                    ### Redrawing target weight horizontal line, with marks for desired end shot time:
                    pygame.draw.line(lcd, col_weight_target, (x0, target_y), (x1, target_y))
                    string_target_weight = str(target_weight) + "g"
                    width = pygame.font.Font(None, 20).render(string_target_weight, True, (0,0,0)).get_width()
                    display_text(string_target_weight, (int((x0 + x1)/2 - width/2), target_y - 19), 20, col_weight_target)
                    pygame.draw.line(lcd, col_weight_target, (int(round(a*(target_time - 1.5)) + x0), target_y - 3), (int(round(a*(target_time - 1.5)) + x0), target_y + 3))
                    pygame.draw.line(lcd, col_weight_target, (int(round(a*(target_time + 1.5)) + x0), target_y - 3), (int(round(a*(target_time + 1.5)) + x0), target_y + 3))
                    ### Redrawing previous shot
                    try:
                        if lastshot_values["w_pointlist"] != [] and lastshot_values["target_weight"] == target_weight:
                            # First: rescaling the values
                            lastshot_w = []
                            for i in range(0, len(lastshot_values["w_pointlist"])):
                                rescaled_x = int(round((lastshot_values["w_pointlist"][i][0] - x0)/max_t*lastshot_values["max_t"] + x0))
                                if rescaled_x <= x1:
                                    lastshot_w.append([rescaled_x, lastshot_values["w_pointlist"][i][1]])
                                else:
                                    break
                            # Then redrawing
                            pygame.draw.lines(lcd, col_lastshot, False, lastshot_w)
                            if lastshot_values["x_endshot"] != 0 and lastshot_values["max_t"] <= max_t:
                                for i in range(y0, y1, 5):
                                    rescaled_x_endshot = int(round((lastshot_values["x_endshot"] - x0)/max_t*lastshot_values["max_t"] + x0))
                                    lcd.set_at((int(round(rescaled_x_endshot)), i), col_lastshot)
                    except:
                        pass
                    pygame.display.update(((26, y0), (280-26+4, 220-y0)))
                    last_x = int(round((last_x-x0) / max_t * last_max_t + x0))
                    new_x = int(round(a * (time_now - min_t) + x0))
                # DISPLAY PREDICTED END TIME/WEIGHT POINTS ON GRAPH
                if predicted_end_time > 0 and flow_smooth > 0:
                    p_now = (time_now) + (target_weight - filtered_weight)/flow_smooth - 1.15
                    x_pred = min(int(round(a * (p_now - min_t) + x0)), x1)
                    if p_now <= max_t:
                        w_end = target_weight - 1.15*flow_smooth
                        width = 0 # If pred_end_time is on the graph; the dot is filled.
                    else:
                        w_end = filtered_weight + flow_smooth*(max_t - (time_now))
                        width = 1 # If pred_end_time is not on the graph; the dot is hollow
                    y_pred = int(round(b * (w_end - min_w) + y1))
                    background_dotpred = pygame.Surface((5,5))
                    background_dotpred.blit(lcd, (0,0), ((x_pred-2, y_pred-2), (5, 5)))  # Copy existing surface into new background surface
                    pygame.draw.circle(lcd, col_weight_line, (x_pred, y_pred), 2, width) # Draw a circle
                    pred_curve_area = ((x_pred-2, y_pred-2), (5, 5)) 
                    changed_areas.append(pred_curve_area)
                    # LIGHTS INDICATING SHOT SPEED
                    breakpoints_now =  [interpolate(seconds_elapsed, (10, target_time +  3.5), (0.80 * target_time, target_time - 4.5)), 
                                        interpolate(seconds_elapsed, (10, target_time +  8.5), (0.80 * target_time, target_time - 1.5)), 
                                        interpolate(seconds_elapsed, (10, target_time + 38.5), (0.80 * target_time, target_time + 1.5))]
                    scenario = 3 - cut(p_now, breakpoints_now)
                    color = color_scenarios[scenario]
                    circle = [0, 1, 2, 2][scenario]
                    for i in range(0, 3):
                        pygame.draw.circle(lcd, col_lights_outline, (x0 + 10 + i*10, 97), 4)
                    pygame.draw.circle(lcd, color, (x0 + 10 + (2-circle)*10, 97), 4)
                    changed_areas.append(((x0 + 6, 93), (28, 8)))
                # PLOT FLOW (and make sure it stays in the background)
                background_flowcurve = pygame.Surface((new_x-last_x+1, y1 - min(last_y_flow, new_y_flow) + 1))
                background_flowcurve.set_colorkey(col_background) # Any black pixel will be transparent
                background_flowcurve.blit(lcd, (0,0), ((min(last_x, new_x), min(last_y_flow, new_y_flow)), (abs(new_x-last_x) + 1, abs(y1 - min(last_y_flow, new_y_flow)) + 1)))
                pygame.draw.polygon(lcd, col_lightred, [(last_x, last_y_flow), (new_x, new_y_flow), (new_x, y1), (last_x, y1)], 0)
                pygame.draw.line(lcd, col_verylightred, (last_x, last_y_flow), (new_x, new_y_flow))
                lcd.blit(background_flowcurve, (min(last_x, new_x), min(last_y_flow, new_y_flow)))
                changed_areas.append(((min(last_x, new_x), min(last_y_flow, new_y_flow)), (abs(new_x-last_x) + 1, abs(y1 - min(last_y_flow, new_y_flow)) + 1)))
                # PLOT CURVE
                pygame.draw.line(lcd, col_weight_line, (last_x, last_y_weight), (new_x, new_y_weight))
                changed_areas.append(((min(last_x, new_x), min(last_y_weight, new_y_weight)), (abs(new_x-last_x) + 3, abs(new_y_weight-last_y_weight) + 3)))
                # DISPLAY FLOW TEXT
                if new_x < 294 - 42 and new_y_flow < y1 - 10 and new_y_flow > target_y + 10 and abs(new_y_flow - new_y_weight) > 14:
                    background_flow = pygame.Surface((45,15))
                    background_flow.blit(lcd, (0,0), ((new_x + 2, new_y_flow - 7), (45, 15)))
                    display_text(str(round(flow_smooth, 1)) + " g/s", (new_x + 2, new_y_flow - 7), 20, col_verylightred)
                    changed_areas.append(((new_x + 2, new_y_flow - 7), (45, 15)))
                # DISPLAY WEIGHT TEXT 
                if new_x < 294 - 42 and new_y_weight < y1 - 10 and new_y_weight > target_y + 10:
                    background_weight = pygame.Surface((45,15))
                    background_weight.blit(lcd, (0,0), ((new_x + 2, new_y_weight - 7), (45, 15)))
                    display_text(str(round(filtered_weight, 1)) + " g", (new_x + 2, new_y_weight - 7), 20, col_weight_text)
                    changed_areas.append(((new_x + 2, new_y_weight - 7), (45, 15)))
                # SAVE SHOT GRAPH HISTORY 
                all_values["w_pointlist"].append([new_x, new_y_weight])
                all_values["f_pointlist"].append([new_x, new_y_flow])
                last_x = new_x
                last_y_weight = new_y_weight
                last_y_flow = new_y_flow
                trigger_refresh_weight_graph = False
            # VERTICAL LINE AT THE END OF SHOT
            if display_end_shot_line_now == True:
                x_endshot = int(round(a*(end_shot_time - start_shot_time) + x0))
                if x_endshot <= x1:
                    try:
                        lcd.blit(background_dotpred, (x_pred-2, y_pred-2), ((0,0),(5, 5)))
                        changed_areas.append(((x_pred-2, y_pred-2), (5, 5)))
                    except:
                        pass
                    pygame.draw.line(lcd, col_weight_target, (x_endshot, y0), (x_endshot, y1), 1)
                    changed_areas.append(((x_endshot, y0), (1, y1 - y0)))
                    pygame.draw.circle(lcd, col_weight_line, (x_endshot, last_y_weight), 2)
                    changed_areas.append(((x_endshot-2, last_y_weight-2), (5, 5)))
                    all_values["x_endshot"] = x_endshot
                    all_values["target_weight"] = target_weight
                    all_values["max_t"] = max_t
                display_end_shot_line_now = False
            for rect in changed_areas:
                pygame.display.update(rect)
        time.sleep(0.01)
    lcd.fill(col_background, rect = ((0,65), (295, 220-65)))
    pygame.display.update(((0,65), (295, 220-65)))


# Add to display_weight_graph:
## - previous shot weight (in darkgrey)
###         * grab data from last log_shotXXX.json file saved and redraw it???
## - rescaling every 10 seconds: 
###         * save weight/flow sequence
###         * adding 10 seconds max_t if time.time()-start_shot_time > max_t
###         * redrawing everything (flow/weight/x axis)
## - show lines of predicted_end_time? (maybe the red/orange/yellow lines are sufficient)

j = 0

def draw_belowgraph():
    global j
    if keep_reading_scale == False:
        string1 = "T: " + str(set_temp) + u"\u2103" + "  -  H: " 
        display_text(string1, area_belowgraph[0], 25, col_text)
        string2 = str(int(power)) + "%  -"
        width = pygame.font.Font(None, 25).render(string2, True, col_text).get_width()
        display_text(string2, (100+59-width, area_belowgraph[0][1]), 25, col_text)
        if steam_on == True:
            string3 = "  -  Steam"
        if steam_on == False:
            # Update the rest of the display only every 2 refreshes
            if (j % 6 < 2): # 0-1
                if time.time() - last_input_time < 2700: # If time.time() - last_input_time >= 2700, the system should either shutdown OR should be detecting that NTPD has reset time (in which case "On time" will be off for less than 5 seconds -- until thread_auto_dim_or_shutdown() reajusts start_script_time and last_input_time)
                    minutes = int((time.time() - start_script_time)/60)
                    string3 = "On: %s min." %(minutes)
                    if (time.time() - start_script_time)/60 < warmup_minutes:
                        j += 5
            elif (j % 6 < 4): # 2-3
                string3 = "Target: %s g." %(target_weight)
            else:
                string3 = "Flow: " + flow_mode
            j += 1
        display_text(string3, (165, area_belowgraph[0][1]), 25, col_text)
    else:
        string = "P: " + str(int(pump_power)*10) + "%"
        display_text(string, (0, area_belowgraph[0][1]), 25, col_text)

def backflush():
    global backflush_on
    backflush_on = True
    i = 1
    while (backflush_on == True) and (i <= 5):
        lcd.fill(col_background, area_menu)
        display_text("Backflush...", (160, 100), 25, col_menu_text)
        display_text("%s/5" % i, (160, 125), 25, col_menu_text)
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
    global tare_weight, prev_weight, raw_weight_values, filter_on, filtered_weight, heat_gain, start_preheat_time
    start_preheat_time = time.time()
    heat_gain = shot_heat_gain(target_weight, target_time, time_preheat, time_shot_heat, alpha, beta_0, beta_1)
    lcd.fill(col_background, rect = area_text_temp)
    text = "Preheat"
    for i in range(1, 4):
        display_text(text + "."*i, (175, 20), 30, col_text)
        pygame.display.update(area_timer)
        time.sleep(time_preheat/3)
    # Important to get a clean tare weight to start with.
    # make zero_weight a mean weight of the last 15 readings, with top and bottom 2 readings thrown away.
    trimmed_raw_weight_values = sorted(raw_weight_values)[2:14]
    zero_weight = mean(trimmed_raw_weight_values) - tare_weight
    tare_weight += zero_weight
    prev_weight = 0.0
    filter_initialize()
    filter_on = True
    filtered_weight = 0.0



def process_weight_reading():
    # Conversion in grams every 10th of a second
    global trigger_process_weight_reading, trigger_refresh_weight_graph
    while keep_reading_scale == True:
        if trigger_process_weight_reading == True:
            smooth_weight()
            weight_series_time.append(weighing_time - start_script_time)
            weight_series.append(current_weight)
            # print "%s: current_weight = %s" %(weighing_time, current_weight)
            if filter_on == True:
                filtered_time.append(weighing_time - start_script_time)
                filtered_weight_series.append(filtered_weight)
                filtered_flow_series.append(flow_smooth)
                adc_values.append(adc_diff10)
                raw_weights.append(raw_weight_values[-1])
                # print "%s: filtered_weight = %s" %(weighing_time, filtered_weight)
            trigger_refresh_weight_graph = True
            lcd.fill(col_background, rect = area_text_temp)
            display_text('{0:.1f}'.format(displayed_weight) + " g.", (5, 5), 60, col_text)
            pygame.display.update(area_text_temp)
            trigger_process_weight_reading = False
        elif trigger_process_weight_reading == False:
            # print "Waiting for next trigger_process_weight_reading"
            time.sleep(.01)
    if keep_reading_scale == False:
        # print "voltage_to_weight thread exited"
        return

weighing_time = time.time()-1
t1 = time.time()
times_weight_overridden = 0

def smooth_weight():
    global current_weight, prev_weight, tare_weight, displayed_weight, weighing_time, previous_time, filtered_weight, times_weight_overridden
    # # In rare very cases where the scale issues an unexpectedly high or low reading during a shot, it is better to ignore it than to throw off the automatic flow adjustment. This can only be done 4 times in a row (weight reading ignored for .5 second)
    # if shot_pouring == True and abs(raw_weight_values[-1] - raw_weight_values[-2]) > 10 and times_weight_overridden <= 4:
    #     raw_weight_values[-1] = raw_weight_values[-2]
    #     times_weight_overridden += 1
    # else:
    #     times_weight_overridden = 0
    current_weight = mean([raw_weight_values[-1], raw_weight_values[-2], raw_weight_values[-3]]) - tare_weight
    previous_time = weighing_time
    weighing_time = t1
    # print "%s: converting V into g: last raw_weight_values" %t1
    if filter_on == True:
        # print "filtering: previous_time %s; weighing_time %s" %(previous_time, weighing_time)
        filter(current_weight, prev_weight, weighing_time, previous_time, Q0, R0)
        displayed_weight = filtered_weight if filtered_weight >= 0 else 0.001
    elif filter_on == False:
        filtered_weight = current_weight
        displayed_weight = current_weight
    prev_weight = current_weight


def filter_initialize():
    global Q0, R0, x, P
    Q0 =np.array([[0.04, 0.00],
                  [0.00, 0.08]])
    R0 = np.array([[2.0, -1.0],
                  [-1.0, 2.0]])
    x = np.array([0.0, 0.0])
    P = np.array([[0.0, 0.0],
                  [0.0, 0.0]])

# def filter_initialize():
#     global Q0, R0, x, P
#     Q0 =np.array([[10.0, 0.00],
#                   [0.00, 10.0]])
#     R0 = np.array([[2.0, -1.0],
#                   [-1.0, 2.0]])
#     x = np.array([0.0, 0.0])
#     P = np.array([[0.0, 0.0],
#                   [0.0, 0.0]])
#

flow_list6 = deque([0], 6)
flow_smooth = 0

def filter(current_weight, prev_weight, time1, time0, Q0, R):
    global x, P, filtered_weight, filtered_flow, flow_list5, flow_smooth
    try:
        dt = time1 - time0
        z = [current_weight, (current_weight-prev_weight)/dt] 
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        if (seconds_elapsed) < 7 and current_weight < 1 and prev_weight < 1: 
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

def save_lastshot():
    json.dump({"all_values" : all_values}, open("/home/pi/lastshot.json", "w"))

def shutdown():
    # Save data in json file (y, heat_series,)
    # Save settings (temperature, preinfusion, etc.) in another json file to be loaded later on.
    # save_temperature_logs()
    GPIO.cleanup()
    save_settings()
    save_lastshot()
    pygame.quit()
    display_brightness(10)
    try:
        stream.close()
    except:
        pass
    # os._exit(1)
    # sys.exit()
    os.system("sudo shutdown now")
    

##################################################################
# BUTTON FUNCTIONS AND THREADS WAITING FOR BUTTONS TO BE PRESSED #
##################################################################

class Menu(object):
    # Can be an Item [an Action, NumericParam, CategoricalParam]
    #        or a Menu [a list of children: Items and other Menus]
    def __init__(self, title, children = [], icons = (icon_back, icon_up, icon_down, icon_check), side_graph_function = None):
        self.title = title
        self.children = children
        self.side_graph_function = side_graph_function
    
    def display_setup(self):
        global head, tail, items_in_current_menu
        items_in_current_menu = []
        for child_title in self.children:
            items_in_current_menu.append(child_title)
        if head == 0 and tail == 0:
            head, tail = 0, min(len(items_in_current_menu), (max_n_items - 1))
        if selection > tail:
            while selection > tail:
                head += 1
                tail += 1
        if selection < head:
            while selection < head:
                head -= 1
                tail -= 1
        
    def display(self):
        self.display_setup()
        max_height = 240 - 60 - 25
        text_height = 24 * (tail - head + 1) 
        margin_height = (max_height - text_height)/2
        lcd.fill(col_background, area_menu)
        row = 0
        for item in items_in_current_menu[head:(tail+1)]:
            # if item == items_in_current_menu[selection]:
            #     display_text(">", (150, 60 + margin_height + 25*row), 17, col_menu_text, condensed = True)
            # display_text(item, (160, 60 + margin_height + 25*row), 17, col_menu_text, condensed = True)
            if item == items_in_current_menu[selection]:
                ctext = col_menu_text_select
                ctext_bg = col_menu_text_select_background
                lcd.fill(ctext_bg, ((155, 60 + margin_height + 24*row - 1), (125, 22)))
            else:
                ctext = col_menu_text
                ctext_bg = col_background
            display_text(item, (160, 60 + margin_height + 24*row), 17, ctext, condensed = True)
            if isinstance(all_items[item], Param) == False and isinstance(all_items[item], Action) == False:
                lcd.fill(ctext_bg, ((269, 60 + margin_height + 24*row - 1), (11, 22)))
                display_text(u"\u2022", (269, 60 + margin_height + 24*row), 17, ctext, condensed = True)
            row += 1
        pygame.display.update(area_menu)
        if self.side_graph_function != None:
            self.side_graph_function()
            return
        if all_items[items_in_current_menu[selection]].side_graph_function != None:
            all_items[items_in_current_menu[selection]].side_graph_function()
            return
    
    def press_up(self):
        global selection
        if selection > 0:
            selection -= 1
        else:
            selection = len(items_in_current_menu) - 1
        
    def press_down(self):
        global selection
        if selection < len(items_in_current_menu) - 1:
            selection += 1
        else:
            selection = 0
    
    def select(self):
        global current_menu, selection, head, tail
        menu_stack.append({"current_menu": current_menu, "selection": selection, "head": head, "tail": tail, "items_in_current_menu": items_in_current_menu})
        current_menu = items_in_current_menu[selection]
        if isinstance(all_items[current_menu], Param) == False: #Only if what is selected is not a parameter:
            selection, head, tail = 0, 0, 0


class Action(Menu):
    
    def __init__(self, title, function, end = None):
        self.title = title
        self.function = function
        self.end = end
        self.children = []
        self.side_graph_function = None
    
    def select(self):
        global current_menu
        menu_stack.append({"current_menu": current_menu, "selection": selection, "head": head, "tail": tail, "items_in_current_menu": items_in_current_menu})
        current_menu = items_in_current_menu[selection]
        self.function()
    
    def display(self):
        # Maybe some stuff to display while the action is running (lcd.fill(col_background))???
        pass

class Param(Menu):
    
    def __init__(self, title, varname, long_title):
        self.title = title
        self.varname = varname
        self.long_title = long_title
        self.children = []
    
    def display_long_title(self):
        print self.long_title


class NumericParam(Param):
    
    def __init__(self, title, varname, limits, increment, decimals, unit, value_scale = 1.0, long_title = None, side_graph_function = None):
        self.title = title
        self.varname = varname
        self.long_title = long_title
        self.limits = limits
        self.increment = increment
        self.decimals = decimals
        self.unit = unit
        self.value_scale = value_scale
        if long_title == None:
            self.long_title = self.title
        else:
            self.long_title = long_title
        self.children = []
        self.side_graph_function = side_graph_function
    
    def enforce_limits(self):
        if type(self.limits[0]) == str:
            lower_lim = globals()[self.limits[0]]
        else:
            lower_lim = self.limits[0]
        if type(self.limits[1]) == str:
            upper_lim = globals()[self.limits[1]]
        else:
            upper_lim = self.limits[1]
        globals()[self.varname] = clip(globals()[self.varname], lower_lim, upper_lim)
        
    def display(self):
        self.enforce_limits()
        value_string = str("%." + str(self.decimals) + "f%s") %(globals()[self.varname] * self.value_scale, self.unit)
        lcd.fill(col_background, area_menu)
        display_text(self.long_title, (170, 100), 17, col_menu_text, condensed = True)
        display_text(value_string, (170, 135), 17, col_menu_value, condensed = True)
        pygame.display.update(area_menu)
        if self.side_graph_function != None:
            self.side_graph_function()
        
    def press_up(self):
        globals()[self.varname] += self.increment
        self.enforce_limits()
     
    def press_down(self):
        globals()[self.varname] -= self.increment
        self.enforce_limits()
    

class CategoricalParam(Param):
    
    def __init__(self, title, varname, categorical_values, categorical_labels = None, long_title = None, side_graph_function = None):
        self.title = title
        self.varname = varname
        self.long_title = long_title
        self.categorical_values = categorical_values
        if long_title == None:
            self.long_title = self.title
        else:
            self.long_title = long_title
        if categorical_labels == None:
            self.categorical_labels = categorical_values
        else:
            self.categorical_labels = categorical_labels
        self.children = []
        self.side_graph_function = side_graph_function
     
    def display(self):
        value_string = str(globals()[self.varname])
        lcd.fill(col_background, area_menu)
        display_text(self.long_title, (170, 100), 17, col_menu_text, condensed = True)
        display_text(value_string, (170, 135), 17, col_menu_value, condensed = True)
        pygame.display.update(area_menu)
        if self.side_graph_function != None:
            self.side_graph_function()
    
    def press_up(self):
        cycling_all_values = itertools.cycle(self.categorical_values)
        for i in self.categorical_values:
            if globals()[self.varname] == cycling_all_values.next():
                globals()[self.varname] = cycling_all_values.next()
                break
        
    
    def press_down(self):
        cycling_all_values = itertools.cycle(reversed(self.categorical_values))
        for i in self.categorical_values:
            if globals()[self.varname] == cycling_all_values.next():
                globals()[self.varname] = cycling_all_values.next()
                break

def create_menu_map(items_list):
    items_dict = {}
    for item in items_list:
        items_dict[item.title] = item
    for item in items_list:
        for child in item.children:
            if child not in items_dict.keys():
                print "Warning: no MenuItem listed for child %s of parent %s" %(child, item.title)
    return items_dict
    



def flush():
    global flush_on, keep_reading_pot, pump_power, trigger_refresh_display_pump
    flush_on = True
    keep_reading_pot = True
    trigger_refresh_display_pump = True
    thread.start_new_thread(read_adc, ())
    thread.start_new_thread(display_pump_power, ())
    thread.start_new_thread(output_pump, ())
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

def run_shutdown():
    print "Now shutting down"

def run_cancel():
    global current_menu, selection, head, tail, items_in_current_menu, menu_stack
    menu_stack.pop()
    prev_stack = menu_stack.pop()
    (current_menu, selection, head, tail, items_in_current_menu) = (prev_stack["current_menu"], prev_stack["selection"], prev_stack["head"], prev_stack["tail"], prev_stack["items_in_current_menu"])
    all_items[current_menu].display()



def run_test_scale():
    global keep_reading_scale, filter_on, flush_on
    keep_reading_scale = True
    filter_on = False
    flush_on = True
    thread.start_new_thread(flush, ())
    thread.start_new_thread(read_adc, ())
    thread.start_new_thread(process_weight_reading, ())

def end_test_scale():
    global keep_reading_scale, flush_on, pump_power
    keep_reading_scale = False
    flush_on = False

def run_show_last_shot():
    global display_lastshot
    try:
        display_lastshot = True
        lcd.fill(col_background, ((0, 65), (280, 175)))
        lastshot_surface = pygame.image.load("/home/pi/lastshot.png")
        lcd.blit(lastshot_surface, (27, 70))
        pygame.display.update(((0, 65), (280, 175)))
    except:
        display_lastshot = False
        refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)

def end_show_last_shot():
    global display_lastshot, current_menu, selection, head, tail, items_in_current_menu
    display_lastshot = False
    refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)    
    
def pass_function():
    pass

def clean_exit():
    print "Exiting after cleaning up GPIO"
    GPIO.output(heat_pin, 0)
    GPIO.output(pump_pin, 0)
    GPIO.cleanup()
    pygame.quit()
    display_brightness(10)
    try:
        stream.close()
    except:
        pass
    sys.exit()

def preset_profile():
    # Just a few hard-coded preset profiles
    global pp0, pp1, pp2, pp3, pp4, time_t1, time_t2, time_t3, time_t4, profile_01, profile_12, profile_23, profile_34, time_auto_flow, target_time, current_menu, selection, head, tail, items_in_current_menu, prev_stack
    if items_in_current_menu[selection] == "Half Power":
        pp0, pp1, pp2, pp3, pp4                        = 5, 5, 5, 5, 5
        time_t1, time_t2, time_t3, time_t4             =    0, 0, 0, 0
        profile_01, profile_12, profile_23, profile_34 =  "Flat", "Flat", "Flat", "Flat"
        time_auto_flow = 12 
        target_time = 31
        time_preheat = 3.0
        time_shot_heat = 23.0
        alpha = -25.0
    elif items_in_current_menu[selection] == "Full Power":
        pp0, pp1, pp2, pp3, pp4                        = 10, 10, 10, 10, 10
        time_t1, time_t2, time_t3, time_t4             =    0, 0, 0, 0
        profile_01, profile_12, profile_23, profile_34 =  "Flat", "Flat", "Flat", "Flat"
        time_auto_flow = 12
        target_time = 31
        time_preheat = 4.0
        time_shot_heat = 23.0
        alpha = -25.0
    elif items_in_current_menu[selection] == "Smooth Rise":
        pp0, pp1, pp2, pp3, pp4                        = 1, 7, 7, 7, 7
        time_t1, time_t2, time_t3, time_t4             =    8, 8, 8, 8
        profile_01, profile_12, profile_23, profile_34 =  "Gradual", "Flat", "Flat", "Flat"
        time_auto_flow = 12 
        target_time = 31
        time_preheat = 2.0
        time_shot_heat = 23.0
        alpha = -25.0
    elif items_in_current_menu[selection] == "Smooth & Slow":
        pp0, pp1, pp2, pp3, pp4                        = 1, 1, 7, 7, 7
        time_t1, time_t2, time_t3, time_t4             =    4, 8, 8, 8
        profile_01, profile_12, profile_23, profile_34 =  "Flat", "Gradual", "Flat", "Flat"
        time_auto_flow = 12 
        target_time = 31
        time_preheat = 2.0
        time_shot_heat = 23.0
        alpha = -25.0
    elif items_in_current_menu[selection] == "Preinfusion":
        pp0, pp1, pp2, pp3, pp4                        = 5, 0, 1, 7, 7
        time_t1, time_t2, time_t3, time_t4             =    3, 6, 8, 8
        profile_01, profile_12, profile_23, profile_34 =  "Flat", "Flat", "Gradual", "Flat"
        time_auto_flow = 12
        target_time = 31
        time_preheat = 3.0
        time_shot_heat = 23.0
        alpha = 0.0
    elif items_in_current_menu[selection] == "Long Preinfusion":
        pp0, pp1, pp2, pp3, pp4                        = 1, 0,  1,  7,  7
        time_t1, time_t2, time_t3, time_t4             =    8, 12, 15, 15
        profile_01, profile_12, profile_23, profile_34 =  "Flat", "Flat", "Gradual", "Flat"
        time_auto_flow = 18
        target_time = 40
        time_preheat = 2.0
        time_shot_heat = 33.0
        alpha = 25.0
    elif items_in_current_menu[selection] == "Rise & Drop":
        pp0, pp1, pp2, pp3, pp4                        = 1, 5, 10, 10, 1
        time_t1, time_t2, time_t3, time_t4             =    5, 13, 18, 28
        profile_01, profile_12, profile_23, profile_34 =  "Gradual", "Gradual", "Gradual", "Gradual"
        time_auto_flow = 30
        target_time = 31
        time_preheat = 3.0
        time_shot_heat = 23.0
        alpha = -25.0
    display_profile_graph()
    prev_stack = menu_stack.pop()
    (current_menu, selection, head, tail, items_in_current_menu) = (prev_stack["current_menu"], prev_stack["selection"], prev_stack["head"], prev_stack["tail"], prev_stack["items_in_current_menu"])

all_items = create_menu_map(
                    [Menu(title = "Main", children = ["Last Shot", "Set Temperature", "Target Weight", "Target Time", "Shot Profile", "Flow Control", "Heat Control", "Other Stuff", "Exit"]),
                     # Action(title = "Flush", function = run_flush, end = end_flush),
                     Action(title = "Last Shot", function = run_show_last_shot, end = end_show_last_shot),
                     NumericParam(title = "Set Temperature", varname = "set_temp", limits = [80, 95], unit = u"\u00B0" + "C", increment = 1, decimals = 0),
                     NumericParam(title = "Target Weight", varname = "target_weight", limits = [20, 50], unit = " g.", increment = 1, decimals = 0),
                     NumericParam(title = "Target Time", varname = "target_time", limits = [20, 60], unit = " s.", increment = 1, decimals = 0),
                     Menu(title = "Shot Profile", children = ["Preset Profiles", "Pump Profile", "Heat Profile"], side_graph_function = display_profile_graph),
                            Menu(title = "Preset Profiles", children = ["Half Power", "Full Power", "Smooth Rise", "Smooth & Slow", "Preinfusion", "Long Preinfusion", "Rise & Drop"], side_graph_function = display_profile_graph),
                                Action(title = "Half Power"      , function = preset_profile),
                                Action(title = "Full Power"      , function = preset_profile),
                                Action(title = "Smooth Rise"     , function = preset_profile),
                                Action(title = "Smooth & Slow"   , function = preset_profile),
                                Action(title = "Preinfusion"     , function = preset_profile),
                                Action(title = "Long Preinfusion", function = preset_profile),
                                Action(title = "Rise & Drop", function = preset_profile),
                            Menu(title = "Pump Profile", children = ["Pump Power P0", "Pump Power P1", "Pump Power P2", "Pump Power P3", "Pump Power P4", "Time T1", "Time T2", "Time T3", "Time T4", "Time Auto Flow", "Profile 0-T1", "Profile T1-T2", "Profile T2-T3", "Profile T3-T4"], side_graph_function = display_profile_graph),
                                NumericParam(title = "Pump Power P0", varname = "pp0", limits = ["min_pp", 10], unit = "%", increment = 1, decimals = 0, value_scale = 10, side_graph_function = display_profile_graph),
                                NumericParam(title = "Pump Power P1", varname = "pp1", limits = [0, 10], unit = "%", increment = 1, decimals = 0, value_scale = 10, side_graph_function = display_profile_graph),
                                NumericParam(title = "Pump Power P2", varname = "pp2", limits = [0, 10], unit = "%", increment = 1, decimals = 0, value_scale = 10, side_graph_function = display_profile_graph),
                                NumericParam(title = "Pump Power P3", varname = "pp3", limits = [0, 10], unit = "%", increment = 1, decimals = 0, value_scale = 10, side_graph_function = display_profile_graph),
                                NumericParam(title = "Pump Power P4", varname = "pp4", limits = ["min_pp", 10], unit = "%", increment = 1, decimals = 0, value_scale = 10, side_graph_function = display_profile_graph),
                                NumericParam(title = "Time T1", varname = "time_t1", limits = [0, "time_t2"], unit = " s.", increment = 1, decimals = 0, side_graph_function = display_profile_graph),
                                NumericParam(title = "Time T2", varname = "time_t2", limits = ["time_t1", "time_t3"], unit = " s.", increment = 1, decimals = 0, side_graph_function = display_profile_graph),
                                NumericParam(title = "Time T3", varname = "time_t3", limits = ["time_t2", "time_t4"], unit = " s.", increment = 1, decimals = 0, side_graph_function = display_profile_graph),
                                NumericParam(title = "Time T4", varname = "time_t4", limits = ["time_t3", 60], unit = " s.", increment = 1, decimals = 0, side_graph_function = display_profile_graph),
                                CategoricalParam(title = "Profile 0-T1", varname = "profile_01", categorical_values = ["Gradual", "Flat"], side_graph_function = display_profile_graph),
                                CategoricalParam(title = "Profile T1-T2", varname = "profile_12", categorical_values = ["Gradual", "Flat"], side_graph_function = display_profile_graph),
                                CategoricalParam(title = "Profile T2-T3", varname = "profile_23", categorical_values = ["Gradual", "Flat"], side_graph_function = display_profile_graph),
                                CategoricalParam(title = "Profile T3-T4", varname = "profile_34", categorical_values = ["Gradual", "Flat"], side_graph_function = display_profile_graph),
                            Menu(title = "Heat Profile", children = ["Heat Level", "Heat Change", "Preheat Time", "Heat Time"], side_graph_function = display_intrashot_graph),
                      Menu(title = "Flow Control", children = ["Flow Mode", "Time Auto Flow"]),
                            CategoricalParam(title = "Flow Mode", varname = "flow_mode", categorical_values = ["Auto", "Manual", "Program"]),
                            NumericParam(title = "Time Auto Flow", varname = "time_auto_flow", limits = [8, "target_time"], unit = " s.", increment = 1, decimals = 0, side_graph_function = display_profile_graph),
                      Menu(title = "Heat Control", children = ["PID", "Intrashot", "Warm-Up"]),
                            Menu(title = "PID", children = ["kP", "kI", "kD", "k0"]),
                                NumericParam(title = "kP", varname = "kP", limits = [0, .20], unit = "", increment = .005, decimals = 3),
                                NumericParam(title = "kI", varname = "kI", limits = [0, .30], unit = "", increment = .01, decimals = 2),
                                NumericParam(title = "kD", varname = "kD", limits = [0,  4], unit = "", increment = .10, decimals = 2),
                                NumericParam(title = "k0", varname = "k0", limits = [0, .06], unit = "", increment = .005, decimals = 3),
                            Menu(title = "Intrashot", children = ["Heat Level", "Heat Profile", "Preheat Time", "Heat Time"], side_graph_function = display_intrashot_graph),
                                NumericParam(title = "Heat Level", varname = "beta_0", limits = [0, 50], unit = "%", increment = 1, decimals = 0, side_graph_function = display_intrashot_graph),
                                NumericParam(title = "Heat Change", varname = "alpha", limits = [-100, 100], unit = "%", increment = 5, decimals = 0, side_graph_function = display_intrashot_graph),
                                NumericParam(title = "Preheat Time", varname = "time_preheat", limits = [2, 5], unit = " s.", increment = 1, decimals = 0, side_graph_function = display_intrashot_graph),
                                NumericParam(title = "Heat Time", varname = "time_shot_heat", limits = [10, "target_time"], unit = " s.", increment = 1, decimals = 0, side_graph_function = display_intrashot_graph),
                            Menu(title = "Warm-Up", children = ["Warm-Up Temperature", "Warm-Up Time"]),
                                NumericParam(title = "Warm-Up Temperature", varname = "warmup_temp", limits = [set_temp, 110], unit = u"\u00B0" + "C", increment = 1, decimals = 0),
                                NumericParam(title = "Warm-Up Time", varname = "warmup_minutes", limits = [0, 5], unit = " min.", increment = 0.5, decimals = 1),
                       Menu(title = "Other Stuff", children = ["Test Scale", "Save Settings", "Cancel Changes", "Restore Defaults"]),
                               Action(title = "Test Scale", function = run_test_scale, end = end_test_scale),
                               Action(title = "Restore Defaults", function = reset_settings, end = pass_function),
                               Action(title = "Cancel Changes", function = load_settings, end = pass_function),
                               Action(title = "Save Settings", function = save_settings, end = pass_function),
                       Menu(title = "Exit", children = ["Shut Down", "Quit Python", "Cancel"]),
                                Action(title = "Shut Down", function = shutdown),
                                Action(title = "Quit Python", function = clean_exit),
                                Action(title = "Cancel", function = run_cancel)
                    ])


display_lastshot = False


max_n_items = 6
items_in_current_menu = [None]
selection = 0

def button1(channel):
    time_button_press = time.time()
    time.sleep(.01)
    if GPIO.input(button1_pin) != GPIO.LOW: # Button appears to have been released after 0.01 sec. This can't be an actual button press.
        print "Probably a false positive button1 press"
        return
    print "Button 1 pressed"
    global last_input_time, menu_stack, menu, current_menu, selection, head, tail, items_in_current_menu, menu_stack
    last_input_time = time.time()
    display_brightness(max_brightness)
    highlight_pressed_icon(1)
    if menu == 0:
        # Entering menu
        menu = 1
        reset_graph_area()
        refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)
        current_menu = "Main"
        selection = 0
        head = 0
        tail = min(len(all_items[current_menu].children), (max_n_items - 1))
        menu_stack = [] # (current_menu, selection, head, tail)
        all_items[current_menu].display()
    else:
        if isinstance(all_items[current_menu], Action) == True:
            all_items[items_in_current_menu[selection]].end() 
            prev_stack = menu_stack.pop()
            (current_menu, selection, head, tail, items_in_current_menu) = (prev_stack["current_menu"], prev_stack["selection"], prev_stack["head"], prev_stack["tail"], prev_stack["items_in_current_menu"])
            all_items[current_menu].display()
        elif menu_stack == []:
            menu = 0
            items_in_current_menu = [None]
            selection = 0
            reset_graph_area()
        else:
            prev_stack = menu_stack.pop()
            (current_menu, selection, head, tail, items_in_current_menu) = (prev_stack["current_menu"], prev_stack["selection"], prev_stack["head"], prev_stack["tail"], prev_stack["items_in_current_menu"])
            all_items[current_menu].display()
    refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)
    refresh_buttons()


    
def button2(channel):
    time.sleep(.01)
    if GPIO.input(button2_pin) != GPIO.LOW:
        print "Probably a false positive button2 press"
        return
    print "Button 2 pressed"
    global flush_on, last_input_time, trigger_update_log_shot, current_menu, selection, head, tail, items_in_current_menu, menu_stack, all_items
    last_input_time = time.time()
    display_brightness(max_brightness)
    highlight_pressed_icon(2)
    if menu == 0:
        if flush_on == False:
            flush_on = True
            thread.start_new_thread(flush, ())
        else:
            flush_on = False
        reset_graph_area()
    if menu == 1:
        all_items[current_menu].press_up()
        all_items[current_menu].display()
    refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)
    refresh_buttons()

def button3(channel):
    time.sleep(.01)
    if GPIO.input(button3_pin) != GPIO.LOW:
        print "Probably a false positive button3 press"
        return
    print "Button 3 pressed"
    global last_input_time, trigger_update_log_shot, current_menu, selection, head, tail, items_in_current_menu, menu_stack, all_items
    last_input_time = time.time()
    display_brightness(max_brightness)
    highlight_pressed_icon(3)
    if menu == 0:
        # Will need to install RTD temperature sensor to get steam.
        if steam_on == False:
            #run_steam() # Function that makes the pump pulse a tiny bit of water into the boiler every 10 or 15 seconds once steam_temp is reached. 
            #            # But that would require installing a relay controlling the pump directly (and bypassing the 3 way-valve and the brew switch).
            # or just set global var: 
            # steam_on = True
            pass
        else:
            #end_steam()
            # or just reset global var:
            # steam_on = False
            pass
        reset_graph_area()
    if menu == 1:
        all_items[current_menu].press_down()
        all_items[current_menu].display()
    refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)
    refresh_buttons()

def button4(channel):
    time.sleep(.01)
    if GPIO.input(button4_pin) != GPIO.LOW:
        print "Probably a false positive button4 press"
        return
    print "Button 4 pressed"
    global shot_pouring, pump_power, keep_reading_scale, menu, last_input_time, current_menu, selection, head, tail, items_in_current_menu, menu_stack
    last_input_time = time.time()
    display_brightness(max_brightness)
    highlight_pressed_icon(4)
    if menu == 0 and flush_on == False and steam_on == False: # No menu displayed
        if shot_pouring == True:
            end_shot()
        elif shot_pouring == False and wait == False:
            shot_pouring = True
            pump_power = 0
            keep_reading_scale = True
            thread.start_new_thread(read_adc, ())
            thread.start_new_thread(process_weight_reading, ())
            thread.start_new_thread(tare_and_preheat, ())
            thread.start_new_thread(time_shot, ())
            thread.start_new_thread(pour_shot, ())
            thread.start_new_thread(display_weight_graph, ())
            thread.start_new_thread(display_pump_power, ())
    elif menu == 1: # Settings menu
        if menu_stack != []:
            prev_stack = menu_stack[-1]
            items_in_prev_menu = prev_stack["items_in_current_menu"]
            prev_selection = prev_stack["selection"]
        if isinstance(all_items[current_menu], Param) == False and isinstance(all_items[current_menu], Action) == False:  
            # If current_menu is neither a Param nor an Action: select it
            all_items[items_in_current_menu[selection]].select() 
            print "Selected %s" %items_in_current_menu[selection]
        elif isinstance(all_items[current_menu], Action) == True:
            # If current_menu is already an Action: end the action
            all_items[items_in_current_menu[selection]].end()
            reset_graph_area()
            prev_stack = menu_stack.pop()
            (current_menu, selection, head, tail, items_in_current_menu) = (prev_stack["current_menu"], prev_stack["selection"], prev_stack["head"], prev_stack["tail"], prev_stack["items_in_current_menu"])
            refresh_buttons()
        elif (isinstance(all_items[current_menu], Param) == True
                and prev_selection < len(items_in_prev_menu) - 1
                and isinstance(all_items[items_in_prev_menu[prev_selection + 1]], Param) == True): 
            # Looks a bit complicated but:
            # If current_menu is already a Param, then make the button fast forward to the next menu_item if it's also a a Param
            prev_stack = menu_stack.pop()
            (current_menu, selection, head, tail, items_in_current_menu) = (prev_stack["current_menu"], prev_stack["selection"], prev_stack["head"], prev_stack["tail"], prev_stack["items_in_current_menu"])
            selection += 1
            all_items[items_in_current_menu[selection]].select()
        refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)
        all_items[current_menu].display()
    refresh_buttons()


###########
# PLOTLY  #
###########

if len(sys.argv) > 1 and sys.argv[1] == "test":
    pass
else:
    try:
        with open('/home/pi/plotly_config.json') as config_file:
            plotly_user_config = json.load(config_file)
            py.sign_in(plotly_user_config["plotly_username"], plotly_user_config["plotly_api_key"])
    
        url = py.plot([
            {
                'x': [], 'y': [], 'type': 'scatter',
                'stream': {
                    'token': plotly_user_config['plotly_streaming_tokens'][0],
                    'maxpoints': 130
                }
            }], filename='Gaggia Boiler Temperature')
    
        stream = py.Stream(plotly_user_config['plotly_streaming_tokens'][0])
        stream.open()
    except:
        pass

##################################
# THREADS REFRESHING THE DISPLAY #
##################################

def thread_read_temp():
    global trigger_heat, trigger_refresh_graph
    while True:
        read_temp()
        adjust_heating_power()
        try:
            stream.write({'x': datetime.datetime.now(), 'y': y[-1]})
        except:
            pass
        trigger_heat = True
        trigger_refresh_graph = True

def thread_heat():
    global trigger_heat
    while True:
        if trigger_heat:
            trigger_heat = False
            output_heat()
        else:
            time.sleep(.01)


def thread_refresh_graph():
    global trigger_refresh_graph
    while True:
        if trigger_refresh_graph:
            refresh_graph(y, graph_top, graph_bottom, area_graph, area_text_temp)
            trigger_refresh_graph = False
        time.sleep(.02)

# def thread_refresh_buttons():
#     while True:
#         refresh_buttons(menu, shot_pouring, steam_on, backflush_on, flush_on)
#         time.sleep(.02)

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
        if time.time() - last_input_time >= 300 and brightness == max_brightness:
            display_brightness(10)
        if time.time() - last_input_time >= 2700:
            shutdown()
        old_time = time.time()
        time.sleep(5)

uptime_s = uptime.uptime()
print "Uptime in seconds %s" %(uptime_s)

old_time = time.time()
start_script_time = time.time()
last_input_time = start_script_time
print "Start script time: " + time.strftime('%Y-%m-%d-%H:%M:%S')

thread.start_new_thread(thread_auto_dim_or_shutdown, ())
thread.start_new_thread(thread_read_temp, ())
thread.start_new_thread(thread_heat, ())
# thread.start_new_thread(thread_refresh_timer_display, ())
thread.start_new_thread(thread_refresh_graph, ())

refresh_buttons()
# thread.start_new_thread(thread_refresh_buttons, ())

GPIO.add_event_detect(button1_pin, GPIO.FALLING, callback=button1, bouncetime = 100)
GPIO.add_event_detect(button2_pin, GPIO.FALLING, callback=button2, bouncetime = 100)
GPIO.add_event_detect(button3_pin, GPIO.FALLING, callback=button3, bouncetime = 100)
GPIO.add_event_detect(button4_pin, GPIO.FALLING, callback=button4, bouncetime = 100)

os.system("gpio -g mode 18 pwm")
os.system("gpio pwmc 1000")
display_brightness(max_brightness)

try:
    while True:
        pass
        time.sleep(1)
except:
    clean_exit()


sys.stderr.close()
sys.stderr = sys.__stderr__