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

from Adafruit_ADS1x15 import ADS1x15
from collections import deque

# Remember that the PiTFT display uses the following GPIO pins: all the hardware SPI pins (SCK, MOSI, MISO, CE0, CE1), and GPIO 24 and 25.
# The 4 microswitches on the side of the display use: GPIO 17, 22, 23, 27 (from top to bottom)

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
 
# def read_sensor_raw():
#     f = open(device_file, 'r')
#     lines = f.readlines()
#     f.close()
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
        
def read_sensor_raw():
    catdata = subprocess.Popen(['cat',device_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out,err = catdata.communicate()
    out_decode = out.decode('utf-8')
    lines = out_decode.split('\n')
    return lines



# Setting up input pins for buttons:

GPIO.setup(button1_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button2_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button3_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(button4_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
# I2C connection to load cell.
# ADC connected to potentiometer

# Setting the output pins:

GPIO.setup(pump_pin, GPIO.OUT)
# GPIO.setup(threewayvalve_pin, GPIO.OUT)
GPIO.setup(heat_pin, GPIO.OUT)

GPIO.output(pump_pin, 0)
GPIO.output(heat_pin, 0)

# ADC for scale voltage readings

ADS1115 = 0x01	# 16-bit ADC
adc = ADS1x15(ic=ADS1115)

sps = 16 # sps can be 8, 16, 32, 64, 128, 250, 475, 860
sensitivity_factor = 2 # The sensitivity factor determines the length of the voltsdiff list, on which the moving average is computed: the longer the list, the less sensitive the measurement. A sensitivity factor of 4 means that the moving average is run on the voltages measured in the last one fourth of a second.
adc_resolution = 2048
scaling = [-287.35410, 430.74201]

print "Scale resolution = %0.4f grams" %(round(adc_resolution,0)/(2**15)*scaling[1]/1000.0)

voltsdiff=deque([-adc.readADCDifferential01(adc_resolution, sps)/1000.0]*int(sps/sensitivity_factor), int(sps/sensitivity_factor))
tare_weight = 0.0
prev_weight = [0.0]*4
mva_voltsdiff = 0.0


# Setting the pump pulse sequences corresponding to power levels in 10% increments.

pulse_sequence = [[0,0,0,0,0,0,0,0,0,0], # 0% power
          [0,0,0,0,1,0,0,0,0,0], # 10%
          [0,0,0,0,1,0,0,0,0,1], # 20%
          [0,0,0,1,0,0,1,0,0,1], # 30%
          [1,0,0,1,0,0,1,0,0,1], # 40%
          [1,0,1,0,1,0,1,0,1,0], # 50%
          [0,1,1,0,1,1,0,1,1,0], # 60%
          [1,1,1,0,1,1,0,1,1,0], # 70%
          [1,1,1,1,0,1,1,1,1,0], # 80%
          [1,1,1,1,0,1,1,1,1,1], # 90%
          [1,1,1,1,1,1,1,1,1,1]] # 100% power

# Closed-loop temperature control: PID parameters:

kP = 0.07
kI = 0.05
kD = 0.90
iMin = 0.0
iMax = 1.0

# Open-loop temperature control

power_when_pulling_shot = 36

# Time in seconds when the pump needs to stay idle before shot after button 4 is pressed, in order to tare the scale.

time_tare = 1.5

#####################################################################
# Global variables used to share state across functions and threads #
#####################################################################

last_timer = 0
last_weight = 0

shot_pouring = False 
old_shot_pouring = False
seconds_elapsed = 0.0

time_shot_stopped = time.time()-10
trigger_stop_scale = True

menu = 0 # Other options: 0 = main menu; 1 = settings menu
old_menu = 1
steaming_on = False
old_steaming_on = False
backflush_on = False
old_backflush_on = False

shot_temp = 90
steam_temp = 120
set_temp = shot_temp
old_set_temp = 0
power = 0
pump_power = 0 
preinf = True

submenu = 0
m_item_selected = 0

temp0 = read_sensor()
y = deque([temp0, temp0, temp0], 3600) 
power_series = deque([0,0,0], 3600)
shot_series = deque([False, False, False], 3600)

# With a refresh speed of 0.5 seconds, 3600 data points = 30 minutes of temperature logging. Perhaps overkill.
# Making sure that y starts with a length of 3, so that the PID function can be run right after read_temp.

trigger_refresh_temp_display = False
trigger_adjust_heating = False
trigger_refresh_timer = False

####################################
# UI area positions and dimensions #
####################################

# Screen resolution is 320 * 240.
# Careful here: pygame.Rect uses does not use coordinates for rectangles: (left, top, width, height)
area_graph = ((0,65),(280,155))
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

def read_temp():
    global y, trigger_refresh_temp_display, trigger_adjust_heating
    current_temp_reading = read_sensor()
    # if current_temp_reading < y[-1] - 5: # A drop of more than 5 degrees between readings is clearly an error.
    #     print "Implausible reading (" + str(current_temp_reading) + ") replaced by previous value of (" + str(y[-1])+")"
    #     current_temp_reading = y[-1]
    y.append(current_temp_reading)
    trigger_refresh_temp_display = True
    trigger_adjust_heating = True
    shot_series.append(shot_pouring)
    
def pid(y):
    # P: how far from the set_temp is y?
    P = set_temp - y[-1]
    # I: how far from the set_temp has y been on average over the last 30 seconds?
    I = sum(set_temp - y[-i] for i in xrange(1, min(len(y), 60)))/min(len(y), 60)
    # Make sure values of I stay between 0 and 1, or they will have too much weight on the final value returned by the PID(?)
    if I > 1:
        I = 1
    elif I < 0:
        I = 0
    # D: how has y evolved over the last 2 seconds (4 readings)?
    if len(y) < 4:
        D = 0
    elif len(y) >= 4:
        D = (y[-4] + y[-3])/2 - (y[-2] + y[-1])/2
    # Make sure that power only takes even percentage values -- because the duty cycle is .50 sec = 25 A/C waves = 50 half waves.
    # SSR time on can be adjusted in 1/50 = 2% increments
    if (abs(P) <= 1) & (abs(D) <= 1): # use a refined function when temp is close to set point and nearly stable. PID says power should be 0 at equilibrium, but that's clearly wrong. The system will always cool down, and some power will always need to be applied.
        power = random.choice([2, 4]) # on average power close to equilibrium should be 3%.
    else:
        power = int(round((kP*P + kI*I + kD*D)*50))*2
    # We can't go above 100% and below 0% heat.
    if power > 100:
        power = 100
    elif power < 0:
        power = 0
    return(float(power))

def adjust_heating_power():
    global power
    if shot_pouring == False:
        if steaming_on == False: 
            if (y[-1] > .85 * set_temp) | (y[-1] < 1.15 * set_temp):
                power = pid(y)
            elif y[-1] <= .85 * set_temp:
                power = 100
            elif y[-1] >= 1.15 * set_temp:
                power = 0
        elif steaming_on == True:
            if y[-1] < set_temp:
                power = 100
            if y[-1] >= set_temp:
                power = 0
    elif shot_pouring == True:
        power = power_when_pulling_shot
    power_series.append(power)

def output_heat():
    if power > 0:
        GPIO.output(heat_pin, True)
        time.sleep(.85 * power/100)
    if power < 100:
        GPIO.output(heat_pin, False)
        time.sleep(.85 * (100-power)/100)
    GPIO.output(heat_pin, False)

def output_pump(pump_power):
    if pump_power < 0 | pump_power > 10:
        print "Error, choose pump_power between 0 and 10"
    else:
        pulses = pulse_sequence[pump_power] # Is this necessary? Should only reset pulses if pump_power changes
        for i in range(0, 10):
            GPIO.output(pump_pin, pulses[i])
            time.sleep(.02) # wait 20 ms between pulses; could increase this to .04 to make it easier on the relay and the pump...

def pour_shot():
    # This function will be run in its own thread
    print "pour_shot thread started"
    # Remember to open the 3-way valve here.
    global pump_power, shot_pouring, time_shot_stopped
    time.sleep(time_tare+.5)
    while shot_pouring == True:
        if seconds_elapsed <= 9:
            pump_power = int(seconds_elapsed) + 1 # increase pump power by 10% each second for the first 10 seconds.
        else:
            pump_power = 10
        if current_weight > 34.0:
            end_shot()
        output_pump(pump_power)
    # Make sure that the thread can exit
    if shot_pouring == False:
        GPIO.output(pump_pin, 0)
        print "pour_shot thread exited"
        thread.exit()

def counting_seconds():
    # This function will be run in its own thread
    print "counting_seconds thread started"
    global seconds_elapsed, trigger_refresh_timer
    time.sleep(time_tare+.5)
    start = time.time()
    while shot_pouring == True:
        seconds_elapsed = math.floor((time.time() - start)*10)/10
        trigger_refresh_timer = True
        time.sleep(.1)
    # Make sure that the thread can exit
    if shot_pouring == False:
        print "counting_seconds thread exited"
        thread.exit()

######################################
# DISPLAY INFO / MENUS ON THE SCREEN #
######################################

# Setting up the screen 


os.putenv('SDL_FBDEV', '/dev/fb1')
pygame.init()
pygame.font.init()
pygame.mouse.set_visible(False)
lcd = pygame.display.set_mode((320, 240))

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
        if (coord_val < graph_bottom) & (coord_val > graph_top): # remember that y coordinates start at 0 from the top of the display
            pygame.draw.line(lcd, col_text, (graph_left, coord_val), (graph_right-20, coord_val)) # change this 300 value, based on graph_right
            display_text(str(val), (graph_right-15, coord_val-10), 20, col_text) # change this 305 value, based on graph_right, leave space for icons

def draw_lines(subset_y, y_axis, tick, graph_top, graph_bottom, color_series):
    y_coord  = [coordinates(y_axis, subset_y[j], graph_top, graph_bottom) for j in xrange(0, len(subset_y))]
    for j in xrange(1, len(y_coord)):
        pygame.draw.line(lcd, color_series[j-1], (graph_left + npixels_per_tempreading*(j-1), y_coord[j-1]),(graph_left + npixels_per_tempreading*j, y_coord[j]), 1) # 2 is the line thickness here.
    coord_set_temp = coordinates(y_axis, set_temp, graph_top, graph_bottom)
    if (coord_set_temp < graph_bottom) & (coord_set_temp > graph_top): # remember that y coordinates start at 0 from the top of the display
        pygame.draw.line(lcd, col_templine, (graph_left, coord_set_temp), (graph_right-20, coord_set_temp)) # change this 300 value, based on graph_right

def draw_power(power_data):
    for j in xrange(0, len(power_data)):
        pygame.draw.line(lcd, col_orange, (graph_left+npixels_per_tempreading*j, 220), (graph_left+npixels_per_tempreading*j, int(220-power_data[j]/4)),1)
        
def draw_belowgraph():
    string = "T: " + str(set_temp) + u"\u2103" + "  -  H: " + str(int(power)) + "%"
    if steaming_on == True:
        string = string + "  -  Steam"
    if steaming_on == False:
        if preinf == True:
            string = string + "  -  Preinfusion"
        else:
            string = string + " - No preinfusion"
    display_text(string, area_belowgraph[0], 25, col_text)

def refresh_timer_display(seconds_elapsed, area_timer):
    lcd.fill(col_background, rect = area_timer)
    display_text("%4s" % '{0:.1f}'.format(seconds_elapsed) + "s", (175, 5), 60, col_text)
    # display_text('{:>8}'.format(str(seconds_elapsed) + "s."), (180, 5), 60, col_text)
    #display_text(str(seconds_elapsed)+" s.", (180, 5), 70, col_text)
    pygame.display.update(area_timer)

def refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp):
    # Erase the areas to be updated
    lcd.fill(col_background, rect = area_timer)
    lcd.fill(col_background, rect = area_graph)
    lcd.fill(col_background, rect = area_text_temp)
    lcd.fill(col_background, rect = area_belowgraph)
    # Transform the series of 150 most recent ys into coordinates on the screen
    n_datapoints = int(math.floor((graph_right-20-graph_left)/npixels_per_tempreading))
    subset_y = [y[k] for k in xrange(max(0, len(y)-n_datapoints), len(y))]
    subset_power_series = [power_series[k] for k in xrange(max(0, len(power_series)-n_datapoints), len(power_series))]
    subset_shot_series = [shot_series[k] for k in xrange(max(0, len(shot_series)-n_datapoints), len(shot_series))]
    color_series = [col_red if whatshappening else col_white for whatshappening in subset_shot_series]
    # Find the range of y to be plotted, the tick marks, and draw.
    y_axis = axis_min_max(subset_y)
    tick = BestTick(y_axis[1]-y_axis[0])
    draw_power(subset_power_series)
    draw_axes(y_axis, tick, graph_top, graph_bottom-25)
    draw_lines(subset_y, y_axis, tick, graph_top, graph_bottom-25, color_series)
    if trigger_stop_scale == True:
        draw_belowgraph()
        if subset_y[-1] >= 100:
            display_text(str(int(round(subset_y[-1]))) + u"\u2103", (5, 5), 60, col_text)
        elif subset_y[-1] < 100:
            display_text('{0:.2f}'.format(subset_y[-1]) + u"\u2103", (5, 5), 60, col_text) # u"\u2103" is the unicode degrees celsisus sign.
        pygame.display.update(area_text_temp)
        if (last_weight != 0) & (last_timer != 0):
            display_text("Last shot", (180, 8), 25, col_text)
            display_text("%0.0f s. / %0.0f g." %(last_timer, last_weight), (180, 26), 25, col_text)
        pygame.display.update(area_timer)
    pygame.display.update(area_graph)
    pygame.display.update(area_belowgraph)
    

icon_start = pygame.image.load(os.path.join('icons', 'start.png'))
icon_plus = pygame.image.load(os.path.join('icons', 'plus.png'))
icon_minus = pygame.image.load(os.path.join('icons', 'minus.png'))
icon_up = pygame.image.load(os.path.join('icons', 'up.png'))
icon_down = pygame.image.load(os.path.join('icons', 'down.png'))
icon_more = pygame.image.load(os.path.join('icons', 'more.png'))
icon_settings = pygame.image.load(os.path.join('icons', 'settings.png'))
icon_check = pygame.image.load(os.path.join('icons', 'check.png'))
icon_back = pygame.image.load(os.path.join('icons', 'back.png'))
icon_stop = pygame.image.load(os.path.join('icons', 'stop.png'))

def draw_buttons(menu, shot_pouring, steaming_on, backflush_on):
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
        if (steaming_on == True) | (backflush_on == True):
            lcd.blit(icon_stop, (295, 216))
        else:
            lcd.blit(icon_back, (295, 36))
            lcd.blit(icon_up, (295, 96))
            lcd.blit(icon_down, (295, 156))
            lcd.blit(icon_check, (295, 216))

def refresh_buttons(menu, shot_pouring, steaming_on, backflush_on):
    # Same as time: only refresh if something changes.
    global old_menu, old_set_temp, old_shot_pouring, old_steaming_on, old_backflush_on
    if (menu != old_menu) | (shot_pouring != old_shot_pouring) | (steaming_on != old_steaming_on) | (backflush_on != old_backflush_on):
        print "Refreshing button icons"
        lcd.fill(col_background, rect = area_icons)
        draw_buttons(menu, shot_pouring, steaming_on, backflush_on)
        pygame.display.update(area_icons)
        old_menu = menu
        old_shot_pouring = shot_pouring
        old_steaming_on = steaming_on
        old_backflush_on = backflush_on

def reset_graph_area(menu, shot_pouring):
    global area_graph, graph_top, graph_bottom, graph_left, graph_right, npixels_per_tempreading
    if (menu == 0) & (shot_pouring == False):
        print "Big graph area"
        lcd.fill(col_background, rect = area_graph)
        pygame.display.update(area_graph)
        area_graph = ((0,65),(280,155))
        npixels_per_tempreading = 2
    elif (menu == 1)|(shot_pouring == True):
        print "Small graph area"
        lcd.fill(col_background, rect = area_graph)
        pygame.display.update(area_graph)
        area_graph = ((0,65),(140,155))
        npixels_per_tempreading = 1
    graph_top = area_graph[0][1]
    graph_bottom = area_graph[0][1] + area_graph[1][1] 
    graph_left  = area_graph[0][0]
    graph_right = area_graph[0][0] + area_graph[1][0]

def display_pump_power():
    old_pump_power = -1        
    while trigger_stop_scale == False:
        if (pump_power != old_pump_power) & (pump_power >= 0):
            print "display pump power update"
            lcd.fill(col_background, rect = (200, 65, 70, 155))
            display_text("Pump", (200, 180), 25, col_text)
            display_text(str(pump_power*10) + "%", (202 if pump_power == 10 else 206, 200), 25, col_text)
            for box in range(1, 11):
                lcd.fill(col_verydarkgrey, rect= ((215, 170-(box-1)*10), (20, 8)))
            for box in range(1, pump_power + 1):
                lcd.fill(col_lightblue, rect= ((215, 170-(box-1)*10), (20, 8)))
            pygame.display.update((200, 65, 70, 155))
            old_pump_power = pump_power
        time.sleep(.01)
    if trigger_stop_scale == True:
        print "Exiting display pump power thread"
        thread.exit()

area_menu = ((150, 65), (130, 155))

def backflush():
    global backflush_on
    backflush_on = True
    i = 1
    while (backflush_on == True) & (i <= 5):
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
    print "Exiting backflush thread"
    thread.exit()

def volts():
    global voltsdiff
    voltsdiff.append(-adc.readADCDifferential01(adc_resolution, sps)/1000.0)

def mean(x):
    return math.fsum(x)/len(x)

def sd(x):
    sqdev = [0]*len(x)
    for i in range(0, len(x)-1):
        sqdev[i] = (x[i] - mean(x))**2
    return ((math.fsum(sqdev))/(len(x)-1))**.5

def convert_volts():
    global current_weight, prev_weight, mva_voltsdiff, tare_weight
    mva_voltsdiff = mean(voltsdiff)
    current_weight = scaling[0] + mva_voltsdiff*scaling[1] - tare_weight
    if abs(current_weight) <= 0.1:
        prev_tare_weight = tare_weight
        tare_weight += current_weight
        current_weight = current_weight + prev_tare_weight - tare_weight
    if (abs(round(current_weight,1)) < 0.1) & (current_weight < 0):
        current_weight = -current_weight # This is just to get rid of these annoying "-0.0" weight measurements.
    if ((abs(current_weight - prev_weight[0])<.1) |
       ((abs(current_weight - prev_weight[0])<.2) & (prev_weight[0] == prev_weight[1]))):
        current_weight = prev_weight[0]
    prev_weight = [current_weight, prev_weight[0], prev_weight[1], prev_weight[2]]

def tare():
    global tare_weight, prev_weight, voltsdiff
    lcd.fill(col_background, rect = area_text_temp)
    text = "Taring"
    for i in range(1, 4):
        display_text("Taring" + "."*i, (175, 20), 30, col_text)
        pygame.display.update(area_timer)
        time.sleep(time_tare/3)
    tare_weight += current_weight
    prev_weight = [0.0]*4
    print "Tare"

def read_voltage():
    # Voltages are read 16 times per second
    while trigger_stop_scale == False:
        volts()
        time.sleep(1/round(sps, 0) + .0001) # E.g. if sps = 250: take a measurement every 0.0041 sec
    if trigger_stop_scale == True:
        print "read_voltage thread exited"
        thread.exit()

def voltages_to_weight():
    # Conversion in grams every 10th of a second
    while trigger_stop_scale == False:
        convert_volts()
        lcd.fill(col_background, rect = area_text_temp)
        display_text('{0:.1f}'.format(current_weight) + " g.", (5, 5), 60, col_text)
        pygame.display.update(area_text_temp)
        time.sleep(.1)
        sd_grams = sd(voltsdiff)*scaling[1]/(len(voltsdiff)**0.5)
        # print "w = %0.1f, sd = %0.4f" % (current_weight, sd_grams)
    if trigger_stop_scale == True:
        print "voltage_to_weight thread exited"
        thread.exit()

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

def display_menu_items(items, n_item_selected, number_of_choices):
    max_height = 240 - 60 - 25
    text_height = 25 * number_of_choices
    margin_height = (max_height - text_height)/2
    lcd.fill(col_background, area_menu)
    for i in range(0, number_of_choices):
        display_text(items[i], (170, 60 + margin_height + 25*i), 25, col_white)
    display_text(">", (160, 60 + margin_height - 2 + 25*(n_item_selected % number_of_choices)), 25, col_white)
    pygame.display.update(area_menu)

def display_main_menu():
    global items, n_items_selected, n
    items = ["Steam", "Shut Down", "Shot mode", "Temperature", "Flow", "Backflush"]
    n = len(items)
    display_menu_items(items, n_item_selected, n)

def display_confirm_shutdown_menu():
    global items, m_item_selected, m
    items = ["Shut Down", "Cancel"]
    m = len(items)
    display_menu_items(items, m_item_selected, m)

def end_shot():
    global time_shot_stopped, shot_pouring
    shot_pouring = False
    pump_power = 0
    GPIO.output(pump_pin, 0)
    # Remember to close the 3-way valve here.
    time_shot_stopped = time.time()
    thread.start_new_thread(wait_after_shot_and_refresh, ())

def wait_after_shot_and_refresh():
    while time.time() - time_shot_stopped < 5:
        time.sleep(.1)
    if time.time() - time_shot_stopped >= 5:
        global trigger_refresh_temp_display, trigger_stop_scale, last_weight, last_timer
        reset_graph_area(menu, shot_pouring)
        lcd.fill(col_background, area_timer)
        pygame.display.update(area_timer)
        refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
        trigger_refresh_temp_display = True
        trigger_stop_scale = True
        last_weight = current_weight
        last_timer = seconds_elapsed
        thread.exit()
    
def button1(channel):
    print "Button 1 pressed"
    global menu, n_item_selected, submenu
    if submenu == 0:
        menu = 1-menu
    elif submenu == 1:
        menu = 1
        submenu = 0
    reset_graph_area(menu, shot_pouring)
    refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
    if menu == 1:
        n_item_selected = 0
        display_main_menu()
    
def button2(channel):
    print "Button 2 pressed"
    if menu == 0:
        global set_temp, shot_temp
        set_temp += 1
        shot_temp = set_temp
        refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
    if menu == 1:
        global n_item_selected, m_item_selected
        if submenu == 0:
            n_item_selected -= 1
            display_main_menu()
        elif submenu == 1:
            m_item_selected -= 1
            display_confirm_shutdown_menu()

def button3(channel):
    print "Button 3 pressed"
    if menu == 0:
        global set_temp, shot_temp
        set_temp -= 1
        shot_temp = set_temp
        refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
    if menu == 1:
        global n_item_selected, m_item_selected
        if submenu == 0:
            n_item_selected += 1
            display_main_menu()
        elif submenu == 1:
            m_item_selected += 1
            display_confirm_shutdown_menu()

def button4(channel):
    # print "Button 4 pressed"
    global shot_pouring, pump_power, time_shot_stopped, trigger_stop_scale, steaming_on, set_temp, button4_repress, m_item_selected, n_item_selected, menu, submenu, backflush_on
    
    # Check if button 4 is called from within a submenu
    # This will change the behavior of button4: first button4 press: submenu is displayed; second button4 press: choice is selected.
    if submenu == 1:
        button4_repress = True
    elif submenu == 0:
        button4_repress = False
        m_item_selected = 0
    
    if menu == 0: # Main menu
        if shot_pouring == True:
            # Need to start a timer time_since_shot_ended here. Display ended time for 10 seconds; keep measuring weight for another 10 seconds; stay in shot display mode for 10 seconds.
            end_shot()
        elif shot_pouring == False :
            # here: tare the scale and thread.start_new_thread(update_scale, ())
            # here: preheat for 1-2 seconds if needed
            shot_pouring = True
            pump_power = 0
            reset_graph_area(menu, shot_pouring)
            refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
            trigger_stop_scale = False
            thread.start_new_thread(read_voltage, ())
            thread.start_new_thread(voltages_to_weight, ())
            thread.start_new_thread(tare, ())
            thread.start_new_thread(counting_seconds, ())
            thread.start_new_thread(pour_shot, ())
            thread.start_new_thread(display_pump_power, ())
            # Note: it's easy to start a thread from an outside function: button1 can start 2 threads: counting_seconds() and pour_shot()
            # but I don't know how to exit a thread remotely, from an outside function. thread.exit only works from the inside of the thread.
            # Make sure that each function called by thread.start_new_thread() has an if condition to trigger thread.exit() when its job is done.
    elif menu == 1: # Settings menu
        if n_item_selected % n == 0: 
            # Steam
            if (steaming_on == False):
                set_temp = steam_temp
                steaming_on = True
            elif steaming_on == True:
                set_temp = shot_temp
                steaming_on = False
        elif n_item_selected % n == 1: 
            # This is the "Shut Down" choice.
            submenu = 1
            # Asking for confirmation
            display_confirm_shutdown_menu()
            if (button4_repress == True) & (m_item_selected % m == 0): 
                # Save data in json file (y, power_series,)
                # Save settings (temperature, preinfusion, etc.) in another json file to be loaded later on.
                GPIO.cleanup()
                pygame.quit()
                os.system("sudo shutdown now")
            elif (button4_repress == True) & (m_item_selected % m == 1):
                submenu = 0
                m_item_selected = 0
                button4_repress = False
                display_main_menu()
        elif n_item_selected % n == 5: 
            # Backflush
            if backflush_on == False:
                thread.start_new_thread(backflush, ())
            elif backflush_on == True:
                backflush_on = False
    
        
GPIO.add_event_detect(button1_pin, GPIO.BOTH, callback=button1, bouncetime = 500)
GPIO.add_event_detect(button2_pin, GPIO.BOTH, callback=button2, bouncetime = 500)
GPIO.add_event_detect(button3_pin, GPIO.BOTH, callback=button3, bouncetime = 500)
GPIO.add_event_detect(button4_pin, GPIO.BOTH, callback=button4, bouncetime = 500)


##################################
# THREADS REFRESHING THE DISPLAY #
##################################

def thread_read_temp():
    while True:
#        t = time.time()
        read_temp()
        adjust_heating_power()
#        print time.time() - t
        output_heat()

# Check out the if condition inside the while loops. "trigger_" variables force the update/refresh.

def thread_refresh_temp_display():
    global trigger_refresh_temp_display
    while True:
        if trigger_refresh_temp_display:
            refresh_temp_display(y, graph_top, graph_bottom, area_graph, area_text_temp)
            trigger_refresh_temp_display = False
        time.sleep(.01)

def thread_refresh_buttons():
    while True:
        refresh_buttons(menu, shot_pouring, steaming_on, backflush_on)
        time.sleep(.01)

def thread_refresh_timer_display():
    global trigger_refresh_timer
    while True:
        if trigger_refresh_timer:
            refresh_timer_display(seconds_elapsed, area_timer)
            trigger_refresh_timer = False
        time.sleep(.01)

thread.start_new_thread(thread_read_temp, ())
thread.start_new_thread(thread_refresh_timer_display, ())
thread.start_new_thread(thread_refresh_temp_display, ())
thread.start_new_thread(thread_refresh_buttons, ())

try:
    while True:
        pass
        time.sleep(1)
except KeyboardInterrupt:
    print "KeyboardInterrupt"
    print "Exiting after cleaning up GPIO"
    GPIO.cleanup()

# backlight: terminal commands
# gpio -g mode 18 pwm
# gpio pwmc 1000
# gpio -g pwm 18 100
# gpio -g pwm 18 1023
# gpio -g pwm 18 0

# Done:

# * Separated read_temp from adjust_heating_power and refresh_temp_display
# * Rewrote the while loops for the tread_ functions.


# To do:

# When pulling shot: display weight instead of temperature in the topleft corner.

# If weight measurement is accurate and fast enough: calculate flowrate

# When pulling shot:
#
# Weight g.                          Timer sec.
#
# [temp graph] [pump power gauge] [flowrate graph]
#   Temp           Pump               Flow
#   92C            100%               2 g/sec

# Try to comment out thread.start_new_thread(thread_refresh_timer_display, ()) to see if CPU usage is still > 10%.
# Rewrite the pump_pulse and pour_shot functions
#        there needs to be a adjust_pump_power function which only updates a pulses global variable based on seconds_elapsed or user input (e.g. potentiometer). pump_pulse will cycle through pulses; it should be run in a separate thread when a shot is pulled.

# Run the add_heat (controlling the GPIO pin for heat SSR) in a separate thread.

# Is it OK if refresh_temp_display and read_temp are out of sync? (I guess so, but could there be a cleaner solution somehow -- e.g. read_temp tells refresh_temp_display to update as soon as there's new data?).

# When in settings menu, it would be nice to reduce the size of the graph. Just change area_graph when menu == "Settings menu", and don't forget to put it back to its original values when going back in the main menu. Could do the same thing when a shot is being pulled, to free up some screen space -- and display the shot weight, and maybe a pump_power gauge.

# Refine PID at equilibrium, so that when set_temp is reached, enough heat is generated to maintain the temp. When temperature within .5 degree of target: for example power  = 2% or 4%.

# Buttons: Instead of defining instructions conditional on state for each button (which gets messy),
# write conditions on state (is a shot being pulled? is steaming on? are we in the main menu?), and then redefine the buttons functions / update the display (draw the icons & update the text) accordingly.
# have an update_state function that keeps track of state, and changes in state.
# e.g. in one state: button1 will do x, button2 y, etc. , button4 will start a shot [and this is what buttons will look like]
#      in another state:  button1 will do blah, button2, etc. , button4 will heat the boiler for steaming [and this is what buttons will look like]
# Otherwise, it will get too messy very quickly.
# Example of states that will affect the buttons and their meaning:
# {(Menu: 0,1), (Submenu: 0,1,2,3), (Subsubmenu: 0,1,2,3,4), (mode: 0=idle, 1=pulling shot, 2=steaming)} 

# "..." menu
#         - Steam
#         - Shutdown the Raspberry Pi.
# For later:
#         - Heating parameters:
#                 - PID 
#                 - during shot
#         - Auto/manual stop.
#               - if auto: shot time
#            or - if auto: shot weight
#         - Auto/manual pressure.
#               - manual pressure: use the 10k pot.
#               - auto preinf
#               - (auto ramp down)
#               - (do not exceed max flow rate.)

# PID parameters and set_temp: save into a file when changes are made; read from the file when starting up.

# Change color of lines when a shot is being pulled.

# Take care of the load cell / weight and ADC conversion later.
#
# Done (kind of): 
# PID
# Adjust heat during shot (set values)
# Steaming
# Menus and buttons. Get icons here: https://www.google.com/design/icons/



# Need to sync read_temp and adjust_heat:
#
# read_temp() data should have a timestamp.
#
# if time.time() - timestamp > .39:
#     print "read_temp and adjust_heat are getting out of sync"
#     GPIO heat False for the next .39
