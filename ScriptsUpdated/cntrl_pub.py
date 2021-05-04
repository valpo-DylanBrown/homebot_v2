#!/usr/bin/env python3
# Imports
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as gpio
from datetime import datetime
from time import sleep, time
# Definitions
CH3 = 31
CH4 = 29
CH5 = 23
CH6 = 37

CUTOFF_MIN = 4
CUTOFF_MAX = 11

# Globals
global ch4_risingCount
global ch4_pulseWidth
global ch4_timeStart
ch4_risingCount = 0
ch4_pulseWidth=0
ch4_timeStart=0

global ch3_risingCount
global ch3_pulseWidth
global ch3_timeStart
ch3_risingCount = 0
ch3_pulseWidth=0
ch3_timeStart=0

global ch5_risingCount
global ch5_pulseWidth
global ch5_timeStart
ch5_risingCount = 0
ch5_pulseWidth=0
ch5_timeStart=0

global ch6_risingCount
global ch6_pulseWidth
global ch6_timeStart
ch6_risingCount = 0
ch6_pulseWidth=0
ch6_timeStart=0

# Determines the pulse_width of the channel recived.
# This is done manually since the Pi cannot recive analog signals.
def ch4_edgeDetected(channel):

    global ch4_risingCount
    global ch4_pulseWidth
    global ch4_timeStart

    if gpio.input(CH4):
        #rising edge
        ch4_risingCount += 1
        ch4_timeStart = time()
    else:
        #falling edge
        if (ch4_risingCount != 0):
            timePassed = time() - ch4_timeStart
            #make pulseWidth an average
            ch4_pulseWidth = ((ch4_pulseWidth*(ch4_risingCount-1)) + timePassed)/ch4_risingCount
# Determines the pulse_width of the channel recived.
# This is done manually since the Pi cannot recive analog signals.
def ch3_edgeDetected(channel):

    global ch3_risingCount
    global ch3_pulseWidth
    global ch3_timeStart

    if gpio.input(CH3):
        #rising edge
        ch3_risingCount += 1
        ch3_timeStart = time()
    else:
        #falling edge
        if (ch3_risingCount != 0):
            timePassed = time() - ch3_timeStart
            #make pulseWidth an average
            ch3_pulseWidth = ((ch3_pulseWidth*(ch3_risingCount-1)) + timePassed)/ch3_risingCount
# Determines the pulse_width of the channel recived.
# This is done manually since the Pi cannot recive analog signals.
def ch5_edgeDetected(channel):

    global ch5_risingCount
    global ch5_pulseWidth
    global ch5_timeStart

    if gpio.input(CH5):
        #rising edge
        ch5_risingCount += 1
        ch5_timeStart = time()
    else:
        #falling edge
        if (ch5_risingCount != 0):
            timePassed = time() - ch5_timeStart
            #make pulseWidth an average
            ch5_pulseWidth = ((ch5_pulseWidth*(ch5_risingCount-1)) + timePassed)/ch5_risingCount
# Determines the pulse_width of the channel recived.
# This is done manually since the Pi cannot recive analog signals.
def ch6_edgeDetected(channel):

    global ch6_risingCount
    global ch6_pulseWidth
    global ch6_timeStart

    if gpio.input(CH6):
        #rising edge
        ch6_risingCount += 1
        ch6_timeStart = time()
    else:
        #falling edge
        if (ch6_risingCount != 0):
            timePassed = time() - ch6_timeStart
            #make pulseWidth an average
            ch6_pulseWidth = ((ch6_pulseWidth*(ch6_risingCount-1)) + timePassed)/ch6_risingCount

if __name__ == '__main__':
  # Initalize node
  rospy.init_node('ch_publisher')
  # Initalize control publishers
  ch3_pub = rospy.Publisher('ch3_state', Float32, queue_size=10)
  ch4_pub = rospy.Publisher('ch4_state', Float32, queue_size=10)
  ch5_pub = rospy.Publisher('ch5_state', Float32, queue_size=10)
  ch6_pub = rospy.Publisher('ch6_state', Float32, queue_size=10)
  gpio.setmode(gpio.BOARD)

  # Setup gpio pins
  gpio.setup(CH3, gpio.IN)
  gpio.setup(CH4, gpio.IN)
  gpio.setup(CH5, gpio.IN)
  gpio.setup(CH6, gpio.IN)

  # Add double-sided interrupts for GPIO
  gpio.add_event_detect(CH4, gpio.BOTH, callback=ch4_edgeDetected)
  gpio.add_event_detect(CH3, gpio.BOTH, callback=ch3_edgeDetected)
  gpio.add_event_detect(CH5, gpio.BOTH, callback=ch5_edgeDetected)
  gpio.add_event_detect(CH6, gpio.BOTH, callback=ch6_edgeDetected)
  while not rospy.is_shutdown():
    # sleep timer = .2 seconds
    st = .2
    sleep(st)
    # Translate pulse width into duty cycle
    ch4_dutycycle = round(ch4_pulseWidth * ch4_risingCount * 100 * (1/st), 2)
    ch3_dutycycle = round(ch3_pulseWidth * ch3_risingCount * 100 * (1/st), 2)
    ch5_dutycycle = round(ch5_pulseWidth * ch5_risingCount * 100 * (1/st), 2)
    ch6_dutycycle = round(ch6_pulseWidth * ch6_risingCount * 100 * (1/st), 2)

    # If out of range, dont publish the data
    if(ch3_dutycycle > CUTOFF_MIN and ch3_dutycycle < CUTOFF_MAX):
        ch3_pub.publish(ch3_dutycycle)
    if(ch4_dutycycle > CUTOFF_MIN and ch4_dutycycle < CUTOFF_MAX):
        ch4_pub.publish(ch4_dutycycle)
    if(ch5_dutycycle > CUTOFF_MIN and ch5_dutycycle < CUTOFF_MAX):
        ch5_pub.publish(ch5_dutycycle)
    if(ch6_dutycycle > CUTOFF_MIN and ch6_dutycycle < CUTOFF_MAX):
        ch6_pub.publish(ch6_dutycycle)
    # Reset counts and pulse_widths
    ch4_risingCount = 0
    ch3_risingCount = 0
    ch5_risingCount = 0
    ch6_risingCount = 0

    ch4_pulseWidth = 0
    ch3_pulseWidth = 0
    ch5_pulseWidth = 0
    ch6_pulseWidth = 0

  gpio.cleanup()
