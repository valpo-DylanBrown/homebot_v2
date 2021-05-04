#!/usr/bin/env python3
# Imports
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as gpio
from datetime import datetime
from time import sleep, time
# Definitions
CH1 = 35
CH2 = 33

CUTOFF_MIN = 4
CUTOFF_MAX = 11
#Globals
global ch1_risingCount
global ch1_pulseWidth
global ch1_timeStart
ch1_risingCount = 0
ch1_pulseWidth=0
ch1_timeStart=0

global ch2_risingCount
global ch2_pulseWidth
global ch2_timeStart
ch2_risingCount = 0
ch2_pulseWidth=0
ch2_timeStart=0

# Determines the pulse_width of the channel recived.
# This is done manually since the Pi cannot recive analog signals.
def ch1_edgeDetected(channel):

    global ch1_risingCount
    global ch1_pulseWidth
    global ch1_timeStart

    if gpio.input(CH1):
        #rising edge
        ch1_risingCount += 1
        ch1_timeStart = time()
    else:
        #falling edge
        if (ch1_risingCount != 0):
            timePassed = time() - ch1_timeStart
            # make pulseWidth an average
            ch1_pulseWidth = ((ch1_pulseWidth*(ch1_risingCount-1)) + timePassed)/ch1_risingCount
# Determines the pulse_width of the channel recived.
# This is done manually since the Pi cannot recive analog signals.
def ch2_edgeDetected(channel):

    global ch2_risingCount
    global ch2_pulseWidth
    global ch2_timeStart

    if gpio.input(CH2):
        #rising edge
        ch2_risingCount += 1
        ch2_timeStart = time()
    else:
        #falling edge
        if (ch2_risingCount != 0):
            timePassed = time() - ch2_timeStart
            #make pulseWidth an average
            ch2_pulseWidth = ((ch2_pulseWidth*(ch2_risingCount-1)) + timePassed)/ch2_risingCount

if __name__ == '__main__':
  # Initalize node
  rospy.init_node('drive_publisher')
  gpio.setmode(gpio.BOARD)
  # Initalize drive publishers
  ch1_pub = rospy.Publisher('ch1_state', Float32, queue_size=10)
  ch2_pub = rospy.Publisher('ch2_state', Float32, queue_size=10)
  # Setup GPIO pins
  gpio.setup(CH1, gpio.IN)
  gpio.setup(CH2, gpio.IN)
  # Add rising and falling interrupt for GPIO pins.
  gpio.add_event_detect(CH1, gpio.BOTH, callback=ch1_edgeDetected)
  gpio.add_event_detect(CH2, gpio.BOTH, callback=ch2_edgeDetected)
  while not rospy.is_shutdown():
    # sleep timer = .1 seconds
    st = .1
    sleep(st)
    # Translate pulse width into duty cycle
    ch1_dutycycle = round(ch1_pulseWidth * ch1_risingCount * 100 * (1/st), 2)
    ch2_dutycycle = round(ch2_pulseWidth * ch2_risingCount * 100 * (1/st), 2)
    # If out of range, dont publish the data
    if(ch1_dutycycle > CUTOFF_MIN and ch1_dutycycle < CUTOFF_MAX):
        ch1_pub.publish(ch1_dutycycle)
    if(ch2_dutycycle > CUTOFF_MIN and ch2_dutycycle < CUTOFF_MAX):
        ch2_pub.publish(ch2_dutycycle)
    # Reset counts and pulse_widths
    ch1_risingCount = 0
    ch2_risingCount = 0

    ch1_pulseWidth = 0
    ch2_pulseWidth = 0

  gpio.cleanup()
