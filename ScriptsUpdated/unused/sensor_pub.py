#!/usr/bin/env python3
import RPi.GPIO as gpio
import rospy
from std_msgs.msg import Float32
import sys
import signal
import time

SONAR_SPEED = 34300
BL_TRIG = 32
BL_ECHO = 26

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    #gpio.cleanup()
    sys.exit(0)
if __name__ == '__main__':
    rospy.init_node('sensor_publisher')
    signal.signal(signal.SIGINT, signal_handler)
    bl_sens = rospy.Publisher('bl_sens', Float32, queue_size=1) 
    gpio.setmode(gpio.BOARD)
    gpio.setup(BL_TRIG, gpio.OUT)
    gpio.setup(BL_ECHO, gpio.IN)

    rate = rospy.Rate(15)
    try:

        while not rospy.is_shutdown():
            gpio.output(BL_TRIG, gpio.LOW)
            time.sleep(0.1)
            gpio.output(BL_TRIG, gpio.HIGH)
            time.sleep(0.00001)
            gpio.output(BL_TRIG, gpio.LOW)
            while gpio.input(BL_ECHO) == 0:
                pulse_start = time.time()
            while gpio.input(BL_ECHO) == 1:
                pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * SONAR_SPEED) / 2
            distance = round(distance,3)

            bl_sens.publish(distance)

            rate.sleep()

    except(KeyboardInterrupt, SystemExit):
        gpio.cleanup()
        sys.exit(0)
    except:
        gpio.cleanup()