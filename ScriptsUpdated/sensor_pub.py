#!/usr/bin/env python3
import RPi.GPIO as gpio
import rospy
from std_msgs.msg import Float32
import sys
import signal
import time
 
SONAR_SPEED = 34300
SENS1_TRIG = 31
SENS1_ECHO = 29
 
def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    #gpio.cleanup()
    sys.exit(0)
if __name__ == '__main__':
    rospy.init_node('sensor_publisher')
    signal.signal(signal.SIGINT, signal_handler)
    sens1_pub = rospy.Publisher('sens1_dist', Float32, queue_size=1) 
    gpio.setmode(gpio.BOARD)
    gpio.setup(SENS1_TRIG, gpio.OUT)
    gpio.setup(SENS1_ECHO, gpio.IN)
    
    rate = rospy.Rate(15)
    try:
    
        while not rospy.is_shutdown():
            gpio.output(SENS1_TRIG, gpio.LOW)
            time.sleep(0.1)
            gpio.output(SENS1_TRIG, gpio.HIGH)
            time.sleep(0.00001)
            gpio.output(SENS1_TRIG, gpio.LOW)
            while gpio.input(SENS1_ECHO) == 0:
                pulse_start = time.time()
            while gpio.input(SENS1_ECHO) == 1:
                pulse_end = time.time()
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * SONAR_SPEED) / 2
            if pulse_duration >= 0.05:
                print('time out')
                continue
            elif distance >=300 or distance ==0:
                print('out of range')
                continue
            distance = round(distance,3)
            
            sens1_pub.publish(distance)
            
            rate.sleep()
            
    except(KeyboardInterrupt, SystemExit):
        gpio.cleanup()
        sys.exit(0)
    except:
        gpio.cleanup()
            