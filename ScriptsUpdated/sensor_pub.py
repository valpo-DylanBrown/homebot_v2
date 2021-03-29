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
#FR_TRIG = 1
#FR_ECHO = 2
 
def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    #gpio.cleanup()
    sys.exit(0)
def get_sensor_distance(SENS_TRIG, SENS_ECHO):
    gpio.output(SENS_TRIG, gpio.LOW)
    time.sleep(0.1)
    gpio.output(SENS_TRIG, gpio.HIGH)
    time.sleep(0.00001)
    gpio.output(SENS_TRIG, gpio.LOW)
    while gpio.input(SENS_ECHO) == 0:
        pulse_start = time.time()
    while gpio.input(SENS_ECHO) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = (pulse_duration * SONAR_SPEED) / 2
    #if pulse_duration >= 0.05:
        #print('time out')
        #continue
    #elif distance >=300 or distance ==0:
        #print('out of range')
        #continue
    distance = round(distance,3)
    return distance
if __name__ == '__main__':
    rospy.init_node('sensor_publisher')
    signal.signal(signal.SIGINT, signal_handler)
    bl_pub = rospy.Publisher('BL_dist', Float32, queue_size=1) 
    #fr_pub = rospy.Publisher('FR_dist', Float32, queue_size=1)
    
    gpio.setmode(gpio.BOARD)
    gpio.setup(BL_TRIG, gpio.OUT)
    gpio.setup(BL_ECHO, gpio.IN)
    #gpio.setup(FR_TRIG, gpio.OUT)
    #gpio.setup(FR_ECHO, gpio.IN)
    
    rate = rospy.Rate(15)
    try:
    
        while not rospy.is_shutdown():
            
            bl_distance = get_sensor_distance(BL_TRIG, BL_ECHO)
            bl_pub.publish(bl_distance)
            #fr_distance = get_sensor_distance(FR_TRIG, FR_ECHO)
            #fr_pub.publish(fr_distance)
            rate.sleep()
            
    except(KeyboardInterrupt, SystemExit):
        gpio.cleanup()
        sys.exit(0)
    except:
        gpio.cleanup()
            