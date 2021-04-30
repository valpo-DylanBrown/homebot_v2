#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as gpio

global lg_act_on
lg_act_on = 0
global ch6_enable
ch6_enable = 0

# LG_ACT_EXTEND = 5
# LG_ACT_RETRACT = 7
# SM_ACT_EXTEND = 10
# SM_ACT_RETRACT = 8
LG_ACT_EXTEND = 8
LG_ACT_RETRACT = 10
SM_ACT_EXTEND = 5
SM_ACT_RETRACT = 7
MOTOR_ENA1 = 11
MOTOR_ON = 13
MOTOR_OFF = 15
DEADZONE_HIGH = 9
DEADZONE_LOW = 6

def ch3_state_callback(msg):
    global lg_act_on
    if(ch6_enable == 0):
      gpio.output(LG_ACT_EXTEND, gpio.LOW)
      gpio.output(LG_ACT_RETRACT, gpio.LOW)
      lg_act_on = 0
      return
    if msg.data > DEADZONE_HIGH:
      gpio.output(LG_ACT_RETRACT, gpio.LOW)
      gpio.output(LG_ACT_EXTEND, gpio.HIGH)
      lg_act_on = 1
    elif msg.data < DEADZONE_LOW:
      gpio.output(LG_ACT_EXTEND, gpio.LOW)
      gpio.output(LG_ACT_RETRACT, gpio.HIGH)
      lg_act_on = 1
    else:
      gpio.output(LG_ACT_EXTEND, gpio.LOW)
      gpio.output(LG_ACT_RETRACT, gpio.LOW)
      lg_act_on = 0
def ch5_state_callback(msg):
    if(ch6_enable == 0):
      gpio.output(SM_ACT_EXTEND, gpio.LOW)
      gpio.output(SM_ACT_RETRACT, gpio.LOW)
      return
    if msg.data > DEADZONE_HIGH-0.5:
      gpio.output(SM_ACT_EXTEND, gpio.LOW)
      gpio.output(SM_ACT_RETRACT, gpio.HIGH)
    elif msg.data < DEADZONE_LOW+0.5:
      gpio.output(SM_ACT_RETRACT, gpio.LOW)
      gpio.output(SM_ACT_EXTEND, gpio.HIGH)
    else:
      gpio.output(SM_ACT_RETRACT, gpio.LOW)
      gpio.output(SM_ACT_EXTEND, gpio.LOW)
def ch4_state_callback(msg):
    global lg_act_on
    if(ch6_enable == 0):
      gpio.output(MOTOR_ENA1, gpio.LOW)
      gpio.output(MOTOR_OFF, gpio.LOW)
      gpio.output(MOTOR_ON, gpio.LOW)
      return
    if lg_act_on:
      gpio.output(MOTOR_ENA1, gpio.LOW)
      gpio.output(MOTOR_OFF, gpio.LOW)
      gpio.output(MOTOR_ON, gpio.LOW)
    elif msg.data > DEADZONE_HIGH:
      gpio.output(MOTOR_ENA1, gpio.HIGH)
      gpio.output(MOTOR_OFF, gpio.LOW)
      gpio.output(MOTOR_ON, gpio.HIGH)
    elif msg.data < DEADZONE_LOW:
      gpio.output(MOTOR_ENA1, gpio.HIGH)
      gpio.output(MOTOR_ON, gpio.LOW)
      gpio.output(MOTOR_OFF, gpio.HIGH)
    else:
      gpio.output(MOTOR_ENA1, gpio.LOW)
      gpio.output(MOTOR_OFF, gpio.LOW)
      gpio.output(MOTOR_ON, gpio.LOW)
def ch6_state_callback(msg):
    global ch6_enable
    # 1 = enable, 0 = disable, -1 = override
    if (msg.data > 10.5) or (msg.data < 4.5):
        return
    if msg.data > 8.5:
        ch6_enable = 0
    elif msg.data < 6.5:
        ch6_enable = 1
    else:
        ch6_enable = -1
if __name__ == '__main__':
    rospy.init_node('actuator_motor_subscriber')
    gpio.setmode(gpio.BOARD)
    gpio.setup(LG_ACT_EXTEND, gpio.OUT)
    gpio.setup(LG_ACT_RETRACT, gpio.OUT)
    gpio.setup(SM_ACT_EXTEND, gpio.OUT)
    gpio.setup(SM_ACT_RETRACT, gpio.OUT)
    gpio.setup(MOTOR_ENA1, gpio.OUT)
    gpio.setup(MOTOR_OFF, gpio.OUT)
    gpio.setup(MOTOR_ON, gpio.OUT)
    rospy.Subscriber('ch6_state', Float32, ch6_state_callback)
    rospy.Subscriber('ch3_state', Float32, ch3_state_callback)
    rospy.Subscriber('ch4_state', Float32, ch4_state_callback)
    rospy.Subscriber('ch5_state', Float32, ch5_state_callback)
    rospy.spin()
    gpio.cleanup()
