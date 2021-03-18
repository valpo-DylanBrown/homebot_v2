#!/usr/bin/env python3

from __future__ import print_function

import threading
from std_msgs.msg import Float32
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import signal
global ch1_movement
global ch2_movement
global BL_dist
global FR_dist
ch1_movement = 0
ch2_movement = 0
BL_dist = 0
FR_dist = 0
BACK_SENSOR_THRESHOLD = 50
#FRONT_SENSOR_THRESHOLD = 50

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    #gpio.cleanup()
    sys.exit(0)
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('RosAria/cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = .5
        self.turn = 1
        self.condition = threading.Condition()
        self.done = False


        # dont think we need a rate
        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, th):
        self.condition.acquire()
        self.x = x
        self.th = th

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.speed = 0.0
        self.turn = 0.0
        self.update(0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

def ch1_state_callback(msg):
    global ch1_movement
    # TODO: Take this out
    if (msg.data > 10.5) or (msg.data < 4.5):
        return
    if msg.data > 8.5:
        ch1_movement = -1
    elif msg.data < 6.5:
        ch1_movement = 1
    else:
        ch1_movement = 0

def ch2_state_callback(msg):
    global ch2_movement
    # TODO: Take this out
    if (msg.data > 10.5) or (msg.data < 4.5):
        return
    if msg.data > 8.5:
        ch2_movement = 1
    elif msg.data < 6.5:
        ch2_movement = -1
    else:
        ch2_movement = 0
# TODO do something with this data.
def BL_dist_callback(msg):
    global BL_dist
    BL_dist = msg.data
#def FR_dist_callback(msg):
    #global FR_dist
    #FR_dist = msg.data

if __name__=="__main__":

    rospy.init_node('qbot_drive')
    signal.signal(signal.SIGINT, signal_handler)
    # TODO Get rid of these params
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    th = 0
    rospy.Subscriber('ch1_state', Float32, ch1_state_callback)
    rospy.Subscriber('ch2_state', Float32, ch2_state_callback)
    rospy.Subscriber('BL_dist', Float32, BL_dist_callback)
    #rospy.Subscriber('FR_dist', Float32, FR_dist_callback)

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, th)
        while(1):
          if(BL_dist <= BACK_SENSOR_THRESHOLD):
            pub_thread.update(0,0)
            #print("back left triggered: ", BL_dist)
            continue
          #if(FR_dist <= FRONT_SENSOR_THRESHOLD):
            #pub_thread.update(0,0)
            #print("front right triggered")
            #continue
          else:
            x = ch2_movement
            if(x == -1):
                th = x * ch1_movement
            else:
                th = ch1_movement
            pub_thread.update(x, th)
            #print("Sensor 1 dist: ", sens1_dist)
    except(KeyboardInterrupt, SystemExit):
        print("requested stop")

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        sys.exit(0)
