import RPi.GPIO as GPIO #RPI GPIO Library
import keyboard
import time

# Constants
LG_ACTUATOR_EXTEND = 38 #in3 of relay
LG_ACTUATOR_RETRACT = 40 #in4 of relay
SM_ACTUATOR_EXTEND = 35 #in1 of relay
SM_ACTUATOR_RETRACT = 37 #in2 of relay
MOTOR_ENA1 = 32 
MOTOR_FWD = 29
MOTOR_REV = 31

actuator_pins = [LG_ACTUATOR_EXTEND, LG_ACTUATOR_RETRACT, SM_ACTUATOR_EXTEND, SM_ACTUATOR_RETRACT]
motor_pins = [MOTOR_ENA1, MOTOR_FWD, MOTOR_REV]
GPIO.setmode(GPIO.BOARD)

lrg_act_extending = False
lrg_act_retracting = False
sm_act_extending = False
sm_act_retracting = False

# Pin Setup
for i in actuator_pins:
	GPIO.setup(i, GPIO.OUT)
	GPIO.output(i, GPIO.HIGH)

for i in motor_pins:
	GPIO.setup(i, GPIO.OUT)
	GPIO.output(i, GPIO.LOW)

# Function Defs
def lg_actuator_extend():
	global lrg_act_extending
	if lrg_act_extending == False:
		GPIO.output(LG_ACTUATOR_EXTEND, GPIO.LOW)
	else:
		GPIO.output(LG_ACTUATOR_EXTEND, GPIO.HIGH)
	
	lrg_act_extending = not lrg_act_extending
	
def lg_actuator_retract():
	global lrg_act_retracting
	if lrg_act_retracting == False:
		GPIO.output(LG_ACTUATOR_RETRACT, GPIO.LOW)
	else:
		GPIO.output(LG_ACTUATOR_RETRACT, GPIO.HIGH)
	
	lrg_act_retracting = not lrg_act_retracting

def sm_actuator_extend():
	global sm_act_extending
	if sm_act_extending == False:
		GPIO.output(SM_ACTUATOR_EXTEND, GPIO.LOW)
	else:
		GPIO.output(SM_ACTUATOR_EXTEND, GPIO.HIGH)
	
	sm_act_extending = not sm_act_extending
	
def sm_actuator_retract():
	global sm_act_retracting
	if sm_act_retracting == False:
		GPIO.output(SM_ACTUATOR_RETRACT, GPIO.LOW)
	else:
		GPIO.output(SM_ACTUATOR_RETRACT, GPIO.HIGH)
	
	sm_act_retracting = not sm_act_retracting


def motor_forward():
	GPIO.output(MOTOR_ENA1, GPIO.HIGH)
	GPIO.output(MOTOR_FWD, GPIO.HIGH)
	GPIO.output(MOTOR_REV, GPIO.LOW)

def motor_reverse():
	GPIO.output(MOTOR_ENA1, GPIO.HIGH)
	GPIO.output(MOTOR_FWD, GPIO.LOW)
	GPIO.output(MOTOR_REV, GPIO.HIGH)
	
def motor_off():
	GPIO.output(MOTOR_ENA1, GPIO.LOW)
	GPIO.output(MOTOR_FWD, GPIO.LOW)
	GPIO.output(MOTOR_REV, GPIO.LOW)

def key_press(key):
	if(key.name == 'w'):
		lg_actuator_extend()
	elif(key.name == 's'):
		lg_actuator_retract()
	elif (key.name == 'd'):
		sm_actuator_extend()
	elif (key.name == 'a'):
		sm_actuator_retract()
	elif(key.name == 'j'):
		motor_forward()
	elif(key.name == 'k'):
		motor_off()
	elif(key.name == 'l'):
		motor_reverse()
	elif(key.name == 'esc'):
		print("quitting")
keyboard.on_press(key_press)
try:
	while True: 
		time.sleep(0)
except KeyboardInterrupt:
	
	print("quit")
	GPIO.cleanup()

