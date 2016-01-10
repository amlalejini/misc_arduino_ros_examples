#!/usr/bin/python

import rospy, serial, atexit
from sensor_msgs.msg import Joy

'''
This node interfaces with an arduino programmed with the servo_control_example code.

Input is received from joy node.

Serial servo command packet format: label:value\n
'''

###################################
# Controller mappings in joy message
CONTROLLER_BUTTONS = {"A": 0, "B":1, "X": 2, "Y": 3, "R1": 5, "L1": 4, "BACK": 6, "START": 7} # TODO: FINISH THIS, USE BELOW
CONTROLLER_AXES = {"LSTICKV": 1, "LSTICKH": 0}
# Servo control
JOY_SERVO_AXIS = CONTROLLER_AXES["LSTICKH"]
DFLT_JOY_AXIS_THRESH = 0.5
# Below are default settings (overriden by param file)
# Servo Constants
DFLT_MAX_SERVO_ANGLE = 150
DFLT_MIN_SERVO_ANGLE = 5
DFLT_SERVO_INC = 5
# Initial Angle
DFLT_INIT_SERVO_ANGLE = 90
# Joystick topic name
DFLT_JOY_TOPIC_NAME = "joy"
# Arduino Port
DFLT_ARDUINO_PORT = "/dev/ttyACM1"
# Arduino Baud
DFLT_ARDUINO_BAUD = 9600
###################################

class JoyToServo(object):

    def __init__(self):
        # Initialize as ros node
        rospy.init_node("JoyToServo")

        # Initialize some variables
        self.current_angle = rospy.get_param("servo/initial_angle", DFLT_INIT_SERVO_ANGLE)
        self.max_servo_angle = rospy.get_param("servo/max_angle", DFLT_MAX_SERVO_ANGLE)
        self.min_servo_angle = rospy.get_param("servo/min_angle", DFLT_MIN_SERVO_ANGLE)
        self.servo_increment = rospy.get_param("servo/increment", DFLT_SERVO_INC)

        self.joystick_topic = rospy.get_param("joystick/topic", DFLT_JOY_TOPIC_NAME)
        self.joy_axis_thresh = rospy.get_param("joystick/axis_thresh", DFLT_JOY_AXIS_THRESH)
        self.joy_received = False
        self.controller_state = Joy()

        self.arduino = None                     # This will keep our serial connection to the arduino
        self.arduino_port = rospy.get_param("arduino/port", DFLT_ARDUINO_PORT)
        self.arduino_baud = rospy.get_param("arduino/baud", DFLT_ARDUINO_BAUD)

        # Setup subscription to joystick topic
        rospy.Subscriber(self.joystick_topic, Joy, self.joy_callback)

        # Attempt to connect to arduino
        while not rospy.is_shutdown():
            try:
                self.arduino = serial.Serial(self.arduino_port, self.arduino_baud, timeout = 1)
            except:
                rospy.logerr("Failed to connect to Arduino. Will continue trying.")
                rospy.sleep(3)
            else:
                rospy.loginfo("Connected to Arduino on port %s." % self.arduino_port)
                rospy.loginfo("Initializing servo angle to: %d" % self.current_angle)
                self.arduino.write(str("0:%d\n" % self.current_angle))
                break

        # Register cleanup function to run on exit
        atexit.register(self._cleanup)

    def limit(self, value):
        '''
        This function clips the given value to max or min if > max or < min
        '''
        if value > self.max_servo_angle:
            return self.max_servo_angle
        elif value < self.min_servo_angle:
            return self.min_servo_angle
        else:
            return value

    def joy_callback(self, msg):
        '''
        Joy topic callback function (called anytime a message is sent over joy topic)
        '''
        # Only care about meaningful joy messages
        if abs(msg.axes[JOY_SERVO_AXIS]) > self.joy_axis_thresh:
            self.controller_state = msg
            self.joy_received = True

    def run(self):
        '''
        Run function: process incoming messages (translate and send to arduino as servo commands)
        '''
        # Wait for a message to come over joystick topic before running
        rospy.wait_for_message(self.joystick_topic, Joy)
        # Set the rate this node will run at (running as fast as we can will kill ROS)
        rate = rospy.Rate(5)
        # Run!
        while not rospy.is_shutdown():
            if self.joy_received:
                # Grab most recent controller state
                current_state = self.controller_state
                cntrl_cmd = current_state.axes[JOY_SERVO_AXIS]
                # Calculate target angle from controller command
                #  If negative (< -0.5), decrement current angle by 2
                targ_angle = self.current_angle
                if cntrl_cmd < -self.joy_axis_thresh:
                    targ_angle = self.limit(self.current_angle - self.servo_increment)
                elif cntrl_cmd > self.joy_axis_thresh:
                    targ_angle = self.limit(self.current_angle + self.servo_increment)
                else:
                    targ_angle = self.limit(self.current_angle)
                #  If positive (> 0.5), increment current angle by 2
                #  Otherwise, do nothing
                self.arduino.write("0:%d\n" % targ_angle)
                self.current_angle = targ_angle
                rospy.loginfo("Setting servo angle to %d" % self.current_angle)
                self.joy_received = False
            rate.sleep()

    def _cleanup(self):
        """Called at exit to close connection to Arduino"""
        self.arduino.close()

if __name__ == "__main__":
    node = JoyToServo()
    node.run()
