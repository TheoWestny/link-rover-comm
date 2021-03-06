#!/usr/bin/env python

import rospy
import os
import time
import threading
import Adafruit_PCA9685
from std_msgs.msg import String

os.system("sudo pigpiod")
time.sleep(1)


import pigpio


def robot_init():
    global pwm
    pwm = Adafruit_PCA9685.PCA9685()

    # GPIO04
    global ESC
    ESC = 4

    global pi
    pi = pigpio.pi()
    pi.set_servo_pulsewidth(ESC, 0)

    global max_value
    max_value = 2000

    global min_value
    min_value = 700


class engine_listener(threading.Thread):

    def __init__(self, sub):
        threading.Thread.__init__(self)
        self.sub = sub
        self.oldcommand = None
        self.startspeed_forward = 1560
        self.startspeed_reverse = 1470
        self.speed = 1500
        pi.set_servo_pulsewidth(ESC, self.speed)

    def sent_command(self, data):
        self.drive(data.data)

    def drive(self, data):
        print(self.speed)
        if data == 'forward': 
            if self.oldcommand == "reverse":
                self.speed = self.startspeed_forward
            if self.speed != 1600:
                self.speed += 5
            if self.oldcommand != "forward":
                self.oldcommand = "forward" 
        elif data == 'reverse':
            if self.oldcommand == "forward":
                self.speed = self.startspeed_reverse
            if self.speed != 1400:
                self.speed -= 5
            if self.oldcommand != "reverse":
                self.oldcommand = "reverse"
        else:
            pass
        pi.set_servo_pulsewidth(ESC, self.speed)

    def run(self):
        rospy.Subscriber(self.sub, String, self.sent_command)
        rospy.spin()


class steering_listener(threading.Thread):

    # Init-function
    def __init__(self, sub):
        threading.Thread.__init__(self)
        self.sub = sub
        self.pwm_channel = 1
        self.last_command = None
        self.command_limit = None
        self.turn = 390
        self.speed = self.turn - 390
        self.turn_times = 0

    def sent_command(self, data):
        self.drive(data.data)

    def drive(self, data):
        print(self.turn)
        if data == 'right':
            if self.last_command != "right":
                self.last_command = "right"
                self.turn_times = 0
            elif self.last_command == "right":
                self.turn_times += 1
                self.turn = 410
            #if self.turn_times == 3:
            #    pass
            #if self.command_limit == "right":
            #    pass
            #elif self.command_limit == "left":
            #    self.command_limit = None
            #elif self.turn == 415:
            #    self.turn = 400
                #self.command_limit = "right"
            #else:
                #self.turn += 5
            #    self.turn = 410
        elif data == 'left':
            if self.last_command != "left":
                self.last_command = "left"
                self.turn_times = 0
            elif self.last_command == "left":
                self.turn_times += 1
                self.turn = 365
            #if self.turn_times == 3:
            #    pass
            #elif self.command_limit == "right":
            #    self.command_limit = None
            #elif self.turn == 365:
            #    self.turn = 400
            #    self.command_limit = "left"
            #else:
                #self.turn -= 5
            #    self.turn = 370
        else:
            pass
        if self.turn_times < 5:
            pwm.set_pwm(self.pwm_channel,0,self.turn)
            time.sleep(0.1)
            pwm.set_pwm(self.pwm_channel,0,390)

    def run(self):
        rospy.Subscriber(self.sub, String, self.sent_command)
        rospy.spin()


class robot_kinetics:

    def __init__(self):
        self.controlEnabled = False
        self.calibrateStarted = False
        self.calibrateContinued = False
        self.calibrateFinished = False

        self.pub = rospy.Publisher('response', String, queue_size=1)

    # self.pub.publish("alive")

    def calibrate(self):
        if not self.calibrateStarted:
            self.pub.publish("Switch of the ESC and press the button again")
            pi.set_servo_pulsewidth(ESC, 0)
            print("switch of the ESC and press the button again")

            time.sleep(5)
        elif self.calibrateStarted and not self.calibrateContinued:
            self.pub.publish("Connect the battery now... Wait for the ESC to beep then press the button again")
            pi.set_servo_pulsewidth(ESC, max_value)
            print("connect the battery now... w8 4 the ESC to beep then press the button again")
        else:
            self.pub.publish("Please wait for a few seconds...")

            time.sleep(5)
            pi.set_servo_pulsewidth(ESC, min_value)

            time.sleep(5)
            pi.set_servo_pulsewidth(ESC, 0)

            time.sleep(5)
            pi.set_servo_pulsewidth(ESC, min_value)

            time.sleep(1)
            print("done calibrating..")
            pi.set_servo_pulsewidth(ESC, 1500)

    # Put the arm-function after the calibration seems unnessecary. But so does calibrating every time...
    def arm(self):
        pi.set_servo_pulsewidth(ESC, 1500)
        print("Connect the battery and press Enter")
        inp =input()
        if inp == '':
            pi.set_servo_pulsewidth(ESC, 0)
            time.sleep(1)
            pi.set_servo_pulsewidth(ESC, max_value)
            time.sleep(1)
            pi.set_servo_pulsewidth(ESC, min_value)
            time.sleep(1)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if data.data == 'control' and not self.controlEnabled and not self.calibrateStarted:
            self.__thread_starter__()
            self.controlEnabled = True

        elif data.data == 'calibrate':
            if not self.calibrateStarted:
                self.calibrate()
                self.calibrateStarted = True
            elif self.calibrateStarted and not self.calibrateContinued:
                self.calibrate()
                self.calibrateContinued = True
            elif self.calibrateStarted and self.calibrateContinued:
                self.calibrate()
                self.calibrateStarted = False
                self.calibrateContinued = False
            else:
                pass
        else:
            print("TESTING INIT_COMMAND FUNCTION")

    def initial_command(self):
        rospy.Subscriber('function', String, self.callback)
        rospy.spin()

    def __thread_starter__(self):
        self.eThread = engine_listener('engine')
        self.sThread = steering_listener('steering')
        self.eThread.start()
        self.sThread.start()

    def main(self):
        rospy.init_node('ROBOT', anonymous=True)
        self.initial_command()


def run():
    robot_init()
    global ROBOT
    ROBOT = robot_kinetics()
    ROBOT.main()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        ROBOT.eThread.terminate()
        ROBOT.sThread.terminate()
        pass


