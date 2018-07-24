#!/usr/bin/env python

import rospy
import os
import time
import threading
#import Adafruit_PCA9685
from std_msgs.msg import String

#os.system("sudo pigpiod")
time.sleep(1)
#import pigpio


def robot_init():
	
	#pwm = Adafruit_PCA9685.PCA9685()	
	
	#GPIO04
	ESC = 4

	#pi = pigpio.pi()
	#pi.set_servo_pulsewidth(ESC, 0)

	max_value = 2000
	min_value = 700


class engine_listener(threading.Thread):
	
	def __init__(self, sub):
		threading.Thread.__init__(self)
		self.sub = sub
		self.speed = 1500
	
	def sent_command(self, data):
		self.drive(data.data)

	def drive(self, data):
		print(self.speed)
		if data == 'forward' and self.speed != 1588:
			self.speed += 1
		elif data == 'reverse' and self.speed != 1417:
			self.speed -= 1
		else:
			pass
		#pi.set_servo_pulsewidth(ESC, self.speed)

	def run(self):
		rospy.Subscriber(self.sub, String, self.sent_command)
		rospy.spin()

	
	

class steering_listener(threading.Thread):

	#Init-function
	def __init__(self, sub):
		threading.Thread.__init__(self)
		self.sub = sub
		self.turn = 425

	def sent_command(self, data):
		self.drive(data.data)
			
	
	def drive(self, data):
		print(self.turn)
		if data == 'left' and self.turn != 510:
			self.turn += 5
		elif data == 'right' and self.turn != 335:
			self.turn -= 5
		else:
			pass
		#pwm.set_pwm(0,0,self.turn)
			

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

	
	def calibrate(self):
		if not self.calibrateStarted:
			#pi.set_servo_pulsewidth(ESC, 0)
			print("switch of the ESC and press the button again")
			#self.pub.publish("switch of the ESC and press the button again")
			time.sleep(5)
		elif self.calibrateStarted and not self.calibrateContinued:
			#pi.set_servo_pulsewidth(ESC, max_value)
			print("connect the battery now... w8 4 the ESC to beep then press the button again")
			#self.pub.publish("connect the battery now... w8 4 the ESC to beep then press the button again")
		else:
			
			time.sleep(5)
			#pi.set_servo_pulsewidth(ESC, min_value)
			
			time.sleep(5)
			#pi.set_servo_pulsewidth(ESC, 0)
			
			time.sleep(5)
			#pi.set_servo_pulsewidth(ESC, 0)
			
			time.sleep(1)
			print("done calibrating..")
	

	def callback(self,data):
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
		if data.data == 'control' and not self.controlEnabled and not self.calibrateStarted:
			self.thread_starter()
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

	def thread_starter(self):
		self.eThread = engine_listener('engine')
		self.sThread = steering_listener('steering')
		self.eThread.start()
		self.sThread.start()

	def main(self):
		rospy.init_node('ROBOT', anonymous=True)
		self.initial_command()

def run():
	robot_init()
	ROBOT = robot_kinetics()
	ROBOT.main()

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		ROBOT.engine_thread.terminate()
		ROBOT.steerng_thread.terminate()
		pass




