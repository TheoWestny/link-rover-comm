#!/usr/bin/env python

import rospy
import os
import sys
import time
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import QThread
from std_msgs.msg import String


#Separate class for threading is declared
class ros_handler(QThread):

	#Init-function
	def __init__(self, node):
		QThread.__init__(self)
		self.node = node
		self.pub = rospy.Publisher(node, String, queue_size=1)
		self.rate = rospy.Rate(10)

		if node == 'engine':
			self.W_state = False
			self.S_state = False
		elif node == 'steering':
			self.A_state = False
			self.D_state = False

	def __del__(self):
		self.wait()

	def state_changer(self, keyInput, state):
		if keyInput == 'W':
			if state:
				self.W_state = True
			else:
				self.W_state = False
		elif keyInput == 'S':
			if state:
				self.S_state = True
			else:
				self.S_state = False
		elif keyInput == 'A':
			if state:
				self.A_state = True
			else:
				self.A_state = False
		elif keyInput == 'D':
			if state:
				self.D_state = True
			else:
				self.D_state = False
			

	def run(self):
		if self.node == 'engine':
			while True:
				try:
					if self.W_state:
						self.pub.publish("forward")
					elif self.S_state:
						self.pub.publish("reverse")
					self.rate.sleep()
				except rospy.ROSInterruptException:
					pass
		elif self.node == 'steering':
			while True:
				try:
					if self.A_state:
						self.pub.publish("left")
					elif self.D_state:
						self.pub.publish("right")
					self.rate.sleep()
				except rospy.ROSInterruptException:
					pass
		else:
			pass

#Interface class is declared
class Window(QtGui.QMainWindow):

	#Initialization-funtion
	def __init__(self):
		super(Window, self).__init__()
		self.setGeometry(50,50,500,300)
		self.setWindowTitle("LINK ROVER")
		#self.setWindowIcon(QtGui.QIcon('logo.png'))  <-- add logo here 

		#Create action connected to the "File" selection in the mainmenu		
		fileAction = QtGui.QAction("&Select", self)
		fileAction.setShortcut("Ctrl+Q")
		fileAction.setStatusTip('Leave the App')
		fileAction.triggered.connect(self.close_application)

		editAction = QtGui.QAction("&Edit", self)
		editAction.triggered.connect(self.print_something)
		editAction.setShortcut("Ctrl+E")
			
		self.statusBar()
		
		#Create a mainmenu at the top of the app
		mainMenu = QtGui.QMenuBar()
		mainMenu.setNativeMenuBar(False)
		self.setMenuBar(mainMenu)
		
		#Create a "File" selection
		fileMenu = mainMenu.addMenu('&File')
		fileMenu.addAction(fileAction)
		
		#Create a "Edit selection"		
		editMenu = mainMenu.addMenu('&Edit')
		editMenu.addAction(editAction)


		self.pub = rospy.Publisher('function', String, queue_size=1)
		
		self.thread_starter()

	#Enable threading for different ROS publishers and starts ROS-node "GUI"
	def thread_starter(self):
		rospy.init_node("GUI", anonymous=True)
		self.engine_thread = ros_handler('engine')
		self.steering_thread = ros_handler('steering')
		self.engine_thread.start()
		self.steering_thread.start()

		self.home()

	#Create button and connect to events
	def home(self):

		#Design QUIT-Button
		quitBtn = QtGui.QPushButton("Quit", self)	
		quitBtn.clicked.connect(self.close_application)
		quitBtn.resize(100,100)
		quitBtn.move(50,50)
		
		#Desgin control-button that enables control via keyboard
		controlBtn = QtGui.QPushButton("CONTROL", self)
		controlBtn.clicked.connect(self.control_publish)
		controlBtn.resize(100,100)
		controlBtn.move(175,50)

		#Design calibrate-button that enables ESC calibration
		calibrateBtn = QtGui.QPushButton("Calibrate", self)
		calibrateBtn.clicked.connect(self.calibrate_publish)
		calibrateBtn.resize(100,100)
		calibrateBtn.move(300,50)

		self.create_buttons()

	def create_buttons(self):

		#Move-btn-stylesheet
		self.moveBtnStyle_active = 'QPushButton {padding: 7px;background-color: #00FF00;color: #000000;border-style: outset; border-width: 2px; border-radius: 5px}'
		self.moveBtnStyle_idle = 'QPushButton {padding: 7px;background-color: #000000;color: #00FF00;border-style: outset; border-width: 2px; border-radius: 5px}'

		#Create a button to handle forward event
		self.forwardBtn = QtGui.QPushButton(u"\u2191", self)
		self.forwardBtn.resize(100, 50)
		self.forwardBtn.move(200,200)
		self.forwardBtn.clicked.connect(self.forward)
		self.forwardBtn.setStyleSheet(self.moveBtnStyle_idle)

		#Create a button to handle reverse event
		self.reverseBtn = QtGui.QPushButton(u"\u2193", self)
		self.reverseBtn.resize(100, 50)
		self.reverseBtn.move(200,250)
		self.reverseBtn.clicked.connect(self.reverse)
		self.reverseBtn.setStyleSheet(self.moveBtnStyle_idle)

		#Create a button to handle right event
		self.rightBtn = QtGui.QPushButton(u"\u2192", self)
		self.rightBtn.resize(100, 50)
		self.rightBtn.move(300,225)
		self.rightBtn.clicked.connect(self.right)
		self.rightBtn.setStyleSheet(self.moveBtnStyle_idle)

		#Create a button to handle left event
		self.leftBtn = QtGui.QPushButton(u"\u2190", self)
		self.leftBtn.resize(100, 50)
		self.leftBtn.move(100,225)
		self.leftBtn.clicked.connect(self.left)
		self.leftBtn.setStyleSheet(self.moveBtnStyle_idle)


	#Function to handle key-pressed events
	def keyPressEvent(self, event):
		if event.key() == QtCore.Qt.Key_W:
			self.forward()
		elif event.key() == QtCore.Qt.Key_S:
			self.reverse()
		elif event.key() == QtCore.Qt.Key_A:
			self.left()
		elif event.key() == QtCore.Qt.Key_D:
			self.right()

	#Function to handle key-released events
	def keyReleaseEvent(self, event):
		if event.key() == QtCore.Qt.Key_W:
			self.end_forward()
		elif event.key() == QtCore.Qt.Key_S:
			self.end_reverse()
		elif event.key() == QtCore.Qt.Key_A:
			self.end_left()
		elif event.key() == QtCore.Qt.Key_D:
			self.end_right()

	def control_publish(self):
		self.pub.publish("control")

	def calibrate_publish(self):
		self.pub.publish("calibrate")
			
	
	#Function to publish data to the engine_node
	def engine_publish(self, data):
		self.engine_pub.publish(data)

	#Function to publish data to the steering_node
	def steering_publish(self, data):
		self.steering_pub.publish(data)

	def forward(self):
		self.engine_thread.state_changer('W', True)
		self.forwardBtn.setStyleSheet(self.moveBtnStyle_active)
	
	def reverse(self):
		self.engine_thread.state_changer('S', True)
		self.reverseBtn.setStyleSheet(self.moveBtnStyle_active)

	def left(self):
		self.steering_thread.state_changer('A', True)
		self.leftBtn.setStyleSheet(self.moveBtnStyle_active)
	
	def right(self):
		self.steering_thread.state_changer('D', True)
		self.rightBtn.setStyleSheet(self.moveBtnStyle_active)

	def end_forward(self):
		self.engine_thread.state_changer('W', False)
		self.forwardBtn.setStyleSheet(self.moveBtnStyle_idle)

	def end_reverse(self):
		self.engine_thread.state_changer('S', False)
		self.reverseBtn.setStyleSheet(self.moveBtnStyle_idle)

	def end_left(self):
		self.steering_thread.state_changer('A', False)
		self.leftBtn.setStyleSheet(self.moveBtnStyle_idle)

	def end_right(self):
		self.steering_thread.state_changer('D', False)
		self.rightBtn.setStyleSheet(self.moveBtnStyle_idle)

	#Terminates GUI and closes open threads
	def close_application(self):
		print("hej")
		self.engine_thread.terminate()
		self.steering_thread.terminate()
		sys.exit()
			
	#Test-function
	def print_something(self):
		print("something")
	

	
#Starts up the GUI
def run():
	app = QtGui.QApplication(sys.argv)
	GUI = Window()
	GUI.show()
	sys.exit(app.exec_())
	

if __name__ == '__main__':
	try:
		run()
	except rospy.ROSInterruptException:
		pass
