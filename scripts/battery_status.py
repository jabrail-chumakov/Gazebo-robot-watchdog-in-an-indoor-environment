#!/usr/bin/env python

"""
.. module:: battery_status
	:platform: Unix
	:synopsis: Python module for the management of the battery status
   
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

The task for the second assignment in the Experimental Robotics course
of the Robotics Engineering Master program is to create a ROS node for
controlling the battery level of a robot.
The software architecture should allow for the publishing of the battery
state on the topic "/state/battery_low". The battery level can transition
from high to low in two ways: either by setting the battery_low flag to
"True" after a random delay defined in the architecture, or manually by
retrieving the input from the user. When the battery level becomes low,
the transition is published. Additionally, the node should implement a
service that charges the robot when the battery level becomes low.
This service is blocking, and after the battery is charged, the client receives a response.

Publishes to:
	/state/battery_low battery level
	
Service:
	/state/recharge recharge robot
		
"""

# Import necessary modules and libraries
import time
import rospy
import random
import threading

# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style

# Import messages, actions and services
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

# Import constant name defined to structure the architecture
from assignment2 import architecture_name_mapper as anm

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_BATTERY_STATE

# Define a class for Robot Battery State node
class RobotBatteryState:
	"""
	The purpose of this class is to manage the state of the robot's battery, which involves changing
	the battery state to low and recharging the battery when the robot reaches the charging location 'E'.
	"""
	def __init__(self):
		""" 
		This method is responsible for initializing the RobotBatteryState class.
		
		Args:
			self: current class instance.
		"""
		# Initialise the node and battery level
		rospy.init_node(anm.NODE_ROBOT_BATTERY_STATE, log_level = rospy.INFO)
		self._battery_low = False

		# Set time period to charge robot's battery
		self._random_battery_charge = rospy.get_param(anm.PARAM_BATTERY_CHARGE, [5.0, 15.0])
		log_msg = (Fore.YELLOW + f'Battery of the robot will be charged between {self._random_battery_charge[0]} and {self._random_battery_charge[1]} seconds.\n')
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 

		# Initialize and define the service to recharge the state of the battery
		rospy.Service(anm.TOPIC_RECHARGE, SetBool, self.battery_charger)
		# Set time period after which robot will go to charge station in Room "E"
		self._randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
		if self._randomness:
			self._random_battery_time = rospy.get_param(anm.PARAM_BATTERY_TIME, [220.0, 235.0])
			log_msg = (Fore.YELLOW + f'Robot will start charging in {self._random_battery_time[0]} or {self._random_battery_time[1]} seconds.\n')
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Makes text from yellow to normal
			print('\033[39m') 

		# Start a new thread to monitor the battery level
		th = threading.Thread(target = self.is_battery_low)
		th.start()
		
		# Log information about battery low topic
		log_msg = (Fore.GREEN + f'Node: `{anm.NODE_ROBOT_BATTERY_STATE}` | Topic: `{anm.TOPIC_BATTERY_LOW}`.\n')
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Makes text from green to normal
		print('\033[39m') 


	def is_battery_low(self):
		""" 
		This method runs on a separate thread and publishes any changes in the battery status.
		
		Args:
			self: current class instance.
		"""
		# Create a publisher to publish battery low message
		publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size = 1, latch = True)
		if self._randomness: 
			# Start a thread to randomly notify battery low message
			self.random_battery_notifier(publisher)
			
	# Method for battery charging service
	def battery_charger(self, request):
		""" 
		This service is used to charge the robot's battery. When the client requests the service,
		the server simulates the charging process by waiting for a duration of time between
		self._random_battery_charge[0] and self._random_battery_charge[1]. Once this time has passed,
		the server changes the battery state to high and sends a response to the client.
		
		Args:
			self: current class instance.
			request: boolean to indicate charging beginning.
		
		Returns:
			response: boolean to indicate charging end.
		
		"""
		# Respond to battery charging request
		response = SetBoolResponse()  
		if request.data == True:
			log_msg = Fore.RED + f'\nBattery leter is low, going to recharge.'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Makes text from red to normal
			print('\033[39m') 
			delay_charge = random.uniform(self._random_battery_charge[0], self._random_battery_charge[1])
			rospy.sleep(delay_charge)
			log_msg = Fore.GREEN + f'The battery level is 100% now and it was charged for {delay_charge} seconds.\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Makes text from green to normal
			print('\033[39m') 
			self._battery_low = False   
			response.success = True     
		else:
			response.success = False
		return response

	# Method for battery status notification
	def random_battery_notifier(self, publisher):
		""" 
		This method publishes a message indicating that the battery level has become low.
		The delay before publishing is randomized within the range of [self._random_battery_time[0],
		self._random_battery_time[1]]. The message is sent through the 'publisher' input parameter
		as a boolean value, where 'True' indicates that the battery is low.
		
		Args:
			self: current class instance.
			publisher: boolean to indicate charge level.
		
		"""
		while not rospy.is_shutdown():
			if self._battery_low == False:
				delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
				rospy.sleep(delay)
				self._battery_low = True	
				publisher.publish(Bool(self._battery_low))
				log_msg = f'Battery level is low after {delay} seconds.\n'
				self.print_info(log_msg)

	# Method for printing information messages
	def print_info(self, msg):
		""" 
		The method prints log information only when random testing is in progress, so that users
		can easily use the keyboard interface without being distracted by excessive information.
		
		Args:
			self: current class instance.
			msg: message for the logger
		
		"""
		if self._randomness:
			rospy.loginfo(anm.tag_log(msg, LOG_TAG))


if __name__ == "__main__":
	# Create an instance of the RobotBatteryState class
	RobotBatteryState()
	# Spin the ROS node
	rospy.spin()

