#!/usr/bin/env python

"""
.. module:: fsm_helper
	:platform: Unix
	:synopsis: Python module for the management of the battery status
   
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

This is a ROS node for the second assignment of the Robotics Engineering Master program's Experimental Robotics course.
Its software architecture enables the initialization of a helper class for the Final State Machine that governs the
behavior of a surveillance robot. The node provides a way to keep the code in the state_machine.py node more organized
and easy to understand by defining each task in this node.

Publishes to:
	/cmd_vel control robot's velocities
	/robot/joint1_position_controller/command rotate arm's base joint
Subscribes to:
	/state/battery_low subscribes to battery state
	
Service:
	/state/recharge robot charging
	/armor_interface_srv for ontology communication
	/world_init obtain world information from aruco markers
	
Action Service:
	/move_base move robot between rooms
"""

import threading
import random
import rospy
import rospkg
import os
import time
import sys
import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from threading import Lock
from actionlib import SimpleActionClient

# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style

# Import the message to rotate the camera during the surveillance phase
from sensor_msgs.msg import JointState

# Import constant name defined to structure the architecture.
from assignment2 import architecture_name_mapper as anm

# Import the messages used by services and publishers.
from std_msgs.msg import Bool, Float64, Float32
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

# Import services and messages to build the world
from assignment2.srv import WorldInit, WorldInitResponse
from assignment2.msg import RoomConnection

# Armor import to work with the ontology
from armor_msgs.srv import ArmorDirective, ArmorDirectiveRequest, ArmorDirectiveResponse
from armor_msgs.srv import ArmorDirectiveList, ArmorDirectiveListRequest, ArmorDirectiveListResponse

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE

# get the file path for rospy_tutorials
rp = rospkg.RosPack()
assignment_path = rp.get_path('assignment2')

# Define the file path in which the ontology is stored
ONTOLOGY_FILE_PATH = os.path.join(assignment_path, "topological_map", "topological_map.owl")
ONTOLOGY_FILE_PATH_DEBUG = os.path.join(assignment_path, "topological_map", "new_topological_map.owl")
WEB_PATH = 'http://bnc/exp-rob-lab/2022-23'

# Initialize and define the client to use armor
cli_armorontology = rospy.ServiceProxy('/armor_interface_srv', ArmorDirective) 
# Initialize and define the request message for armor
armorontology_req = ArmorDirectiveRequest()
# Initialize and define the arg list to pass to the ontology
ARGS = []
# Define the number for which the state of the action client is done
DONE = 3 # since the get_state() function returns 3 when the action server achieves the goal



def ontology_manager(command, primary_command_spec, secondary_command_spec, ARGS):
	""" 
	This function serves the purpose of communicating with the ARMOR service in order to obtain
	and update information about the environment in the ontology. It is used as a replacement for the ARMOR API.
		
	Args:
		command: command to execute (e.g. ADD, LOAD, ...).
		primary_command_spec: primary command specification (optional).
		secondary_command_spec: secondary command specification (optional).
		ARGS: the list of arguments (e.g. list of individuals to add).
	Returns:
		armorontology_res: returns a list of queried objects.
	"""
	armorontology_req.armor_request.client_name = 'example'
	armorontology_req.armor_request.reference_name = 'ontoRef'
	armorontology_req.armor_request.command = command
	armorontology_req.armor_request.primary_command_spec = primary_command_spec
	armorontology_req.armor_request.secondary_command_spec = secondary_command_spec
	armorontology_req.armor_request.args = ARGS
	rospy.wait_for_service('/armor_interface_srv')
	try:
			armorontology_res = (cli_armorontology(armorontology_req)).armor_response.queried_objects
			return armorontology_res
	except rospy.ServiceException as e:
			print('Service call failed: %s' %e)
			sys.exit(1)
			
 	
def ontology_format(old_list, start, end):
	""" 
	This function accepts a list as input and generates a new list that is based on the original one.
	The new list contains all elements of the original list starting from the specified 'start' index
	and ending at the specified 'end' index. The resulting list only includes characters and numbers
	within the range of the specified indexes.
		
	Args:
		old_list: refers to the list that requires formatting.
		start: refers to the index of the element in the original list that will be the first to be copied to the new list.
		end: refers to the index of the element in the original list that will be the last to be copied to the new list.
	Returns:
		new_list: refers to the list that has been properly formatted according to the specified requirements.
	"""
	new_list = [num[start:end] for num in old_list]
	return new_list



class Helper:
	"""
	This class has been designed to separate the implementation of the Finite State Machine from the rest of the code in
	the state_machine.py node. By managing synchronization with subscribers, services, and action servers, it helps to
	ensure that the code is more readable and easier to understand.
	"""
	def __init__(self):
		""" 
		This function is used to initiate the Helper class.
		
		Args:
			self: current class instance.
		"""
		# Initialize the variables used in the class 
		self.battery_low = False            # Set to True if the battery of the robot is low
		self.map_completed = False          # Set to True when the ontology is complete
		self.aruco_detected = False         # Set to true when all the aruco markers have been detected
		self.reasoner_done = False          # Set to True after querying the ontology
		self.motion_completed = False       # Set to True when the robot arrives in the desired location
		self.charge_reached = False         # Set to True when the charging station is reached
		self.check_completed = False        # Set to True when the robot has finished surveillance action
		
		self._rooms_coord = []                 # List to store the coordinates X and Y of each room 
		self._connections = []                 # List of connections containing _connected_to and _through_door lists
		self._connected_to = []                # List of rooms connected to a specific room
		self._through_door = []                # List of doors used for connecting rooms
		self._rooms = []                       # List of all rooms 
		self._doors = []                       # List of all doors 
		self._next_goal = ''                   # Variable to store the name of the room that will be visited
		self._prev_goal = anm.INIT_LOCATION    # Previous location, the robot starts from location 'E'
		self.markers_detected = 0              # Count the number of detected markers
		self.charge_loc = anm.CHARGE_LOCATION  # Define the charging location
		
		self.rate = rospy.Rate(10) # Loop at 10hz
		
		# Initialize the current time
		self.timer_now = str(int(time.time()))  
		# Initialize and define the mutex to work with shared variables
		self.mutex = Lock()
		
		# Load the ontology
		ARGS = [ONTOLOGY_FILE_PATH, WEB_PATH, 'true', 'PELLET', 'false']
		ontology_manager('LOAD', 'FILE', '', ARGS)	
		log_msg = f'Ontology successuly loaded'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		
		# Subscribe to the topic that controls the battery level.
		self.battery_sub = rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.battery_callback)
		
		# Publisher to move rotate the camera for surveillance action
		self.pub_base_joint = rospy.Publisher(anm.TOPIC_JOINT_BASE, Float64, queue_size = 10)
		
		# Publisher to make the robot explore the initial room
		self.twist_pub = rospy.Publisher(anm.TOPIC_TWIST, Twist, queue_size = 10)
		
		# Initialize and define the server to build the map
		self.world_srv = rospy.Service(anm.TOPIC_WORLD, WorldInit, self.handle_world_init)
		
		# Initialize and define the client for the recharge service
		rospy.wait_for_service(anm.TOPIC_RECHARGE)
		self.recharge_cli = rospy.ServiceProxy(anm.TOPIC_RECHARGE, SetBool)
		
		# Initialize and define the action client for the move base action service
		self.move_cli = actionlib.SimpleActionClient(anm.ACTION_MOTION, MoveBaseAction)
		self.move_cli.wait_for_server()
			
		
		
	def handle_world_init(self, request):
		""" 
		This service retrieves information about the map from aruco markers detected by the system.
		The collected information is then used to construct an ontology of the environment, enabling
		the robot to navigate and behave appropriately within the indoor setting.
		
		Args:
			self: current class instance.
			request: string and floating-point values are used to indicate the name of the room, its
			spatial coordinates, and its connections to other rooms through the doors that connect them.
		
		Returns:
			response: a boolean value is used to indicate whether the information has been successfully retrieved or not.
		"""
		self._rooms.append(request.room)
		self._rooms_coord.append([request.x, request.y])
		for i in range(len(request.connections)):
			self._connected_to = request.connections[i].connected_to
			self._through_door = request.connections[i].through_door
			new_element = [self._connected_to, self._through_door]
			if new_element not in self._connections:
				self._connections.append(new_element)
			ARGS = ['hasDoor', self._connections[i][0], self._connections[i][1]]
			ontology_manager('ADD', 'OBJECTPROP', 'IND', ARGS)
		self.markers_detected = self.markers_detected + 1
		if self.markers_detected == anm.MARKERS_NUMBER:
			# Debug
			log_msg = (f'\nROOM:\n{self._rooms}\n'
				   f'COORDINATES:\n{self._rooms_coord}\n'
			           f'CONNECTIONS:\n{self._connections}\n')
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self.aruco_detected = True     
		return WorldInitResponse(status = True)
	
		
	def aruco_done(self):
		""" 
		Retrieve the value of the variable that indicates whether all the markers have been detected, and whether
		the information gathered from them can be used to update the ontology. The function will return "True" if
		all the arucos have been detected, and "False" otherwise.
		
		Args:
			self: current class instance.
		
		Returns:
			self.aruco_detected: a boolean value that indicates the current status of aruco detection.
		"""
		return self.aruco_detected
			
		
	def build_environment(self):
		""" 
		This method initializes the environment ontology by using the ARMOR service. Once all the markers have been detected,
		this method is called to upload all the new information about each room to the ontology. By utilizing the ARMOR service,
		it is possible to initialize and define all the necessary components to ensure the correct operation of the program.
		
		Args:
			self: current class instance.
		"""
		# Get the number of rooms
		rooms_number = range(0,len(self._rooms))
		# Get the number of connections
		connections_number = range(0,len(self._connections))
		# Link doors and rooms according to the connections informations
		for con in connections_number:
			ARGS = ['hasDoor', self._connections[con][0], self._connections[con][1]]
			ontology_manager('ADD', 'OBJECTPROP', 'IND', ARGS)
			self._doors.append(self._connections[con][1])
		# Disjoint rooms, doors and robot
		ARGS = self._rooms + self._doors + ['Robot1']
		ontology_manager('DISJOINT', 'IND', '', ARGS)
		# State the robot initial position
		ARGS = ['isIn', 'Robot1', self.charge_loc]
		ontology_manager('ADD', 'OBJECTPROP', 'IND' , ARGS)
		# Get a time in the past (before the timestamp of the robot)
		self.timer_now = str(int(1000000000)) # Make every room URGENT at the beginning  
		# Start the timestamp in every location to retrieve when a location becomes urgent
		for i in rooms_number:
			ARGS = ['visitedAt', self._rooms[i], 'Long', self.timer_now]
			ontology_manager('ADD', 'DATAPROP', 'IND', ARGS)
			# Reason about the ontology to assign the timestamp
			ARGS = ['']
			ontology_manager('REASON', '', '', ARGS)
			ARGS = ['visitedAt', self._rooms[i]]
			last_location = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
			last_location = ontology_format(last_location, 1, 11) 
		# Update the timestamp of corridor 'E' since the robot spawns in it
		self.timer_now = str(int(time.time())) # initial location is not urgent
		ARGS = ['visitedAt', self.charge_loc, 'Long', self.timer_now, last_location[0]]
		ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
		# Save ontology for DEBUG purposes
		#ARGS = [ONTOLOGY_FILE_PATH_DEBUG] # <--- uncomment this line for ontology debug
		#ontology_manager('SAVE', '', '', ARGS) # <--- uncomment this line for ontology debug
		log_msg = Fore.GREEN + f'\nMap has been constructed\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		# Before continuing make the robot rotate on itself to understand the sorroundings
		self.explore_init_room() 
		self.map_completed = True   # Set to True only the one involved in the state
		
		
	def explore_init_room(self):	
		""" 
		This method enables the robot to rotate in place once the ontology has been completely uploaded.
		This rotation takes place within the initial room before the robot starts processing and reasoning
		about the environment. The purpose of this rotation is to help the robot gain a better understanding
		of its surroundings and to allow time for the map to be uploaded in Rviz, thus preventing collisions
		with walls or other obstacles.
		
		Args:
			self: current class instance.
		"""
		cmd = Twist()
		# No linear motion
		cmd.linear.x = 0
		cmd.linear.y = 0
		cmd.linear.z = 0
		# Rotate on itself
		cmd.angular.x = 0
		cmd.angular.y = 0
		cmd.angular.z = -1.6
		# start the timer
		start_time = rospy.get_time()
		log_msg = Fore.GREEN + f'\nMove around to investigate from different angles!\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		# Run the loop for six seconds to allow the ROBOT to undertand its sorroundings
		while rospy.get_time() - start_time < 9:
			self.twist_pub.publish(cmd)
			self.rate.sleep()
		# Stop rotation
		cmd.angular.z = 0
		self.twist_pub.publish(cmd)
		
		
	def world_done(self):
		""" 
		Retrieve the value of the variable that indicates whether the environment has been created
		using the ARMOR service to define the ontology. The function will return "True" if the map
		has been successfully created, and "False" otherwise.
		
		Args:
			self: current class instance.
		
		Returns:
			self.map_completed: a boolean value that indicates the current status of the map generation.
		"""
		return self.map_completed
		
			
	def reason(self):
		""" 
		This method communicates with the pre-existing ontology to gather information and determine the
		robot's next move based on the desired surveillance behavior. Firstly, reachable rooms and their
		status (e.g., ROOM, URGENT, CORRIDOR) are retrieved. Then, each reachable room is checked, and
		the robot will move first to URGENT locations. If no URGENT locations are present, it will stay on CORRIDORS.
		If there are no CORRIDORS, the robot will move to a random ROOM. Finally, the method returns the next
		location that the robot will visit.
		
		Args:
			self: current class instance.
			
		Returns:
			self._next_goal: refers to the next location that the reasoner has determined the robot should reach.
		"""
		# Reset the boolean variables
		self.reset_var()
		# Reason about the onoloy
		ARGS = ['']
		ontology_manager('REASON', '', '', ARGS)
		# Retreive the locations that the robot can reach
		ARGS = ['canReach', 'Robot1']
		can_reach = ontology_manager('QUERY', 'OBJECTPROP', 'IND', ARGS)
		can_reach = ontology_format(can_reach, 32, -1)
		random.shuffle(can_reach) # Make the choice randomic
		log_msg = f'\nREACHABLE LOCATIONS: {can_reach}\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Retrieve the status of the reachable locations
		loc_status = []
		all_status = []
		for loc in range(0, len(can_reach)):
			ARGS = [can_reach[loc], 'false']
			loc_status = ontology_manager('QUERY', 'CLASS', 'IND', ARGS)  
			loc_status = ontology_format(loc_status, 32, -1)
			log_msg = f'\nROOM: {can_reach[loc]} -> STATUS: {loc_status}\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			all_status.append(loc_status)
		# Check the status of the room (i.e. ROOM, CORRIDOR, URGENT)
		urgent_loc = []
		possible_corridor = []
		for sta in range(0, len(all_status)):
			for urg in range(0, len(all_status[sta])):
				# If location is urgent and it is reachable
				if all_status[sta][urg] == 'URGENT':
					urgent_loc.append(can_reach[sta])
				# If location is a corridor and it is reachable
				elif all_status[sta][urg] == 'CORRIDOR':
					possible_corridor.append(can_reach[sta])
		# Retrieve the next location taht will be checked by the robot
		if len(urgent_loc) == 0:
			log_msg = f'\nNO URGENT LOCATIONS\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			if len(possible_corridor) == 0:
				log_msg = f'\nNO REACHABLE CORRIDORS\nCHOOSE A RANDOMIC REACHABLE ROOM\n'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
				self._next_goal = can_reach # take the reachable rooms
			else:
				log_msg = f'\nCORRIDORS: {possible_corridor}\n'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
				self._next_goal = possible_corridor # take the reachable corridors
		else:
			log_msg = f'\nURGENT: {urgent_loc}\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self._next_goal = urgent_loc # take the urgent rooms
		# If the next goal is a list, take only the first one
		if type(self._next_goal) == list:
			self._next_goal = self._next_goal[0]
		self.reasoner_done = True   # Set to True only the one involved in the state
		return self._next_goal
		
		
	def reason_done(self):
		""" 
		This function retrieves the value of the variable that indicates whether the reasoning phase,
		which involves retrieving information from the ontology using the ARMOR service, has been completed.
		The function will return a boolean value of "True" if the reasoning phase is complete, and "False" otherwise.
		
		Args:
			self: current class instance.
		
		Returns:
			self.reasoner_done: a boolean value that indicates the current status of the reasoner.
		
		"""
		return self.reasoner_done
	
	
	def go_to_charge(self):
		""" 
		This function enables the robot to navigate towards the charging station when its battery level is low.
		It sets the charging station as the robot's target location and calls the self.go_to_goal method to reach it.
		The variable charge_reached is set to True once the robot successfully reaches room 'E', indicating that it
		is ready to be charged.
		
		Args:
			self: current class instance.
		"""
		# Reset the boolean variables
		self.reset_var()
		# Set the next location to be the charging station
		self._next_goal = self.charge_loc
		log_msg = Fore.RED + f'\nBattery is low! Moving to charging station!'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		self.go_to_goal()
		# Loops until the plan action service is Not DONE
		while self.move_cli.get_state() != DONE: 
			self.rate.sleep() # Wate time
		# Update the position of the ROBOT in the ontology
		ARGS = ['isIn', 'Robot1', self._next_goal, self._prev_goal]
		ontology_manager('REPLACE', 'OBJECTPROP', 'IND' , ARGS)
		self._prev_goal = self._next_goal
		# Reason about the onoloy
		ARGS = ['']
		ontology_manager('REASON', '', '', ARGS)
		# Retreive the last time the robot moved
		ARGS = ['now', 'Robot1']
		last_motion = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
		last_motion = ontology_format(last_motion, 1, 11)
		# Retreive the last time a specific location has been visited
		ARGS = ['visitedAt', self._next_goal]
		last_location = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
		last_location = ontology_format(last_location, 1, 11) 
		# Update the time
		self.timer_now = str(int(time.time())) 
		# Update the timestamp since the robot moved
		ARGS = ['now', 'Robot1', 'Long', self.timer_now, last_motion[0]]
		ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
		# Update the timestamp since the robot visited the location
		ARGS = ['visitedAt', self._next_goal, 'Long', self.timer_now, last_location[0]]
		ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
		log_msg = f'\nThe ROBOT arrived at the CHARGING STATION'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		self.charge_reached = True   # Set to True only the one involved in the state
		
	def charge_ready(self):
		""" 
		Retrieve the value of the variable indicating if the robot has reached the charging location
		and is ready to be charged. The returned value will be "True" if the charging location is reached,
		and "False" otherwise.
		
		Args:
			self: current class instance.
		
		Returns:
			self.charge_reached: a boolean variable that indicates whether the robot has reached the charging location.
		"""
		return self.charge_reached
		
	
	def battery_callback(self, msg):
		""" 
		This function is responsible for handling the subscription to the topic /state/battery_low in order to get updates on the battery status.
		
		Args:
			self: current class instance.
			msg: refers to the object that is set up to receive and process messages published to the topic /state/battery_low,
			in order to monitor the state of the robot's battery.
		"""
		self.mutex.acquire()    # take the mutex
		try: 
			self.battery_low = msg.data    # change the flag of battery low with the received message
			if self.battery_low == True:
				log_msg = f'\n@@@### BATTERY: LOW! RECHARGE! ###@@@\n'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			if self.battery_low == False:
				log_msg = f'\n@@@### BATTERY: FULL ###@@@\n'
				rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		finally:
			self.mutex.release()    # release the mutex
	
			
	def ret_battery_low(self):
		""" 
		This function retrieves the value of a variable that indicates whether the robot's battery level is low or not.
		The returned value will be "True" if the battery level is low, and "False" otherwise.
		
		Args:
			self: current class instance.
		
		Returns:
			self.battery_low: a boolean value indicating the current status of the battery.
		"""
		return self.battery_low
			
			
	def recharge_srv(self):
		""" 
		This is a service used to recharge the robot's battery. When the battery is low and the robot
		is at the charging location, a request is sent to the service to start charging the battery.
		After a specified time, the service returns a result indicating that the battery is fully charged.
		Once the service has finished, the variable battery_low is set to "False" to indicate that the robot's
		battery is now high. The service is blocking, meaning that the program execution is paused until
		the service returns a result.
		
		Args:
			self: current class instance.
		"""
		request = SetBoolRequest()
		request.data = True
		response = self.recharge_cli(request)
		log_msg = f'\nThe Robot has been recharged! Ready for action!!\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		self.battery_low = False
	
		
	def go_to_goal(self):
		""" 
		This function sets a goal in space based on the decision made by the Reasoner. The coordinates of the goal are
		retrieved and passed to the MoveBase action service, which uses a SLAM algorithm to find a path between the
		robot and the goal, taking into account the environment and obstacles. The robot follows this path and can
		adjust it in real-time if new obstacles are detected by sensors. MoveBase also ensures that the robot stays on the desired path.
		
		Args:
			self: current class instance.
		"""
		# Reset the boolean variables
		self.reset_var()
		# Get the goal and its coordinates
		try:
			room_index = self._rooms.index(self._next_goal)
		except:
			log_msg = Fore.RED + f'Goal cannot be found!\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Reset console color to default
			print('\033[39m') 
		log_msg = Fore.CYAN+ f'\nThe robot is planning to go to:\nRoom: {self._rooms[room_index]}\nCoordinates: ({self._rooms_coord[room_index][0]},{self._rooms_coord[room_index][1]})\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		goal = MoveBaseGoal()
		# Set the desired goal that we want to reach
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = self._rooms_coord[room_index][0]
		goal.target_pose.pose.position.y = self._rooms_coord[room_index][1]
		goal.target_pose.pose.orientation.w = 1
		# Sends the goal to the action server.
		self.move_cli.send_goal(goal)
		
	
	def check_motion(self):
		""" 
		This method verifies if the robot has successfully reached its goal by checking if the state of the
		MoveBase action service is equal to "DONE". Once the robot has reached its goal, the ontology is updated.
		This involves placing the robot in its new location in the ontology, as well as updating the timestamps
		of the robot and goal location to ensure proper behavior.
		
		Args:
			self: current class instance.
		"""
		# Execute only when the plan action service is done
		if self.move_cli.get_state() == DONE and self.battery_low == False:
			log_msg = Fore.GREEN + f'\nThe robot reached final goal!'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Reset console color to default
			print('\033[39m') 
			# Update the position of the ROBOT in the ontology
			ARGS = ['isIn', 'Robot1', self._next_goal, self._prev_goal]
			ontology_manager('REPLACE', 'OBJECTPROP', 'IND' , ARGS)
			self._prev_goal = self._next_goal
			# Reason about the onoloy
			ARGS = ['']
			ontology_manager('REASON', '', '', ARGS)
			# Retreive the last time the robot moved
			ARGS = ['now', 'Robot1']
			last_motion = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
			last_motion = ontology_format(last_motion, 1, 11)
			# Retreive the last time a specific location has been visited
			ARGS = ['visitedAt', self._next_goal]
			last_location = ontology_manager('QUERY', 'DATAPROP', 'IND', ARGS)
			last_location = ontology_format(last_location, 1, 11) 
			# Update the time
			self.timer_now = str(int(time.time())) 
			# Update the timestamp since the robot moved
			ARGS = ['now', 'Robot1', 'Long', self.timer_now, last_motion[0]]
			ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
			# Update the timestamp since the robot visited the location
			ARGS = ['visitedAt', self._next_goal, 'Long', self.timer_now, last_location[0]]
			ontology_manager('REPLACE', 'DATAPROP', 'IND', ARGS)
			# Cancel the goal to be sure
			self.cancel_motion()
			self.motion_completed = True  # Set to True only the one involved in the state
	
		
	def motion_done(self):
		""" 
		Retrieve the value of the variable that indicates whether the robot has reached its destination or not.
		The output value will be "True" if the robot has successfully reached the goal, and "False" if it hasn't reached it yet.
		
		Args:
			self: current class instance.
		
		Returns:
			self.motion_completed: refers to a boolean variable that indicates the status of the robot's movement.
		"""
		return self.motion_completed
		
	
	def cancel_motion(self):
		""" 
		This function stops any current goals that have been sent to the MoveBase action service.
		
		Args:
			self: current class instance.
		"""
		log_msg = Fore.RED + f'Cancel goal\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		# Cancel the goal
		self.move_cli.cancel_all_goals()
		rospy.sleep(1)
				
				
	def reset_var(self):
		""" 
		This function resets all the variables used to determine when a task has completed its execution.
		
		Args:
			self: current class instance.
		"""
		self.mutex.acquire()    # take the mutex
		self.reasoner_done = False
		self.motion_completed = False
		self.charge_reached = False
		self.check_completed = False
		self.mutex.release()    # release the mutex
		
	
	def do_surveillance(self):
		# Reset the boolean variables
		self.reset_var()
		# Surveillance task
		log_msg = f'\nThe robot starts surveilling room: {self._next_goal}\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		joint_angle = 0
		while self.battery_low == False and joint_angle <= 3.1: # If bettery low there won't be surveillance task
			# Rotate the base joint of the robot
			self.pub_base_joint.publish(joint_angle)
			joint_angle += 0.1
			self.rate.sleep()
		# Make the camera look the fron side of the robot
		joint_angle = 0
		self.pub_base_joint.publish(joint_angle)
		if self.battery_low == False:
			log_msg = f'\nThe robot checked location: {self._next_goal}\n\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			self.check_completed = True  # Set to True only the one involved in the state
		else:
			log_msg = Fore.RED + f'\nStop surveilling! Battery low!\n\n'
			rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
			# Reset console color to default
			print('\033[39m') 
			self.check_completed = False  # Set to False since action was not completed
	
	def surveillance_done(self):
		"""
		This function indicates that surveillance is done.

		Returns:
			self.check_completed: refers to a boolean variable that indicates the surveillance is completed.
		"""
		return self.check_completed
		
