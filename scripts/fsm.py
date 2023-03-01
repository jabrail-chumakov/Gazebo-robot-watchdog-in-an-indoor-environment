#!/usr/bin/env python

"""
.. module:: fsm
	:platform: Unix
	:synopsis: Python module for the management of the battery status
   
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>


This is a description of a ROS node developed for the second assignment of the Experimental Robotics course
in the Robotics Engineering Master program. The software architecture is designed to create a Final State Machine
that controls the behavior of a surveillance robot deployed in an indoor environment. The robot's objective
is to visit different locations such as rooms and corridors and spend some time in each. The robot starts at
the charging location and waits until it receives instructions to build the topological map. It then moves to
new locations, performs surveillance tasks, and checks other locations. This behavior continues until the
program is shut down. When the robot's battery is low, it returns to the charging location and waits
before resuming the same behavior. The robot prioritizes staying in corridors and visits a room that
has not been visited for a while.The node includes defined subscriptions, publishers, services, and
service actions used in the final_state_machine.py helper node.
		
"""

# Import necessary modules and libraries
import rospy
import smach
import time
import random
import roslib
import smach_ros
from std_msgs.msg import String, Float64, Bool, Float32
from assignment2.fsm_helper import Helper

# Import colorama to make colorful text printed on to console
from colorama import Fore, Back, Style

# Import constant name defined to structure the architecture
from assignment2 import architecture_name_mapper as anm

# The list of names that identify the states of the Finite State Machine.
STATE_CHARGE = 'CHARGE'                # State where the robot recharges its battery.
STATE_BUILD_WORLD = 'BUILDWORLD'       # State where the environment is build using informations from the aruco.
STATE_REASONER = 'REASONER'            # State that decides the next location that will be visisted.
STATE_MOTION = 'MOTION'                # State that allows the robot to move in the environement.
STATE_REACH_CHARGE = 'REACHCHARGE'     # State used to let the robot reach the charging station.
STATE_SURVEILLANCE = 'SURVEILLANCE'    # State that checks the location in which the root stops.


# The list of names that identify the transitions of the Finite State Machine.
TRANS_BATTERY_LOW = 'battery_low'      # The transition from the `REASONER`, 'MOTION' and `SURVEILLANCE` states toward the `REACHCHARGE` state.
TRANS_BATTERY_OK = 'battery_ok'        # The transition from the `CHARGE` state to the `REASONER` state.
TRANS_CHECK_LOC = 'check_loc'          # The transition from the `MOTION` state to the `SURVEILLANCE` state.
TRANS_INFO_DONE = 'info_done'          # The transition from the `REASONER` state to the `MOTION` state.
TRANS_WORLD_DONE = 'world_done'        # The transition from the `BUILDWORLD` state with to the 'REASONER' state.
TRANS_CHARGE_ON = 'charge_on'          # The transition from the 'REACHCHARGE' state toward the `CAHRGE` state.
TRANS_CHECK_DONE = 'check_done'        # The transition from the 'SURVEILLANCE' state toward the `REASONER` state.


# Initialize and define the tag for identifying logs producer.
LOG_TAG = anm.NODE_STATE_MACHINE										  

class BuildWorld(smach.State):
	""" 
	Class defining BUILDWORLD.
	"""
	def __init__(self, helper):
		""" 
		This is a method that sets the state to BUILDWORLD.
		
		Args:
			self: current class instance.
			helper: this refers to an allocated instance of the Helper() class located in the state_machine_helper.py file.
		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper
								 
	def execute(self, userdata):
		""" 
		This is a method that runs before leaving the BUILDWORLD state. Its purpose is to create the environment by
		retrieving information from aruco markers. The method loops until all 7 markers have been detected.
		Once detected, the build_environment() method is called to add information to the existing ontology using
		the Armor server. When the environment is created, the program transitions to the next state.
		
		Args:
			self: current class instance.
			userdata: userdata between the states of the finite state machine.
		Returns:
			TRANS_WORLD_DONE: transition from the BUILDWORLD to the REASONER state.
		
		"""
		log_msg = Fore.GREEN + f'\n\nThe state of "BUILD WORLD" is being executed.\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG)) 
		# Reset console color to default
		print('\033[39m') 
		r = rospy.Rate(10)
		while not self._helper.aruco_done():
			# Wait until all arucos have been detected before building the world
			r.sleep()
		# Build the world once the markers have been detected
		self._helper.build_environment()     
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.world_done():
					return TRANS_WORLD_DONE
			finally:
					self._helper.mutex.release()
					

					
class Charge(smach.State):
	""" 
	Class defining CHARGE.
	"""
	def __init__(self, helper):
		""" 
		This is a method that sets the state to CHARGE.
		
		Args:
			self: current class instance.
			helper: this refers to an allocated instance of the Helper() class located in the state_machine_helper.py file.
		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper

	def execute(self, userdata):
		""" 
		This is a method that runs before leaving the CHARGE state. Its purpose is to recharge the robot by calling
		the recharge_srv() method defined in the helper node. The method continues to recharge the battery until it
		is fully charged, at which point the program transitions to the next state.
		
		Args:
			self: current class instance.
			userdata: userdata between the states of the finite state machine.
		Returns:
			TRANS_BATTERY_OK: transition from the CHARGE to the REASONER state.
		
		"""
		log_msg = Fore.GREEN + f'\n\nThe state of "CHARGE" is being executed.\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		self._helper.recharge_srv()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if not self._helper.ret_battery_low():
					return TRANS_BATTERY_OK
			finally:
				self._helper.mutex.release()			

			

class ReachCharge(smach.State):
	""" 
	Class defining REACH_CHARGE.
	"""
	def __init__(self, helper):
		""" 
		This is a method that sets the state to REACH_CHARGE.
		
		Args:
			self: current class instance.
			helper: this refers to an allocated instance of the Helper() class located in the state_machine_helper.py file.
		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper
			
	def execute(self, userdata):
		""" 
		This is a method that runs before leaving the REACH_CHARGE state. Its purpose is to guide the robot to the charging
		location 'E' by calling the go_to_charge() method defined in the helper node. The method continues to guide the robot
		until it reaches the charging location, at which point the program transitions to the next state.
		
		Args:
			self: current class instance.
			userdata: userdata between the states of the finite state machine.
		Returns:
			TRANS_CHARGE_ON: transition from the REACHCHARGE to the CHARGE state.
		
		"""
		log_msg = Fore.GREEN + f'\n\nThe state of "REACH CHARGE" is being executed.\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		self._helper.go_to_charge()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.charge_ready():
					return TRANS_CHARGE_ON
			finally:
				self._helper.mutex.release()
		


class Reasoner(smach.State):
	""" 
	Class defining REASONER.
	"""
	def __init__(self, helper):
		""" 
		This is a method that sets the state to REASONER.
		
		Args:
			self: current class instance.
			helper: this refers to an allocated instance of the Helper() class located in the state_machine_helper.py file.
		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper
			
	def execute(self, userdata):
		""" 
		This is a method that runs before leaving the REASONER state. Its purpose is to enable the robot to reason and
		achieve the desired behavior for the surveillance robot by calling the reason() method defined in the helper node.
		After querying the ontology, the battery level is checked. If the battery level is low during execution, the next
		state to be executed will be REACHCHARGE. Otherwise, the MOTION state will be executed.
		
		Args:
			self: current class instance.
			userdata: userdata between the states of the finite state machine.
		Returns:
			TRANS_BATTERY_LOW: transition from the REASONER to the REACHCHARGE state.
			TRANS_INFO_DONE: transition from the REASONER to the MOTION state.
		go 
		"""
		log_msg = Fore.GREEN + f'\n\nThe state of "REASONER" is being executed.\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		goal_location = self._helper.reason()
		log_msg = f'NEXT GOAL: {goal_location}\n\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.ret_battery_low():
					self._helper.cancel_motion()
					return TRANS_BATTERY_LOW
				if self._helper.reason_done():
					return TRANS_INFO_DONE
			finally:
				self._helper.mutex.release()        		



class Motion(smach.State):
	""" 
	Class defining MOTION.
	"""
	def __init__(self, helper):
		""" 
		This is a method that sets the state to MOTION.
		
		Args:
			self: current class instance.
			helper: this refers to an allocated instance of the Helper() class located in the state_machine_helper.py file.
		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper 
			
	def execute(self, userdata):
		""" 
		This is a function that runs before leaving the MOTION state. Its purpose is to enable the robot to create a path
		from its current position to the desired position retrieved from the REASONER. It also ensures that the robot follows
		the programmed path throughout the entire motion. The robot travels autonomously from its current position to the target
		using a SLAM algorithm to locate itself in the environment and map the surroundings. The action is completed when the
		robot reaches the target. If the battery level is low during execution, the next state to be executed will be REACHCHARGE.
		Otherwise, the SURVEILLANCE state will be executed.
		
		Args:
			self: current class instance.
			userdata: userdata between the states of the finite state machine.
		Returns:
			TRANS_BATTERY_LOW: transition from the MOTION to the REACHCHARGE state.
			TRANS_CHECK_LOC: transition from the MOTION to the SURVEILLANCE state.
		
		"""
		log_msg = Fore.GREEN + f'\n\nThe state of "MOTION" is being executed.\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		self._helper.go_to_goal()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				self._helper.check_motion()
				if self._helper.ret_battery_low():
					self._helper.cancel_motion()
					return TRANS_BATTERY_LOW
				if self._helper.motion_done():
					return TRANS_CHECK_LOC
			finally:
				self._helper.mutex.release()



class Surveillance(smach.State):
	""" 
	Class defining SURVEILLANCE.
	"""
	def __init__(self, helper):
		""" 
		This is a method that sets the state to SURVEILLANCE.
		
		Args:
			self: current class instance.
			helper: this refers to an allocated instance of the Helper() class located in the state_machine_helper.py file.
		"""
		smach.State.__init__(self, outcomes = [TRANS_BATTERY_LOW, TRANS_BATTERY_OK, TRANS_CHECK_LOC, TRANS_INFO_DONE, TRANS_WORLD_DONE, TRANS_CHARGE_ON, TRANS_CHECK_DONE])
		self._helper = helper 
		
	def execute(self, userdata):
		""" 
		This is a method that runs before leaving the SURVEILLANCE state. Its purpose is to simulate a surveillance task
		when the robot reaches a specific location. The method rotates the base joint of the robot's arm 360 degrees while
		the camera at the top of the arm captures images. During execution, the battery level is checked. If the battery
		level is low, the next state to be executed will be REACHCHARGE. Otherwise, the REASONER state will be executed.
		
		Args:
			self: current class instance.
			userdata: userdata between the states of the finite state machine.
		Returns:
			TRANS_BATTERY_LOW: transition from the SURVEILLANCE to the REACHCHARGE state.
			TRANS_CHECK_DONE: transition from the SURVEILLANCE to the REASONER state.
		
		"""
		log_msg = Fore.GREEN + f'\n\nThe state of "SURVEILLANCE" is being executed.\n'
		rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
		# Reset console color to default
		print('\033[39m') 
		self._helper.do_surveillance()
		while not rospy.is_shutdown():
			self._helper.mutex.acquire()
			try:
				if self._helper.ret_battery_low():
					self._helper.cancel_motion()
					return TRANS_BATTERY_LOW
				if self._helper.surveillance_done():
					return TRANS_CHECK_DONE
			finally:
				self._helper.mutex.release()


			
def main():
	"""
	This method initializes the Final State Machine of the node state_machine.py using the SMACH modules.
	The SMACH modules provide a framework for creating hierarchical state machines in ROS. The state_machine.py
	node depends on the state_machine_helper.py node, which contains a Helper() instance that is passed to every state of the FSM. 
    """
	rospy.init_node(anm.NODE_STATE_MACHINE, log_level = rospy.INFO)
	
	helper = Helper()
	
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes = ['container_interface_surv'])
	sm.userdata.sm_counter = 0

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(STATE_BUILD_WORLD, BuildWorld(helper),
							transitions={TRANS_BATTERY_LOW:STATE_BUILD_WORLD,
								     TRANS_CHARGE_ON:STATE_BUILD_WORLD, 
								     TRANS_BATTERY_OK:STATE_BUILD_WORLD,
								     TRANS_CHECK_LOC:STATE_BUILD_WORLD,
								     TRANS_INFO_DONE:STATE_BUILD_WORLD,
								     TRANS_WORLD_DONE:STATE_REASONER,
								     TRANS_CHECK_DONE:STATE_BUILD_WORLD})
																										
		smach.StateMachine.add(STATE_CHARGE, Charge(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_CHARGE,
								     TRANS_CHARGE_ON:STATE_CHARGE,
								     TRANS_BATTERY_OK:STATE_REASONER,
								     TRANS_CHECK_LOC:STATE_CHARGE,
							             TRANS_INFO_DONE:STATE_CHARGE,
								     TRANS_WORLD_DONE:STATE_CHARGE,
								     TRANS_CHECK_DONE:STATE_CHARGE})
													
		smach.StateMachine.add(STATE_REASONER, Reasoner(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE,
								     TRANS_CHARGE_ON:STATE_REASONER, 
								     TRANS_BATTERY_OK:STATE_REASONER,
								     TRANS_CHECK_LOC:STATE_REASONER,
								     TRANS_INFO_DONE:STATE_MOTION,
								     TRANS_WORLD_DONE:STATE_REASONER,
								     TRANS_CHECK_DONE:STATE_REASONER})
													
		smach.StateMachine.add(STATE_MOTION, Motion(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE, 
								     TRANS_CHARGE_ON:STATE_MOTION,
							             TRANS_BATTERY_OK:STATE_MOTION,
								     TRANS_CHECK_LOC:STATE_SURVEILLANCE,
								     TRANS_INFO_DONE:STATE_MOTION,
								     TRANS_WORLD_DONE:STATE_MOTION,
								     TRANS_CHECK_DONE:STATE_MOTION})
													
		smach.StateMachine.add(STATE_REACH_CHARGE, ReachCharge(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE, 
								     TRANS_CHARGE_ON:STATE_CHARGE,
								     TRANS_BATTERY_OK:STATE_REACH_CHARGE,
								     TRANS_CHECK_LOC:STATE_REACH_CHARGE,
								     TRANS_INFO_DONE:STATE_REACH_CHARGE,
								     TRANS_WORLD_DONE:STATE_REACH_CHARGE,
								     TRANS_CHECK_DONE:STATE_REACH_CHARGE})
										
		smach.StateMachine.add(STATE_SURVEILLANCE, Surveillance(helper), 
							transitions={TRANS_BATTERY_LOW:STATE_REACH_CHARGE, 
								     TRANS_CHARGE_ON:STATE_SURVEILLANCE,
								     TRANS_BATTERY_OK:STATE_SURVEILLANCE,
								     TRANS_CHECK_LOC:STATE_SURVEILLANCE,
								     TRANS_INFO_DONE:STATE_SURVEILLANCE,
								     TRANS_WORLD_DONE:STATE_SURVEILLANCE,
								     TRANS_CHECK_DONE:STATE_REASONER})
										  
	# Create and start the introspection server for visualization
	sis = smach_ros.IntrospectionServer('server_surv', sm, '/SM_ROOT_SURV')
	sis.start()

	# Execute the state machine
	outcome = sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()