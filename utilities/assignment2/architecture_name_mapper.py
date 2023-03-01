#!/usr/bin/env python
import rospy

# Define the location in which the robot starts
INIT_LOCATION = 'E'

# Define the location in which the robot goes to recharge itself.
CHARGE_LOCATION = 'E'

# Define the number of markers that needs to be detected
MARKERS_NUMBER = 7

# The boolean parameter to active random testing.
# If the value is `False` a keyboard-based interface will be used to produce stimulus 
# (i.e., battery signals). Instead, random stimulus will be generated if `True`. In the 
# latter case, the architecture also requires all the parameters with the scope 
# `test/random_sense/*`, which are not used if the boolean parameter is `False`.
PARAM_RANDOM_ACTIVE = 'test/random_sense/active'
# ---------------------------------------------------------


# The name of the node representing the shared knowledge required for this scenario.
NODE_ROBOT_BATTERY_STATE = 'robot-battery-state'

# The name of the topic where the joint base command is published
TOPIC_JOINT_BASE = '/robot/joint1_position_controller/command'

# The name of the topic where to make the robot move in the initial room
TOPIC_TWIST = '/cmd_vel'

# The name of the topic where the battery state is published.
TOPIC_BATTERY_LOW = 'state/battery_low'

# The name of the service solving the recharge of the robot.
TOPIC_RECHARGE = 'state/recharge'

# The name of the service to use the MoveBase action service
ACTION_MOTION = '/move_base'

# The delay for the battery to become low, i.e., from high to low.
# It should be a list `[min_time, max_time]`, and the battery level change
# will occur after a random number of seconds within such an interval.
PARAM_BATTERY_TIME = 'test/random_sense/battery_time'

# The delay of the charging time regarding the battery, i.e., from low to high.
# It should be a list `[min_time, max_time]`, and the battery will
# be charged after a random number of seconds within such an interval.
PARAM_BATTERY_CHARGE = 'test/random_sense/battery_charge'
# ---------------------------------------------------------


# The name of the node representing the shared knowledge required for this scenario.
NODE_STATE_MACHINE = 'state-machine'

# The name of the service solving the initialization of the world.
TOPIC_WORLD = '/world_init'
# ---------------------------------------------------------

# Function used to label each log with a producer tag.
def tag_log(msg, producer_tag):
    return '@%s>> %s' % (producer_tag, msg)
