# <img src="https://user-images.githubusercontent.com/67557966/206814922-97bb77f5-e22d-4c57-9305-a03ac2743efd.png" width="5%" height="5%"> Robot watchdog in an indoor environment with Gazebo and RVIZ

A second ROS-based exercise for the Experimental Robotics Laboratory course held at the University of Genoa.

Author: [Jabrail Chumakov](https://github.com/jabrail-chumakov)

## <img src="https://user-images.githubusercontent.com/67557966/206814981-d1297a1d-dcdd-403e-af7c-09a4381b28a2.png" width="5%" height="5%"> Documentation

The documentation can be found [here](https://jabrail-chumakov.github.io/Robot-watchdog-in-an-indoor-environment/) and [here](https://jabrail-chumakov.github.io/Gazebo-robot-watchdog-in-an-indoor-environment/).

## <img src="https://user-images.githubusercontent.com/67557966/206815462-747ddfde-04db-4143-837a-256282f3651d.png" width="5%" height="5%"> Introduction

<p align="center">
  <img width="500" height="300" src="https://user-images.githubusercontent.com/67557966/222198118-b5776692-0ae3-4105-ba64-2bf31c8ff02c.jpg">
</p>

This is a software architecture that uses ROS to simulate a surveillance robot in an indoor environment. The robot uses aruco markers to understand its surroundings, including room names, coordinates, and connections to other areas. The robot autonomously moves around the map to perform surveillance tasks until its battery becomes low. It then goes to a charging location and returns to the surveillance behavior once the battery is full. The program interacts with an ontology to retrieve necessary information for its behavior. The main focus is on the gazebo simulator, with a finite state machine implemented by the **fsm.py** node to show the current state, important actions, and transitions. The **aruco_detection.cpp** and **camera_movement.cpp** nodes show information about detected markers and the robot's arm motion. The Rviz screen displays what the robot sees through its camera. The **battery_status.py** node generates a battery_low signal and manages the charging action. The user interface is kept simple to avoid confusion.

The task is to create a robot that can move through the corridors of a building and periodically enter each room. The robot has a battery that drains when it moves through a room. The main goal is to use Smach, a tool for creating finite state machines, to adjust the robot's behavior based on its location and battery level. This will help the robot navigate the environment more efficiently. The environment is generally separated into the following 7 rooms and 7 doors:
* **Room E:** The robot's starting location, which also houses the charging station.
* **Corridor 1:** Next to room E, accessible from another corridor through doors D7 or D5.
* **Corridor 2:** Located next to room E and accessible by door D6 or door D5 from another corridor. 
* **Room R1:** A room that can be entered from corridor 1 through door D1. 
* **Room R2:** A room that can be accessed from corridor 1 through door D2.
* **Room R3:** This room is reachable from corridor 2 via door D3.
* **Room R4:** This room is reachable from corridor 2 via door D4.
<p align="center">
  <img width="400" height="400" src="https://user-images.githubusercontent.com/67557966/222202796-7fdb0c64-ad27-4c1b-a029-79824974bc29.png">
</p>

## <img src="https://user-images.githubusercontent.com/67557966/206816524-f958630f-b19d-4195-89b2-6fd6371d400b.png" width="5%" height="5%"> Installation

Follow these steps to install the software.
 - Install required repositories such as [ARMOR](https://github.com/EmaroLab/armor), [ROSJAVA](https://github.com/EmaroLab/armor) and [Topological Map](https://github.com/buoncubi/topological_map)
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`) and name it `assignment2`.
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - You also need to install a few third-party libraries such as [colorama](https://pypi.org/project/colorama/) and [xterm](https://installati.one/ubuntu/21.04/xterm/) that I utilized for this project. 

Colorama is a Python library that allows you to print colored text on terminals that support ANSI escape sequences. This can be useful for creating colorful output in your Python programs, especially when working in the command line. XTerm is a terminal emulator for the X Window System. It is a program that allows users to interact with the Unix-like operating system using a command-line interface. XTerm supports a wide range of features, including color schemes, scrollback buffer, customizable fonts and key bindings, and support for international character sets. 

- After that you can write the following command to execute launch file:

```bash
$ roslaunch assignment2 assignment.launch
```

The software exploits [roslaunch](http://wiki.ros.org/roslaunch) and 
[rospy](http://wiki.ros.org/rospy) for using python with ROS. Rospy allows defining ROS nodes, 
services and related messages.

Also, the software uses [actionlib](http://wiki.ros.org/actionlib/DetailedDescription) to define
action servers. In particular, this software is based on 
[SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137).
Thus, you should use the [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html)
to solve the exercise.

The Finite States Machine that I used in this project based on [SMACH](http://wiki.ros.org/smach). You can check the 
[tutorials](http://wiki.ros.org/smach/Tutorials) related to SMACH, for an overview of its 
functionalities. In addition, you can exploit the [smach_viewer](http://wiki.ros.org/smach_viewer)
node to visualize and debug the implemented Finite States Machine.

## <img src="https://user-images.githubusercontent.com/67557966/222184317-0c0f1c59-65ca-4a94-a54f-3d9f202b3e23.png" width="5%" height="5%"> Folder Organization

This GitHub repository includes a ROS package called **assignment2** which contains various components:
- [launch/](launch/): Includes the necessary setup to launch its components.
  - [assignment.launch](launch/assignment.launch): main launch file which run all required scripts and processes.
- [docs/](docs/): Includes package's HTML documentation.
- [msg/](msg/): Includes the messages exchanged through ROS topics.
  - [RoomConnection.msg](msg/RoomConnection.msg): Specifies the connections for each location.
- [param/](param/): Includes parameters files used by move base to facilitate the robot's movement in the environment.
- [scripts/](scripts/): Implementation of each software component.
  - [battery_status.py](scripts/battery_status.py): Managing the amount of power that the robot has available.
  - [fsm.py](scripts/fsm.py): Сreates the state machine that represents the final software architecture.
- [src/](src/): Contains the C++ files.
  - [aruco_detection.cpp](src/aruco_detection.cpp): Permits the identification of ArUco markers that have been positioned in the surrounding area.
  - [camera_movement.cpp](src/camera_movement.cpp): Enables the robot arm to be manipulated in order to detect markers.
  - [marker_server.cpp](src/marker_server.cpp): The server needed to obtain data from ArUco has already been given.
- [srv/](srv/): Contains the service files.
  - [RoomInformation.srv](srv/RoomInformation.srv): Transmits the identification number of the marker that was detected and receives information about the surrounding environment.
- [topological_map/](topological_map/): Includes an initial ontology that is adjusted during the initial phase to create a new environment for the task at hand.
- [urdf/](urdf/): The robot and its arm's **urdf** files.
- [utilities/assignment2/](utilities/assignment2/): There are additional Python files included that are utilized by the scripts located in the **scripts** folder.
  - [fsm_helper.py](utilities/assignment2//fsm_helper.py): Methods that are invoked in the **fsm.py** node to enhance the readability and organization of the code.
  - [architecture_name_mapper.py](utilities/assignment2//architecture_name_mapper.py): Lists the names of all the nodes, topics, servers, actions, and parameters used in this architecture, if applicable.
- [worlds/](worlds/): Ccontains the world definition used.
- [CMakeLists.txt](CMakeLists.txt): File to configure this package.
- [README.md](README.md): README file of this repository.
- [package.xml](package.xml): File to configure this package.
- [setup.py](setup.py): File to **import** python modules from the **utilities** folder into the files in the **script** folder. 

## <img src="https://user-images.githubusercontent.com/67557966/206817137-e914ebdf-1bc6-4565-a21d-ece2751eb222.png" width="5%" height="5%"> Software 

The software architecture for the surveillance robot consists of multiple scripts. Two of these scripts are written in Python and are named **fsm.py** and **battery_status.py**. Additionally, there is a Python script called **fsm_helper.py** that contains all the methods used in **fsm.py**. Furthermore, there are three other nodes written in C++ named **aruco_detection.cpp**, **camera_movement.cpp**, and **marker_server.cpp**.

### State diagram

<p align="center">
  <img width="400" height="300" src="https://user-images.githubusercontent.com/67557966/222208835-cdd51540-f91a-4b21-8bb6-b2127271a400.png">
</p>

The state machine consists of a total of seven states, which are as follows:
- **Build World** refers to a state in which the Tbox of an ontology is loaded and modified in order to create a specific environment based on a request. This process results in the creation of the Abox of the ontology. If necessary, the ontology can be saved for debugging purposes by removing the comment symbol from a few lines of code in the **fsm_helper.py** script.
- **Reasoner** is a state in which the ontology is queried to obtain important information required for the robot's surveillance behavior. During this state, the reachable rooms are examined and the robot decides which room to visit next based on factors such as its level of urgency or the type of location.
- **Motion** is a state where a path is calculated from the current location of the robot to its desired destination. This is accomplished by using the move base algorithm, which has the ability to adjust the path if new obstacles are detected along the way. The **Motion** state also controls the movement of the robot, ensuring that it follows the path generated by the algorithm.
- **Surveillance** is a state where the robot scans the room with its camera after arriving at a new location. However, it is important to note that this implementation is not functional, as the robot simply rotates its camera 360 degrees without any actual ability to react to stimuli. 
- **Reach Charge** is a state that causes the robot to move towards the charging location when its battery level becomes low. In this state, the next location that the robot needs to reach is set as the charging location **E**. The **go_to_goal()** method, which is also used in the **Motion** state, is then called to direct the robot towards the target location. The purpose of the **Reach Charge** state is to ensure that the robot can recharge its battery when necessary.
- **Charge** is a state where the robot charges its battery after it has become low. This state is implemented using a blocking service that simulates the recharge action for a real battery. While in this state, time is wasted as the robot charges its battery until the timer expires, at which point the battery becomes fully charged. The purpose of the **Charge** state is to ensure that the robot's battery is replenished so that it can continue to operate effectively.

### Temporal diagram
<p align="center">
  <img width="1000" height="450" src="https://user-images.githubusercontent.com/67557966/222238095-d1361e02-0c2f-4664-bd96-3f5c183ba930.png">
</p>

The temporal diagram illustrates the sequence of events that occur in a robotic architecture from the moment it is launched. The nodes shown in the diagram begin running and communicating with each other to perform various tasks. Gazebo provides information to **slam_gmapping** algorithm to update the map while also sending joint state information to **camera_movement.cpp** to move the robot arm's joints. **aruco_detection.cpp** acquires camera images, processes them to detect aruco markers, and sends their IDs to **marker_server.cpp** to retrieve environmental information, which is then sent to **fsm.py** to build an ontology of the environment. Once all markers are detected, the build map phase begins, and the ontology is loaded to create an Abox. The robot then enters the surveillance behavior phase, where it queries the ontology to retrieve information about its location and uses a reasoner method to implement a surveillance policy. Once a **battery_low = True** signal is issued, the robot goes to the charging location, and a charging request is sent to **battery_status.py**. Once the battery is fully charged, the robot returns to the surveillance behavior phase until a new **battery_low = True** signal is issued.

## <img src="https://user-images.githubusercontent.com/67557966/206817629-1e16e808-8c7f-4814-b2d1-e462ca7293b0.png" width="5%" height="5%"> Video demonstration

[<img src="https://user-images.githubusercontent.com/67557966/206757072-fc9b14c4-52ad-4594-9308-0a2355f47035.png" width="70%">](https://www.youtube.com/watch?v=_LHYHsRIdUc&ab_channel=JabrailChumakov)

## <img src="https://user-images.githubusercontent.com/67557966/206817931-2a30ae94-d10d-43d4-8492-70423cd0d2be.png" width="5%" height="5%"> Working hypothesis

The robot is initially placed in the **E** corridor at coordinates **[x: -6.0 | y: 11]** on the map frame. Before moving, the robot scans its surroundings using a camera mounted on its arm to detect seven markers. Once all markers are detected, the robot updates its ontology and scans the environment using a laser scanner. 

Based on a specific policy, the robot decides which room to visit next and navigates autonomously using the SLAM algorithm. The map is updated in real-time and the planned path may change based on any detected obstacles. When the robot reaches its destination, it performs a surveillance task by rotating its arm 360 degrees to scan the entire room. The robot then repeats the reasoning phase and moves to the next location according to the surveillance policy. When the battery is low, the robot navigates to the charging location in the **E** corridor and simulates charging until the battery is fully charged. The surveillance policy prioritizes locations that have not been visited in a long time and are considered urgent based on a threshold value. The robot mainly stays in corridors while moving. In order to create a simpler model of a surveillance robot, certain assumptions were made during the project's development:
- The robot moves in a static environment with no moving obstacles. 
- Each room has one door, and corridors have at least two. 
- The robot starts and recharges at a predetermined location. 
- Battery low signals are random and immediate, but do not interrupt ongoing reasoning. 
- The robot scans target locations with a 360-degree camera rotation. 
- Battery recharge is simulated as a waste of time. 
- All locations are initially marked as urgent, except for the starting location. 
- Aruco markers in the environment provide complete information. The environment is colored white to aid aruco marker detection.
 
### <img src="https://user-images.githubusercontent.com/67557966/206818143-1b16e859-2602-46ee-b252-ebc4e1dc0ad7.png" width="5%" height="5%"> System's limitations

While a robot deployed in an indoor environment for surveillance purposes may have a number of useful features, it is also likely to have some limitations. Some potential limitations of such a system could include:

- The environment is restricted to one floor and lacks stairs or slopes, limiting the robot's mobility.
- The fixed structure limits the number of rooms, corridors, and doors, making the map static and reducing the possibility of triggering alert signals.
- The robot model in the simulation is not representative of a real mobile robot and serves solely for testing purposes.
- After detecting all initial aruco markers, the nodes for aruco detection are disabled, leaving the robot unable to detect any subsequent markers.
- The robot deems a location urgent based solely on whether it has not been visited during the assigned time slot, ignoring other potential stimuli.
- The robot can only assess the urgency of adjacent locations that it can reach at a specific time.
- The robot's battery charge is a waste of time and restricts its actual task performance.

### <img src="https://user-images.githubusercontent.com/67557966/206818238-659a2844-463a-446a-b274-e113975b4cab.png" width="5%" height="5%"> Possible technical improvements

To enhance the software architecture, some limitations in the system can be addressed by adopting more practical assumptions. A list of potential suggestions is as follows:

- Create a dynamic environment to evaluate the robot's performance by allowing it to conduct actual surveillance activities and send out an alert when a mobile entity is detected.
- Replace the **planar move** controller with a more practical one, such as a **skid drive** controller.
- Install an actual battery in the robot to enable it to recharge when the battery runs low.
- Develop a more realistic robot model, using an existing model implemented by developers, which can also be used in real-life scenarios.
