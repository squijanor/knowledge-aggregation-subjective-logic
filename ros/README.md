# Knowledge Aggregation with Subjective Logic in Multi-Agent Self-Adaptive Cyber-Physical Systems

This repository contains the source code and resources to perform Knowledge Aggregation with Subjective Logic in a ROS-based simulation of Self-Adaptive multi-robot system.

## Setting up the environment

### ROS
The current implementation use ROS Melodic and Gazebo to execute the testbed. To install ROS, you may want to follow the [official installation istructions](http://wiki.ros.org/melodic/Installation/Ubuntu). We recommend to use the full desktop distribution of ROS, since it installs the needed Gazebo resources.

Additionally, you need to install the navigation stack and turtlebot3 ROS packages. Use the following commands to install these packages:
 - Navigation stack: `sudo apt install ros-melodic-navigation`
 - Turtlebot3: `sudo apt install ros-melodic-turtlebot3`

### Subjective Logic library using jpy

Knowledge aggregation is performed using a Subjective Logic library implememted in Java (the source code of the library can be found in the [Subjective Logic library](../subjective_logic)). Since our ROS nodes are written in Python, we need a Java-Python bridge to use such library.

jpy is a bi-directional Python-Java bridge which you can use to embed Java code in Python programs or the other way round. Installation of jpy is currently done by building from sources. To install jpy, follow the most up to date instructions in [installation guide](https://jpy.readthedocs.io/en/latest/install.html).


## Running the simulation

### Building ans sourcing ROS packages

For convinience, we recommend to source your ROS environment and the testbed workspace by adding the following lines to your .bashrc file:
```
`source /opt/ros/melodic/setup.bash`
`source /path/to/workspace/devel/setup.bash`
```

Since we are using turtlebot3 Burger for simulation, we need to add the respective environment varibale:
```
export TURTLEBOT3_MODEL=burger
```

To build the testbed ROS packages:
1. Go to the project workspace root directory.
2. Execute `catkin_make`

### Configuration parameters

In order to change the simulation behavior, we need to configure the following paramaters:
 - `use_sl`. This parameter enable/disable the use of knowledge aggregation. By default this feature is enabled.
 - `sl_operator`. Set the kind of operator to be used for knowledge aggregation. Possible values are:
   - `CBF` for Cumulative Belief Fusion
   - `CCF` for Consensus & Compromise Fusion
 - `sl_classpath`. Path where to look for the subjective library jar file
 - `adaptive_scheduling`. Enable/disable the use of adaptation logic in the simulation. By default, self-adaptation is enabled.
 - `gen_seed`. Seed to be used to generate random locations where to spawn a new task. The default seed is 100.
 - `spawn_interval`. Time interval (in seconds) to create a new task. The default interval is 10s

This parameters are set accordingly in the launch or config files in the following sections.

### Run the simulation using launch files
By using roslaunch tool we can set directly the above mentioned parameters. For example, to simulate a scenario with 2 robots with knowledge aggregation enabled using Cumulative Belief Fusion, and spawn interval of 10s, we could execute the following command in a terminal:

```
roslaunch master.launch use_sl:=true sl_operator:=CBF sl_classpth:=-Djava.class.path=/home/user/sl-java-lib.jar adaptive_scheduling:=true gen_seed:=100 spawn_interval:=10
```

For convinience, bash scripts for different test scenarios are provided in the directory [test](../ros/test) of this repository.

This scripts make use of the launch files contained in the the `robot_meta` and `launch_simulation` ROS packages. If we were to add more robots in the simulation, their initial position, or the robot's simulation descriptors, we need to change these values manually in the corresponding launch file.

