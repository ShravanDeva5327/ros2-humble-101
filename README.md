# ROS2 Humble 101 

[![Alt Text 1](atlas2.gif)](https://www.youtube.com/watch?v=-e1_QhJ1EhQ&t=12s)

## Installation

For detailed instructions on installing ROS2 Humble, please refer to the official [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html).

## What is ROS?

ROS, or Robot Operating System, is an open-source framework that provides a collection of libraries and tools to help you build and manage robot applications. It enables developers to modularize a robot’s software into independent processes called nodes, which can communicate with one another using a standardized interface.

For example, imagine a robotic arm in a manufacturing assembly line. In such a project:
- **Modularity**: Each function of the robotic arm—such as movement control, sensor feedback, and error detection—can be developed as separate nodes.
- **Communication**: Nodes exchange information seamlessly via topics (for continuous data, like sensor readings) and services (for specific requests, like executing a movement command).
- **Scalability**: As the project grows, additional capabilities (such as vision processing or advanced control algorithms) can be integrated without a complete redesign.

Don't worry about these new terms; we will discuss them in further sections and you'll quickly get used to them.

## Underlay and Overlay Workspaces

In ROS2, workspaces are structured in a layered manner to help manage and organize packages effectively:

- **Underlay**: This is your foundational workspace. It contains a stable set of ROS2 packages that many other projects or workspaces might depend on.
- **Overlay**: This workspace sits on top of an underlay. It includes additional or custom packages that extend the functionality of the underlay, without altering the underlay's contents.

### Sourcing `setup.bash` Files

After building your workspaces, you need to source the `setup.bash` files to set up your environment correctly. This step ensures that all the ROS2 commands, executables, and packages are available in your current shell session.


```bash
# Source the underlay workspace
source /opt/ros/humble/setup.bash
```
``` bash
# Source the overlay workspace
source /path/to/ws/install/setup.bash
```
Alternatively, you can add these lines to your `~/.bashrc` file to have them sourced automatically in every new terminal session.
``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /path/to/ws/install/setup.bash" >> ~/.bashrc
```

**Pro Tip:**
``` text
Forgetting to source these setup.bash files is a common mistake that often leads to errors such as:
 - ros2: command not found
 - package not found

Always ensure you source the appropriate setup file(s) in every new terminal session before running any ROS2 commands.
```
## Turtlesim
**Turtlesim** is a lightweight simulator designed for learning the basics of ROS 2. It visually demonstrates how ROS 2 operates, helping you understand nodes, topics, and services in a fun and interactive way.

### 1. Installation
Start by sourcing your ROS 2 setup files in a new terminal (not needed if you sourced setup file in bashrc), then install turtlesim:

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

### 2. Starting Turtlesim
Launch the turtlesim node:
```
ros2 run turtlesim turtlesim_node
```
In another terminal (after sourcing ROS 2), run the teleop node to control the turtle:
```
ros2 run turtlesim turtle_teleop_key
```
### 3. Explore ROS Components
You can list nodes, topics, services, and actions to see how components interact:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```
More details about these components are discussed later.

## rqt
**rqt** is a graphical user interface tool that provides a more intuitive way to interact with ROS 2 components, complementing command line operations.

### 1. Installation
Open a new terminal and install rqt along with its plugins:

```bash
sudo apt update
sudo apt install '~nros-humble-rqt*'
```

### 2. Launching rqt
Start rqt by running:

```
rqt
```
Once open, navigate to `Plugins > Services > Service Caller` to interact with ROS 2 services.

### 3. Using rqt to Spawn a New Turtle
- In the Service Caller window, select the /spawn service.
- Enter a unique name for the new turtle (e.g., turtle2) and valid coordinates (e.g., x=1.0, y=1.0).
- Click the Call button to execute the service and spawn a new turtle.

`Note: If you try to spawn a turtle with a name that already exists (like turtle1), you will receive an error.`

## Understanding Nodes
Each node in ROS 2 is designed to perform a single, modular function—like controlling wheel motors or publishing sensor data from a laser range-finder. In a complete robotic system, many nodes work in concert, and a single executable can host one or more nodes.

![Nodes in ROS 2](Nodes1.gif)

A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

### ros2 run
The `ros2 run` command launches an executable from a package. For example, to start the turtlesim node, open a new terminal and run:

```
ros2 run turtlesim turtlesim_node
```
Here, `turtlesim` is the package name and `turtlesim_node` is the executable.
We still don’t know the node name, however. You can find node names by using `ros2 node list`.

### ros2 node list
`ros2 node list` will show you the names of all running nodes.

Open a new terminal while turtlesim is still running in the other one, and enter the following command:
```
ros2 node list
```
The terminal will return the node name:
```
/turtlesim
```
Open another new terminal and start the teleop node, Return to the terminal where you ran ros2 node list and run it again. You will now see the names of two active nodes:
```
/turtlesim
/teleop_turtle
```

### Remapping
Remapping lets you reassign default node properties (like node name, topic names, etc.). For example, to remap the `/turtlesim` node to a custom name `/my_turtle`, run:

```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

After this, `ros2 node list` will display:

```
/my_turtle
/turtlesim
/teleop_turtle
```

### ros2 node info
To see detailed information about a node—its publishers, subscribers, services, and actions—use:
```
ros2 node info <node_name>
```
For example, to inspect the node `my_turtle`:
```
ros2 node info /my_turtle
```

This command displays the node’s ROS graph connections.
```
/my_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /my_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /my_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /my_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /my_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /my_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /my_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

You can run the same command for `/teleop_turtle` to compare their connections.

