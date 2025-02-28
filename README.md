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
