# QiBullet-ROS-Setup

## 1. Install Ubuntu VirtualBox

Download VirtualBox https://www.virtualbox.org/
Download image https://releases.ubuntu.com/focal/

Create a new VM for Ubuntu 20.04

Note: This will not work for Ubuntu 22.04.
## 2. Setup Ubuntu Environment 
Adapted from https://wiki.ros.org/noetic/Installation/Ubuntu
### 2.1 Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
### 1.3 Set up your keys
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 1.4 Installation
First, make sure your Debian package index is up-to-date:
```
sudo apt update
```

Desktop-Full Install ros-noetic. This is to also install the packages that are needed to run the simulated Pepper. This may take a while. 
```
sudo apt install ros-noetic-desktop-full
```

### 1.5 Environment setup
You must source this script in every **bash** terminal you use ROS in.
source /opt/ros/noetic/setup.bash
In Bash, this automatically sources the script every time a new shell is launched. 
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.6 Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, [rosinstall](https://wiki.ros.org/rosinstall) is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

#### 1.6.1 Initialize rosdep
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## 2 Set up ROS Workspace for NaoQi Driver

### 2.1 Create Catkin Workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```
### 2.2 Set up Environment
Source your new setup.* sh file and make sure ROS_PACKAGE_PATH environment variable includes the directory you're in to make sure your workspace is properly overlayed by the setup script
```
source devel/setup.bash
echo $ROS_PACKAGE_PATH /home/youruser/catkin_ws/src:/opt/ros/noetic/share
```
### 2.3 Set Up NaoQi Driver 
Install NaoQi Driver Dependencies
```
sudo apt-get update
sudo apt-get install -y git-all
sudo apt-get install ros-noetic-naoqi-libqi
sudo apt-get install ros-noetic-naoqi-libqicore
sudo apt-get install ros-noetic-naoqi-bridge-msgs
sudo apt install net-tools
```

Clone Naoqi Driver into directory
```
git clone https://github.com/ros-naoqi/naoqi_driver.git ~/catkin_ws/src/naoqi_driver
```

Source ROS script 
```
source /opt/ros/noetic/setup.bash
```

Install Naoqi Driver
```
rosdep install -i -y --from-paths ~/catkin_ws/src/naoqi_driver
. /opt/ros/noetic/setup.sh
```

You may have to install and update rosdep before this step, if you get errors.

```
sudo apt install python3-rosdep2
rosdep update
```

Make the project 
```
catkin_make
```
## 3 Create QiBullet Project

### 3.1 Install Dependencies
The following modules are required:
numpy, pybullet, pip
```
sudo apt update
sudo apt install pip
pip install numpy
pip install pybullet
```

### 3.2 Install qiBullet module
```
pip install --user qibullet
```

### 3.3 Create QiBullet Project
```
catkin_create_pkg qibullet std_msgs rospy 
```

### 3.4 Navigate to QiBullet Project Folder and Create qi.py 
```
roscd qibullet
```
NOTE: (if roscd doesn't work, try running: )
```
. ~/catkin_ws/devel/setup.bash
```

Create a file called "qi.py" with the following code snippet (adapted from qibullet robot_ros_test.py)
```
#!/usr/bin/env python
# coding: utf-8

import sys
import rospy
import pybullet
import pybullet_data
from qibullet import NaoVirtual
from qibullet import RomeoVirtual
from qibullet import PepperVirtual
from qibullet import NaoRosWrapper
from qibullet import RomeoRosWrapper
from qibullet import PepperRosWrapper
from qibullet import SimulationManager

if __name__ == "__main__":
    simulation_manager = SimulationManager()

    if (sys.version_info > (3, 0)):
        rob = input("Which robot should be spawned? (pepper/nao/romeo): ")
    else:
        rob = raw_input("Which robot should be spawned? (pepper/nao/romeo): ")

    client = simulation_manager.launchSimulation(gui=True)

    if rob.lower() == "nao":
        wrap = NaoRosWrapper()
        camera_id = NaoVirtual.ID_CAMERA_TOP
        robot = simulation_manager.spawnNao(client, spawn_ground_plane=True)
    elif rob.lower() == "pepper":
        wrap = PepperRosWrapper()
        camera_id = PepperVirtual.ID_CAMERA_BOTTOM
        robot = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    elif rob.lower() == "romeo":
        wrap = RomeoRosWrapper()
        camera_id = RomeoVirtual.ID_CAMERA_DEPTH
        robot = simulation_manager.spawnRomeo(client, spawn_ground_plane=True)
    else:
        print("You have to specify a robot, pepper, nao or romeo.")
        simulation_manager.stopSimulation(client)
        sys.exit(1)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    urdf = pybullet.loadURDF(
        "samurai.urdf",
        globalScaling=1.0)

    wrap.launchWrapper(robot, "/naoqi_driver")

    handle = robot.subscribeCamera(camera_id)

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass
    except rospy.ROSInterruptException:
        pass
    finally:
        wrap.stopWrapper()
        simulation_manager.stopSimulation(client)
```
Make it executable
```
chmod +x qi.py
```
Modify your CMakeLists file to add the following:
```
catkin_install_python(PROGRAMS qi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
### 3.5 Navigate to catkin_ws and make the project
```
catkin_make
```

### 3.6 Run your roscore and run the project
```
roscore
```
In a new terminal (if not already in the folder): 
```
cd ~/catkin_ws
```
If needed, source the ROS workspace
```
. ~/catkin_ws/devel/setup.bash
```
Run the project you just created
```
rosrun qibullet qi.py
```
When prompted, run Pepper and press "Enter"

![Pasted image 20231130134813](https://github.com/emily-zhang021/QiBullet-ROS-Setup/assets/52023695/0fd3b031-6596-4a08-b034-f7082c1ab7c2)

If running for the first time, accept the agreements to install the robot meshes onto your computer. 

If installation was successful, Pepper will appear on another window.

![Pasted image 20231130135032](https://github.com/emily-zhang021/QiBullet-ROS-Setup/assets/52023695/edb6c0fb-aeb2-457d-a850-33d00c839b51)
