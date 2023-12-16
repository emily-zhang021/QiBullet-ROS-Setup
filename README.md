# QiBullet-ROS-Setup

This tutorial assumes that you have already gone through the steps of setting up a catkin workspace that is ready to use for ROS.

Original QiBullet Repository: https://github.com/softbankrobotics-research/qibullet

If you have not set up your environment yet, please follow this tutorial: https://github.com/rosielab/pepper-ros-joints-package

## Create QiBullet Project

### Install Dependencies
The following modules are required:
numpy, pybullet, pip
```
sudo apt update
sudo apt install pip
pip install numpy
pip install pybullet
```

### Install qiBullet module
```
pip install --user qibullet
```

### Create QiBullet Project
```
catkin_create_pkg qibullet std_msgs rospy 
```

### Navigate to QiBullet Project Folder and Create qi.py 
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
