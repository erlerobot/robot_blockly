# robot_blockly

------

This package has been renamed to meet the ROS naming conventions http://wiki.ros.org/ROS/Patterns/Conventions

------

`robot_blockly` is a ROS package that provides web-based visualization and block programming tools for robots and drones.

*Note: The package has been renamed from ros_rosimple to robot_blockly to meet http://wiki.ros.org/ROS/Patterns/Conventions*
![](img/ROSimple-peek.png)
![](img/ROSimple-code.png)

### Installation:

Pre-requisites:

```
sudo apt-get install python-pip
sudo pip install rosdep rosinstall_generator wstool rosinstall
sudo pip install autobahn trollius
```

Then create a new workspace:
```
mkdir -p blockly/src
cd blockly/src
wstool init

If you have added an ssh key for your machine to your github account, use:
wstool set robot_blockly --git git@github.com:shadow-robot/robot_blockly

Otherwise run:
wstool set robot_blockly --git https://github.com/shadow-robot/robot_blockly.git

wstool up
```

To build the workspace (or rebuild it - you can do this while the robot_blockly_backend node is running!):
```
cd ../
catkin_make_isolated --pkg robot_blockly --install
```

To start the backend and build in HTTP server:
```
source install_isolated/setup.bash 
roslaunch robot_blockly robot_blockly.launch 
```

Point your browser to: [http://localhost:8000/pages/blockly.html](http://localhost:8000/pages/blockly.html)

### Create your own blocks
- Open [http://localhost:8000/blockly/demos/blockfactory/index.html](http://localhost:8000/blockly/demos/blockfactory/index.html)
- Design you own block and then add the metadata to: `frontend/blockly/blocks` and `frontend/blockly/generator`
- Launch `python build.py` to regenerate the blocks.

### License
blockly has been built based on [blockly](http://github.com/erlerobot/blockly), [ACE](http://github.com/erlerobot/ace-builds) and Bootstrap. Refer to their sources for the corresponding licenses.

Unless specified, the rest of the code is freed under a GPLv3 License.

### Documentation
- [Erle Robotics blockly docs](http://erlerobotics.com/docs/ROS/Blockly/Intro.html)
- [ROS Wiki](http://wiki.ros.org/blockly)


# Robots where blockly has been implemented:
- [Erle-Spider](http://erlerobotics.com/blog/product/erle-spider-the-ubuntu-drone-with-legs/)
- [Erle-Rover](https://erlerobotics.com/blog/product/erle-rover/) (Work in progress)

Do you need help getting your robot supported? Launch your questions at http://forum.erlerobotics.com.
