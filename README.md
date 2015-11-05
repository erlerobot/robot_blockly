# ROSimple

ROSimple is a ROS package that provides visualization and block-based programming tools for robots and drones that use the Robot Operating System.

![](img/ROSimple-peek.png)
![](img/ROSimple-code.png)

### Robots where ROSimple has been implemented:
- [Erle-Spider](http://erlerobotics.com/blog/product/erle-spider-the-ubuntu-drone-with-legs/)

### Installation:
```
cd <catkin_ws_root>/src
git clone https://github.com/erlerobot/ros_rosimple
cd ros_rosimple/frontend
git clone https://github.com/erlerobot/blockly
git clone https://github.com/erlerobot/ace-builds
cd ../scripts
# make sure that Apache is running properly in your robot and then
./deploy.sh

cd <catkin_ws_root>
source install_isolated/setup.bash
catkin_make_isolated --pkg ros_rosimple

# now go to http://192.168.1.68/frontend/pages/blockly.html
#  and start playing!

```

### License
ROSimple is build based on [blockly](http://github.com/erlerobot/blockly) and [ACE](http://github.com/erlerobot/ace-builds) and other open source frameworks. Unless specified, the code is freed under a GPLv3 License.