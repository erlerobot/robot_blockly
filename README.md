# ROSimple

ROSimple is a ROS package that provides visualization and block-based programming tools for robots and drones that use the Robot Operating System.

![](img/ROSimple-peek.png)
![](img/ROSimple-code.png)

### Robots where ROSimple has been implemented:
- [Erle-Spider](http://erlerobotics.com/blog/product/erle-spider-the-ubuntu-drone-with-legs/)

### Installation:
#### Frontend
```
cd <catkin_ws_root>/src
git clone https://github.com/erlerobot/ros_rosimple
cd ros_rosimple/frontend
git clone https://github.com/erlerobot/blockly
git clone https://github.com/erlerobot/ace-builds
cd ../scripts

# install apache
sudo apt-get install apache2
# make sure that Apache is running properly in your robot and then
./deploy.sh

```

#### Backend
```
# Install dependencies
sudo pip3 install rosdep rosinstall_generator wstool rosinstall
sudo pip3 install autobahn
cd <catkin_ws_root>
catkin_make_isolated --pkg rosimple --install
source install_isolated/setup.bash
rosrun rosimple rosimple_backend.py

# now go to http://erle-brain-2.local/
#  and start playing!

```

### Create your own blocks
- Open `frontend/demos/blockfactory/index.html`
- Design you own block and then add the metadata to: `frontend/blockly/blocks` and `frontend/blockly/generator`
- Launch `python build.py` to regenerate the blocks.

### License
ROSimple is build based on [blockly](http://github.com/erlerobot/blockly) and [ACE](http://github.com/erlerobot/ace-builds) and other open source frameworks. Unless specified, the code is freed under a GPLv3 License.
