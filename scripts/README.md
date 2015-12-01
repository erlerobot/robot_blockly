### Requirements:
```
# a ROS distro installed
sudo apt-get install python3-pip
pip3 install autobahn
pip3 install asyncio
pip3 install catkin_pkg rospkg
```

#### Installing the package
```bash
# Clone the source under catkin_ws/src
# Then:

root@localhost:/root/catkin_ws# catkin_make_isolated --pkg ros_blockly --install
Base path: /root/catkin_ws
Source space: /root/catkin_ws/src
Build space: /root/catkin_ws/build_isolated
Devel space: /root/catkin_ws/devel_isolated
Install space: /root/catkin_ws/install_isolated

==> Processing catkin package: 'ros_blockly'
==> Creating build directory: 'build_isolated/ros_blockly'
==> Building with env: '/root/catkin_ws/install_isolated/env.sh'
==> cmake /root/catkin_ws/src/ros_blockly -DCATKIN_DEVEL_PREFIX=/root/catkin_ws/devel_isolated/ros_blockly -DCMAKE_INSTALL_PREFIX=/root/catkin_ws/install_isolated -G Unix Makefiles in '/root/catkin_ws/build_isolated/ros_blockly'
-- The C compiler identification is GNU 4.7.3
-- The CXX compiler identification is GNU 4.7.3
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Using CATKIN_DEVEL_PREFIX: /root/catkin_ws/devel_isolated/ros_blockly
-- Using CMAKE_PREFIX_PATH: /root/catkin_ws/install_isolated;/opt/ros/indigo
-- This workspace overlays: /root/catkin_ws/install_isolated;/opt/ros/indigo
-- Found PythonInterp: /usr/bin/python (found version "2.7.6") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /root/catkin_ws/build_isolated/ros_blockly/test_results
-- Looking for include file pthread.h
-- Looking for include file pthread.h - found
-- Looking for pthread_create
-- Looking for pthread_create - not found
-- Looking for pthread_create in pthreads
-- Looking for pthread_create in pthreads - not found
-- Looking for pthread_create in pthread
-- Looking for pthread_create in pthread - found
-- Found Threads: TRUE  
-- Found gtest sources under '/usr/src/gtest': gtests will be built
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.6.14
-- Configuring done
-- Generating done
-- Build files have been written to: /root/catkin_ws/build_isolated/ros_blockly
==> make -j4 -l4 in '/root/catkin_ws/build_isolated/ros_blockly'
==> make install in '/root/catkin_ws/build_isolated/ros_blockly'
Install the project...
-- Install configuration: ""
-- Installing: /root/catkin_ws/install_isolated/_setup_util.py
-- Installing: /root/catkin_ws/install_isolated/env.sh
-- Installing: /root/catkin_ws/install_isolated/setup.bash
-- Installing: /root/catkin_ws/install_isolated/setup.sh
-- Installing: /root/catkin_ws/install_isolated/setup.zsh
-- Installing: /root/catkin_ws/install_isolated/.rosinstall
-- Installing: /root/catkin_ws/install_isolated/lib/pkgconfig/ros_blockly.pc
-- Installing: /root/catkin_ws/install_isolated/share/ros_blockly/cmake/ros_blocklyConfig.cmake
-- Installing: /root/catkin_ws/install_isolated/share/ros_blockly/cmake/ros_blocklyConfig-version.cmake
-- Installing: /root/catkin_ws/install_isolated/share/ros_blockly/package.xml
-- Installing: /root/catkin_ws/install_isolated/share/ros_blockly/scripts/server.py
<== Finished processing package [7 of 12]: 'ros_blockly'
```

#### Installing apache and getting the files deployed
```
sudo apt-get install apache2
cd <dir-to-blockly>/web
git clone http://github.com/erlerobot/blockly
cd ../scripts
./deploy.sh
```
