# remote_controller_support

# android_app
Requires installing Android StudioGradle build script will download and link up the dependencies

# ros_app
Requires much more work:

We used Mavlink Kinetic release, which is 4.4.0 version, Mavros is 0.24.0
Get Onboard-SDK-ROS
- https://github.com/dji-sdk/Onboard-SDK-ROS.git

Get MAVROS
- sudo apt-get install geographiclib-tools
- sudo apt-get install libgeographiclib-dev
- rosdep update- sudo apt-get install ros-indigo-geographic-msgs- sudo apt-get install ros-indigo-tf2-eigen
- sudo apt-get install python-rosinstall-generator
- rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
- rosinstall_generator --upstream --rosdistro kinetic mavros | tee -a /tmp/mavros.rosinstall
- wstool merge -t src /tmp/mavros.rosinstall
- wstool update -t src -j4
- rosdep install --from-paths src --rosdistro kinetic --ignore-src -y
- sudo mkdir /usr/share/geographiclib
- sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

To not have issues during compiling, just use one thread
- catkin_make_isolated -j1
- source devel/setup.bash

Get Mavlink and header generation tool
- pip install --user future- sudo apt-get install python-tk

Inside src folder- git clone https://github.com/mavlink/mavlink.git --recursive mavlink_repo
- git clone https://github.com/mavlink/mavlink.git --recursive- add to .bashrc: PYTHONPATH=your_path_to_mavlink_clone
- NOTE: the PYTHONPATH variables needs to be defined before sourcing any workspace
- See: https://answers.ros.org/question/236707/ros-environmental-variable-problem/
- cd your_path_to_mavlink_clone- python mavgenerate.py- Instructions about this here: https://mavlink.io/en/getting_started/generate_source.html

To add a new mavlink dialect- cd mavlink/message_definitions/v1.0
- cd mavlink_repo/message_definitions/v1.0- dji_icg.xml is located inside the camera_drones repository “camera-drones/dji_icg.xml”
- Create a dji_icg.xml, look to other dialects for guidance

Currently a bug in mavgen_java.py- crc.update_checksum(len); is wrong- Change to crc.update_checksum(payloadSize);
Get my repository for code in ROS from ros_app
- git clone into second_workspace/src
- catkin_make
- source devel/setup.bash after Onboard-SDK-ROS repository
