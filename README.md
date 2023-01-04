# Roboteq Controller ROS [![Badge License]][License]
*ROS driver for RoboteQ motor controller via serial communication*
- *This repository was mirrored from DoanNguyenTrong's fork of the original repository from Roboteq-Inc ([link](https://github.com/DoanNguyenTrong/roboteq_controller_ros)), hence contain previous commit history.*
- *This repository was not forked as there is no intention to merge with the original branch*
- *The packages were tested using SBL2360T controller via serial communication.*
#

## Notes

### From Roboteq-Inc

- This ROS driver only supports firmware version 2.0 or 2.0+.
- You can check your firmware version from Roborun+ console tab by querying - "?fid".
- If firmware is not the latest one then please update it with the latest one available on Roboteq website or contact "techsupport.roboteq@mail.nidec.com".
- This repository contains the ROS driver for Roboteq controllers. The package requires ROS system to be installed properly to your system  and proper connection of Roboteq controller.
- The roboteq driver is designed to be dynamic and users can publish the controller queries as per their requirements. The publishing queries is not limited to any value. Users can change or add queries in configuration file. For that go to config/query.yaml

### From DoanNguyenTrong

- The original code worked at a fixed rate of 5 Hz while querying system's states. He did major mofifications in his work, making it work at any frequency you want to.
- The original code did also specified 3 separate query frequencies in query.yaml: frequencyH, frequencyL, and frequencyG. However, it's not the case (or quite complicated). In his work, he cleaned all of it and only keep a default `frequency` for all queries as it is sufficient for it to work.
- He only used the `driver.launch`, hence `diff_odom` is kept as original. Later, he might make it work, but probably by modifying the `roboteq_controller_node`, not putting in a separated file to make it a little bit efficent.

## Added features
- Two new branches were created from `FW2.1` branch, namely `ps-ros1` and `ps-ros2` for ROS1 and ROS2 development respectively. The `ps-ros1` and `ps-ros2` branch were developed on ROS Melodic and ROS2 Foxy.
- Removed unnecessary files and cleaned some functions.
- Gear reduction (motor to wheel) parameter for closed loop rpm calculation is added in `roboteq_controller_node`.
- Added `roboteq_proc_node` which publishes 4 new topics: proc/digital_input, proc/digital_output, proc/fault_flag, proc/runtime_status_flag, proc/battery_level. Digital input, digital output, fault flag, and runtime status flag are published in a structured boolean form instead of integer. Battery level can be mapped from volts reading and published according to your hardware.


## Installation

1.  Clone the repository
```shell
cd catkin_ws/src/
git clone -b ps-ros1 https://github.com/prostraintech/roboteq_controller_ros.git
```

2. Build the packages 
```shell
cd catkin_ws
catkin_make
source devel/setup.bash
```

3. Launch the packages 
```shell
roslaunch roboteq_controller driver.launch
```

<!----------------------------------------------------------------------------->
[Badge License]: https://img.shields.io/github/license/prostraintech/roboteq_controller_ros
[License]: LICENSE