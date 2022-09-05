# LP-01 SDK

# Prepare
```shell
git clone https://github.com/tsari-lidar-perception/lidarplus-sdk --recurse-submodules
```

# Requirement
* libboost_thread
* libboost_system
* proto 3.11+
* ZLG usbcan-4e (optional)

# Build
```shell
(Turn on BUILD_ZLGCAN in CMakeLists.txt, if you wants to build ZLGCAN receive sample)

mkdir build && cd build
cmake ..
make -j
```

# RUN
## 1.Receive Object Detection
cpp samples
```shell
build/sample/udp_recv_sample/cpp/recv_detection_udp
build/sample/can_recv_sample/recv_detection_zlgcan
```
python samples
```shell
python3 sample/udp_recv_sample/python/recv_detection.py
```
## 2.Receive Localization
cpp samples
```shell
build/sample/udp_recv_sample/cpp/recv_localization_udp
```
# ROS Sample
```shell
cd sample/ros
catkin_make
source devel/setup.bash
roslaunch rosbag_proxy rosbag_proxy.launch
rosbag play xxx.bag
```