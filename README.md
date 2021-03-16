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

# CPP Sample
```shell
cpp_sample/recev_detection_udp.cpp
cpp_sample/recev_detection_zlgcan.cpp
```

# Python3 Sample
```shell
python3_sample/rece_detection.py
```