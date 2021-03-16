# LP-01 SDK

# Prepare
git clone https://github.com/tsari-lidar-perception/lidarplus-sdk --recurse-submodules

# Requirement
* libboost_thread
* libboost_system
* proto 3.11+
* ZLG usbcan-4e (optinal)

# Build
```shell
  (Turn on BUILD_ZLGCAN in CMakeLists.txt, if you wants to build ZLGCAN receive sample)
  mkdir build
  cd build
  cmake ..
  make -j
```

# Cpp Sample
```shell
cpp_sample/recev_detection_udp.cpp
cpp_sample/recev_detection_zlgcan.cpp
```

# Python Sample
python3_sample/rece_detection.py
