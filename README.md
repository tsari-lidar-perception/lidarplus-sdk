# ECN-SDK

# Prepare
git clone https://github.com/tsari-ecn/ecn-sdk.git --recurse-submodules

# Requirement
libboost_thread
libboost_system
proto

# Build
  mkdir build

  cd build

  cmake ..

  make -j

# Cpp Sample
cpp_sample/recev_detection.cpp

# Python Sample
python3_sample/rece_detection.py
