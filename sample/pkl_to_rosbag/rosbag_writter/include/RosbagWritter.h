
#ifndef __ROSBAG_WRITTER__H
#define __ROSBAG_WRITTER__H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>

struct Imu_t {
  Imu_t() {
    gyro_x = 0;
    gyro_y = 0;
    gyro_z = 0;
    acc_x = 0;
    acc_y = 0;
    acc_z = 0;
    timestamp = 0;
  }
  double gyro_x;           // rad / s
  double gyro_y;           // rad / s
  double gyro_z;           // rad / s
  double acc_x;            // m / s^2
  double acc_y;            // m / s^2
  double acc_z;            // m / s^2
  uint64_t timestamp;      // us
};

class RosbagWritter
{
public:
  RosbagWritter(std::string file);
  virtual ~RosbagWritter();
  void writeScan(std::string topic, const std::string frame, uint64_t timestamp,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr input);
  void writeImage(std::string topic, const std::string frame, uint64_t timestamp,
                  cv::Mat input);
  void writeCompressedImage(std::string topic, const std::string frame, uint64_t timestamp,
                            cv::Mat input);
  void writeImu(std::string topic, const std::string frame, Imu_t &imu);

private:
  void *mBag;
};

#endif //__ROSBAG_WRITTER__H