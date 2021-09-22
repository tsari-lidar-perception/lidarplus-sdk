
#ifndef __ROSBAG_WRITTER__H
#define __ROSBAG_WRITTER__H

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>

class RosbagWritter
{
public:
  RosbagWritter(std::string file);
  virtual ~RosbagWritter();
  void writeScan(std::string topic, const std::string frame, uint64_t timestamp,
                 pcl::PointCloud<pcl::PointXYZI> input);
  void writeImage(std::string topic, const std::string frame, uint64_t timestamp,
                  cv::Mat input);

private:
  void *mBag;
};

#endif //__ROSBAG_WRITTER__H