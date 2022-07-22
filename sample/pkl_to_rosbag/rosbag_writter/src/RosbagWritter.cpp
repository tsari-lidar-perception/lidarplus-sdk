#include "RosbagWritter.h"

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>

RosbagWritter::RosbagWritter(std::string file)
{
  mBag = new rosbag::Bag();
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->open(file, rosbag::bagmode::Write);
}
RosbagWritter::~RosbagWritter()
{
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->close();
}

void RosbagWritter::writeScan(std::string topic, const std::string frame, uint64_t timestamp,
                              pcl::PointCloud<pcl::PointXYZI> input)
{
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::PointCloud<pcl::PointXYZI> cloudPcl = input;
  pcl::toROSMsg(cloudPcl, cloudMsg);
  cloudMsg.header.stamp.sec = timestamp / 1000000;
  cloudMsg.header.stamp.nsec = (timestamp % 1000000) * 1000;
  cloudMsg.header.frame_id = frame;
  ros::Time topicTime;
  topicTime.sec = timestamp / 1000000;
  topicTime.nsec = (timestamp % 1000000) * 1000;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, cloudMsg);
}

void RosbagWritter::writeImage(std::string topic, const std::string frame, uint64_t timestamp,
                               cv::Mat input)
{
  // const cv::Mat image = input;
  cv_bridge::CvImage cvi;
  cvi.header.stamp.sec = timestamp / 1000000;
  cvi.header.stamp.nsec = (timestamp % 1000000) * 1000;
  cvi.header.frame_id = frame;
  cvi.encoding = "bgr8";
  cvi.image = input;

  sensor_msgs::Image im;
  cvi.toImageMsg(im);
  ros::Time topicTime;
  topicTime.sec = timestamp / 1000000;
  topicTime.nsec = (timestamp % 1000000) * 1000;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, im);
}

void RosbagWritter::writeImu(std::string topic, const std::string frame, Imu_t &imu)
{
  sensor_msgs::Imu imuMsg;
  ros::Time topicTime;
  topicTime.sec = imu.timestamp / 1000000;
  topicTime.nsec = (imu.timestamp % 1000000) * 1000;

  imuMsg.header.stamp = topicTime;
  imuMsg.angular_velocity.x = imu.gyro_x;
  imuMsg.angular_velocity.y = imu.gyro_y;
  imuMsg.angular_velocity.z = imu.gyro_z;
  imuMsg.linear_acceleration.x = imu.acc_x;
  imuMsg.linear_acceleration.y = imu.acc_y;
  imuMsg.linear_acceleration.z = imu.acc_z;
  auto mBagPtr = (rosbag::Bag *)mBag;
  mBagPtr->write(topic, topicTime, imuMsg);
}