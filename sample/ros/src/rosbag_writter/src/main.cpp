#include "RosbagWritter.h" 
#include "getpkldata.h"

#include <string>
#include <vector>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py=pybind11;


int main() {
  py::scoped_interpreter python;
  std::vector<std::string> filelists;
  filelists = GetFiles();

  RosbagWritter wbag("rosbag");
  for(auto c: filelists){
    py::dict data_dict = get_pkldata(c);

    for(auto it: data_dict["points"].attr("keys")()){
      std::string lidarname =  it.cast<std::string>();
      py::dict points_obj = data_dict["points"];
      pcl::PointCloud<pcl::PointXYZI> points_cloud = toPclPointCloud(points_obj[lidarname.c_str()].cast<py::array_t<float>>());

      int pos = 0;
      lidarname = "lidar" + lidarname;
      while (std::string::npos != (pos = lidarname.find("-")) )
      {
        lidarname.erase(pos, 1);
      }
      wbag.writeScan(lidarname, "000000", data_dict["frame_start_timestamp"].cast<uint64_t>(), points_cloud);
    }

    for(auto it: data_dict["image"].attr("keys")()){
      std::string imagename =  it.cast<std::string>();
      py::dict image_obj = data_dict["image"].cast<py::dict>();
      py::bytes image_bytes = image_obj[imagename.c_str()].cast<py::bytes>();
      cv::Mat image_ = cv::imdecode(cv::Mat(1, py::len(image_bytes), CV_8UC1, &image_bytes), CV_LOAD_IMAGE_UNCHANGED);

      imagename = "image" + imagename;
      wbag.writeImage(imagename, "000000", data_dict["frame_start_timestamp"].cast<uint64_t>(), image_);
    }

  }
  return 0;
}
