#include "RosbagWritter.h"
#include "GetPklData.h"

#include <string>
#include <vector>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>
namespace py = pybind11;

int main(int argc, char *argv[])
{
  py::scoped_interpreter python;
  std::vector<std::string> filelists;

  int opt;
  int option_index = 0;

  std::string input_p;
  std::string output_p;
  struct option long_options[] = {
      {"input_path", optional_argument, NULL, 'i'},
      {"output_path", optional_argument, NULL, 'o'},
      {0, 0, 0, 0}};

  while ((opt = getopt_long(argc, argv, "i:o:", long_options, &option_index)) != -1)
  {
    switch (opt)
    {
    case 'i':
      input_p = optarg;
      break;
    case 'o':
      output_p = optarg;
      break;
    }
  }

  filelists = getFiles(input_p);
  if (filelists.empty())
  {
    printf("[ERROR] No .pkl file found!\n");
    return 0;
  }

  std::string bag_name = output_p + std::string("rosbag.bag");

  RosbagWritter wbag(bag_name);
  for (auto c : filelists)
  {
    auto file_path = input_p + c;
    py::dict data_dict = getPklData(file_path);

    for (auto it : data_dict["points"].attr("keys")())
    {
      std::string lidarname = it.cast<std::string>();
      py::dict points_obj = data_dict["points"];
      pcl::PointCloud<pcl::PointXYZI> points_cloud = toPclPointCloud(points_obj[lidarname.c_str()].cast<py::array_t<float>>());

      int pos = 0;
      lidarname = "lidar" + lidarname;
      while (std::string::npos != (pos = lidarname.find("-")))
      {
        lidarname.erase(pos, 1);
      }
      wbag.writeScan(lidarname, "base_link", data_dict["frame_start_timestamp"].cast<uint64_t>(), points_cloud);
    }

    for (auto it : data_dict["image"].attr("keys")())
    {
      std::string imagename = it.cast<std::string>();
      py::dict image_obj = data_dict["image"];
      py::bytes image_bytes = image_obj[imagename.c_str()].cast<py::bytes>();

      cv::Mat cvmat_1c = toCvMatImage(image_bytes);
      cv::Mat image_ = cv::imdecode(cvmat_1c, CV_LOAD_IMAGE_UNCHANGED);

      imagename = "image" + imagename;
      wbag.writeImage(imagename, "base_link", data_dict["frame_start_timestamp"].cast<uint64_t>(), image_);
    }
  }
  return 0;
}
