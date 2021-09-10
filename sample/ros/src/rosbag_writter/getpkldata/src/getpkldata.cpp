#include "getpkldata.h"

namespace py=pybind11;


std::vector<std::string> GetFiles()
{   
    const char *src_dir;
    const char *ext;
    src_dir = "./";
    ext = ".pkl";
    std::vector<std::string> result;
    std::string directory(src_dir);
    std::string m_ext(ext);
    //printf("ext length:%d\n",m_ext.length());
 
    DIR *dir = opendir(src_dir);
    if ( dir == NULL )
    {
        printf("[ERROR] %s is not a directory or not exist!", src_dir);
        return result;
    }

    struct dirent* d_ent = NULL;
    char dot[3] = ".";
    char dotdot[6] = "..";

    while ( (d_ent = readdir(dir)) != NULL )
    {
        if ( (strcmp(d_ent->d_name, dot) != 0) && (strcmp(d_ent->d_name, dotdot) != 0) )
        {
            if ( d_ent->d_type != DT_DIR)
            {
                std::string d_name(d_ent->d_name);
                if (strcmp(d_name.c_str () + d_name.length () - m_ext.length(), m_ext.c_str ()) == 0)
                {
                    result.push_back(std::string(d_ent->d_name));
                }
            }
        }
    }
    // sort the returned files
    sort(result.begin(), result.end());
    closedir(dir);
    return result;
}


py::dict get_pkldata(std::string fliename)
{   
    // py::scoped_interpreter python;
    py::object open = py::module::import("builtins").attr("open");
    py::object pickle = py::module::import("pickle");
    py::object file = open(fliename, "rb");
    py::object data_dict = pickle.attr("load")(file);
    file.attr("close")();
    py::dict data=data_dict.cast<py::dict>();
    return data;
}


pcl::PointCloud<pcl::PointXYZI> toPclPointCloud(py::array_t<float> input) {
  auto ref_input = input.unchecked<2>();
  
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for(int i = 0; i < ref_input.shape(0); i++){
    pcl::PointXYZI point;
    point.x = ref_input(i, 0);
    point.y = ref_input(i, 1);
    point.z = ref_input(i, 2);
    point.intensity = 0;
    cloud.push_back(point);
    }
  return cloud;
}

// F_TEST
// int main() {
//   py::scoped_interpreter python;
//   std::vector<std::string> filelists;
//   filelists = GetFiles();

//   for(auto c: filelists){
//     py::dict data_dict = get_pkldata(c);

//     for(auto it: data_dict["points"].attr("keys")()){
//       std::string lidarname =  it.cast<std::string>();
//       py::dict points_obj = data_dict["points"];
//       pcl::PointCloud<pcl::PointXYZI> points_cloud = toPclPointCloud(points_obj[lidarname.c_str()].cast<py::array_t<float>>());
//     }

//     for(auto it: data_dict["image"].attr("keys")()){
//       std::string imagename =  it.cast<std::string>();
//       py::dict image_obj = data_dict["image"].cast<py::dict>();
//       py::bytes image_bytes = image_obj[imagename.c_str()].cast<py::bytes>();
//       cv::Mat image_ = cv::imdecode(cv::Mat(1, py::len(image_bytes), CV_8UC1, &image_bytes), CV_LOAD_IMAGE_UNCHANGED);
//   }
//   return 0;
//   }
// }