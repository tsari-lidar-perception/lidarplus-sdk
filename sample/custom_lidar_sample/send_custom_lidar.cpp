
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include "UDPServer.h"

#pragma pack(1)
struct CustomLidarPackage {
  char head[2];
  char version[2];
  uint32_t frame_id;
  uint64_t timestamp;
  uint32_t point_num;
  float points_buf[4 * 74];
  char tail[2];
};
#pragma pack()

#define POINT_MAX 500000

uint64_t getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return static_cast<uint64_t>(tv.tv_sec * 1000000 + tv.tv_usec);
}

int main(int argc, char **argv) {
    std::string test_pcd = "custom_lidar_sample/test.raw";
    FILE* fp = nullptr;
    fp = fopen(test_pcd.c_str(), "rb");
    if (nullptr == fp) {
        std::cout << "can not open file: " << test_pcd << std::endl;
        return -1;
    }
    
    float* point_array = new float[3 * POINT_MAX];
    int array_size = fread(point_array, sizeof(float), 3 * POINT_MAX, fp);
    fclose(fp);

    std::unique_ptr<UDPServer> udp_server(new UDPServer(12436));
    int frame_id = 0;
    while (true) {
        CustomLidarPackage lidarPackage;
        lidarPackage.head[0] = ' ';
        lidarPackage.head[1] = 'e';
        lidarPackage.version[0] = '1';
        lidarPackage.version[1] = '0';
        lidarPackage.tail[0] = 'c';
        lidarPackage.tail[1] = 'n';
        lidarPackage.frame_id = frame_id;
        int point_num = 0;
        for (int index = 0; index < array_size; index = index + 3) {
            lidarPackage.points_buf[point_num * 4 + 0] = point_array[index + 0];
            lidarPackage.points_buf[point_num * 4 + 1] = point_array[index + 1];
            lidarPackage.points_buf[point_num * 4 + 2] = point_array[index + 2];
            lidarPackage.points_buf[point_num * 4 + 3] = 0;
            point_num++;
            if (point_num == 74 || (index == (array_size - 3))) {
                lidarPackage.timestamp = getCurrentTime();
                lidarPackage.point_num = point_num;
                point_num = 0;
                char buf[1206] = {0};
                memcpy(buf, &lidarPackage, sizeof(CustomLidarPackage));
                udp_server->UDPSendtoBuf("127.0.0.1", 2688, buf, 1206);
                usleep(15); //avoid buffer overrun
            }
        }
        frame_id++;
        std::cout << "frame id: " << frame_id << std::endl;
    }
    delete[] point_array;
}