#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <vector>
#include <map>
#include <zlgcan/zlgcan.h>
#include "parse_can.h"
#include "detection.pb.h"

static DEVICE_HANDLE  dHandle;
static IProperty *property_can0;
static CHANNEL_HANDLE can0;
void initZLGCAN() {
    dHandle = ZCAN_OpenDevice(ZCAN_USBCAN_4E_U, 0x0, 0);
    if (dHandle == INVALID_DEVICE_HANDLE) {
        printf("ZCAN_OpenDevice failed\n");
    };

    property_can0 = GetIProperty(dHandle);
    can0 = ZCAN_InitCAN(dHandle, 0, NULL);
    if (can0 == INVALID_CHANNEL_HANDLE) {
        printf("ZCAN_InitCAN failed\n");
        ZCAN_CloseDevice(dHandle);
    }
    char path[128];
    snprintf(path, 128, "info/channel/channel_%d/baud_rate", 0);
    property_can0->SetValue(path, "500000");
    ZCAN_StartCAN(can0);
}

void closeZLG()
{
    std::cout << "Close ZLG CAN CARD" << std::endl;
    ZCAN_ResetCAN(can0);
    ReleaseIProperty(property_can0);
}

void sigroutine(int dunno)
{
    switch (dunno) {
    case 1:
        closeZLG();
        exit(0);
        break;
    case 2:
        closeZLG();
        exit(0);
        break;
    }
    return;
}

void parse_obstacles(Detection &det, int num_obstacle, std::map<int, ZCAN_Receive_Data> &can_data) {
    for(int i = 0; i < num_obstacle; i++) {
        if (can_data.find(3 * i) == can_data.end() || can_data.find(3 * i + 1) == can_data.end() || can_data.find(3 * i + 2) == can_data.end()) {
            std::cout << "lost the obstacle " << i << " message" << std::endl;
        }
        Obstacle_Data_A oba = parse_obstacle_a(can_data[3 * i].frame.data);
        Obstacle_Data_B obb = parse_obstacle_b(can_data[3 * i + 1].frame.data);
        Obstacle_Data_C obc = parse_obstacle_c(can_data[3 * i + 2].frame.data);
        Object* obj = det.add_object();
        obj->set_id(oba.id);
        obj->set_type(Object_Type(oba.type));
        obj->set_confidence(obb.confidence / 100.0f);
        Box3D* box = obj->mutable_box();
        Point3D *center = box->mutable_center();
        center->set_x(oba.pos_x);
        center->set_y(oba.pos_y);
        center->set_z(oba.pos_z);
        box->set_length(obb.length);
        box->set_width(obb.width);
        box->set_height(obb.height);
        box->set_heading(obc.angle / 180.0 * 3.14);
        obj->set_velocity_x(oba.rel_vel_x);
        obj->set_velocity_y(0);
    }
    can_data.clear();
}

int main(int argc, char **argv) {
    std::cout << "CAN receive detection results" << std::endl;
    signal(SIGHUP, sigroutine);
    signal(SIGINT, sigroutine);
    initZLGCAN();
    Detection det;
    std::map<int, ZCAN_Receive_Data> can_data;
    int num_obstacle = 0;
    while (true) {
        ZCAN_Receive_Data canReceiveData;
        memset(&canReceiveData, 0, sizeof(ZCAN_Receive_Data));
        int16_t len = ZCAN_Receive(can0, &canReceiveData, 1, 0);
        if (len <= 0) {
            usleep(10000);
            continue;
        }
        if (canReceiveData.frame.can_id == 0x80000568) {
            parse_obstacles(det, num_obstacle, can_data);
            std::cout << det.DebugString() << std::endl;
            det = Detection();
            Obstacle_Status status = parse_obstacle_status(canReceiveData.frame.data);
            Header* head = det.mutable_header();
            head->set_version(std::to_string(status.application_version));
            head->set_timestamp(status.timestamp);
            head->set_relative_timestamp(status.relative_timestamp);
            num_obstacle = status.num_obstacles;
        } else if ((num_obstacle > 0) && ((canReceiveData.frame.can_id - 0x80000569) < (num_obstacle * 3))) {
            int can_id = canReceiveData.frame.can_id - 0x80000569;
            can_data[can_id] = canReceiveData;
        }
    }
}