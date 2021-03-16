#ifndef __PARSE_CAN_H
#define __PARSE_CAN_H

#include <vector>
#include <stdint.h>

struct Obstacle_Status
{
    uint8_t num_obstacles;
    uint8_t timestamp;
    uint8_t relative_timestamp;
    uint8_t application_version;
    uint8_t protocol_version;
    uint8_t close_car;
};

struct Obstacle_Data_A
{
    uint8_t id;
    float pos_x;
    float pos_y;
    float pos_z;
    float rel_vel_x;
    uint8_t type;
    uint8_t status;
    uint8_t valid;
};

struct Obstacle_Data_B
{
    float length;
    float width;
    float height;
    uint8_t age;
    uint8_t confidence;
};

struct Obstacle_Data_C
{
    float angle_rate;
    float accel_x;
    uint8_t replaced;
    float angle;
};

struct Frame_Obstacle
{
    Obstacle_Status status;
    std::vector<Obstacle_Data_A> obas;
    std::vector<Obstacle_Data_B> obbs;
    std::vector<Obstacle_Data_C> obcs;
};

Obstacle_Status parse_obstacle_status(uint8_t data[8]);
Obstacle_Data_A parse_obstacle_a(uint8_t data[8]);
Obstacle_Data_B parse_obstacle_b(uint8_t data[8]);
Obstacle_Data_C parse_obstacle_c(uint8_t data[8]);

#endif //__PARSE_CAN_H