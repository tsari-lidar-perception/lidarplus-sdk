#include "parse_can.h"

int convert(uint16_t value, int bits) {
    if((value & (1 << (bits - 1))) == 0)
        return (int)value;
    else
        return (int)((int)value - (1 << bits));
}

Obstacle_Status parse_obstacle_status(uint8_t data[8]) {
    Obstacle_Status status;
    status.num_obstacles = data[0];
    status.timestamp = data[1];
    status.relative_timestamp = data[2];
    status.application_version = data[3];
    status.protocol_version = data[4];
    status.close_car = data[5] & 0x01;
    return status;
}

Obstacle_Data_A parse_obstacle_a(uint8_t data[8]) {
    Obstacle_Data_A data_a;
    data_a.id = data[0];
    data_a.pos_x = convert((data[1] & 0x00ff) | ((data[2]  & 0x0f) << 8), 12) * 0.0625f;
    data_a.pos_y = convert(((data[3] & 0x00ff) << 4) | ((data[2] & 0xf0) >> 4), 12) * 0.0625f;
    data_a.pos_z = convert(data[4], 8) * 0.0625f;
    data_a.rel_vel_x = convert((data[5] & 0x00ff) | ((data[6] & 0x0f) << 8), 12) * 0.0625f;
    data_a.type = (data[6] & 0xe0) >> 5;
    data_a.status = (data[7] & 0xe0) >> 5;
    data_a.valid = data[7] & 0x03;
    return data_a;
}

Obstacle_Data_B parse_obstacle_b(uint8_t data[8]) {
    Obstacle_Data_B data_b;
    data_b.length = data[0] * 0.12f;
    data_b.width = data[1] * 0.05f;
    data_b.height = data[2] * 0.05f;
    data_b.age = data[3];
    data_b.confidence = data[4];
    return data_b;
}

Obstacle_Data_C parse_obstacle_c(uint8_t data[8]) {
    Obstacle_Data_C data_c;
    data_c.angle_rate = convert((data[0] & 0x00ff) | ((data[1] & 0x00ff) << 8), 16) * 0.01f;
    data_c.accel_x = convert((data[4] & 0x00ff) | ((data[5] & 0x03) << 8), 10) * 0.03f;
    data_c.replaced = (data[5] & 0x80) >> 7;
    data_c.angle = convert((data[6] & 0x00ff) | ((data[7] & 0x00ff) << 8), 16) * 0.01f;
    return data_c;
}