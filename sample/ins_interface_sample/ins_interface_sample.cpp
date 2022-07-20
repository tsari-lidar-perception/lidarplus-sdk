
#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "UDPServer.h"

struct InsDataType {
    std::string header; // header, $GPCHC
    int gps_week;       // weeks from 1980-1-6
    double gps_time;    // seconds from 0:00:00 of this sunday
    double heading;     // [deg]
    double pitch;       // [deg]
    double roll;        // [deg]
    double gyro_x;      // optional, [deg/s]
    double gyro_y;      // optional, [deg/s]
    double gyro_z;      // optional, [deg/s]
    double acc_x;       // optional, [g]
    double acc_y;       // optional, [g]
    double acc_z;       // optional, [g]
    double latitude;    // [deg]
    double longitude;   // [deg]
    double altitude;    // [deg]
    double Ve;          // optional, [m/s]
    double Vn;          // optional, [m/s]
    double Vu;          // optional, [m/s]
    double baseline;    // optional, [m] baseline length
    int NSV1;           // optional, number of satellite of major antenna
    int NSV2;           // optional, number of satellite of secondary antenna
    int Status;         // optional, INS status
    int age;            // optional, latency of INS
    int Warnning;       // optional, INS Warnning
    std::string Cs;     // optional, verification
};

#define SECS_PER_WEEK (60L*60*24*7)
#define LEAP_SECOND 18

uint64_t getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return static_cast<uint64_t>(tv.tv_sec * 1000000 + tv.tv_usec);
}

int GPSweek() {
  double diff = (getCurrentTime() - 315964800000000ULL) / 1000000.0;
  return (int) (diff / SECS_PER_WEEK);
}

double GPSsecond() {
  double diff = (getCurrentTime() - 315964800000000ULL) / 1000000.0;
  return (diff - (int) (diff / SECS_PER_WEEK) * SECS_PER_WEEK + LEAP_SECOND);
}

char dec2hex(int d) {
  if (0 <= d && d <=9) return d + '0';
  if (d >= 10) return d - 10 + 'A';
  return '\0';
}

std::string formatGPCHC(InsDataType ins) {
    char str[1024] = "";
    sprintf(str, "%s,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%.10lf,%.10lf,%.10lf,%lf,%lf,%lf,%d,%d,%d,%d,%d,",
            ins.header.c_str(), ins.gps_week, ins.gps_time, ins.heading, ins.pitch, ins.roll, ins.gyro_x, ins.gyro_y, ins.gyro_z,
            ins.acc_x, ins.acc_y, ins.acc_z, ins.latitude, ins.longitude, ins.altitude, ins.Ve, ins.Vn, ins.Vu,
            ins.baseline, ins.NSV1, ins.NSV2, ins.Status, ins.age, ins.Warnning);

    std::string msg = std::string(str);
    uint8_t datasum = 0;
    for (int i = 1; i < msg.size(); i++) {
        datasum ^= msg[i];
    }
    char Cs[4] = "";
    Cs[0] = '*';
    Cs[1] = dec2hex(datasum / 16);
    Cs[2] = dec2hex(datasum % 16);
    Cs[3] = '\0';

    msg = msg + std::string(Cs);
    return msg;
}

int main(int argc, char **argv) {
    std::unique_ptr<UDPServer> udp_server(new UDPServer(12437));
    while (true) {
        InsDataType data;
        data.header   = "$GPCHC";
        data.gps_week = GPSweek();
        data.gps_time = GPSsecond();
        data.heading = 289.19;
        data.pitch = -0.42;
        data.roll = 0.21;
        data.gyro_x = -0.23;
        data.gyro_y = 0.07;
        data.gyro_z = -0.06;
        data.acc_x = 0.0009;
        data.acc_y = 0.0048;
        data.acc_z = -1.0037;
        data.latitude = 38.8594969;
        data.longitude = 121.5150073;
        data.altitude = 121.51;
        data.Ve = -0.023;
        data.Vn = 0.011;
        data.Vu = 0.0000;
        data.baseline = 1.500;
        data.NSV1 = 14;
        data.NSV2 = 6;
        data.Status = 4;
        data.age = 0;
        data.Warnning = 0;
        std::string message = formatGPCHC(data);
        udp_server->UDPSendtoBuf("127.0.0.1", 9888, (char *)(message.c_str()), message.size());
        usleep(10000);
    }
}
