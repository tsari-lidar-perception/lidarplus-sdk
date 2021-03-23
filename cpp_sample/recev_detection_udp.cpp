#include <iostream>
#include <memory>

#include "UDPServer.h"
#include "detection.pb.h"

int main(int argc, char **argv) {
    std::cout << "UDP receive detection results" << std::endl;
    std::unique_ptr<UDPServer> udp_server(new UDPServer(9000));
    while (true) {
        char recv_buf[26000] = "";
        int receveSize = udp_server->UDPServerReceive(recv_buf, 26000);
        std::cout << "received " << receveSize << " bytes" << std::endl;
        Detection det;
        bool isValid = det.ParseFromArray(recv_buf, receveSize);
        if (isValid) {
            std::cout << det.DebugString() << std::endl;
        }
    }
}