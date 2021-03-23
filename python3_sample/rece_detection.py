import sys
import os
import socket

sys.path.append(os.getcwd())

from proto import detection_pb2

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip_port = ('127.0.0.1', 9000)
    server.bind(ip_port)
    while True:
        data, peer = server.recvfrom(26000)
        det = detection_pb2.Detection()
        det.ParseFromString(data)
        print(det)

if __name__ == '__main__':
    main()