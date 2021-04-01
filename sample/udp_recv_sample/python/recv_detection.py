import sys
import os
import socket

sys.path.append(os.getcwd())

from dependency.proto import detection_pb2

def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip_port = ('127.0.0.1', 9000)
    server.bind(ip_port)
    while True:
        data, peer = server.recvfrom(65536)
        det = detection_pb2.Detection()
        det.ParseFromString(data)
        freespace = detection_pb2.Freespace()
        freespace.ParseFromString(det.freespace)
        det.ClearField('freespace')
        print(det)

if __name__ == '__main__':
    main()