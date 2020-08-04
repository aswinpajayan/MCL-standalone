#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" Unix socket server . used for plotting
"""
import socket
import os


def start_server(socket_path):
    """ function to start a unix socket server

    :socket_path: path to create a unix socket file
    """
    if os.path.exists(socket_path):
        os.remove(socket_path)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(socket_path)

    print("Listening...")
    while True:
        datagram = server.recv(1024)
        if not datagram:
            break
        else:
            print("-" * 20)
            print(datagram.decode('utf-8'))
            if(datagram.decode('utf-8') == "DONE"):
                break
    print("-" * 20)
    print("Shutting down...")
    server.close()
    os.remove(socket_path)
    print("Done")

if __name__ == "__main__":
    print("Opening socket...")
    start_server("/tmp/python/data_socket")
