#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" unix domain socket client code
"""

import os
import socket

import robot

def start_client(socket_path):
    """ function to start the client
        :socket_path: path to create the file
    """
    # create a robot object
    magellan = robot.Robot(x=0, y=-4, phi=0)
    magellan.set_map([[0, 0], [3, 5], [5, -7], [-1, 8], [-8, -9], [-8, 8]])

    # open a socket
    if os.path.exists(socket_path):
        client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        client.connect(socket_path)
        print("Initialised client socket.")
        print("Sending 'DONE' shuts down the server and quits.")
        for _ in range(1000):
            magellan.move([1, 0.25])
            cur_state = magellan.get_state().tostring()
            readings = magellan.sense().reshape(-1, 1).tostring()
            client.send(cur_state)
            client.send(readings)
        client.send("DONE".encode('utf-8'))
        client.close()
    else:
        print("Couldn't Connect! to {}".format(socket_path))
        print("Shutting down the worker")

if __name__ == "__main__":
    print("Connecting....")
    start_client("/tmp/python-data_socket")
