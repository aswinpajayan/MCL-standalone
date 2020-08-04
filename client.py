#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" unix domain socket client code
"""

import socket
import os


def start_client(socket_path):
    """ function to start the client
        :socket_path: path to create the file
    """
    if os.path.exists(socket_path):
        client = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        client.connect(socket_path)
        print("Ready.")
        print("Ctrl-C to quit.")
        print("Sending 'DONE' shuts down the server and quits.")
        while True:
            try:
                x = input("> ")
                if x != '':
                    print("SEND:", x)
                    client.send(x.encode('utf-8'))
                    if(x == "DONE"):
                        print("Shutting down.")
                        break
            except KeyboardInterrupt as k:
                print("Shutting down.")
                client.close()
                break
    else:
        print("Couldn't Connect!")
        print("Done")

if __name__ == "__main__":
    print("Connecting....")
    start_client("/tmp/python-data-socket")
