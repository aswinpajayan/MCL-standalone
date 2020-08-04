#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" Unix socket server . used for plotting
"""
import logging
import os
import socket

import matplotlib.pyplot as plt
import numpy as np

#global constants
ANIMATE = True
VIEW_PROB = True
XLIM, YLIM = 10, 10
NUM = 100

# pylint:disable-msg=C0103
# to plot the targets and scans optionally
# based on GLOBAL_VARIABLE
if(ANIMATE):
    logging.info("interactive plotting is turned on")
    plt.ion()
    fig, ax = plt.subplots(1, 1)
    # ax.set_autoscaley_on(True)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_xlim([-XLIM, XLIM])
    ax.set_ylim([-YLIM, YLIM])
    ax.grid()
    ax.legend()
    line_particles = []
    line_targets = []
    line_scans = []
    line_poses = []
    line_heading = []
    line_heading_p = []
    line_particles, = ax.plot([], [], 'g.', label="waypoint", ms=5)
    line_targets, = ax.plot([], [], 'r.', label="targets", ms=5)
    line_heading, = ax.plot([], [], 'r', lw=3, alpha=0.9, ms=15)
    line_heading_p, = ax.plot([], [], 'g', alpha=0.9, ms=15)
    line_poses, = ax.plot([], [], 'ro', label="robot", ms=15.0, alpha=0.8)
    #ax.scatter(MAP_L[:, 0], MAP_L[:, 1], c='k', marker='*', label="landmarks")
    line_scans, = ax.plot([], [], 'k:')
    fig0, ax0 = plt.subplots(1, 1)
    # ax.set_autoscaley_on(True)
    ax0.set_xlabel('index')
    ax0.set_ylabel('probs')
    ax0.set_xlim([0, NUM])
    ax0.set_ylim([0, 1.2])
    ax0.grid()
    ax0.legend()
    line_prob = []
    line_prob, = ax0.plot([], [], 'g', alpha=0.9, ms=15)
else:
    logging.info("interactive plotting is turned off")
    logging.info("To turn on animation set ANIMATE=True in my_mcl.py")



def start_server(socket_path):
    """ function to start a unix socket server

    :socket_path: path to create a unix socket file
    """
    #  pylint: disable-msg=W0603
    global line_poses, line_heading, line_particles, line_scans  # pylint: disable-msg=C0103
    global line_heading_p, line_prob, NUM  # pylint: disable-msg=C0103
    # pylint: enable-msg=W0603

    if os.path.exists(socket_path):
        os.remove(socket_path)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(socket_path)

    logging.info("Listening...")
    while True:
        datagram = server.recv(1024)
        if not datagram:
            break
        else:
            try:
                logging.info("-" * 20)
                cur_state = datagram
                cur_state = np.fromstring(cur_state, np.float64)
                logging.debug(cur_state, type(cur_state))
                if(ANIMATE):
                    delta = 0.50
                    line_poses.set_data([cur_state[0]], [cur_state[1]])
                    delta_x = delta*np.cos(cur_state[2])
                    delta_y = delta*np.sin(cur_state[2])
                    line_heading.set_data([cur_state[0], cur_state[0]+delta_x], [cur_state[1], cur_state[1]+delta_y])
                    fig.canvas.draw()
                    fig.canvas.flush_events()

                # plt.stem(cur_state[0], cur_state[1])
                # plt.stem(cur_state[0], cur_state[1])
            except ValueError as e:
                logging.warn("Exception occured while handling numpy deserialise")
                logging.info("Checking termination condition")
                if(datagram.decode('utf-8') == "DONE"):
                    print("Recieved termination command (DONE) from client")
                    break
                else:
                    raise e
    logging.info("-" * 20)
    logging.info("Shutting down...")
    server.close()
    os.remove(socket_path)
    print("Removed socket")

if __name__ == "__main__":
    print("Opening socket...")
    start_server("/tmp/python/data_socket")
