#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" Unix socket server . used for plotting
"""
import logging
import os
import socket
# import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy  import signal


from particle_filter import MCL

#global constants
ANIMATE = True
VIEW_PROB = True
XLIM, YLIM = 10, 10
NUM = 100
MAP = np.array([[0, 0], [3, 5], [5, -7], [-1, 8], [-8, -9], [-8, 8]], dtype=np.float)

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
    line_targets, = ax.plot([], [], 'r.', label="end_points", ms=15)
    line_heading, = ax.plot([], [], 'r', lw=3, alpha=0.9, ms=15)
    line_heading_p, = ax.plot([], [], 'g', alpha=0.9, ms=15)
    line_poses, = ax.plot([], [], 'ro', label="robot", ms=15.0, alpha=0.8)
    #ax.scatter(MAP_L[:, 0], MAP_L[:, 1], c='k', marker='*', label="landmarks")
    line_scans, = ax.plot([], [], 'k:')
    # fig0, ax0 = plt.subplots(1, 1)
    # # ax.set_autoscaley_on(True)
    # ax0.set_xlabel('index')
    # ax0.set_ylabel('probs')
    # ax0.set_xlim([0, NUM])
    # ax0.set_ylim([0, 1.2])
    # ax0.grid()
    # ax0.legend()
    # line_prob = []
    # line_prob, = ax0.plot([], [], 'g', alpha=0.9, ms=15)
else:
    logging.info("interactive plotting is turned off")
    logging.info("To turn on animation set ANIMATE=True in my_mcl.py")


def generate_ML_Field(MAP, MAP_size, res):
    """generate_ML_Field: generates an ML field for point landmarks

    :MAP: point land marks of the form [[x0, y0], [x1, y1], ... [xn, yn]]
    :MAP_size: width of MAP in normal units(m) assuming a square MAP
    :res: resolution of MAP (pixels per unit)
    :returns: ML_Field as a grayscale probabilty image

    """
    _std, _k_size = 0.1 * res, 0.5 * res
    ML_Field = np.zeros((MAP_size * res, MAP_size * res), dtype=np.float)
    cov = np.diag([_std ** 2, _std ** 2])
    mean = np.array([0, 0], dtype=np.float)
    x_axis = np.arange(-_k_size, _k_size)
    y_axis = np.arange(-_k_size, _k_size)
    x_values, y_values = np.meshgrid(x_axis, y_axis)
    grid = np.empty(x_values.shape + (2,))
    grid[:, :, 0] = x_values
    grid[:, :, 1] = y_values
    cov_inv = np.linalg.inv(cov)
    z_norm = np.einsum('...k,kl,...l->...', grid - mean, cov_inv, grid - mean)
    kernel = np.exp(-0.5*z_norm)
    kernel = kernel / np.amax(kernel)

    loc = np.array((MAP * res), dtype=np.int)
    for point in loc:
        ML_Field[point[0] + MAP_size * res // 2 -1, -point[1] + MAP_size * res //2-1] = 1.0
    ML_Field = signal.convolve2d(ML_Field, kernel, mode='same')
    # cv2.imshow('test',ML_Field)
    # k = cv2.waitKey(0)
    # while(k != 'x'):
    #     k = cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return ML_Field

def pf(my_pf, readings):
    """pf: carry out particle filter localisation

    :my_pf: MCL object
    :readings: readings from robot

    """
    global line_targets  # pylint: disable-msg=W0603
    my_pf.propose([1, 0.25])
    my_pf.calc_weights(readings)
    line_targets.set_data(my_pf.particles[:, 0], my_pf.particles[:, 1])
    return




def start_server(socket_path):
    """ function to start a unix socket server

    :socket_path: path to create a unix socket file
    """
    #  pylint: disable-msg=W0603
    global line_poses, line_heading, line_particles, line_scans  # pylint: disable-msg=C0103
    global line_heading_p, line_targets, NUM  # pylint: disable-msg=C0103
    # pylint: enable-msg=W0603

    # create object for PF
    my_pf = MCL(1000, [0, -4], 10)
    ML_Field = generate_ML_Field(MAP, 20,20)
    line_targets.set_data(my_pf.particles[:, 0], my_pf.particles[:, 1])
    state_recieved = False
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
                cur_data = datagram
                cur_data = np.fromstring(cur_data, np.float64)
                logging.debug(cur_data, type(cur_data))
                if(ANIMATE):
                    if(not state_recieved):
                        delta = 0.50
                        x, y, phi = cur_data
                        line_poses.set_data([x], [y])
                        delta_x = delta*np.cos(phi)
                        delta_y = delta*np.sin(phi)
                        line_heading.set_data([x, x+delta_x], [y, y+delta_y])
                        fig.canvas.draw()
                        fig.canvas.flush_events()
                        state_recieved = True
                    elif(state_recieved):
                        pf(my_pf, cur_data)
                        readings = cur_data.reshape(-1, 2)
                        delta_x = readings[:, 0] * np.cos(phi + readings[:, 1])
                        delta_y = readings[:, 0] * np.sin(phi + readings[:, 1])
                        #line_targets.set_data([x+delta_x], [y+delta_y])
                        # print('readings: {} endp: {}'.format(readings, [x+delta_x, y+delta_y]))
                        a = np.ones(len(delta_x)) * x 
                        b = np.ones(len(delta_y)) * y
                        c = np.vstack((a,x+delta_x)).T.reshape(-1,1)
                        d = np.vstack((b,y+delta_y)).T.reshape(-1,1)
                        line_scans.set_data(c,d)
                        # time.sleep(1)
                        state_recieved = False


                # plt.stem(cur_data[0], cur_state[1])
                # plt.stem(cur_data[0], cur_state[1])
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
    start_server("/tmp/python-data_socket")
