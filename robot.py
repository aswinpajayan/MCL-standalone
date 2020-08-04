#!/usr/bin/env python3
""" module to mimic the behaviour of a robot """


import numpy as np


class Robot(object):
    """Robot class for mimicing robot motion and measurements"""

    def __init__(self, x, y, phi):
        """ Robot class takes in state and creates a robot class
        :param float x: initial xposition
        :param float y: initial y coordinate
        :param float phi: initial position in radians
        """
        self.state = np.array([np.float64(x), np.float64(y), np.float64(phi)], dtype=np.float64)
        self.noise_motion = 0.0
        self.niose_measurement = 0.0
        self.sep = 0.4   #  wheel seperation
        self.radius = 0.1   #  wheel radius


    def set_state(self, new_state):
        """ method takes in `new_state` which is size three `np.array`
            Parameters
            :param np.float new_state: robot state `np.array([x, y, phi], float)`
        """
        self.state = np.array(new_state, dtype=np.float64)

    def get_state(self):
        """ :returns: present state [x, y, phi] """
        return self.state

    def move(self, cmd):
        """method takes `cmd` which is a size 2 `np.array`
        lin_vel should be in rad per sec

        :cmd: [lin_vel, ang_vel]

        """
        x, y, phi = self.state
        lin_vel, ang_vel = np.array(cmd, dtype=np.float)
        delta_t = 1
        v_left = (lin_vel + ang_vel * self.sep / 2)
        v_right = (lin_vel - ang_vel * self.sep / 2)
        slip_left = v_left * self.radius * delta_t
        slip_right = v_right * self.radius * delta_t
        slip_sum = slip_left + slip_right
        slip_diff = slip_left - slip_right
        x = x + (slip_sum / 2) * np.cos(phi + slip_diff / (2 * self.sep))
        y = y + (slip_sum / 2) * np.sin(phi + slip_diff / (2 * self.sep))
        phi = phi + slip_diff / self.sep
        self.state = np.array([x, y, phi], dtype=np.float64)
