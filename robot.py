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
        self._MAP = np.array([[0, 0], [3, 5], [5, -7], [-1, 8], [-8, -9], [-8, 8]], dtype=np.float)
        self._NUM_OF_LM = 6



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


    def set_map(self, landmarks):
        """set_map: set the map defined by land marks

        :landmarks: `[x0, y0][x1, y1]` ie x,y location of land marks

        """
        self._MAP = np.array(landmarks, dtype=np.float).reshape(-1, 2)  # pylint:disable-msg=C0103
        self._NUM_OF_LM = len(self._MAP)


    def sense(self):
        """sense: generate sensor readings, using environment map
        :returns: sensor `readings` of the form [range0, bearing0] correspoding to every landmark

        """
        cur_pos = self.state[:2]
        diff = self._MAP - cur_pos + 0.1 * (np.random.rand(self._NUM_OF_LM, 2) - 0.05)
        diffx, diffy = diff[:, 0], diff[:, 1]
        ranges = np.hypot(diffx, diffy)
        bearings = np.arctan2(diffy, diffx) - self.state[2]
        return np.vstack((ranges, bearings)).T
