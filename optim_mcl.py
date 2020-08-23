#!/usr/bin/env python3
"""
    script to run particle filter but slightly better
"""

import matplotlib.pyplot as plt
import numpy as np
import random


class Robot(object):
    """
        class to take care of basic robot operations
    """
    MAP = np.array([[0.0, 0.0], [1.0, 1.0]])
    MAP_SIZE = 2
    WORLD_SIZE = 100
    def __init__(self, pose):
        """ constructor

            :pose: array/list of pose [x, y, phi]

        """
        self.pose = [float(value) for value in pose]
        self.motion_noise = [0.0, 0.0]
        self.sense_noise = 0.0
        self.__turn_noise = 0.0
        self.__forward_noise = 0.0
        self.__sense_noise = 0.0
        self.__motion_cmd = [0.0, 0.0]


    def set_noise(self, turn_noise, forward_noise, sense_noise):
        """ set the robot noise

            :turn_noise: uncertainity in rotation
            :forward_noise: uncertainity in translation
            :sense_nose: uncertainity in measurement

        """
        self.__turn_noise, self.__forward_noise = turn_noise, forward_noise
        self.__sense_noise = sense_noise

    def set_motion_cmd(self, turn, forward):
        """

            :turn: set the robot motion
            :forward: translation in float

        """
        self.__motion_cmd = [float(turn), float(forward)]


    def motion_update(self, x, y, phi):
        """function to get upated robot pose
        """
        phi += self.__motion_cmd[0] + np.random.normal(size=len(phi)) * self.__turn_noise
        phi %= np.pi * 2
        delta = self.__motion_cmd[1] + np.random.normal(size=len(x)) * self.__forward_noise
        x += np.cos(phi) * delta
        y += np.sin(phi) * delta
        return [x, y, phi]

    def move(self, turn, forward):
        """ function to move robot """
        self.set_motion_cmd(float(turn), float(forward))
        x, y, phi = self.pose
        phi += self.__motion_cmd[0] + np.random.normal() * self.__turn_noise
        phi %= np.pi * 2
        delta = self.__motion_cmd[1] + np.random.normal() * self.__forward_noise
        x += np.cos(phi) * delta #  % Robot.WORLD_SIZE
        y += np.sin(phi) * delta #  % Robot.WORLD_SIZE
        self.pose = [x, y, phi] 

    @classmethod
    def __set_map__(cls, landmarks):
        Robot.MAP = np.array(landmarks, dtype=np.float).reshape(-1, 2)
        Robot.MAP_SIZE = len(landmarks)

    def sense(self):
        """sensor measurement
        """
        cur_pose = np.array(self.pose[:2], dtype=np.float).reshape(1,2)
        diff = Robot.MAP - cur_pose
#         print(diff.shape,cur_pose.shape,Robot.MAP_SIZE)
        return np.hypot(diff[:, 0], diff[:, 1]) + np.random.normal(size=Robot.MAP_SIZE) * self.sense_noise
