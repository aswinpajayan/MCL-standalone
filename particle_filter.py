#!/usr/bin/env python3
""" module to perform MCL localisation """


import numpy as np


class MCL(object):

    """MCL class to perform localisation. Initialise with number of particles"""

    def __init__(self, NUM, mean, std):
        """MCL class have to be initialised with number of particles

        :NUM: number of particles to perform localisation
        :mean: mean for initialising Particles [x, y]
        :std: standard deviation for initialising particles

        """
        self._NUM = NUM  # pylint: disable-msg=C0103
        self.particles = np.random.rand(self._NUM, 3)
        self.particles[:, 0] = self.particles[:, 0] * std - (std / 2) + mean[0]
        self.particles[:, 1] = self.particles[:, 1] * std - (std / 2) + mean[1]
        self.particles[:, 2] = self.particles[:, 2] * 2 * np.pi - np.pi
        self.weights = np.ones(NUM, dtype=np.float) / NUM
        self._sep = 0.4   #  wheel seperation
        self._radius = 0.1   #  wheel radius


    def propose(self, cmd):
        """function to add motion to the particles. (i.e. creating
        samples of the proposal distribution

        :cmd: motion command [lin_vel, ang_vel]

        """
        lin_vel, ang_vel = np.float(cmd[0]), np.float(cmd[1])
        # self.particles[:, 0] += lin_vel * np.cos(self.particles[:, 2])
        # self.particles[:, 1] += lin_vel * np.sin(self.particles[:, 2])
        # self.particles[:, 2] += ang_vel

        v_left = (lin_vel + ang_vel * self._sep / 2)
        v_right = (lin_vel - ang_vel * self._sep / 2)
        slip_left = v_left * self._radius
        slip_right = v_right * self._radius
        slip_sum = slip_left + slip_right
        slip_diff = slip_left - slip_right
        self.particles[:, 0] += (slip_sum / 2) * np.cos(self.particles[:, 2] + slip_diff / (2 * self._sep))
        self.particles[:, 1] += (slip_sum / 2) * np.sin(self.particles[:, 2] + slip_diff / (2 * self._sep))
        self.particles[:, 2] += slip_diff / self._sep



    def calc_weights(self, readings):
        """calculate the importance weights from sensor measurement and ML field

        :readings: sensor readings from the robot `array` size num_of_lm * 2
        [[range1, bearing1]
        [range2, bearing2]]

        """
        self.weights = np.ones(self._NUM, dtype=np.float)
        for reading in readings:
            endpoints_x = self.particles[:, 0] + reading[0] * np.cos(reading[1] + self.particles[:, 2])
            endpoints_y = self.particles[:, 0] + reading[0] * np.cos(reading[1] + self.particles[:, 2])
            

    def set_ML_field(self, ML_Field):
        """set_ML_field: set ML field to PF

        :ML_field: grayscale map for ML field 0 - no likelihood, 1 - max possibility

        """
        self._ML_Field = ML_Field
