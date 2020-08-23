#!/usr/bin/env python3
""" module to perform MCL localisation """


import numpy as np
from optim_mcl import Robot


class MCL(object):

    """MCL class to perform localisation. Initialise with number of particles"""

    def __init__(self, NUM, max):
        """MCL class have to be initialised with number of particles

        :NUM: number of particles to perform localisation

        """
        self.__NUM = NUM  # pylint: disable-msg=C0103
        self.particles = np.random.rand(self.__NUM, 3)
        self.particles[:, :2] = self.particles[:, :2] * max * 2 - max
        self.particles[:, 2] = self.particles[:, 2] * 2 * np.pi - np.pi
        self.weights = np.ones(NUM, dtype=np.float) / NUM
        self.__MAP = np.array([[0, 0], [1, 1]], dtype=np.float)



    def gaussian(self, mu, sigma, x):
        """ calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma """
        return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

    def calc_weights(self, readings, map_l):
        """ calculate importance weights """
        readings = readings.reshape(-1,1)
        self.weights = np.ones(self.__NUM, dtype=np.float)
        for i, p in enumerate(map_l):
            diff = self.particles[:, :2] - p
            dist = np.hypot(diff[:, 0], diff[:, 1])
            self.weights *= self.gaussian(dist, 5.0, readings[i, 0])
            # print(type(self.weights), self.weights.shape)
        sum_weights = np.sum(self.weights)
        if(sum_weights != 0):
            self.weights = self.weights / sum_weights
        else:
            print('encountered all zeros')
            self.weights = np.ones(self.__NUM) / self.__NUM


    def sample(self):
        """
        function to perform low variance sampling
        taken from S thruns udacity course
        """
        indeces = []
        # print(type(self.weights), self.weights.shape)
        index = int(np.random.random() * self.__NUM)
        beta = 0.0
        mw = max(self.weights)
        for _ in range(self.__NUM):
            beta += np.random.random() * 2 * mw
            while(beta > self.weights[index]):
                beta -= self.weights[index]
                index = (index + 1) % self.__NUM
            indeces.append(index)
        self.particles = self.particles[indeces]


    def motion_update(self, robot):
        """ sampling proposal distribution from particles """
        pass

