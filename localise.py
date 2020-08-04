#!/usr/bin/env python3
""" script to start server and client processes """


# importing the multiprocessing module
import multiprocessing
import numpy as np


# importing user modules and scripts
import robot
import plotter_process
import worker_localiser


def main(socket_path):
    """ main method which creates all the processes
        :socket_path: path to create unix socket
    """
    # creating processes
    process_plotter = multiprocessing.Process(target=plotter_process.start_server, args=(socket_path, ))
    process_worker = multiprocessing.Process(target=worker_localiser.start_client, args=(socket_path, ))
    # process_worker = multiprocessing.Process(target=test_robot)

    # starting process 1
    process_plotter.start()
    # starting process 2
    process_worker.start()

    # wait until process 1 is finished
    process_plotter.join()
    # wait until process 2 is finished
    process_worker.join()

    # both processes finished
    print("Done!")


def test_robot():
    """ Testing the robot """
    magellan = robot.Robot(x=0, y=0, phi=0)
    print(magellan.get_state())
    magellan.set_state(np.array([0, 1, 0], dtype=np.float))
    print(magellan.get_state())
    magellan.move([0.25, 0.25])
    print(magellan.get_state())


if __name__ == "__main__":
    SOCKET_PATH = "/tmp/python-data-socket"
    main(SOCKET_PATH)
