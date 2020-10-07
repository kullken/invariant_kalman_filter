#!/usr/bin/env python

import argparse

import numpy as np
import matplotlib.pyplot as plt

def create_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
            "file",
            help="Name of file to process"
    )
    parser.add_argument(
            "--folder",
            help="Path to where to find file",
            default="/home/vk/mav_ws/src/invariant_kalman_filter/test/results/data/"
    )

    return parser

def get_cmdline_args():
    return create_arg_parser().parse_args()

def plot_error(times, pos_err, vel_err, rot_err, descr=""):
    figure = plt.figure()
    ax = figure.add_subplot(111)

    ax.plot(times, pos_err, label="Position error [m]")
    ax.plot(times, vel_err, label="Velocity error [m/s]")
    ax.plot(times, rot_err, label="Rotation error [rad]")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Error")
    ax.set_title(descr)
    ax.legend()

    return

def plot_position(times, pos_pred, pos_true, descr=""):
    figure = plt.figure()
    ax = figure.add_subplot(111)

    ax.plot(times, pos_pred[0], label="x - Predicted")
    ax.plot(times, pos_pred[1], label="y - Predicted")
    ax.plot(times, pos_pred[2], label="z - Predicted")

    ax.plot(times, pos_true[0], "--", label="x - True")
    ax.plot(times, pos_true[1], "--", label="y - True")
    ax.plot(times, pos_true[2], "--", label="z - True")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Position [m]")
    ax.set_title(descr)
    ax.legend()


if __name__ == "__main__":
    args = get_cmdline_args()
    file_ending = ".csv"
    file_path = args.folder + args.file + file_ending

    with open(file_path, 'r') as f:
        description = f.readline()[2:]

    data = np.loadtxt(file_path, skiprows=2, unpack=True)

    times = data[0]
    pos_err = data[1]
    vel_err = data[2]
    rot_err = data[3]

    pos_pred = data[4:7]
    vel_pred = data[7:10]
    quat_pred = data[10:14]

    pos_true = data[14:17]
    vel_true = data[17:20]
    quat_true = data[20:24]

    plot_error(times, pos_err, vel_err, rot_err, description)
    plot_position(times, pos_pred, pos_true, description)

    plt.show()