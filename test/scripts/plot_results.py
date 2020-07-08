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
            default="/home/vk/mav_ws/src/invariant_kalman_filter/test/results/"
    )

    return parser

def get_cmdline_args():
    return create_arg_parser().parse_args()

if __name__ == "__main__":
    args = get_cmdline_args()
    file_ending = ".csv"
    file_path = args.folder + args.file + file_ending

    t, pos_errs, vel_errs, rot_errs = np.loadtxt(file_path, skiprows=2, unpack=True)

    plt.plot(t, pos_errs, label="Position error [m]")
    plt.plot(t, vel_errs, label="Velocity error [m/s]")
    plt.plot(t, rot_errs, label="Rotation error [rad?]")

    plt.xlabel("Time [s]")
    plt.ylabel("Error")
    plt.title(args.file)
    plt.legend()
    plt.show()