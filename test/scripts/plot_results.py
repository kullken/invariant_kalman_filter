#!/usr/bin/env python

import argparse

import numpy as np
import matplotlib.pyplot as plt

vec3_type = np.dtype([
    ("x", float),
    ("y", float),
    ("z", float),
])

quat_type = np.dtype([
    ("x", float),
    ("y", float),
    ("z", float),
    ("w", float),
])

csv_data_type = np.dtype([
    ("time",      float),
    ("pos_err",   float),
    ("vel_err",   float),
    ("rot_err",   float),
    ("pos_pred",  vec3_type),
    ("vel_pred",  vec3_type),
    ("quat_pred", quat_type),
    ("pos_true",  vec3_type),
    ("vel_true",  vec3_type),
    ("quat_true", quat_type),
])

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

    ax.plot(times, pos_pred["x"], label="x - Predicted")
    ax.plot(times, pos_pred["y"], label="y - Predicted")
    ax.plot(times, pos_pred["z"], label="z - Predicted")

    ax.plot(times, pos_true["x"], "--", label="x - True")
    ax.plot(times, pos_true["y"], "--", label="y - True")
    ax.plot(times, pos_true["z"], "--", label="z - True")

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

    data = np.loadtxt(file_path, dtype=csv_data_type, skiprows=2)

    plot_error(data["time"], data["pos_err"], data["vel_err"], data["rot_err"], description)
    plot_position(data["time"], data["pos_pred"], data["pos_true"], description)

    plt.show()
