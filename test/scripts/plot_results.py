#!/usr/bin/env python

import argparse

import numpy as np
import matplotlib.pyplot as plt

vec3_type = np.dtype([
    ("x", float),
    ("y", float),
    ("z", float),
])

csv_data_type = np.dtype([
    ("time",     float),
    ("pos_err",  float),
    ("vel_err",  float),
    ("rot_err",  float),
    ("pos_pred", vec3_type),
    ("vel_pred", vec3_type),
    ("rot_pred", vec3_type),
    ("pos_true", vec3_type),
    ("vel_true", vec3_type),
    ("rot_true", vec3_type),
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

def plot_error(data, description):
    figure, axes = plt.subplots(nrows=1, ncols=3, figsize=(12,4), sharex=True, sharey=True)
    pos_axes, vel_axes, rot_axes = axes

    figure.suptitle(description)
    pos_axes.set_xlabel("Time [s]")
    pos_axes.set_ylabel("Error [m]")
    vel_axes.set_xlabel("Time [s]")
    vel_axes.set_ylabel("Error [m/s]")
    rot_axes.set_xlabel("Time [s]")
    rot_axes.set_ylabel("Error [rad]")

    for case_data in data:
        pos_axes.plot(case_data["time"], case_data["pos_err"])
        vel_axes.plot(case_data["time"], case_data["vel_err"])
        rot_axes.plot(case_data["time"], case_data["rot_err"])

    return

def plot_state(data, description):
    figure, axes = plt.subplots(nrows=3, ncols=3, figsize=(10,10), sharex="row", sharey="row")
    ((px_axes, py_axes, pz_axes), (vx_axes, vy_axes, vz_axes), (rx_axes, ry_axes, rz_axes)) = axes

    figure.suptitle(description)

    # Plot trajectory for different initial errors.
    plot_args = {"linewidth": 0.5}
    for case_data in data:
        px_axes.plot(case_data["time"], case_data["pos_pred"]["x"], **plot_args)
        py_axes.plot(case_data["time"], case_data["pos_pred"]["y"], **plot_args)
        pz_axes.plot(case_data["time"], case_data["pos_pred"]["z"], **plot_args)

        vx_axes.plot(case_data["time"], case_data["vel_pred"]["x"], **plot_args)
        vy_axes.plot(case_data["time"], case_data["vel_pred"]["y"], **plot_args)
        vz_axes.plot(case_data["time"], case_data["vel_pred"]["z"], **plot_args)

        rx_axes.plot(case_data["time"], case_data["rot_pred"]["x"], **plot_args)
        ry_axes.plot(case_data["time"], case_data["rot_pred"]["y"], **plot_args)
        rz_axes.plot(case_data["time"], case_data["rot_pred"]["z"], **plot_args)

    # Plot ground truth values.
    px_axes.plot(data[0]["time"], data[0]["pos_true"]["x"], "k--", label="True value")
    py_axes.plot(data[0]["time"], data[0]["pos_true"]["y"], "k--", label="True value")
    pz_axes.plot(data[0]["time"], data[0]["pos_true"]["z"], "k--", label="True value")

    vx_axes.plot(data[0]["time"], data[0]["vel_true"]["x"], "k--", label="True value")
    vy_axes.plot(data[0]["time"], data[0]["vel_true"]["y"], "k--", label="True value")
    vz_axes.plot(data[0]["time"], data[0]["vel_true"]["z"], "k--", label="True value")

    rx_axes.plot(data[0]["time"], data[0]["rot_true"]["x"], "k--", label="True value")
    ry_axes.plot(data[0]["time"], data[0]["rot_true"]["y"], "k--", label="True value")
    rz_axes.plot(data[0]["time"], data[0]["rot_true"]["z"], "k--", label="True value")

    px_axes.set_ylabel("x position [m]")
    py_axes.set_ylabel("y position [m]")
    pz_axes.set_ylabel("z position [m]")

    vx_axes.set_ylabel("x velocity [m/s]")
    vy_axes.set_ylabel("y velocity [m/s]")
    vz_axes.set_ylabel("z velocity [m/s]")

    rx_axes.set_ylabel("roll [rad]")
    ry_axes.set_ylabel("pitch [rad]")
    rz_axes.set_ylabel("yaw [rad]")

    for ax in axes.flatten():
        ax.set_xlabel("Time [s]")

    for ax in axes.flatten():
        ax.legend()


if __name__ == "__main__":
    args = get_cmdline_args()
    file_ending = ".csv"
    file_path = args.folder + args.file + file_ending

    with open(file_path, 'r') as f:
        description = f.readline()[2:]
        f.readline()
        f.readline()
        test_case_count = int(f.readline())
        f.readline()
        rows_per_case = int(f.readline())

    FILE_HEADER_ROWS = 6
    CASE_HEADER_ROWS = 2

    data = []
    for n in range(test_case_count):
        skip_rows = FILE_HEADER_ROWS + CASE_HEADER_ROWS*(n+1) + rows_per_case*n
        data.append(np.loadtxt(file_path, dtype=csv_data_type, skiprows=skip_rows, max_rows=rows_per_case))

    plot_error(data, description)
    plot_state(data, description)

    plt.show()
