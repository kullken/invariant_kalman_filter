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
    figure, axes = plt.subplots(nrows=3, ncols=3, figsize=(10,10), sharex="all", sharey="row")
    ((px_axes, py_axes, pz_axes), (vx_axes, vy_axes, vz_axes), (rx_axes, ry_axes, rz_axes)) = axes

    figure.suptitle(description)

    for case_data in data:
        px_axes.plot(case_data["time"], case_data["pos_pred"]["x"])
        py_axes.plot(case_data["time"], case_data["pos_pred"]["y"])
        pz_axes.plot(case_data["time"], case_data["pos_pred"]["z"])

        vx_axes.plot(case_data["time"], case_data["vel_pred"]["x"])
        vy_axes.plot(case_data["time"], case_data["vel_pred"]["y"])
        vz_axes.plot(case_data["time"], case_data["vel_pred"]["z"])

        rx_axes.plot(case_data["time"], case_data["rot_pred"]["x"])
        ry_axes.plot(case_data["time"], case_data["rot_pred"]["y"])
        rz_axes.plot(case_data["time"], case_data["rot_pred"]["z"])

    px_axes.plot(data[0]["time"], data[0]["pos_true"]["x"], "--", label="x - True")
    py_axes.plot(data[0]["time"], data[0]["pos_true"]["y"], "--", label="y - True")
    pz_axes.plot(data[0]["time"], data[0]["pos_true"]["z"], "--", label="z - True")

    px_axes.legend()
    py_axes.legend()
    pz_axes.legend()

    vx_axes.plot(data[0]["time"], data[0]["vel_true"]["x"], "--", label="x - True")
    vy_axes.plot(data[0]["time"], data[0]["vel_true"]["y"], "--", label="y - True")
    vz_axes.plot(data[0]["time"], data[0]["vel_true"]["z"], "--", label="z - True")

    vx_axes.legend()
    vy_axes.legend()
    vz_axes.legend()

    rx_axes.plot(data[0]["time"], data[0]["rot_true"]["x"], "--", label="x - True")
    ry_axes.plot(data[0]["time"], data[0]["rot_true"]["y"], "--", label="y - True")
    rz_axes.plot(data[0]["time"], data[0]["rot_true"]["z"], "--", label="z - True")

    rx_axes.legend()
    ry_axes.legend()
    rz_axes.legend()


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
