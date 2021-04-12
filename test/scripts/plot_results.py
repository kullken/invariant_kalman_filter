#!/usr/bin/env python

import argparse

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

vec3_type = np.dtype([
    ("x", float),
    ("y", float),
    ("z", float),
])

ground_truth_dtype = np.dtype([
    ("time",     float),
    ("pos", vec3_type),
    ("vel", vec3_type),
    ("rot", vec3_type),
])

test_case_dtype = np.dtype([
    ("time",     float),
    ("nees", float),
    ("pos_err", vec3_type),
    ("vel_err", vec3_type),
    ("rot_err", vec3_type),
    ("pos", vec3_type),
    ("vel", vec3_type),
    ("rot", vec3_type),
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

def plot_error_norm(data, description):
    figure, axes = plt.subplots(nrows=1, ncols=3, figsize=(12,4), sharex=True, sharey=True)
    pos_axes, vel_axes, rot_axes = axes

    # figure.suptitle(description)
    pos_axes.set_xlabel("Time [s]")
    pos_axes.set_ylabel("Error [m]")
    vel_axes.set_xlabel("Time [s]")
    vel_axes.set_ylabel("Error [m/s]")
    rot_axes.set_xlabel("Time [s]")
    rot_axes.set_ylabel("Error [rad]")

    plot_args = {"linewidth": 0.5}

    def vec_norm(vec):
        return np.sqrt(vec["x"]**2 + vec["y"]**2 + vec["z"]**2)

    for case_data in data:
        pos_axes.plot(case_data["time"], [vec_norm(error) for error in case_data["pos_err"]], **plot_args)
        vel_axes.plot(case_data["time"], [vec_norm(error) for error in case_data["vel_err"]], **plot_args)
        rot_axes.plot(case_data["time"], [vec_norm(error) for error in case_data["rot_err"]], **plot_args)

    return

def plot_nees(data):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    nees_sum = 0
    for case_data in data:
        nees_sum += case_data["nees"]

    time = data[0]["time"]
    ax.plot(time, nees_sum)

    return

def plot_state_dispersion(data, ground_truth, description):
    figure, axes = plt.subplots(nrows=3, ncols=3, figsize=(10,10), sharex="row", sharey="row")
    ((px_axes, py_axes, pz_axes), (vx_axes, vy_axes, vz_axes), (rx_axes, ry_axes, rz_axes)) = axes

    # figure.suptitle(description)

    # Plot trajectory for different initial errors.
    plot_args = {"linewidth": 0.5}
    for case_data in data:
        px_axes.plot(case_data["time"], case_data["pos"]["x"], **plot_args)
        py_axes.plot(case_data["time"], case_data["pos"]["y"], **plot_args)
        pz_axes.plot(case_data["time"], case_data["pos"]["z"], **plot_args)

        vx_axes.plot(case_data["time"], case_data["vel"]["x"], **plot_args)
        vy_axes.plot(case_data["time"], case_data["vel"]["y"], **plot_args)
        vz_axes.plot(case_data["time"], case_data["vel"]["z"], **plot_args)

        rx_axes.plot(case_data["time"], case_data["rot"]["x"], **plot_args)
        ry_axes.plot(case_data["time"], case_data["rot"]["y"], **plot_args)
        rz_axes.plot(case_data["time"], case_data["rot"]["z"], **plot_args)

    # Plot ground truth values.
    px_axes.plot(ground_truth["time"], ground_truth["pos"]["x"], "k--", label="True value")
    py_axes.plot(ground_truth["time"], ground_truth["pos"]["y"], "k--", label="True value")
    pz_axes.plot(ground_truth["time"], ground_truth["pos"]["z"], "k--", label="True value")

    vx_axes.plot(ground_truth["time"], ground_truth["vel"]["x"], "k--", label="True value")
    vy_axes.plot(ground_truth["time"], ground_truth["vel"]["y"], "k--", label="True value")
    vz_axes.plot(ground_truth["time"], ground_truth["vel"]["z"], "k--", label="True value")

    rx_axes.plot(ground_truth["time"], ground_truth["rot"]["x"], "k--", label="True value")
    ry_axes.plot(ground_truth["time"], ground_truth["rot"]["y"], "k--", label="True value")
    rz_axes.plot(ground_truth["time"], ground_truth["rot"]["z"], "k--", label="True value")

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

    return

def plot_error_dispersion(data_set):
    figure, axes = plt.subplots(nrows=3, ncols=3, figsize=(10,10), sharex="row", sharey="row")
    ((px_axes, py_axes, pz_axes), (vx_axes, vy_axes, vz_axes), (rx_axes, ry_axes, rz_axes)) = axes

    # Plot trajectory for different initial errors.
    plot_args = {"linewidth": 0.5}
    for data_run in data_set:
        px_axes.plot(data_run["time"], data_run["pos_err"]["x"], **plot_args)
        py_axes.plot(data_run["time"], data_run["pos_err"]["y"], **plot_args)
        pz_axes.plot(data_run["time"], data_run["pos_err"]["z"], **plot_args)

        vx_axes.plot(data_run["time"], data_run["vel_err"]["x"], **plot_args)
        vy_axes.plot(data_run["time"], data_run["vel_err"]["y"], **plot_args)
        vz_axes.plot(data_run["time"], data_run["vel_err"]["z"], **plot_args)

        rx_axes.plot(data_run["time"], data_run["rot_err"]["x"], **plot_args)
        ry_axes.plot(data_run["time"], data_run["rot_err"]["y"], **plot_args)
        rz_axes.plot(data_run["time"], data_run["rot_err"]["z"], **plot_args)

    # Plot zero lines as reference
    for ax in axes.flatten():
        ax.plot(data_set[0]["time"], np.zeros_like(data_set[0]["time"]), "k--", **plot_args)

    px_axes.set_ylabel("x position error [m]")
    py_axes.set_ylabel("y position error [m]")
    pz_axes.set_ylabel("z position error [m]")

    vx_axes.set_ylabel("x velocity error [m/s]")
    vy_axes.set_ylabel("y velocity error [m/s]")
    vz_axes.set_ylabel("z velocity error [m/s]")

    rx_axes.set_ylabel("roll error [rad]")
    ry_axes.set_ylabel("pitch error [rad]")
    rz_axes.set_ylabel("yaw error [rad]")

    for ax in axes.flatten():
        ax.set_xlabel("Time [s]")

    return

def set_3d_axis(ax):
    margin = 0.5
    xmin = min(ground_truth["pos"]["x"]) - margin
    xmax = max(ground_truth["pos"]["x"]) + margin
    ymin = min(ground_truth["pos"]["y"]) - margin
    ymax = max(ground_truth["pos"]["y"]) + margin
    zmin = min(ground_truth["pos"]["z"]) - margin
    zmax = max(ground_truth["pos"]["z"]) + margin

    side_length = max(xmax - xmin, ymax - ymin, zmax - zmin)

    ax.set_xlim3d(-side_length/2, side_length/2)
    ax.set_ylim3d(-side_length/2, side_length/2)
    ax.set_zlim3d(0, side_length)

    return

def plot_3D(data, ground_truth, description):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    set_3d_axis(ax)

    # Plot ground truth
    ax.plot(
        ground_truth["pos"]["x"],
        ground_truth["pos"]["y"],
        ground_truth["pos"]["z"],
        "k--", label="True value"
    )

    plot_args = {"linewidth": 1.0}
    for case_data in data:
        ax.plot(
            case_data["pos"]["x"],
            case_data["pos"]["y"],
            case_data["pos"]["z"],
            **plot_args
        )

    return


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

    ground_truth = np.loadtxt(file_path, dtype=ground_truth_dtype, skiprows=FILE_HEADER_ROWS+CASE_HEADER_ROWS, max_rows=rows_per_case)

    data = []
    for n in range(test_case_count):
        skip_rows = FILE_HEADER_ROWS + CASE_HEADER_ROWS*(n+2) + rows_per_case*(n+1)
        data.append(np.loadtxt(file_path, dtype=test_case_dtype, skiprows=skip_rows, max_rows=rows_per_case))

    plot_error_norm(data, description)
    plot_state_dispersion(data, ground_truth, description)
    plot_error_dispersion(data)
    plot_3D(data, ground_truth, description)
    plot_nees(data)

    plt.show()
