#!/usr/bin/env python
from __future__ import division

import argparse

import numpy as np
import scipy.stats
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

vec3_type = np.dtype([
    ("x", float),
    ("y", float),
    ("z", float),
])

ground_truth_dtype = np.dtype([
    ("time", float),
    ("pos", vec3_type),
    ("vel", vec3_type),
    ("rot", vec3_type),
])

test_case_dtype = np.dtype([
    ("time", float),
    ("nees", float),
    ("nis",  float),
    ("pos_nees", float),
    ("vel_nees", float),
    ("rot_nees", float),
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
    parser.add_argument(
            "--dof",
            help="Degrees of freedom of measurement innovation. Used for confidence intervals of NIS.",
            default="3"
    )

    return parser

def get_cmdline_args():
    return create_arg_parser().parse_args()

def plot_error_norm(data):
    _figure, axes = plt.subplots(nrows=1, ncols=3, figsize=(12,4), sharex=True, sharey=True)
    pos_axes, vel_axes, rot_axes = axes

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

def plot_confidence_intervals(data, nis_dof):
    """Plot NEES- and NIS-values over time, and their corresponding confidence intervals."""

    _figure, axes = plt.subplots(nrows=3, ncols=1, figsize=(10,10))

    plot_full_nees(data, axes[0])
    plot_seperate_nees(data, axes[1])
    plot_nis(data, nis_dof, axes[2])

    return

def plot_full_nees(data, ax=None):
    time = data[0]["time"]

    nees_sum = np.zeros_like(time)
    for case_data in data:
        nees_sum += case_data["nees"]

    degrees_of_freedom = len(data) * 9
    confidence_interval = 0.95
    lower_bound, upper_bound = scipy.stats.chi2.interval(confidence_interval, degrees_of_freedom)

    count_inside_bounds = sum(1 for x in nees_sum if lower_bound <= x <= upper_bound)
    hit_ratio = count_inside_bounds / len(nees_sum)

    if ax is None:
        ax = plt.figure().add_subplot(111)

    ax.plot(time, nees_sum, label="{:2.2%} in interval".format(hit_ratio))
    ax.plot(time, np.ones_like(time) * lower_bound, "k--", label="{:2.0%} confidence interval".format(0.95))
    ax.plot(time, np.ones_like(time) * upper_bound, "k--")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("NEES")

    ax.legend()

    return

def plot_seperate_nees(data, ax=None):
    time = data[0]["time"]

    pos_nees = np.zeros_like(time)
    vel_nees = np.zeros_like(time)
    rot_nees = np.zeros_like(time)
    for case_data in data:
        pos_nees += case_data["pos_nees"]
        vel_nees += case_data["vel_nees"]
        rot_nees += case_data["rot_nees"]

    degrees_of_freedom = len(data) * 3
    confidence_interval = 0.95
    lower_bound, upper_bound = scipy.stats.chi2.interval(confidence_interval, degrees_of_freedom)

    pos_hit_ratio = sum(1 for x in pos_nees if lower_bound <= x <= upper_bound) / len(pos_nees)
    vel_hit_ratio = sum(1 for x in vel_nees if lower_bound <= x <= upper_bound) / len(vel_nees)
    rot_hit_ratio = sum(1 for x in rot_nees if lower_bound <= x <= upper_bound) / len(rot_nees)

    if ax is None:
        ax = plt.figure().add_subplot(111)

    plot_args = {"linewidth": 0.75}
    ax.plot(time, pos_nees, label="Position: {:2.2%} in interval".format(pos_hit_ratio), **plot_args)
    ax.plot(time, vel_nees, label="Velocity: {:2.2%} in interval".format(vel_hit_ratio), **plot_args)
    ax.plot(time, rot_nees, label="Rotation: {:2.2%} in interval".format(rot_hit_ratio), **plot_args)
    ax.plot(time, np.ones_like(time) * lower_bound, "k--", label="{:2.0%} confidence interval".format(0.95))
    ax.plot(time, np.ones_like(time) * upper_bound, "k--")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("NEES")

    ax.legend()

    return

def plot_nis(data, dof, ax=None):
    # NIS-values only exist where a measurement update has been done, the rest are dummy-values.
    mask = data[0]["nis"] >= 0
    time = data[0]["time"][mask]

    if not np.any(mask):
        print("No valid NIS-values found.")
        return

    nis_sum = np.zeros_like(time)
    for case_data in data:
        nis_sum += case_data["nis"][mask]

    degrees_of_freedom = len(data) * dof
    confidence_interval = 0.95
    lower_bound, upper_bound = scipy.stats.chi2.interval(confidence_interval, degrees_of_freedom)

    count_inside_bounds = sum(1 for x in nis_sum if lower_bound <= x <= upper_bound)
    hit_ratio = count_inside_bounds / len(nis_sum)

    if ax is None:
        ax = plt.figure().add_subplot(111)

    ax.plot(time, nis_sum, label="{:2.1%} of samples in interval".format(hit_ratio))
    ax.plot(time, np.ones_like(time) * lower_bound, "k--", label="{:2.0%} confidence interval".format(0.95))
    ax.plot(time, np.ones_like(time) * upper_bound, "k--")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("NIS")

    ax.legend()

    return

def plot_state_dispersion(data, ground_truth):
    _figure, axes = plt.subplots(nrows=3, ncols=3, figsize=(10,10), sharex="row", sharey="row")
    ((px_axes, py_axes, pz_axes), (vx_axes, vy_axes, vz_axes), (rx_axes, ry_axes, rz_axes)) = axes

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
    _figure, axes = plt.subplots(nrows=3, ncols=3, figsize=(10,10), sharex="row", sharey="row")
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

def plot_3D(data, ground_truth, duration=float("inf")):
    ax = plt.figure().add_subplot(111, projection="3d")
    set_3d_axis(ax)

    mask = ground_truth["time"] <= duration

    # Plot ground truth
    ax.plot(
        ground_truth["pos"]["x"][mask],
        ground_truth["pos"]["y"][mask],
        ground_truth["pos"]["z"][mask],
        "k--", label="True value"
    )

    plot_args = {"linewidth": 1.0}
    for case_data in data:
        ax.plot(
            case_data["pos"]["x"][mask],
            case_data["pos"]["y"][mask],
            case_data["pos"]["z"][mask],
            **plot_args
        )

    return

def load_data(file_path):
    FILE_HEADER_ROWS = 6
    CASE_HEADER_ROWS = 2

    with open(file_path, 'r') as f:
        description = f.readline()[2:]
        f.readline()
        f.readline()
        test_case_count = int(f.readline())
        f.readline()
        rows_per_case = int(f.readline())

    ground_truth = np.loadtxt(file_path, dtype=ground_truth_dtype, skiprows=FILE_HEADER_ROWS+CASE_HEADER_ROWS, max_rows=rows_per_case)

    data = []
    for n in range(test_case_count):
        skip_rows = FILE_HEADER_ROWS + CASE_HEADER_ROWS*(n+2) + rows_per_case*(n+1)
        data.append(np.loadtxt(file_path, dtype=test_case_dtype, skiprows=skip_rows, max_rows=rows_per_case))

    return ground_truth, data, description


if __name__ == "__main__":
    args = get_cmdline_args()
    file_ending = ".csv"
    file_path = args.folder + args.file + file_ending

    ground_truth, data, _description = load_data(file_path)

    plot_error_norm(data)
    plot_state_dispersion(data, ground_truth)
    plot_error_dispersion(data)
    plot_3D(data, ground_truth)
    plot_confidence_intervals(data, int(args.dof))

    plt.show()
