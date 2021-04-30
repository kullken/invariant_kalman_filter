#!/usr/bin/env python
from __future__ import division

import argparse
import time

import numpy as np

from load_csv import load_data


def get_cmdline_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
            "data_file",
            help="Name of file to load data from."
    )
    parser.add_argument(
            "--data_folder",
            help="Path to where to find the data file.",
            default="/home/vk/mav_ws/src/invariant_kalman_filter/test/results/data/MonteCarlo/"
    )

    return parser.parse_args()

def compute_rmse(data_set, out_rmse):
    pos_error_squared = np.array([[(vec["x"]**2 + vec["y"]**2 + vec["z"]**2) for vec in data_sample["pos_err"]] for data_sample in data_set])
    vel_error_squared = np.array([[(vec["x"]**2 + vec["y"]**2 + vec["z"]**2) for vec in data_sample["vel_err"]] for data_sample in data_set])
    rot_error_squared = np.array([[(vec["x"]**2 + vec["y"]**2 + vec["z"]**2) for vec in data_sample["rot_err"]] for data_sample in data_set])

    out_rmse["time_avg"]["pos"].append(np.mean(np.sqrt(np.mean(pos_error_squared, axis=0))))
    out_rmse["time_avg"]["vel"].append(np.mean(np.sqrt(np.mean(vel_error_squared, axis=0))))
    out_rmse["time_avg"]["rot"].append(np.mean(np.sqrt(np.mean(rot_error_squared, axis=0))))

    out_rmse["sample_avg"]["pos"].append(np.mean(np.sqrt(np.mean(pos_error_squared, axis=1))))
    out_rmse["sample_avg"]["vel"].append(np.mean(np.sqrt(np.mean(vel_error_squared, axis=1))))
    out_rmse["sample_avg"]["rot"].append(np.mean(np.sqrt(np.mean(rot_error_squared, axis=1))))

    return

def print_latex_table(rmse, noise_factors):
    for i in range(len(noise_factors)):
        print("\\hline ${} \\cdot \\mat{{N}}$".format(noise_factors[i]))
        print("& {:0<6.4} & {:0<6.4} & {:0<6.4}  \\\\".format(rmse["pos"][i], rmse["vel"][i], rmse["rot"][i]))
    return


if __name__ == "__main__":
    args = get_cmdline_args()

    noise_factors = ["\\frac{1}{10}", "\\frac{1}{5}", "\\frac{1}{2}", "1", "2", "5", "10"]
    rmse = {
        "time_avg":   {"pos": [], "vel": [], "rot": []},
        "sample_avg": {"pos": [], "vel": [], "rot": []}
    }

    file_ending = ".csv"
    file_stem = args.data_file[:-1]
    start = int(args.data_file[-1])

    t0 = time.time()

    for i in range(start, start + len(noise_factors)):
        file_path = args.data_folder + file_stem + str(i) + file_ending
        _ground_truth, data, _description = load_data(file_path)
        compute_rmse(data, rmse)

    print("")
    print("dt={}".format(time.time() - t0))
    print("")

    print("Time averaged RMSE:")
    print_latex_table(rmse["time_avg"], noise_factors)
    print("")

    print("Sample averaged RMSE:")
    print_latex_table(rmse["sample_avg"], noise_factors)
    print("")
