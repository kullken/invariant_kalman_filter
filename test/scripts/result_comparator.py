#!/usr/bin/env python3

from typing import Tuple
from statistics import mean, harmonic_mean

import argparse
import xml.etree.ElementTree as ET

def create_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
            "file",
            help="Name of a file containing test results"
    )
    parser.add_argument(
            "testsuite",
            help="Name of a test suite"
    )
    parser.add_argument(
            "--file2",
            help="Name of a second file containing test results",
            default=None
    )
    parser.add_argument(
            "--testsuite2",
            help="Name of a second test suite",
            default=None
    )
    parser.add_argument(
            "--folder",
            help="Path to where to find the files",
            default="/home/vk/mav_ws/src/invariant_kalman_filter/test/results/"
    )

    return parser

def get_cmdline_args():
    return create_arg_parser().parse_args()

def get_file_names(args) -> Tuple[str, str]:
    file_name_1 = args.folder + args.file
    if args.file2 is not None:
        file_name_2 = args.folder + args.file2
    else:
        file_name_2 = file_name_1
    return file_name_1, file_name_2

def get_test_names(args) -> Tuple[str, str]:
    test_name_1 = args.testsuite
    if args.testsuite2 is not None:
        file_name_2 = args.testsuite2
    else:
        file_name_2 = test_name_1
    return test_name_1, file_name_2

def get_test_suite_root(file_name: str, test_name: str) -> ET.Element:
    tree = ET.parse(file_name)
    root = tree.getroot()
    for testsuite in root.findall("testsuite"):
        if testsuite.attrib["name"] == "RegressionTests/" + test_name:
            return testsuite
    raise NameError("The testsuite name [{}] was not found in file [{}]".format(test_name, file_name))

def compare_results(old_testsuite: ET.Element, new_testsuite: ET.Element):
    old_abs_pos_error = 0.0
    old_abs_vel_error = 0.0
    old_abs_rot_error = 0.0
    for testcase in old_testsuite:
        old_abs_pos_error += float(testcase.attrib["PositionRMSE"])
        old_abs_vel_error += float(testcase.attrib["VelocityRMSE"])
        old_abs_rot_error += float(testcase.attrib["RotationRMSE"])

    new_abs_pos_error = 0
    new_abs_vel_error = 0
    new_abs_rot_error = 0
    for testcase in new_testsuite:
        new_abs_pos_error += float(testcase.attrib["PositionRMSE"])
        new_abs_vel_error += float(testcase.attrib["VelocityRMSE"])
        new_abs_rot_error += float(testcase.attrib["RotationRMSE"])

    ratio_pos_errors = []
    ratio_vel_errors = []
    ratio_rot_errors = []
    for old_testcase, new_testcase in zip(old_testsuite, new_testsuite):
        MIN_RATIO_VALUE = 1e-6  # Set a min value on errors so we avoid divide by zero.
        old_pos_error = max(float(old_testcase.attrib["PositionRMSE"]), MIN_RATIO_VALUE)
        old_vel_error = max(float(old_testcase.attrib["VelocityRMSE"]), MIN_RATIO_VALUE)
        old_rot_error = max(float(old_testcase.attrib["RotationRMSE"]), MIN_RATIO_VALUE)
        new_pos_error = max(float(new_testcase.attrib["PositionRMSE"]), MIN_RATIO_VALUE)
        new_vel_error = max(float(new_testcase.attrib["VelocityRMSE"]), MIN_RATIO_VALUE)
        new_rot_error = max(float(new_testcase.attrib["RotationRMSE"]), MIN_RATIO_VALUE)
        ratio_pos_errors.append(new_pos_error / old_pos_error)
        ratio_vel_errors.append(new_vel_error / old_vel_error)
        ratio_rot_errors.append(new_rot_error / old_rot_error)

    print("Ratio (new/old) of sum of erros:")
    print("Position: {}".format(new_abs_pos_error / old_abs_pos_error))
    print("Velocity: {}".format(new_abs_vel_error / old_abs_vel_error))
    print("Rotation: {}".format(new_abs_rot_error / old_abs_rot_error))
    print()

    print("Harmonic mean of ratio (new/old) of errors:")
    print("Position: {}".format(harmonic_mean(ratio_pos_errors)))
    print("Velocity: {}".format(harmonic_mean(ratio_vel_errors)))
    print("Rotation: {}".format(harmonic_mean(ratio_rot_errors)))
    print()

    print("Arithmetic mean of ratio (new/old) of errors:")
    print("Position: {}".format(mean(ratio_pos_errors)))
    print("Velocity: {}".format(mean(ratio_vel_errors)))
    print("Rotation: {}".format(mean(ratio_rot_errors)))
    print()


if __name__ == "__main__":
    args = get_cmdline_args()

    file_name_1, file_name_2 = get_file_names(args)
    test_name_1, test_name_2 = get_test_names(args)

    root_1 = get_test_suite_root(file_name_1, test_name_1)
    root_2 = get_test_suite_root(file_name_2, test_name_2)

    compare_results(root_1, root_2)
