import numpy as np

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

def load_data(file_path):
    FILE_HEADER_ROWS = 6
    CASE_HEADER_ROWS = 2

    with open(file_path, 'r') as file:
        description = file.readline()[2:]
        file.readline()
        file.readline()
        test_case_count = int(file.readline())
        file.readline()
        rows_per_case = int(file.readline())

    ground_truth = np.loadtxt(file_path, dtype=ground_truth_dtype, skiprows=FILE_HEADER_ROWS+CASE_HEADER_ROWS, max_rows=rows_per_case)

    data = []
    for n in range(test_case_count):
        skip_rows = FILE_HEADER_ROWS + CASE_HEADER_ROWS*(n+2) + rows_per_case*(n+1)
        data.append(np.loadtxt(file_path, dtype=test_case_dtype, skiprows=skip_rows, max_rows=rows_per_case))

    return ground_truth, data, description
