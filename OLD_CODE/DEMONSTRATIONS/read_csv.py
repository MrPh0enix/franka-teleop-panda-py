import csv
import numpy as np

def pathCSV(path, n):
    # OUTPUT:
    # joints: list of (7,)-vectors
    # poses: list of (4,4)-matrices
    # times: list of scalars
    joints = []
    poses = []
    times = []
    joints_path = ('%sJOINTS%s.csv' % (path, n))
    poses_path = ('%sPOSES%s.csv' % (path, n))
    times_path = ('%sTIMES%s.csv' % (path, n))
    with open(joints_path) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)  # change contents to floats
        for row in reader:  # each row is a list
            joints.append(row)
    with open(poses_path) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)  # change contents to floats
        for row in reader:  # each row is a list
            poses.append(np.reshape(row, (4, 4)).transpose())
    with open(times_path) as csvfile:
        reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)  # change contents to floats
        for row in reader:  # each row is a list
            times.append(float(row[0]))
    return joints, poses, times
    
    
if __name__ == '__main__':
    Nd = 150
    for demo in range(Nd):
        _, poses_raw, times_raw = pathCSV('demonstrations/', demo)
