import csv


def Franka_data2( path, n):
            
    joints = []
    times = []
    output_path = ('%soutput%s.csv' % (path, n))
    
    with open(output_path) as csvfile:
        reader = csv.reader(csvfile)
        data = list(reader)[1:]
        for row in data:
            joint_data = [float(item) for item in row[1:]]
            joints.append(joint_data)
            time_data = float(row[0])
            times.append(time_data)
    return joints, times

joints, times = Franka_data2('NEW_DEMOS/', 1)
print(times)
