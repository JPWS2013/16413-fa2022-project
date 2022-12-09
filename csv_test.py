import csv
import numpy as np

with open('traj_opt_data.csv', 'r') as file:
    chars = '#[() '
    reader = csv.reader(file)
    var_start_list = []
    for i, row in enumerate(reader):
        if i == 0:
            base_pos = []
            for element in row:
                base_pos.append(float(element.strip(chars)))

        elif i == 1:
            arm_start_angles = []
            
            for element in row:
                arm_start_angles.append(float(element.strip(chars)))

        elif i == 2:
            arm_goal_angles = []

            for element in row:
                arm_goal_angles.append(element.strip(chars))

        else:
            row = [float(x) for x in row]
            var_start_list.append(row)

var_start_matrix = np.array(var_start_list)

print('Base position: ', base_pos)
print('Arm start position: ', arm_start_angles)
print('Arm goal position: ', arm_goal_angles)

print('Iniital guess matrix: ', var_start_matrix)

