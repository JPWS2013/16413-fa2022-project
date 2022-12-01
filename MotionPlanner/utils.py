def get_new_theta(start, end):
    dir_vec = np.array(x_new)- np.array(x_nearest.pos)

    delta_theta = math.atan(dir_vec[1]/dir_vec[0])

    if (dir_vec[0]>0):
        new_theta = delta_theta
    else:
        new_theta = -math.pi + delta_theta

    return new_theta