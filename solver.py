from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import os, sys, csv, time

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from src.world import World
from src.utils import COUNTERS, compute_surface_aabb, name_from_type
from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_all_links, get_link_names, get_link_inertial_pose, body_from_name, get_pose, get_link_name, get_bodies, dump_body, create_attachment, get_body_name, get_joint_positions, get_position_waypoints, get_quaternion_waypoints, quat_from_euler, euler_from_quat, get_joint_position, set_joint_position, get_joint_limits, get_joint_names, get_joints, body_collision

UNIT_POSE2D = (0., 0., 0.)

class Solver:
    def __init__(self, start_pos, end_pos, guess_matrix, radius = 0.03):
    #def __init__(self, guess_matrix, time_step = 0.05, duration = 5, radius = 0.05, start_pos = np.array([0.12, -0.5698, 0, -2.8106, -0.0003, 3.0363, 0.7411]), end_pos = np.array([-0.010923567328455445, 0.3756526760797238, -0.7289112485142143, -2.2175170144491707, 2.4638620692328344, 1.9346847840915486, -1.770862013075821])):
        self.solver_objects = dict()
        self.radius = radius
        self.guess_matrix = np.array(guess_matrix)
        self.guess_matrix = self.guess_matrix[:7,:10]
        # print("shape of guess matrix: ", self.guess_matrix.shape)
        # self.time_step = time_step
        # self.duration = duration
        self.prog = MathematicalProgram()
        self.num_rows, self.num_cols = self.guess_matrix.shape
        self.num_time_steps = self.num_cols
        self.num_joints = self.num_rows
        # print("num joints: ", self.num_joints)
        self.var_matrix = self.prog.NewContinuousVariables(self.num_joints, self.num_time_steps , "j")
        # print('var matrix: ', self.var_matrix)
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.upper_joint_limits, self.lower_joint_limits = zip(*[
            (-2.8973, 2.8973), #panda_joint1 limit
            (-1.7628, 1.7628), #panda_joint2 limit
            (-2.8973, 2.8973), #panda_joint3 limit
            (-3.0718, -0.0698), #panda_joint4 limit
            (-2.8973, 2.8973), #panda_joint5 limit
            (-0.0175, 3.7525), #panda_joint6 limit
            (-2.8973, 2.8973) #panda_joint7 limit
        ])
        self.upper_joint_limits = np.array(self.upper_joint_limits)[:self.num_joints]
        self.lower_joint_limits = np.array(self.lower_joint_limits)[:self.num_joints]

        # print("Upper joint limits: ", self.upper_joint_limits)
        # print("Lower joint limits: ", self.lower_joint_limits)

        # print("upper_joint_limits: ", self.upper_joint_limits)
        # print("lower_joint_limits: ", self.lower_joint_limits)


        self.upper_limits_matrix = np.tile(self.upper_joint_limits.reshape(-1,1), self.num_time_steps)
        self.lower_limits_matrix = np.tile(self.lower_joint_limits.reshape(-1,1), self.num_time_steps)
        
        # Set heuristic costs
        def cost_fun(j):
            variable_array = j.reshape((7, 10))
            # print('Variable array in cost func: ', variable_array)
            total_cost = 0
            for i in range(1, 10):
                subtraction_vec = abs(variable_array[:, i] - variable_array[:, i-1])
                total_cost += np.sum(subtraction_vec)

            # print("Total cost: ", total_cost)
            return total_cost

        self.solver_objects['cost'] = self.prog.AddCost(cost_fun, vars=self.var_matrix.flatten())

        # Return separate joint limit checkers, one for each joint over all time
        # def joint_constraint(j):
        #     return j
        #     #return np.array([j[1 :], j[2 :], j[3 :], j[4 :], j[5 :], j[6 :], j[7 :]])

        # for i in range(self.num_joints):
        #     # print('lb for ', i, ': ', self.lower_limits_matrix[i,:])
        #     # print('ub for ', i, ': ', self.upper_limits_matrix[i,:])

        #     self.solver_objects['joint_limit_'+str(i)] = self.prog.AddConstraint(
        #         joint_constraint,
        #         # lb=np.array([np.full((1, self.num_time_steps), -2.8973), np.full((1, self.num_time_steps), -1.7628), np.full((1, self.num_time_steps), -2.8973), np.full((1, self.num_time_steps), -3.0718), np.full((1, self.num_time_steps), -2.8973), np.full((1, self.num_time_steps), -0.0175), np.full((1, self.num_time_steps), -2.8973)]),
        #         # ub=np.array([np.full((1, self.num_time_steps), 2.8973), np.full((1, self.num_time_steps), 1.7628), np.full((1, self.num_time_steps), 2.8973), np.full((1, self.num_time_steps), -0.0698), np.full((1, self.num_time_steps), 2.8973), np.full((1, self.num_time_steps), 3.7525), np.full((1, self.num_time_steps), 2.8973)]),
        #         # vars=self.var_matrix)
        #         lb=self.lower_limits_matrix[i,:],
        #         ub=self.upper_limits_matrix[i,:],
        #         vars=self.var_matrix[i,:].flatten())

        def start_constraint(j):
            # print("Variable in start constraint: ", j)
            return j
        
        def end_constraint(j):
            # print("Variable in end constraint: ", j)
            return j

        self.solver_objects['start_pos_check'] = self.prog.AddConstraint(
            start_constraint,
            lb=(self.start_pos['arm'][:self.num_joints] - self.radius),
            ub=(self.start_pos['arm'][:self.num_joints] + self.radius),
            # lb=np.array([0.3]),
            # ub=np.array([1.5]),
            vars=self.var_matrix[:,0].flatten())

        # print("start constraint: ", self.solver_objects['start_pos_check'])

        self.solver_objects['end_pos_check'] = self.prog.AddConstraint(
            end_constraint,
            lb=(self.end_pos['arm'][:self.num_joints] - self.radius),
            ub=(self.end_pos['arm'][:self.num_joints] + self.radius),
            # lb=np.array([1.4]),
            # ub=np.array([2.1]),
            vars=self.var_matrix[:,-1].flatten())

        #self.prog.SetInitialGuess(self.var_matrix, self.guess_matrix)

    def optimize(self):
        wait_for_user("Press enter to begin planning")
        result = Solve(self.prog)

        return result, self.var_matrix


def create_world(use_gui=False):
        
        add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
        add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)
        
        np.set_printoptions(precision=3, suppress=True)
        world = World(use_gui=use_gui)
        sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.15, 0.65, np.pi / 4))
        spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
        world._update_initial()

        return world

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def import_trajectory_parameters(filename):
    with open(filename,  'r') as file:
        chars = '#[()] '
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
                    arm_goal_angles.append(float(element.strip(chars)))

            else:
                row = [float(x) for x in row]
                var_start_list.append(row)

    start_pos = {
        'arm':  np.array(arm_start_angles),
        'base': np.array(base_pos)
    }
    end_pos = {
        'arm': np.array(arm_goal_angles),
        'base': np.array(base_pos)
    }

    var_start_matrix = np.array(var_start_list)

    return start_pos, end_pos, var_start_matrix

def extract_path(sol_array):
    path = list()

    for joint_conf in sol_array.T:
        path.append(tuple(joint_conf))

    return path


if __name__ == "__main__":

    # Initialize solver
    start_pos, end_pos, guess_matrix = import_trajectory_parameters('traj_opt_data.csv')

    traj_solver = Solver(start_pos, end_pos, guess_matrix)

    # print("Starting the base at ", start_pos['base'])
    # print("Starting the arm at ", start_pos['arm'])
    # if end_pos['arm'].size > 0:
    #     print("Planning for arm movement to the goal: ", end_pos['arm'])

    result, var_matrix = traj_solver.optimize()

    print(f"Solution joint angles:\n {result.GetSolution(var_matrix)}")
    print("Successful?", result.is_success())
    print("Solver is: ", result.get_solver_id().name())
    print('optimal cost = ', result.get_optimal_cost())
    
    if not result.is_success():
        raise ValueError("UNABLE TO FIND SOLUTION!!")
    else: 
        optimal_trajectory = extract_path(result.GetSolution(var_matrix))

    # print("Optimal trajectory: ", optimal_trajectory)

    world = create_world(use_gui=True) #Create the world to visualize the optimized trajectory

    #Position robot at the starting point and the arm in the end positions in the world
    set_joint_positions(world.robot, world.base_joints, end_pos['base'])
    set_joint_positions(world.robot, world.arm_joints, end_pos['arm'])

    wait_for_user("Displaying end position. Press enter to continue")

    #Position robot at the starting point and the arm in the starting positions in the world
    set_joint_positions(world.robot, world.base_joints, start_pos['base'])
    set_joint_positions(world.robot, world.arm_joints, start_pos['arm'])

    wait_for_user("Displaying start position. Press enter to visualize trajectory")

    for conf in optimal_trajectory:
        set_joint_positions(world.robot, world.arm_joints, conf)
        time.sleep(1)

    wait_for_user('Visualization complete! Press enter to end')