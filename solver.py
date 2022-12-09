from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import os, sys, csv

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from src.world import World
from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_all_links, get_link_names, get_link_inertial_pose, body_from_name, get_pose, get_link_name, get_bodies, dump_body, create_attachment, get_body_name, get_joint_positions, get_position_waypoints, get_quaternion_waypoints, quat_from_euler, euler_from_quat, get_joint_position, set_joint_position, get_joint_limits, get_joint_names, get_joints, body_collision

UNIT_POSE2D = (0., 0., 0.)

class solver:
    def __init__(self, guess_matrix, radius = 0.05, start_pos = np.array([0.12, -0.5698, 0, -2.8106, -0.0003, 3.0363, 0.7411]), end_pos = np.array([-0.010923567328455445, 0.3756526760797238, -0.7289112485142143, -2.2175170144491707, 2.4638620692328344, 1.9346847840915486, -1.770862013075821])):
    #def __init__(self, guess_matrix, time_step = 0.05, duration = 5, radius = 0.05, start_pos = np.array([0.12, -0.5698, 0, -2.8106, -0.0003, 3.0363, 0.7411]), end_pos = np.array([-0.010923567328455445, 0.3756526760797238, -0.7289112485142143, -2.2175170144491707, 2.4638620692328344, 1.9346847840915486, -1.770862013075821])):
        self.radius = radius
        self.guess_matrix = np.array(guess_matrix)
        # self.time_step = time_step
        # self.duration = duration
        self.prog = MathematicalProgram()
        self.num_rows, self.num_cols = self.guess_matrix.shape
        self.num_time_steps = self.num_cols
        self.var_matrix = self.prog.NewContinuousVariables(7, self.num_time_steps , "j")
        self.start_pos = start_pos
        self.end_pos = end_pos

        # Set heuristic costs

        def cost_fun(j):
            variable_array = np.array(j)
            total_cost = 0
            for i in range(2, self.num_time_steps ):
                subtraction_vec = abs(variable_array[: i] - variable_array[: i-1])
                total_cost += np.sum(subtraction_vec)
            return total_cost

        cost1 = self.prog.AddCost(cost_fun, vars=self.var_matrix)

        def joint_constraint(j):
            return np.array([j[1 :], j[2 :], j[3 :], j[4 :], j[5 :], j[6 :], j[7 :]])

        constraint1 = self.prog.AddConstraint(
            joint_constraint,
            lb=np.array([np.full((1, self.num_time_steps), -2.8973), np.full((1, self.num_time_steps), -1.7628), np.full((1, self.num_time_steps), -2.8973), np.full((1, self.num_time_steps), -3.0718), np.full((1, self.num_time_steps), -2.8973), np.full((1, self.num_time_steps), -0.0175), np.full((1, self.num_time_steps), -2.8973)]),
            ub=np.array([np.full((1, self.num_time_steps), 2.8973), np.full((1, self.num_time_steps), 1.7628), np.full((1, self.num_time_steps), 2.8973), np.full((1, self.num_time_steps), -0.0698), np.full((1, self.num_time_steps), 2.8973), np.full((1, self.num_time_steps), 3.7525), np.full((1, self.num_time_steps), 2.8973)]),
            vars=self.var_matrix)

        def start_constraint(j):
            return np.array([j[: 0]])

        constraint2 = self.prog.AddConstraint(
            start_constraint,
            lb=np.array(self.start_pos) - self.radius,
            ub=np.array(self.start_pos) + self.radius,
            vars=self.var_matrix)

        def end_constraint(j):
            return np.array([j[: -1]])

        constraint3 = self.prog.AddConstraint(
            end_constraint,
            lb=np.array(self.end_pos) - self.radius,
            ub=np.array(self.end_pos) + self.radius,
            vars=self.var_matrix)

        #self.prog.SetInitialGuess(self.var_matrix, self.guess_matrix)

        def optimize():
            result = Solve(self.prog)
            print(f"Is optimization successful? {result.is_success()}")
            print(f"Solution to x: {result.GetSolution(j)}")
            print(f"optimal cost: {result.get_optimal_cost()}")
            return np.array([j[: -1]])

        optimal_traject = optimize()





def create_world(use_gui=False):
        
        add_sugar_box = lambda world, **kwargs: self.add_ycb(world, 'sugar_box', **kwargs)
        add_spam_box = lambda world, **kwargs: self.add_ycb(world, 'potted_meat_can', **kwargs)
        
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

    start_pos = {
        'arm':  arm_start_angles,
        'base': base_pos
    }
    end_pos = {
        'arm': arm_goal_angles,
        'base': base_pos
    }

    var_start_matrix = np.array(var_start_list)

    return start_pos, end_pos, var_start_matrix

if __name__ == "__main__":

    #TODO: Initialize solver

    # Hard-coded start and end points for the chosen trajectory
    # start_pos = {
    #     'arm':  [0.12, -0.5698, 0, -2.8106, -0.0003, 3.0363, 0.7411],
    #     'base:': [0.7, -0.75, -1.57]
    # }
    # end_pos = {
    #     'arm': [-0.010923567328455445, 0.3756526760797238, -0.7289112485142143, -2.2175170144491707, 2.4638620692328344, 1.9346847840915486, -1.770862013075821],
    #     'base': [0.7, -0.75, -1.57]
    # }
    start_pos, end_pos, guess_matrix = import_trajectory_parameters('traj_opt_data.csv')

    print("Starting the base at ", start_pos['base'], " and the arm at ", start_pos['arm'])
    if end_pos['arm']:
        print("Planning for arm movement to the goal: ", end_pos['arm'])

    wait_for_user("Press enter to begin planning")

    #TODO: solve for the optimal trajectory
    # optimal_trajectory = solve()

    if optimal_trajectory:
        wait_for_user("Solution found! Press enter to begin visualization")
    else:
        raise ValueError("No optimal solution found!")

    world = create_world(use_gui=True) #Create the world to visualize the optimized trajectory

    #Position robot at the starting point and the arm in the starting positions in the world
    set_joint_positions(world.robot, world.base_joints, start_pos['base'])
    set_joint_positions(world.robot, world.arm_joints, start_pos['arm'])

    for conf in optimal_trajectory:
        set_joint_positions(world.robot, world.arm_joints, conf)
        time.sleep(0.05)

    wait_for_user('Visualization complete! Press enter to end')