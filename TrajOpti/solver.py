from pydrake.solvers import MathematicalProgram, Solve
import numpy as np

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from src.world import World
from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_all_links, get_link_names, get_link_inertial_pose, body_from_name, get_pose, get_link_name, get_bodies, dump_body, create_attachment, get_body_name, get_joint_positions, get_position_waypoints, get_quaternion_waypoints, quat_from_euler, euler_from_quat, get_joint_position, set_joint_position, get_joint_limits, get_joint_names, get_joints, body_collision


class solver:
    def __init__(self, guess_vector)
        self.prog = MathematicalProgram()
        p1 = self.prog.NewContinuousVariables(7, "x")

        # Set heuristic costs

        def cost_fun(z, start_pos, end_pos):
            return cos_z**2 + cos_z + sin_z
        # Add the cost evaluated with x[0] and x[1].
        cost1 = prog.AddCost(cost_fun, vars=[x[0], x[1]])

        # Add the constraint that p1 is on the unit circle centered at (0, 2)
        self.prog.AddConstraint(
            lambda z: [z[0]**2 + (z[1]-2)**2],
            lb=np.array([1.]),
            ub=np.array([1.]),
            vars=p1)

        # Add the constraint that p2 is on the curve y=x*x
        self.prog.AddConstraint(
            lambda z: [z[1] - z[0]**2],
            lb=[0.],
            ub=[0.],
            vars=p2)



        # Set the value of p1 in initial guess to be [0, 1]
        self.prog.SetInitialGuess(p1, [0., 1.])
        # Set the value of p2 in initial guess to be [1, 1]
        self.prog.SetInitialGuess(p2, [1., 1.])


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

if __name__ == "__main__":

    #TODO: Initialize solver

    # Hard-coded start and end points for the chosen trajectory
    start_pos = {
        'arm': [],
        'base:': []
    }
    end_pos = {
        'arm': [].
        'base': []
    }

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