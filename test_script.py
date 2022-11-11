import os
import sys
import argparse
import numpy as np

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, get_joint_positions, set_joint_positions, set_joint_position
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_collision_data, body_collision

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

UNIT_POSE2D = (0., 0., 0.)

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

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

np.set_printoptions(precision=3, suppress=True)
world = World(use_gui=True)
sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
world._update_initial()
tool_link = link_from_name(world.robot, 'panda_hand')
joints = get_movable_joints(world.robot)
print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
sample_fn = get_sample_fn(world.robot, world.arm_joints)

os.system('Clear')

action_option = 0

while action_option != 2:
    action_option = input("\nChoose an action:\n \
        1. Get current pose\n \
        2. stop doing stuff\n \
        3. Set a new pose\n \
        4. Set individual joint angles\n \
        5. Check for collisions\n \
        6. Play with random samples\n \
        7. Move base forward\n")
    
    try:
        action_option = int(action_option.strip())
    except ValueError as e:
        print("Invalid input detected!")
        continue

    if action_option == 1:
        tool_link = link_from_name(world.robot, 'panda_hand')
        start_pose = get_link_pose(world.robot, tool_link)
        print("Pose of panda_hand: ", start_pose)

        joints_pos = get_joint_positions(world.robot, world.arm_joints)
        arm_joint_names = [get_joint_name(world.robot, joint) for joint in world.arm_joints]
        print("Position of joints: ")
        for joint, pos in zip(arm_joint_names, joints_pos):
            print("    ", joint, ":", pos)

    elif action_option == 2:
        continue

    elif action_option == 3:
        joint_angles_str = input("Feed a 7 element tuple of joint angles: ")
        joint_angles_str = joint_angles_str.split(',')
        joint_angles = tuple([float(element) for element in joint_angles_str])
        set_joint_positions(world.robot, world.arm_joints, joint_angles)

    elif action_option == 4:
        
        valid_joint_mvmt = False

        while not valid_joint_mvmt:
            joint_num = input("Which joint do you want to move? ")

            try:
                joint_num = int(joint_num.strip())
                assert joint_num <= 7
                assert joint_num >= 1
            except:
                print("Invalid jointnumber entered")

            joint_angle = input("What angle would you like the joint at? ")

            try:
                joint_angle = float(joint_angle.strip())
            except:
                print("Invalid joint angle entered")

            joint_obj = joint_from_name(world.robot, ('panda_joint'+str(joint_num)))

            # set_joint_position(world.robot, joint_obj, joint_angle)

            try:
                set_joint_position(world.robot, joint_obj, joint_angle)
                valid_joint_mvmt = True
            except:
                print("Unable to set joint angle!")

    
    elif action_option == 5:
        collision_data = get_collision_data(world.robot)
        print('get_collision_data returned:', collision_data)

        body_collision_val = body_collision(world.robot, world.kitchen)
        print('body_collision_val is:', body_collision_val)

    elif action_option == 6:
        print("Getting random sample")
        conf = sample_fn()
        print("Sample: ", conf)

        apply_sample = input("Apply this sample to the robot arm? (y/n)")

        if apply_sample == 'y':
            set_joint_positions(world.robot, world.arm_joints, conf)

    elif action_option == 7:

        successful_mvmt = False

        while not successful_mvmt:
            try:
                translation_amt = input("Indicate how much to move the base:")
                translation_amt = float(translation_amt.strip())
                goal_pos = translate_linearly(world, translation_amt) 
                set_joint_positions(world.robot, world.base_joints, goal_pos)
                successful_mvmt = True

            except ValueError as e:
                print("Invalid movement amount detected!")
                continue
    
    else:
        print(action_option, " is an invalid input option! Please try again.")

wait_for_user()