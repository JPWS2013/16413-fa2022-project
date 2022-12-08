import os
import sys
import argparse, math
import numpy as np

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, get_joint_positions, set_joint_positions, set_joint_position, create_attachment, get_euler, quat_from_euler, any_link_pair_collision, expand_links, get_link_name

from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_collision_data, body_collision, Pose, Point, get_link_names, get_all_links, get_joints, get_joint_names, euler_from_quat, get_joint_limits, is_movable

from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF


from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, get_grasps, set_tool_pose, create_surface_attachment

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
sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.15, 0.65, np.pi / 4))
spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
world._update_initial()
tool_link = link_from_name(world.robot, 'panda_hand')
joints = get_movable_joints(world.robot)
# grasp_gen = get_grasps(world, sugar_box)
# for grasp in grasp_gen:
#     print('GRASP', grasp)
print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
sample_fn = get_sample_fn(world.robot, world.arm_joints)
active_attachment = None
kitchen_links = get_all_links(world.kitchen)
robot_joints = get_joints(world.robot)
joint_names = get_joint_names(world.robot, robot_joints)
finger_joints = dict()
for joint_name, joint in zip(joint_names, robot_joints):
    if 'finger' in joint_name:

        finger_joints[joint_name] = (joint, get_joint_limits(world.robot, joint))

print("Finger joint stats: ", finger_joints)

wait_for_user("Hit enter to move finger joints")

joints, joint_limits = zip(*finger_joints.values())

set_joint_positions(world.robot, joints, [0.065,0.065])

# print("Positions of the finger joints: ", get_joint_positions(world.robot, finger_joints.values()))
wait_for_user()
# print("Getting random sample")
# conf = sample_fn()
# print("Sample: ", conf)
# goal_pos = translate_linearly(world, 1.2) 
# set_joint_positions(world.robot, world.base_joints, goal_pos-0.7)
# wait_for_user()
# #collisions = get_collision_data(world.robot)
# collisions = body_collision(world.robot, world.kitchen, max_distance=0.01)
# print('Collisions:')
# print(collisions)
# print("Movable joints: ", [get_joint_name(world.robot, joint) for joint in joints])
# print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
# print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
# sample_fn = get_sample_fn(world.robot, world.arm_joints)

os.system('Clear')

action_option = 0
drawer_pos = 0

while action_option != 2:
    action_option = input("\nChoose an action:\n \
        1. Get current pose\n \
        2. stop doing stuff\n \
        3. Set a new pose\n \
        4. Set individual joint angles\n \
        5. Check for collisions\n \
        6. Play with random samples\n \
        7. Move base forward\n \
        8. Play demo\n \
        9. Locate objects and get joint angles\n \
        10. Locate indigo drawer handle\n")
    
    try:
        action_option = int(action_option.strip())
    except ValueError as e:
        print("Invalid input detected!")
        continue

    if action_option == 1:
        tool_link = link_from_name(world.robot, 'panda_hand')
        start_pose = get_link_pose(world.robot, tool_link)
        print("Pose of panda_hand: ", start_pose)
        print("Euler angles of panda_hand: ", euler_from_quat(start_pose[1]))

        joints_pos = get_joint_positions(world.robot, world.arm_joints)
        arm_joint_names = [get_joint_name(world.robot, joint) for joint in world.arm_joints]
        print("Position of joints: ")
        for joint, pos in zip(arm_joint_names, joints_pos):
            print("    ", joint, ":", pos)

        base_joints_pos = get_joint_positions(world.robot, world.base_joints)
        print("Base position: X -", base_joints_pos[0], 'Y -', base_joints_pos[1], 'Z -', base_joints_pos[2])

    elif action_option == 2:
        continue

    elif action_option == 3:
        joint_set = input("Do you want to movethe arm (a) or the base (b) or the drawer(d)?")
        joint_set = joint_set.strip()

        if joint_set == 'a':
            joint_angles_str = input("Feed a 7 element tuple of joint angles: ")
            joint_angles_str = joint_angles_str.split(',')
            joint_angles = tuple([float(element) for element in joint_angles_str])
            set_joint_positions(world.robot, world.arm_joints, joint_angles)
        elif joint_set == 'b':
            joint_angles_str = input("Feed a 3 element tuple of x-pos, y-pos, theta for the base: ")
            joint_angles_str = joint_angles_str.split(',')
            joint_angles = tuple([float(element) for element in joint_angles_str])
            set_joint_positions(world.robot, world.base_joints, joint_angles)

        elif joint_set == 'd':
            drawer_joint = joint_from_name(world.kitchen, 'indigo_drawer_top_joint')
            print("Drawer has the following joint limits: ", get_joint_limits(world.kitchen, drawer_joint))
            drawer_pos = input("What position do you want the drawer at?")
            drawer_pos = float(drawer_pos.strip())

            set_joint_position(world.kitchen, drawer_joint, drawer_pos)

        if active_attachment != None:
            active_attachment.assign()



    elif action_option == 4:
        
        valid_joint_mvmt = False

        while not valid_joint_mvmt:
            joint_set = input("Do you want to move the arm (a) or the base (b)?")
            joint_set = joint_set.strip()

            if joint_set == 'a':
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
            
            elif joint_set == 'b':
                joint = input("Do you want to move x, y or theta? ")
                joint = joint.strip()

                joint_angle = input("What value do you want the joint at? ")

                try:
                    joint_angle = float(joint_angle.strip())
                except:
                    print("Invalid joint angle entered")

                joint_obj = joint_from_name(world.robot, joint)

                try:
                    set_joint_position(world.robot, joint_obj, joint_angle)
                    valid_joint_mvmt = True
                except:
                    print("Unable to set joint angle!")

    
    elif action_option == 5:
        collision_data = get_collision_data(world.robot)
        print('get_collision_data returned:', collision_data)

        body_collision_val = body_collision(world.robot, world.kitchen)
        # body_collision_val_new = any_link_pair_collision(world.robot, None, world.robot)
        robot, robot_links = expand_links(world.robot)
        robot_link_names = get_link_names(world.robot, robot_links)
        collision_links = []
        for link_name, link in zip(robot_link_names, robot_links):
            if 'panda_hand' in link_name or 'panda_link' in link_name:
                collision_links.append(link)
        body_collision_val_new = any_link_pair_collision(world.robot, collision_links, world.robot, collision_links,)
        sugar = world.get_body('sugar_box0')
        meat = world.get_body('potted_meat_can1')

        if body_collision(world.robot, sugar):
            print("Collision with sugar box!")
        if body_collision(world.robot, meat):
            print("Collision with meat can!")

        if body_collision(world.kitchen, sugar):
            print("Collision between sugar and kitchen!")
        if body_collision(world.kitchen, meat):
            print("Collision between kitchen and meat can!")

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

    elif action_option == 8:

        wait_for_user('Going to set base in front of sugar box')
        point1_arm = (-0.1, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
        point1_base = (0.8, 0.5, -3.14)
        set_joint_positions(world.robot, world.base_joints, point1_base)
        set_joint_positions(world.robot, world.arm_joints, point1_arm)

        wait_for_user('Grasping sugar box')
        sugar_box_att = create_attachment(world.robot, link_from_name(world.robot, 'panda_hand'), world.get_body(sugar_box))

        wait_for_user('Attempting to move sugar box')
        point2_arm = (3.14, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
        set_joint_positions(world.robot, world.arm_joints, point2_arm)
        sugar_box_att.assign()

        wait_for_user('Attempting to move sugar box')
        point2_arm = (0, 1.25, 0, -0.6, 0 ,3.04 , 0.741)
        set_joint_positions(world.robot, world.arm_joints, point2_arm)
        sugar_box_att.assign()

    elif action_option == 9:

        object_dict = {
            'potted_meat_can1': (0.7, 0.3, -math.pi/2),
            'sugar_box0': (0.75, 0.65 , -math.pi/2)
        }

        user_obj = input("What object would you like to locate? ")
        user_obj = user_obj.strip()

        try:
            link = link_from_name(world.kitchen, user_obj)
            pose = get_link_pose(world.kitchen, link)
            print("Location ", user_obj, " has pose: ", pose)

        except ValueError as e:
            try:
                body = world.get_body(user_obj)
                coord = get_pose(body)[0]
                euler_angles = get_euler(body)
                print("Object ", user_obj, " has coordinates: ", coord)
                print("Euler angles for this object are: ", euler_angles)
            
            except ValueError as e:
                print("Error getting coordinates for the following link: ", e, " Exiting!")
                sys.exit(1)

        set_joint_positions(world.robot, world.base_joints, object_dict[user_obj])

        tool_link = link_from_name(world.robot, 'panda_hand')

        obj_lower, obj_upper = get_aabb(body)
        obj_height = obj_upper[2] - obj_lower[2]

        print("Object aabb is: ", obj_lower, obj_upper)
        
        user_selection = ''
        x_shift_obj_orig = -0.03
        y_shift_obj_orig = -0.14
        # tool_backoff_dist = 0.1 #Distance to back the tool off the object
        x_backoff = (x_shift_obj_orig*math.cos(euler_angles[2])) - (y_shift_obj_orig*math.sin(euler_angles[2]))
        y_backoff = (x_shift_obj_orig*math.sin(euler_angles[2])) + (y_shift_obj_orig*math.cos(euler_angles[2]))

        tool_euler_angles = (euler_angles[0], (-0.5*math.pi), -euler_angles[2])
        coord = ((coord[0]+x_backoff), (coord[1]+y_backoff), (coord[2]+(obj_height/2))) #Raise z by 0.05 to be at the right height to grip object
        pose = (coord, quat_from_euler(tool_euler_angles))

        print("Calculated tool pose: ", pose)
        wait_for_user("Setting tool pose now, press enter to confirm")
        # set_tool_pose(world, pose)



        while user_selection != 'end':
            
            success = False

            for i in range(10):
                conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)

                if conf != None:
                    success = True
                    break

                if i == 9:
                    print("Unable to run inverse kinematics successfully!")

            if success:
                print("The joint angles to achieve this pose is: ", conf)

                set_joint_positions(world.robot, world.arm_joints, conf)

            user_selection = input("End or continue? ")
            user_selection.strip()
    
    elif action_option == 10:

        set_joint_positions(world.robot, world.base_joints, (0.7, 0.6, -math.pi/2))

        user_choice = input("Would you like the drawer opened or closed?")
        user_choice = user_choice.strip()

        target_pose = get_link_pose(world.kitchen, link_from_name(world.kitchen, 'indigo_drawer_handle_top'))

        target_pos, target_quat = target_pose
        tool_target_pos = ((target_pos[0]+0.1), target_pos[1], target_pos[2])
        orig_target_euler = euler_from_quat(target_quat)

        tool_target_euler = (-orig_target_euler[0], orig_target_euler[1], (0.5*math.pi))
        tool_target_quat = quat_from_euler(tool_target_euler)

        tool_link = link_from_name(world.robot, 'panda_hand')

        print("Attempting to get joint angles for target pose: ", target_pos, tool_target_euler)

        target_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, (tool_target_pos, tool_target_quat), max_time=0.05), None)

        if target_joint_angles == None:
            print("Unable to get target joint angles!")

        else:
            print("Setting arm to target joint angles!")
            set_joint_positions(world.robot, world.arm_joints, target_joint_angles)

            target_x, target_y, target_z = tool_target_pos
            
            if user_choice.lower() == 'opened':
                end_pos = ((target_x + 0.4), target_y, target_z)
            else:
                end_pos = ((target_x - 0.4), target_y, target_z)

            target_end_pose = (end_pos, tool_target_quat)

            wait_for_user('Hit enter to open/close drawer')
            
            target_end_joint_angles = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, target_end_pose, max_time=0.05), None)

            if target_end_joint_angles == None:
                print("Unable to get target end joint angles!")
            
            else:
                set_joint_positions(world.robot, world.arm_joints, target_end_joint_angles)

                if user_choice.lower() == 'opened':
                    drawer_pos = 0.4
                else:
                    drawer_pos = 0

                set_joint_position(world.kitchen, joint_from_name(world.kitchen, 'indigo_drawer_top_joint'), drawer_pos)

    elif action_option == 11:
        if active_attachment == None:
            active_attachment = create_attachment(world.robot, link_from_name(world.robot, 'panda_hand'), world.get_body('sugar_box0'))
        else:
            active_attachment = None

    elif action_option == 12:
        drawer_link = link_from_name(world.kitchen, 'indigo_drawer_handle_top')
        surface_attachment = create_surface_attachment(world, 'potted_meat_can1', 'indigo_drawer_handle_top')
        if drawer_pos == 0:
            set_joint_position(world.kitchen, joint_from_name(world.kitchen, 'indigo_drawer_top_joint'), 0.4)
            drawer_pos = 0.4
        else:
            set_joint_position(world.kitchen, joint_from_name(world.kitchen, 'indigo_drawer_top_joint'), 0)
            drawer_pos = 0
        surface_attachment.assign()

    elif action_option == 13:
        user_obj = input("What object would you like to work with? ")
        user_obj = user_obj.strip()

        set_joint_positions(world.robot, world.base_joints, [0.7, 0.67, -1.57])
        start_grip_pose  = ((-0.05807611844574878, 0.5439339828220179, -0.4540235005339055), (-0.2705980500730985, -0.6532814824381882, -0.27059805007309856, 0.6532814824381883))
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, start_grip_pose, max_time=0.05), None)

        if conf != None:
            set_joint_positions(world.robot, world.arm_joints, conf)
        else:
            raise ValueError("Unable to get joint angles!")

        attachment = create_attachment(world.robot, link_from_name(world.robot, 'panda_hand'), world.get_body(user_obj))

        wait_for_user('Ready to move sugar box, hit enter to continue')
        
        tool_pos, tool_quat = get_link_pose(world.robot, tool_link)
        link = link_from_name(world.kitchen, 'back_right_stove')
        surf_pos, surf_quat = get_link_pose(world.kitchen, link)

        actual_grip_height = tool_pos[2] - surf_pos[2]
        print("Actual grip height: ", actual_grip_height)

        link = link_from_name(world.kitchen, 'hitman_countertop')
        target_pose = get_link_pose(world.kitchen, link)
        link_lower, link_upper = get_aabb(world.kitchen, link)
        front_edge_x_pos = max(link_upper[0], link_lower[0])
        
        print("Front edge xposition: ", front_edge_x_pos)
        print("link_lower:", link_lower)
        print("link_upper", link_upper)

        target_pose = (((target_pose[0][0]+0.3), target_pose[0][1], (target_pose[0][2]+actual_grip_height)), quat_from_euler((0, -math.pi/2, 0)) )
        print("Target pose for ", user_obj, ':', target_pose)
        
        set_joint_positions(world.robot, world.base_joints, [0.7, -0.75, -1.57])
        
        conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, target_pose, max_time=0.05), None)

        if conf != None:
            set_joint_positions(world.robot, world.arm_joints, conf)
        else:
            print("Unable to get joint angles!")

        attachment.assign()

    else:
        print(action_option, " is an invalid input option! Please try again.")

wait_for_user()