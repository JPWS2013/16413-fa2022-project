import numpy as np
from pddl_parser.PDDL import PDDL_Parser
import os, sys, traceback
import MotionPlanner.RRT_star as mp

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_all_links, get_link_names, get_link_inertial_pose, body_from_name, get_pose, get_link_name, get_bodies, dump_body

# import MotionPlanner.RRT_star as mp
import ActivityPlanner.ff_planner as ap

UNIT_POSE2D = (0., 0., 0.)

class ExecutionEngine():
    def __init__(self, problem_filepath, domain_filepath):
        self.problem_filepath = problem_filepath
        self.domain_filepath = domain_filepath
        self.parser = self.init_parser()
        self.world = self.create_world()

        self.location_map = self.create_location_map()

        self.motion_planner = mp.MotionPlanner(self.world, self.location_map)

    def end(self):
        print("Destroying world object")
        self.world.destroy()

    def run(self):
        print("===============================")
        print("Step 1: Getting activity plan.....")
        act_plan = self.get_activity_plan()

        print("Identified activity plan: ")
        for action in act_plan:
            print(action.name, action.parameters)
        print("===============================")

        # Expected plan:
        # move ('a', 'start_pos', 'hitman_countertop')
        # move ('a', 'hitman_countertop', 'back_right_stove')
        # grip ('a', 'sugar_box0', 'back_right_stove')
        # move ('a', 'back_right_stove', 'hitman_countertop')
        # placeon ('a', 'sugar_box0', 'hitman_countertop')
        # move ('a', 'hitman_countertop', 'indigo_countertop')
        # move ('a', 'indigo_countertop', 'indigo_drawer_top')
        # open ('a', 'indigo_drawer_top')
        # move ('a', 'indigo_drawer_top', 'indigo_countertop')
        # grip ('a', 'potted_meat_can1', 'indigo_countertop')
        # move ('a', 'indigo_countertop', 'indigo_drawer_top')
        # placein ('a', 'potted_meat_can1', 'indigo_drawer_top')
        # close ('a', 'indigo_drawer_top')
        print("Step 2: Executing actions in the activity plan...")
        for action in act_plan:
            self.execute_action(action)
            # start_pos, end_pos = get_positions()
            # motion_plan = self.mp.solve(start_pos, end_pos)
            # self.execute(motion_plan)

    def execute_action(self, action):
        print("    - Executing action: ", action.name, action.parameters[1:])

        wait_for_user()

        if action.name == 'move':
                arm, start_pos_name, end_pos_name = action.parameters
                if start_pos_name == 'start_pos':
                    start_pos = (0,0,0) #TODO get the actual initial position
                    end_pos = self.location_map[end_pos_name]
                path = self.motion_planner.solve(start_pos, end_pos)
                self.move_robot(path)
        #     parameters = action.parameters[1:]
        #     case 'open':
        #         self.world.open_door
        #     case 'close':
        #         self.world.close_door
        #     case 'grip':
        #         self.world.close_gripper
        #     case 'placein' | 'placeon':
        #         self.world.open_gripper

    def move_robot(self, path):
        for single_path in path:
            #set the joint positions
            raise Exception("Function to move the robot not implemented! Code will not execute further!")
            continue

    
    def get_activity_plan(self):
        grounded_actions = ap.ground_actions(self.parser)
        selected_actions = ap.enforced_hill_climb(self.parser.state, self.parser, grounded_actions)

        return selected_actions

    def init_parser(self):
        parser = PDDL_Parser()

        parser.parse_domain(self.domain_filepath)
        parser.parse_problem(self.problem_filepath)

        return parser

    def create_world(self):
        
        add_sugar_box = lambda world, **kwargs: self.add_ycb(world, 'sugar_box', **kwargs)
        add_spam_box = lambda world, **kwargs: self.add_ycb(world, 'potted_meat_can', **kwargs)
        
        np.set_printoptions(precision=3, suppress=True)
        world = World(use_gui=False)
        sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
        spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
        world._update_initial()

        return world

    def add_ycb(self, world, ycb_type, idx=0, counter=0, **kwargs):
        name = name_from_type(ycb_type, idx)
        world.add_body(name, color=np.ones(4))
        self.pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
        return name

    def pose2d_on_surface(self, world, entity_name, surface_name, pose2d=UNIT_POSE2D):
        x, y, yaw = pose2d
        body = world.get_body(entity_name)
        surface_aabb = compute_surface_aabb(world, surface_name)
        z = stable_z_on_aabb(body, surface_aabb)
        pose = Pose(Point(x, y, z), Euler(yaw=yaw))
        set_pose(body, pose)
        return pose
    
    def create_location_map(self):
        locations = self.parser.objects['location']
        items = self.parser.objects['item']
        
        location_map = dict()

        # Get the 3D coordinates for all locations in the kitchen
        for loc in locations:
            if loc == 'start_pos':
                continue
            try:
                link = link_from_name(self.world.kitchen, loc)
                coord = get_link_pose(self.world.kitchen, link)[0]
                # print("Location ", loc, " has coordinates: ", coord)
                location_map[loc] = coord

            except ValueError as e:
                print("Error getting coordinates for the following link: ", e, " Exiting!")
                sys.exit(1)

        for each_item in items:
            try:
                body = self.world.get_body(each_item)
                coord = get_pose(body)[0]
                # print("Item ", each_item, "has coordinates ", coord)
            
            except ValueError as e:
                print("Error getting coordinates for the following link: ", e, "Exiting!")

        return location_map

    def execute(motion_plan):
        pass

if __name__ == "__main__":
    
    problem_file = 'ActivityPlanner/pb1.pddl'
    domain_file = 'ActivityPlanner/kitchen.pddl'

    engine = ExecutionEngine(problem_file, domain_file)

    try:
        engine.run()
        engine.end()
    except Exception as e:
        engine.end()
        print(traceback.format_exc())

    