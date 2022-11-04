import numpy as np
from pddl_parser.PDDL import PDDL_Parser
import os, sys

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_all_links, get_link_names, get_link_inertial_pose

import MotionPlanner.RRT_star as mp

UNIT_POSE2D = (0., 0., 0.)

class ExecutionEngine():
    def __init__(self, problem_filepath, domain_filepath):
        self.problem_filepath = problem_filepath
        self.domain_filepath = domain_filepath
        self.parser = self.init_parser()
        self.world = self.create_world()

        self.location_map, self.action_map = self.generate_maps()

        # self.motion_planner = mp.MotionPlanner(world, self.action_map, self.location_map)

    def run():
        act_plan = get_activity_plan(problem_file, domain_file)

        for action in act_plan:
            start_pos, end_pos = get_positions()
            motion_plan = self.mp.solve(start_pos, end_pos)
            self.execute(motion_plan)


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
    
    def generate_maps(self):
        locations = self.parser.objects['location']
        actions = [action.name for action in self.parser.actions]

        action_map = dict()
        location_map = dict()

        # Get the 3D coordinates for all locations in the kitchen
        for loc in locations:
            if loc == 'start_pos':
                continue
            try:
                link = link_from_name(self.world.kitchen, loc)
                coord = get_link_pose(self.world.kitchen, link)[0]
                print("Location ", loc, " has coordinates: ", coord)
                location_map[loc] = coord

            except ValueError as e:
                print("Error getting lcoordinates for the following link: ", e, " Exiting!")
                sys.exit(1)

        # Map the activities to the executable actions
        for action in actions:
            if action == 'open':
                action_map[action] = self.world.open_door
            elif action == 'close':
                action_map[action] = self.world.close_door
            elif action == 'grip':
                action_map[action] = self.world.close_gripper
            elif (action == 'placein') or (action == 'placeon'):
                action_map[action] = self.world.open_gripper
            # elif action == 'move':
            #     action_map[action] = self.mp.solve

        return location_map, action_map

    def execute(motion_plan):
        pass

if __name__ == "__main__":
    
    problem_file = 'ActivityPlanner/pb1.pddl'
    domain_file = 'ActivityPlanner/kitchen.pddl'

    engine = ExecutionEngine(problem_file, domain_file)

    

    