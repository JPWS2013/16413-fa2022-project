import numpy as np
from pddl_parser.PDDL import PDDL_Parser
import os, sys, traceback, math, time
import MotionPlanner.RRT_star as mp

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, create_surface_attachment, surface_from_name

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_all_links, get_link_names, get_link_inertial_pose, body_from_name, get_pose, get_link_name, get_bodies, dump_body, create_attachment, get_body_name, get_joint_positions, get_position_waypoints, get_quaternion_waypoints, quat_from_euler, euler_from_quat, get_joint_position, set_joint_position

from pybullet_tools.ikfast.ikfast import closest_inverse_kinematics
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF

# import MotionPlanner.RRT_star as mp
import ActivityPlanner.ff_planner as ap

UNIT_POSE2D = (0., 0., 0.)

class ExecutionEngine():
    def __init__(self, problem_filepath, domain_filepath):
        # Parse the PDDL files for activity planning
        self.problem_filepath = problem_filepath
        self.domain_filepath = domain_filepath
        self.parser = self.init_parser()

        # Create the simulated world
        self.world = self.create_world(use_gui=False)

        # Get the various links in the world needed for motion planning later
        self.tool_link = link_from_name(self.world.robot, 'panda_hand')
        self.drawer_joint = joint_from_name(self.world.kitchen, 'indigo_drawer_top_joint')
        self.drawer_link = link_from_name(self.world.kitchen, 'indigo_drawer_handle_top')

        # Map location and object names to locations in the world and also get the starting (current) position of the robot in the world
        # location_map: A dictionary mapping kitchen locations to (x,y) positions
        # object_map: A dictionary mapping object names to ((x,y,z), quat) poses that the robot tool needs to be in
        # Object_dict: A dictionary mapping object names to the object itself
        # current_pos: The current position of the robot's various joints (joint1 angle, joint2 angle, joint3 angle, joint4 angle, joint5 angle, joint6 angle, joint7 angle, base_x position, base_y position)
        self.location_map, self.object_map, self.object_dict, self.current_pos = self.create_maps()

        # Establish relevant movement constraints
        self.drawer_mvmt_dist = 0.4 #When opening the drawer, drawer only opens 0.4m

        # print("Object map: ", self.object_map)

        # Set up variables to determine attachments of the robot to objects and furniture
        self.active_attachment = None #Determines the object being grasped by the tool (if not None)
        self.drawer_status = None #Determines whether the drawer is being opened or closed (if not None)

        # Initialize the motion planner object for motion planning
        self.motion_planner = mp.MotionPlanner(self.world.robot, self.world.kitchen, self.world.base_joints, self.world.arm_joints, object_dict)

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
        print("Step 2: Generating Motion Plan")
        plan_dict = dict()

        for i, action in enumerate(act_plan):
            
            plan_dict[action.name+str(action.parameters)] = self.plan_action(action)
            wait_for_user()
            
            if i >= 7:
                break

        # Once motion planning is complete, destroy the old world object so that a new one can be created that uses the gui
        self.end()
        self.world = self.create_world(use_gui=True)

        # Reset the variables keeping track of the current pos to the starting point of the robot before starting to execute the plans
        self.current_pos = self.location_map['start_pos']
        self.current_pos = (self.current_pos)+(-math.pi,)
        # print("Current pos before executing plan:" , self.current_pos)

        print("Step 3: Executing Plan")
        self.execute_plan(plan_dict)

    def execute_plan(self, plan_dict):
        for action, (target, base_path, arm_path) in plan_dict.items():
            print("Executing ", action)
            wait_for_user()
            if 'move' in action.lower():
                # base_path, arm_path = values
                self.move_robot(base_path, arm_path)
            
            elif ('open' in action.lower()) or ('close' in action.lower()):
                arm_path1, arm_path2 = arm_path
                self.move_robot(base_path, arm_path1)
                if 'open' in action.lower():
                    self.drawer_status = 'opening'
                else:
                    self.drawer_status = 'closing'

                self.move_robot(base_path, arm_path2)
                self.drawer_status = None

            elif 'grip' in action.lower():
                if not ((base_path==None) and (arm_path==None)):
                    self.move_robot(base_path, arm_path)

                
                print("Creating attachment to ", target)

                self.active_attachment = create_attachment(self.world.robot, link_from_name(self.world.robot, 'panda_hand'), self.world.get_body(target))

                # self.active_attachment = self.attachments[target]

            elif ('placein' in action) or ('placeon' in action):
                if not ((base_path==None) and (arm_path==None)):
                    self.move_robot(base_path, arm_path)
                self.active_attachment = None

            else:
                raise ValueError("Action does not match pre-mapped actions in execute_plan function")

    def update_robot_position(self, arm_angles, base_pos):

        self.current_pos = arm_angles + base_pos

        set_joint_positions(self.world.robot, self.world.arm_joints, self.current_pos[:7])

        set_joint_positions(self.world.robot, self.world.base_joints, base_pos)

    def get_target_joint_angles(self, target_pose):
        target_joint_angles = next(closest_inverse_kinematics(self.world.robot, PANDA_INFO, self.tool_link, target_pose, max_time=0.05), None)

        if target_joint_angles == None:
            raise ValueError("Unable to get joint angles!")
        else:
            return target_joint_angles


    def plan_action(self, action):
        print("    - Planning for action: ", action.name, action.parameters[1:])

        if action.name == 'move':
            arm, start_pos_name, end_pos_name = action.parameters
            
            if end_pos_name in self.location_map.keys():
                end_pos = self.location_map[end_pos_name]

            # elif end_pos_name in self.object_map.keys():
            #     end_pos = self.object_map[end_pos_name, 'a']
            else:
                raise ValueError(end_pos_name + " not in location map!")
            
            base_path, arm_path = self.motion_planner.plan(end_pos, 'b')

            self.update_robot_position(self.current_pos[0:7], base_path[-1])

            # set_joint_positions(self.world.robot, self.world.base_joints, (base_path[0]+(-math.pi/2)))
            
            return (None, base_path, None)

        if (action.name == 'open') or (action.name == 'close'):
            arm, target = action.parameters


            if target == "indigo_drawer_top":
                orig_target_pose = get_link_pose(self.world.kitchen, self.drawer_link)
            else:
                raise ValueError("Unrecognized target for opening: " + str(target))

            orig_target_pos, orig_target_quat = orig_target_pose
            orig_target_x, orig_target_y, orig_target_z = orig_target_pos
            
            tool_target_pos = ((orig_target_x+0.1), orig_target_y, orig_target_z)
            
            orig_target_euler = euler_from_quat(orig_target_quat)
            tool_target_euler = (-orig_target_euler[0], orig_target_euler[1], (0.5*math.pi))
            tool_target_quat = quat_from_euler(tool_target_euler)

            tool_target_pose = (tool_target_pos, tool_target_quat)

            target_joint_angles = self.get_target_joint_angles(tool_target_pose)

            base_path1, arm_path1 = self.motion_planner.plan(target_joint_angles, 'a')

            self.update_robot_position(arm_path1[0], self.current_pos[7:])

            tool_start_x, tool_start_y, tool_start_z = tool_target_pos
            
            if action.name == 'open':
                end_pos = ((tool_start_x + self.drawer_mvmt_dist), tool_start_y, tool_start_z)
            else:
                end_pos = ((tool_start_x - self.drawer_mvmt_dist), tool_start_y, tool_start_z)

            target_end_pose = (end_pos, tool_target_quat)

            target_end_joint_angles = self.get_target_joint_angles(target_end_pose)

            base_path2, arm_path2 = self.motion_planner.plan(target_end_joint_angles, 'a')

            self.update_robot_position(arm_path2[0], self.current_pos[7:])

            return (target, None, (arm_path1, arm_path2))

        if action.name == 'grip':
            arm, target, location = action.parameters

            target_pose = self.object_map[target]
            location_pos = self.location_map[location]
            print("Current robot base position: ", get_joint_positions(self.world.robot, self.world.base_joints))

            target_joint_angles = self.get_target_joint_angles(target_pose)

            if self.current_pos[:7] == target_joint_angles:
                return (target, None, None)
                
            else:
                base_path, arm_path = self.motion_planner.plan(target_joint_angles, 'a')
                self.update_robot_position(arm_path[0], self.current_pos[7:])
                
                return(target, None, arm_path)

        if (action.name == 'placein') or (action.name == 'placeon'):
            arm, target, location = action.parameters

            link = link_from_name(self.world.kitchen, location)
            target_pose = get_link_pose(self.world.kitchen, link)
            target_pose = ((target_pose[0][0], target_pose[0][1], (target_pose[0][2]+0.1)), target_pose[1] )
            print("Target pose for ", target, ':', target_pose)
            target_joint_angles = self.get_target_joint_angles(target_pose)

            base_path, arm_path = self.motion_planner.plan(target_joint_angles, 'a')
            self.update_robot_position(arm_path[0], self.current_pos[7:])
                
            return(target, None, arm_path)

    def update_objects(self, delta_x=None):
        if self.active_attachment:
            self.active_attachment.assign()

        if self.drawer_status:
            curr_drawer_pos = get_joint_position(self.world.kitchen, self.drawer_joint)
            if self.drawer_status == 'opening':
                set_joint_position(self.world.kitchen, self.drawer_joint, (curr_drawer_pos+abs(delta_x)))
            else:
                set_joint_position(self.world.kitchen, self.drawer_joint, (curr_drawer_pos-abs(delta_x)))

    def move_robot(self, base_path, arm_path):
        if base_path:
            # base_path.reverse()
            
            for next_base_point in base_path:

                set_joint_positions(self.world.robot, self.world.base_joints, next_base_point)
                self.update_objects()
                self.current_pos = self.current_pos[:7] + next_base_point

                time.sleep(0.05)
            

        if arm_path:
            # arm_path.reverse()

            for arm_point in arm_path:
                current_tool_pose = get_link_pose(self.world.robot, self.tool_link)
                set_joint_positions(self.world.robot, self.world.arm_joints, arm_point)
                new_tool_pose = get_link_pose(self.world.robot, self.tool_link)

                delta_x = new_tool_pose[0][0] - current_tool_pose[0][0]
                
                self.update_objects(delta_x)

                time.sleep(0.5)

    
    def get_activity_plan(self):
        grounded_actions = ap.ground_actions(self.parser)
        selected_actions = ap.enforced_hill_climb(self.parser.state, self.parser, grounded_actions)

        return selected_actions

    def init_parser(self):
        parser = PDDL_Parser()

        parser.parse_domain(self.domain_filepath)
        parser.parse_problem(self.problem_filepath)

        return parser

    def create_world(self,use_gui=False, create_att=False):
        
        add_sugar_box = lambda world, **kwargs: self.add_ycb(world, 'sugar_box', **kwargs)
        add_spam_box = lambda world, **kwargs: self.add_ycb(world, 'potted_meat_can', **kwargs)
        
        np.set_printoptions(precision=3, suppress=True)
        world = World(use_gui=use_gui)
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
    
    def create_maps(self):
        locations = self.parser.objects['location']
        items = self.parser.objects['item']
        
        location_map = dict()
        object_map = dict()
        object_dict = dict()

        # Get the 3D coordinates for all locations in the kitchen
        for loc in locations:
            try:
                if loc == 'start_pos':
                    base_init = get_joint_positions(self.world.robot, self.world.base_joints)[0:2]
                    arm_init = get_joint_positions(self.world.robot, self.world.arm_joints)
                    location_map[loc] = arm_init + base_init
                else:
                    link = link_from_name(self.world.kitchen, loc)
                    coord_x, coord_y, coord_z = get_link_pose(self.world.kitchen, link)[0]

                    #For all indigo locations (countertop an drawer), stop beside the drawer so that the robot isn't blocking the drawer
                    if "indigo" in loc:
                        location_map[loc] = (0.7, 0.6)
                    else:
                    #For locations in the kitchen, park just in front of the counter which is about +0.7, leave y coordinate alone
                        location_map[loc] = (0.7, coord_y)

                    # print("Location ", loc, " has coordinates: ", location_map[loc])

            except ValueError as e:
                print("Error getting coordinates for the following link: ", e, " Exiting!")
                raise ValueError(e)


        for each_item in items:
            try:
                body = self.world.get_body(each_item)
                target_coords = self.get_target_object_pose(body, each_item)
                print("Item ", each_item, "has coordinates ", target_coords)
                object_map[each_item] = target_coords
                object_dict[each_item] = body
            
            except ValueError as e:
                print("Error getting coordinates for the following link: ", e, "Exiting!")
                raise ValueError(e)

        return location_map, object_map, object_dict, arm_init + base_init

    def get_target_object_pose(self, body, name):
        #TODO: Get the actual body pose from the world using get_pose(body)[0]
        #Example code:
        # x_backoff = (0.1*math.cos(euler_angles[2])) - (-0.025*math.sin(euler_angles[2]))
        # y_backoff = (0.1*math.sin(euler_angles[2])) + (-0.025*math.cos(euler_angles[2]))

        # euler_angles = (euler_angles[0], (-0.5*math.pi), euler_angles[2])
        # coord = ((coord[0]+x_backoff), (coord[1]+y_backoff), (coord[2]+0.05)) #Raise z by 0.05 to be at the right height to grip object
        # pose = (coord, quat_from_euler(euler_angles))
        # conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)


        # name = get_body_name(body)

        if "potted_meat" in name:
            return ((0.26717514421272204, 0.9903984489160852, -0.4970404981710017), (-0.2705980500730985, -0.6532814824381882, -0.27059805007309856, 0.6532814824381883))

        
        elif "sugar_box" in name:
            return ((-0.11514718625761429, 0.5510050506338834, -0.4540235005339055), (-0.2705980500730985, -0.6532814824381882, -0.27059805007309856, 0.6532814824381883))
        
        else:
            raise ValueError(name)
        
    def execute(motion_plan):
        pass

if __name__ == "__main__":
    
    problem_file = 'ActivityPlanner/pb1.pddl'
    domain_file = 'ActivityPlanner/kitchen.pddl'

    engine = ExecutionEngine(problem_file, domain_file)

    try:
        engine.run()
        wait_for_user()
        engine.end()
    except Exception as e:
        engine.end()
        print(traceback.format_exc())