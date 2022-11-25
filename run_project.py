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

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_all_links, get_link_names, get_link_inertial_pose, body_from_name, get_pose, get_link_name, get_bodies, dump_body, create_attachment, get_body_name, get_joint_positions, get_position_waypoints, get_quaternion_waypoints, quat_from_euler, euler_from_quat

# import MotionPlanner.RRT_star as mp
import ActivityPlanner.ff_planner as ap

UNIT_POSE2D = (0., 0., 0.)

class ExecutionEngine():
    def __init__(self, problem_filepath, domain_filepath):
        self.problem_filepath = problem_filepath
        self.domain_filepath = domain_filepath
        self.parser = self.init_parser()
        self.world = self.create_world(use_gui=False)

        self.drawer_mvmt_dist = 0.2

        self.location_map, self.object_map, self.current_pos = self.create_maps()

        print("Object map: ", self.object_map)

        object_dict = dict()

        for item_name in self.object_map.keys():
            object_dict[item_name] = self.world.get_body(item_name)


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
            
            if i >= 4:
                break
            # start_pos, end_pos = get_positions()
            # motion_plan = self.mp.solve(start_pos, end_pos)
            # self.execute(motion_plan)

        self.end()

        self.world = self.create_world(use_gui=True, create_att=True)

        # print("Plan dict: ", plan_dict)

        self.current_pos = self.location_map['start_pos'][7:]
        self.current_pos = (self.current_pos)+(-math.pi,)
        print("Current pos before executing plan:" , self.current_pos)
        
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
                self.active_attachment = self.attachments[target]
                self.move_robot(base_path, arm_path)
                self.active_attachment = None

            elif 'grip' in action.lower():
                if not ((base_path==None) and (arm_path==None)):
                    self.move_robot(base_path, arm_path)

                self.active_attachment = self.attachments[target]

            elif ('placein' in action) or ('placeon' in action):
                self.active_attachment = None

            else:
                raise ValueError("Action does not match pre-mapped actions in execute_plan function")

    def update_robot_position(self, arm_angles, base_pos):

        self.current_pos = arm_angles + base_pos

        set_joint_positions(self.world.robot, self.world.arm_joints, self.current_pos[:7])

        set_joint_positions(self.world.robot, self.world.base_joints, (base_pos+(-math.pi/2,)))


    def plan_action(self, action):
        print("    - Planning for action: ", action.name, action.parameters[1:])

        if action.name == 'move':
            arm, start_pos_name, end_pos_name = action.parameters
            
            if end_pos_name in self.location_map.keys():
                end_pos = self.location_map[end_pos_name]
                base_path, arm_path = self.motion_planner.plan(end_pos, 'b')

            # elif end_pos_name in self.object_map.keys():
            #     end_pos = self.object_map[end_pos_name, 'a']
            else:
                raise ValueError(end_pos_name + " not in location map!")
            
            self.update_robot_position(self.current_pos[0:7], base_path[0])

            # set_joint_positions(self.world.robot, self.world.base_joints, (base_path[0]+(-math.pi/2)))
            
            return (None, base_path, None)

        if (action.name == 'open') or (action.name == 'close'):
            arm, target = action.parameters
            target_x, target_y, target_z = self.object_map[target]
            
            if action.name == 'open':
                end_pos = ((target_x + self.drawer_mvmt_dist), target_y, target_z)
            else:
                end_pos = ((target_x - self.drawer_mvmt_dist), target_y, target_z)
            base_path, arm_path = self.motion_planner.plan(end_pos, 'a')
            
            self.update_robot_position(arm_path[0], self.current_pos[7:])

            return (target, None, arm_path)

        if action.name == 'grip':
            arm, target, location = action.parameters

            target_pos = self.object_map[target]
            location_pos = self.location_map[location]

            if self.current_pos[7:] == location_pos:
                return (target, None, None)
                
            else:
                base_path, arm_path = self.motion_planner.plan(target_pos, 'a')
                self.update_robot_position(arm_path[0], self.current_pos[7:])
                
                return(target, None, arm_path)

        if (action.name == 'placein') or (action.name == 'placeon'):
            arm, target, location = action.parameters
            return (target, None, None)


    def move_robot(self, base_path, arm_path):
        if base_path:
            base_path.reverse()
            
            for next_base_point in base_path:
                print("Current point: ", self.current_pos)
                
                dir_vec = np.array(next_base_point)- np.array(self.current_pos[:2])

                delta_theta = math.atan(dir_vec[1]/dir_vec[0])

                if (dir_vec[0]>0):
                    new_theta = delta_theta
                else:
                    new_theta = -math.pi + delta_theta
                
                next_pos = next_base_point+(new_theta,)

                print("Dir vec: ", dir_vec, " and delta theta: ", delta_theta, "(", math.degrees(delta_theta), "degrees)")
                print("Next point: ", next_pos, "(New theta of ", math.degrees(new_theta), "degrees)")

                next_pos_quat = quat_from_euler((0,0,new_theta))

                if new_theta != self.current_pos[2]:
                    
                    # print("Rotating base first!")
                    # wait_for_user()

                    for point, next_quat in get_quaternion_waypoints(self.current_pos[:2], quat_from_euler((0,0,self.current_pos[2])), next_pos_quat, step_size = math.pi/32):
                        set_joint_positions(self.world.robot, self.world.base_joints, (point+(euler_from_quat(next_quat)[2],)))
                        time.sleep(0.05)

                self.current_pos = self.current_pos[:2] + (new_theta,)
                # print("Translating base now")
                # wait_for_user()

                for next_point, quat in get_position_waypoints(self.current_pos[:2], dir_vec, next_pos_quat):
                    next_point = tuple(next_point)
                    # print("Next point: ", next_point)
                    set_joint_positions(self.world.robot, self.world.base_joints, (next_point + (new_theta,)))
                    time.sleep(0.01)

                self.current_pos = next_base_point + (new_theta,)
                
                if self.active_attachment:
                    self.active_attachment.assign()

                
                # time.sleep(0.1)

            # set_joint_positions(self.world.robot, self.world.base_joints, (self.current_pos[:2]+(-math.pi/2,)))

            for point, next_quat in get_quaternion_waypoints(self.current_pos[:2], quat_from_euler((0,0,self.current_pos[2])), quat_from_euler((0,0,(-math.pi/2))), step_size=math.pi/32):
                        set_joint_positions(self.world.robot, self.world.base_joints, (point+(euler_from_quat(next_quat)[2],)))
                        time.sleep(0.05)
            

        if arm_path:
            arm_path.reverse()

            for arm_point in arm_path:
                set_joint_positions(self.world.robot, self.world.arm_joints, arm_point)
                if self.active_attachment:
                    self.active_attachment.assign()
                time.sleep(1)

    
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

        if create_att:
            self.attachments = dict()

            self.attachments[sugar_box] = create_attachment(self.world.robot, link_from_name(world.robot, 'panda_hand'), self.world.get_body(sugar_box))

            self.attachments[spam_box] = create_attachment(self.world.robot, link_from_name(world.robot, 'panda_hand'), self.world.get_body(spam_box))

            # self.attachments['indigo_drawer_top'] = create_attachment(self.world.robot, link_from_name(world.robot, 'panda_hand'), surface_from_name('indigo_drawer_top'))
            print('Attachments:', self.attachments)

        self.active_attachment = None
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

                    #For locations in the kitchen, park just in front of the counter which is about +0.8, leave y and z coordinates alone
                    location_map[loc] = (0.7, coord_y)

                    # print("Location ", loc, " has coordinates: ", location_map[loc])

            except ValueError as e:
                print("Error getting coordinates for the following link: ", e, " Exiting!")
                sys.exit(1)

        for each_item in items:
            try:
                body = self.world.get_body(each_item)
                target_coords = self.get_target_object_pose(body, each_item)
                print("Item ", each_item, "has coordinates ", target_coords)
                object_map[each_item] = target_coords
            
            except ValueError as e:
                print("Error getting coordinates for the following link: ", e, "Exiting!")
                raise ValueError(e)

        # print("Location map: ", location_map)
        # print("Object map: ", object_map)

        return location_map, object_map, arm_init + base_init

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
            return (1.465, -1.727, -1.754, -2.252, -0.022, 2.948, -1.004, 0.8, 1.0)
        
        elif "sugar_box" in name:
            return (-0.788, 1.423, -1.229, -1.052, -0.624, 2.531, 2.740, 0.7, 0.65)
        
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

    