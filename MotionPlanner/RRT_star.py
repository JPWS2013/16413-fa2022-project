from ipaddress import collapse_addresses
import numpy as np
from scipy.spatial import KDTree
import sys, os, math
import matplotlib.pyplot as plt

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, quat_from_euler, get_quaternion_waypoints, euler_from_quat, get_position_waypoints, joint_from_name, get_joint_limits
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions, body_collision
from src.utils import translate_linearly

# from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
# from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics


class TreeNode(object):
    def __init__(self, pos, theta=None, cost=0, parent=None, path=None):
        self.parent = parent
        self.pos = pos
        self.interpolated_path = path
        self.cost = cost
        self.theta = theta

class MotionPlanner:
    def __init__(self, robot, kitchen, base_joints, arm_joints, kitchen_items, iterations=100000, base_d=0.75, arm_d = 0.5, goal_int=20, goal_biasing=True, run_rrtstar=False, arm_goal_radius = 0.05, base_goal_radius = 0.021):
        self.robot = robot
        self.kitchen = kitchen
        self.base_joints = base_joints
        self.arm_joints = arm_joints
        self.kitchen_items = kitchen_items
        self.iterations = iterations
        self.base_d = base_d
        self.arm_d = arm_d
        self.goal_int = goal_int
        self.goal_biasing = goal_biasing
        self.run_rrtstar = run_rrtstar

        base_x_joint = joint_from_name(self.robot, 'x')

        base_x_lim_lower, base_x_lim_upper = get_joint_limits(self.robot, base_x_joint)

        # print("Joint", get_joint_name(self.robot, base_y_joint), "has limits: ", base_y_lim_lower, base_y_lim_upper)

        base_custom_limits = {
            base_x_joint: (0, base_x_lim_upper)
        }
        self.get_random_arm_sample = self.get_sample_fn(self.robot, self.arm_joints)
        self.get_random_base_sample = self.get_sample_fn(self.robot, self.base_joints, custom_limits = base_custom_limits)
        self.random_goal_sample = None
        # self.random_base_goal_sample = None
        self.arm_goal_tolerance = arm_goal_radius
        self.base_goal_radius = base_goal_radius
    
    # Create a function to find the distance between two points
    def dist(self, a, b):
        diff = (a[0]-b[0], a[1]-b[1], a[2]-b[2])
        norm = np.linalg.norm(diff)
        return norm

    # Create a function to place x_new between x_rand and x_nearest based on d
    def steer(self, rand_point, nearest_point, body_to_plan):
        if body_to_plan == 'a':
            d = self.arm_d
        else:
            d = self.base_d

        rand_vec = np.array(rand_point)
        nearest_vec = np.array(nearest_point.pos)
        dir_vec = rand_vec - nearest_vec
        mag = np.linalg.norm(dir_vec)
        # if body_to_plan == 'a':
        #     print('rand_vec=', rand_vec, ', nearest_vec=', nearest_vec, 'dir_vec=', dir_vec, 'magnitude: ', mag)
        if d > mag: # If d is greater than the magnitude of the vector make x_new x_rand
            x_new = rand_point
            # print('Magnitude smaller than d. x_new = x_rand')
        else: # Else scale the vector to the magnitude d
            coeff = d/mag
            scaled_dir_vec= coeff*(dir_vec)
            x_new = nearest_vec + scaled_dir_vec
            # print('Magnitude larger than d. x_new =', x_new)
        return tuple(x_new)

    def get_nearest_node(self, point, G):
        """
        Uses scipy's KD Tree implementation to get the nearest node to the point

        Note: Although nodes in G are defined as (x,y,theta), the kdtree is built based only on (x,y)

        Inputs:
            point: (x,y) tuple representing the point for which nearest neighbor must be found
        Returns:
            The (x,y) tuple of the nearest point in the tree
        """
        
        points = np.array([node.pos for node in G])
        kdtree = KDTree(points)

        d,i = kdtree.query(point, k=1)

        for node in G:
            if node.pos == tuple(points[i]):
                return node
        
    def get_sample_fn(self, body, joints, custom_limits={}, **kwargs):
        lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
        generator = interval_generator(lower_limits, upper_limits, **kwargs)
        def fn():
            return tuple(next(generator))
        return fn
    
    def connect_path(self, near_list, x_min, c_min, x_new):
        for node in near_list:
            if (collison_free(node, x_new)) and (node.cost + self.c(node, x_new) < c_min):
                x_min = node
                cmin = node.cost + self.c(node, x_new)
        return (x_min, c_min)

    def rewire(self, G, X_near, x_new):
        return G

    def c(self, start, end):
        start_vec = np.array(start)
        end_vec = np.array(end)
        dir_vec = end_vec - start_vec
        return np.linalg.norm(dir_vec)

    # def collision_free(self, a, b):
    #     return collision_free

    def get_random_sample(self, body_to_plan, sample_goal = False):
        if not sample_goal: 
            if body_to_plan == 'a':
                rand_joints = self.get_random_arm_sample()
            elif body_to_plan == 'b':
                rand_joints = self.get_random_base_sample()[:2]

        else:
            if body_to_plan == 'a':
                rand_joints = self.random_goal_sample()
            elif body_to_plan == 'b':
                rand_joints = self.random_goal_sample()[:2]
                

        return rand_joints

    def get_new_theta(self, start, end):
        dir_vec = np.array(end)- np.array(start)

        delta_theta = math.atan(dir_vec[1]/dir_vec[0])

        if (dir_vec[0]>0):
            new_theta = delta_theta
        else:
            new_theta = -math.pi + delta_theta

        return new_theta
    
    def collision_free(self):

        # Check if robot is in collision with items in the kitchen
        for item_name, item_obj in self.kitchen_items.items():
            if body_collision(self.robot, item_obj):
                return False
        # Check if robot is in collision with itself
        # autorobotic_collision = body_collision(self.robot, self.robot)
        # print('self.robot', self.robot)
        if body_collision(self.robot, self.kitchen):
            return False

        return True


    def obst_free(self, x_nearest, x_new, body_to_plan):
        # print('self.robot', self.robot)
        # collisions = body_collision(self.robot, self.kitchen, max_distance=0.01)
        # if not collisions:
        #     collisions = body_collision(self.robot, kitchen_items['potted_meat_can1'], max_distance=0.01)
        # if not collisions:
        #     collisions = body_collision(self.robot, kitchen_items['sugar_box0'], max_distance=0.01)
        interpolated_path = list()

        if body_to_plan == 'a':
            set_joint_positions(self.robot, self.arm_joints, x_new)
            return self.collision_free(), [x_new], None
        elif body_to_plan == 'b':
            # theta = get_joint_positions(self.robot, self.base_joints)[2]
            current_theta = x_nearest.theta

            dir_vec = np.array(x_new)- np.array(x_nearest.pos)

            delta_theta = math.atan(dir_vec[1]/dir_vec[0])

            if (dir_vec[0]>0):
                new_theta = delta_theta
            else:
                new_theta = -math.pi + delta_theta

            new_quat = quat_from_euler((0,0,new_theta))
            new_pos = (x_new + (new_theta,))

            #Rotate the base first if the current base heading doesn't match the next heading
            if new_theta != x_nearest.theta:
                for point, next_quat in get_quaternion_waypoints(x_nearest.pos, quat_from_euler((0,0,x_nearest.theta)), new_quat, step_size = math.pi/32):
                    next_pos = point+(euler_from_quat(next_quat)[2],)
                    set_joint_positions(self.robot, self.base_joints, next_pos)
                    interpolated_path.append(next_pos)
                    if not self.collision_free():
                        return False, None, None

            #Then translate the base
            for next_point, quat in get_position_waypoints(x_nearest.pos, dir_vec, new_quat, step_size=0.05):
                next_point = tuple(next_point)
                next_pos = (next_point + (new_theta,))
                # print("Next point: ", next_point)
                set_joint_positions(self.robot, self.base_joints, next_pos)
                interpolated_path.append(next_pos)

                if not self.collision_free():
                    return False, None, None

            return True, interpolated_path, new_theta

    def retrive_path(self, x_new):
        current_pos = x_new
        path = []
        while current_pos.parent != None:  
            path = current_pos.interpolated_path + path
            current_pos = current_pos.parent

        # if (path[0][2] <= 0) or (path[0][2] >= -math.pi):
        #     final_rotation_path = [ (point + (euler_from_quat(next_quat)[2],)) for point, next_quat in get_quaternion_waypoints(path[0][:2], path[0][2], quat_from_euler(0,0,-math.pi/2), step_size = math.pi/32) ]
        # else:
        #     final_rotation_path = [ (point + (euler_from_quat(next_quat)[2],)) for point, next_quat in get_quaternion_waypoints(path[0][:2], path[0][2], quat_from_euler(0,0,math.pi/2), step_size = math.pi/32) ]
        
        # path += final_rotation_path
        
        return path

    def in_end_region(self, x_new, end_pos, body_to_plan):
        # print('Goal: ', end_pos)
        # print('x_new:', x_new)
        # set_joint_positions(self.robot, self.arm_joints, x_new)
        # x_new_cart = get_link_pose(self.robot, link_from_name(self.robot, 'panda_hand'))[0]
        # print('x_new_cart: ', x_new_cart)

        if body_to_plan == 'a':
            for sample_j_angle, goal_j_angle in zip(x_new, end_pos):
                if abs(sample_j_angle-goal_j_angle)>self.arm_goal_tolerance:
                    return False
            return True

        elif body_to_plan == 'b':
            diff = np.linalg.norm(np.array(x_new) - np.array(end_pos))
            # print('Diff =', diff)
            if abs(diff) < self.base_goal_radius:
                return True
            else:
                return False

    def create_goal_limits(self, end_pos, body_to_plan):
        limits_dict = dict()

        if body_to_plan == 'a':
            for joint, target_angle in zip(self.arm_joints, end_pos):
                limits_dict[joint] = ((target_angle-self.arm_goal_tolerance), (target_angle + self.arm_goal_tolerance))

        elif body_to_plan == 'b':
            for joint, target_pos in zip(self.base_joints[:2], end_pos[:2]):

                if get_joint_name(self.robot, joint) == 'x':
                    limits_dict[joint] = (target_pos, (target_pos + self.base_goal_radius))

                else:
                    limits_dict[joint] = ((target_pos-self.base_goal_radius), (target_pos + self.base_goal_radius))

        # print("Limits dict: ", limits_dict)

        return limits_dict
    
    def solve(self, start_pos_input, end_pos_input, body_to_plan):

        print("Starting position: ", start_pos_input)
        print("Goal position: ", end_pos_input)

        if body_to_plan == 'b':
            start_pos = start_pos_input[:2]
            start_heading = start_pos_input[2]
            end_pos = end_pos_input[:2]
        else:
            start_pos = start_pos_input
            end_pos = end_pos_input

        goal_joint_limits = self.create_goal_limits(end_pos, body_to_plan)

        if body_to_plan == 'a':
            self.random_goal_sample = self.get_sample_fn(self.robot, self.arm_joints, custom_limits=goal_joint_limits)
        elif body_to_plan == 'b':
            self.random_goal_sample = self.get_sample_fn(self.robot, self.base_joints, custom_limits=goal_joint_limits)

        path = None

        V = [start_pos] # Create a list of node positions

        if body_to_plan == 'b':
            G = [TreeNode(start_pos, start_heading)] # Create a list of TreeNode objects
        else:
            G = [TreeNode(start_pos)]
        found = 0 # Variable to keep track of if we've made it to the goal
        cart_pos_t = []

        # print("List of vertices: ", V)

        wait_for_user("Hit enter to start planning")

        # print("Start position: ", G[0].pos)
        # base_pos = get_joint_positions(self.robot, self.base_joints)
        # print('Starting base position: ', base_pos)

        for i in range(self.iterations): # Iterate
            if i % 200 == 0:
                print("i=", i)

            if (i % 1000 == 0) and (i != 0) and (body_to_plan == 'b'):
                x_t, y_t = zip(*V)
                x_t = [-x for x in x_t]
                fig, ax = plt.subplots()
                ax.scatter(y_t, x_t)
                ax.scatter(end_pos[1], -end_pos[0], c='r')
                cir = plt.Circle((end_pos[1], -end_pos[0]), self.base_goal_radius, color='r',fill=False, linestyle = '--')
                ax.add_patch(cir)
                plt.show()

            if (self.goal_biasing) and (i % self.goal_int == 0): # Every 20 iterations take the random sample from inside the goal area (goal biasing)
                # print("Sampling from goal region")
                x_rand = self.get_random_sample(body_to_plan, sample_goal=True)
                # print("Sample from goal: ", x_rand)
                # if body_to_plan == 'b':
                #     rand_joints = rand_joints + self.get_random_base_sample(sample_goal=True)[0:2]
            else: # Else take the random sample from somewhere in the operating area
                x_rand = self.get_random_sample(body_to_plan)
            # print('Random sample obtained: ', x_rand)
            # wait_for_user()
            x_nearest = self.get_nearest_node(x_rand, G)
            # print('X_nearest: ', x_nearest.pos)
            x_new = self.steer(x_rand, x_nearest, body_to_plan) # Use the stter function to make x_new's position
            # wait_for_user()
            #This runs rrt* instead of rrt
            if self.run_rrtstar:
                continue
                # if ObstacleFree(xnearest, xnew):
                #     X_near = self.near(G = (V, E), xnew, min{gamma_RRG*(log(card V)/ card V)^(1/d), nu})
                #     V.append(x_new)
                #     x_min = xnearest
                #     c_min = x_nearest.cost + self.c(xnearest, xnew)
                #     (x_min, c_min) = self.connect_path(X_near, x_min, c_min)
                #     G.append(TreeNode(x_new, c_min, x_min))
                #     G = self.rewire(G, X_near, x_new)

                #     if self.in_end_region(x_new): # If goal is reached
                #         (found, path)= self.retrive_path(G, end_pos) 
            
            #This runs rrt instead of rrt*
            else:
                is_obstacle_free, interpolated_path, new_theta = self.obst_free(x_nearest, x_new, body_to_plan)
                # print('Is new arm position obstacle free? ', is_obstacle_free)
                if is_obstacle_free and (x_new not in V):
                    # print("Obstacle free point: ", x_new)
                    V.append(x_new)
                    obj = TreeNode(x_new, theta=new_theta, parent=x_nearest, path = interpolated_path)
                    G.append(obj)
                    cart_pos_t.append(get_link_pose(self.robot, link_from_name(self.robot, 'panda_hand'))[0])


                    if self.in_end_region(x_new, end_pos, body_to_plan):
                        path = self.retrive_path(obj)
                        break
        if path:
            # print("Path before returning: ", path)
            return path
        else:
            return None

    def plan(self, end_pos, body_to_plan):
        # hand_pos_vec = np.array(get_link_pose(self.robot, link_from_name(self.robot, 'panda_hand'))[0])

        # end_pos_vec = np.array(end_pos)

        # if np.linalg.norm(end_pos_vec - hand_pos_vec) > self.base_goal_radius:
        #     print("Arm can't reach the goal. Moving the base first")
        #     path_base = self.solve(end_pos, 'b')
            
        #     return None, path_arm
        
        path_arm = None
        path_base = None

        # print("start_pos in plan function: ", start_pos)
        

        if body_to_plan == 'b':
            start_pos = get_joint_positions(self.robot, self.base_joints)
            path_base = self.solve(start_pos, end_pos, body_to_plan)
            if path_base == None:
                raise ValueError("No feasible path for base found!")
        
        elif body_to_plan == 'a':  
            start_pos = get_joint_positions(self.robot, self.arm_joints)  
            path_arm = self.solve(start_pos, end_pos[0:7], body_to_plan)
            if path_arm == None: 
                raise ValueError("No feasible path for arm found!")


        print("PLAN COMPLETE!")
        return path_base, path_arm




