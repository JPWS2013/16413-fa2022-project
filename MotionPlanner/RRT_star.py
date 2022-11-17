from ipaddress import collapse_addresses
import numpy as np
from scipy.spatial import KDTree
import sys, os, math
import matplotlib.pyplot as plt

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions, body_collision
from src.utils import translate_linearly

# from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
# from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics


class TreeNode(object):
    def __init__(self, pos, cost=0, parent=None):
        self.parent = parent
        self.pos = pos
        self.cost = cost

class MotionPlanner:
    def __init__(self, world, location_map, iterations=10000, d=0.3, goal_int=20, goal_biasing=False, run_rrtstar=False, arm_goal_radius = 0.2, base_goal_radius = 0.7):
        self.world = world
        self.location_map = location_map
        self.iterations = iterations
        self.d = d
        self.goal_int = goal_int
        self.goal_biasing = goal_biasing
        self.run_rrtstar = run_rrtstar
        self.get_random_sample = self.get_sample_fn(self.world.robot, self.world.arm_joints)
        self.get_random_base_sample = self.get_sample_fn(self.world.robot, self.world.base_joints)
        self.goal_radius = arm_goal_radius
        self.base_goal_radius = base_goal_radius

        # goal_pos = translate_linearly(self.world, 1.2) 
        # set_joint_positions(self.world.robot, self.world.base_joints, (0.75, 0, -3.14))
    
    def destroy_world(self):
        # print("Destroying world object")
        self.world.destroy()
    
    # Create a function to find the distance between two points
    def dist(self, a, b):
        diff = (a[0]-b[0], a[1]-b[1], a[2]-b[2])
        norm = np.linalg.norm(diff)
        return norm

    # Create a function to place x_new between x_rand and x_nearest based on d
    def steer(self, rand_point, nearest_point, d):
        rand_vec = np.array(rand_point)
        nearest_vec = np.array(nearest_point.pos)
        dir_vec = rand_vec - nearest_vec
        mag = np.linalg.norm(dir_vec)
        # print('rand_vec=', rand_vec, ', nearest_vec=', nearest_vec, 'dir_vec=', dir_vec, 'magnitude: ', mag)
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

    def obst_free(self, x_nearest, x_new, body_to_plan):
        # collisions = body_collision(self.world.robot, self.world.kitchen, max_distance=0.01)
        # if not collisions:
        #     collisions = body_collision(self.world.robot, self.world.get_body('potted_meat_can1'), max_distance=0.01)
        # if not collisions:
        #     collisions = body_collision(self.world.robot, self.world.get_body('sugar_box0'), max_distance=0.01)
        if body_to_plan == 'a':
            set_joint_positions(self.world.robot, self.world.arm_joints, x_new)
        else:
            theta = get_joint_positions(self.world.robot, self.world.base_joints)[2]
            set_joint_positions(self.world.robot, self.world.base_joints, (x_new + (theta,)))
        return not body_collision(self.world.robot, self.world.kitchen)

    def retrive_path(self, x_new):
        current_pos = x_new
        path = []
        while current_pos.parent != None:  
            path.append(current_pos.pos)
            current_pos = current_pos.parent
        return path

    def in_end_region(self, x_new, end_pos, body_to_plan):
        print('Goal: ', end_pos)
        print('x_new:', x_new)
        if body_to_plan == 'a':
            # set_joint_positions(self.world.robot, self.world.arm_joints, x_new)
            x_new_cart = get_link_pose(self.world.robot, link_from_name(self.world.robot, 'panda_hand'))[0]
            print('x_new_cart: ', x_new_cart)
            diff = np.linalg.norm(np.array(x_new_cart) - np.array(end_pos))
            print('Diff =', diff)
            if abs(diff) < self.goal_radius:
                return True
            else:
                return False
        if body_to_plan == 'b':
            diff = np.linalg.norm(np.array(x_new) - np.array(end_pos))
            print('Diff =', diff)
            if abs(diff) < self.base_goal_radius:
                return True
            else:
                return False


    def solve(self, end_pos, body_to_plan):

        path = None

        if body_to_plan == 'a':
            start_pos = get_joint_positions(self.world.robot, self.world.arm_joints)
        else:
            start_pos = get_joint_positions(self.world.robot, self.world.base_joints)[0:2]

        V = [start_pos] # Create a list of node positions
        G = [TreeNode(start_pos, cost=0, parent=None)] # Create a list of TreeNode objects
        found = 0 # Variable to keep track of if we've made it to the goal
        cart_pos_t = []

        # print("Start position: ", G[0].pos)
        # base_pos = get_joint_positions(self.world.robot, self.world.base_joints)
        # print('Starting base position: ', base_pos)

        for i in range(self.iterations): # Iterate
            print("i=", i)
            if i%1000 == 0:
                # if cart_pos_t:
                #     points = [(x, y) for x,y,z in cart_pos_t]
                #     x_t, y_t = zip(*points)
                #     plt.scatter(y_t, x_t, c='b')
                #     plt.scatter(end_pos[1], end_pos[0], c='r')
                #     plt.show()
                wait_for_user()
            # wait_for_user()
            if (self.goal_biasing) and (i % self.goal_int == 0): # Every 20 iterations take the random sample from inside the goal area (goal biasing)
                if body_to_plan == 'a':
                    rand_joints = self.get_random_sample(sample_goal=True)
                else:
                    rand_joints = self.get_random_base_sample(sample_goal=True)[0:2]
            else: # Else take the random sample from somewhere in the operating area
                if body_to_plan == 'a':
                    rand_joints = self.get_random_sample()
                else:
                    rand_joints = self.get_random_base_sample()[0:2]
            # print('Random sample obtained: ', rand_joints)
            # wait_for_user()
            x_nearest = self.get_nearest_node(rand_joints, G)
            # print('X_nearest: ', x_nearest.pos)
            x_new = self.steer(rand_joints, x_nearest, self.d) # Use the stter function to make x_new's position
            # wait_for_user()
            #This runs rrt* instead of rrt
            if self.run_rrtstar:
                print('Here3')
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
                is_obstacle_free = self.obst_free(x_nearest, x_new, body_to_plan)
                # print('Is new arm position obstacle free? ', is_obstacle_free)
                if is_obstacle_free and (x_new not in V):
                    V.append(x_new)
                    obj = TreeNode(x_new, parent=x_nearest)
                    G.append(obj)
                    # cart_pos_t.append(get_link_pose(self.world.robot, link_from_name(self.world.robot, 'panda_hand'))[0])

                    if self.in_end_region(x_new, end_pos, body_to_plan):
                        path = self.retrive_path(obj)
                        break
        if path:
            return path
        else:
            print('No path found')
            return None

    def plan(self, end_pos):
        hand_pos_vec = np.array(get_link_pose(self.world.robot, link_from_name(self.world.robot, 'panda_hand'))[0])

        end_pos_vec = np.array(end_pos)

        if np.linalg.norm(end_pos_vec - hand_pos_vec) > self.base_goal_radius:
            print("Arm can't reach the goal. Moving the base first")
            path_base = self.solve(end_pos[0:2], 'b')
            print("Setting base pose to ", path_base[0])
            set_joint_positions(self.world.robot, self.world.base_joints, (path_base[0]+(-math.pi,)))
            print("Moving the arm now!")
            path_arm = self.solve(end_pos, 'a')

            return path_base, path_arm
        
        else:
            print("Arm is in reach of the goal. No movement of base. Moving the arm")
            
            path_arm = self.solve(end_pos, 'a')

            return None, path_arm




