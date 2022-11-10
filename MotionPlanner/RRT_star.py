from ipaddress import collapse_addresses
import numpy as np
from scipy.spatial import KDTree
import sys, os

SUBMODULE_PATH= os.path.abspath(os.path.join(os.getcwd(), 'padm-project-2022f'))
sys.path.append(SUBMODULE_PATH)
sys.path.extend(os.path.join(SUBMODULE_PATH, d) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics


class TreeNode(object):
    def __init__(self, pos, cost=0, parent=None):
        self.parent = parent
        self.pos = pos
        self.cost = cost

class MotionPlanner:
    def __init__(self, world, location_map, iterations=10000, d=0.3, goal_int=20, goal_biasing=False, run_rrtstar=False):
        self.world = world
        self.initial_pos = self.getinitialpos()
        self.location_map = location_map
        self.iterations = iterations
        self.d = d
        self.goal_int = goal_int
        self.goal_biasing = goal_biasing
        self.run_rrtstar = run_rrtstar
        self.get_random_sample = self.get_sample_fn(self.world.robot, self.world.arm_joints)

    def getinitialpos(self):
        pass
    
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
        if d > mag: # If d is greater than the magnitude of the vector make x_new x_rand
            x_new = rand_point
        else: # Else scale the vector to the magnitude d
            coeff = d/mag
            scaled_dir_vec= coeff*(dir_vec)
            x_new = nearest_vec + scaled_dir_vec
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
            if (collison_free(node, x_new)) and (node.cost + c(node, x_new) < c_min):
                x_min = node
                cmin = node.cost + c(node, x_new)
        return (x_min, c_min)

    def rewire(self, G, X_near, x_new):
        return G

    def c(self, start, end):
        start_vec = np.array(start)
        end_vec = np.array(end)
        dir_vec = end_vec - start_vec
        return np.linalg.norm(dir_vec)

    def collision_free(self, a, b):
        return collision_free

    def obst_free(self, a, b):
        get_collision_data(body, link=BASE_LINK)

    def retrive_path(self, G, end_pos, start_pos, x_new):
        current_pos = x_new
        path = [current_pos]
        while current_pos != start_pos:  
                    for ver in G:
                        if ver.pos == current_pos:
                            current_pos = ver.parent
                            break
                    path.insert(0, current_pos) 
        found = 1 
        return (found, path)

    def in_end_region(x_new):
        pass

    def solve(self, end_pos):
        V = [self.initial_pos] # Create a list of node positions
        start_pos = get_joint_positions(self.world.robot, self.world.arm_joints)
        G = [TreeNode(start_pos, cost=0, parent=None)] # Create a list of TreeNode objects
        found = 0 # Variable to keep track of if we've made it to the goal
        for i in range(self.iterations): # Iterate
            if (self.goal_biasing) and (i % self.goal_int == 0): # Every 20 iterations take the random sample from inside the goal area (goal biasing)
                rand_joints = self.get_random_sample(sample_goal=True)
            else: # Else take the random sample from somewhere in the operating area
                rand_joints = self.get_random_sample()
            x_nearest = self.get_nearest_node(rand_joints, G)
            x_new = self.steer(rand_joints, x_nearest, self.d) # Use the stter function to make x_new's position
            
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
                if self.obst_free(x_nearest, x_new):
                    V.append(x_new)
                    G.append(TreeNode(x_new, parent=x_nearest))
        if found:
            return path
        else:
            print('No path found')