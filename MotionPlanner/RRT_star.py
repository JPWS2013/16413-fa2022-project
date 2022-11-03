import numpy as np
from scipy.spatial import KDTree

class TreeNode(object):
    def __init__(self, pos, parent=None):
        self.parent = parent
        self.pos = pos

class MotionPlanner:
    def __init__(self, world, action_map, location_map, iterations):
        self.world = world
        self.initial_pos = getinitialpos()
        self.action_map = action_map
        self.location_map = location_map
        self.iterations = iterations
        self.d = d

    # Create a function to find the distance between two points
    def dist(a, b):
        diff = (a[0]-b[0], a[1]-b[1], a[2]-b[2])
        norm = np.linalg.norm(diff)
        return norm

    # Create a function to place x_new between x_rand and x_nearest based on d
    def steer(a, b, d):
        diff = (a[0]-b[0], a[1]-b[1], a[2]-b[2]) # Find vector between x_rand and x_nearest
        if d > np.linalg.norm(diff): # If d is greater than the magnitude of the vector make x_new x_rand
            x_new = a
        else: # Else scale the vector to the magnitude d
            coeff = d/np.linalg.norm(diff)
            scaled_diff = (coeff*diff[0], coeff*diff[1])
            x_new = (scaled_diff[0]+b[0], scaled_diff[1]+b[1])
        return x_new

    def get_nearest_node(self, point):
        """
        Uses scipy's KD Tree implementation to get the nearest node to the point

        Inputs:
            point: (x,y) tuple representing the point for which nearest neighbor must be found
        Returns:
            The (x,y) tuple of the nearest point in the tree
        """
        points = np.array(list(self._nodes.keys()))
        kdtree = KDTree(points)

        d,i = kdtree.query(point, k=1)

        return tuple(points[i])
        
    def get_sample_fn(body, joints, custom_limits={}, **kwargs):
        lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
        generator = interval_generator(lower_limits, upper_limits, **kwargs)
        def fn():
            return tuple(next(generator))
        return fn

    def solve(start_pos, end_pos):
        V = [self.initial_pos] # Create a list of node positions
        G = [TreeNode(start_pose, None)] # Create a list of TreeNode objects
        found = 0 # Variable to keep track of if we've made it to the goal
        for i in range(self.iterations): # Iterate
            if i % 20 == 0: # Every 20 iterations take the random sample from inside the goal area (goal biasing)
                x_rand = getsamplefunc(goal)
            else: # Else take the random sample from somewhere in the operating area
                x_rand = gen_point(bounds)
            
            x_nearest = get_nearest(x_rand)

            x_new = steer(x_rand, x_nearest, self.d) # Use the stter function to make x_new's position
            # Create a line and check if we hit anything
            start = x_nearest
            end = x_new
            start_pose_buff = Point(start).buffer(radius, resolution=3)
            end_pose_buff = Point(end).buffer(radius, resolution=3)
            line = LineString([start, end])
            expanded_line = line.buffer(radius, resolution=3)
            clear = 1;
            for obs in environment.obstacles:
                if expanded_line.intersects(obs) or obs.contains(Point(x_new)):
                    clear = 0
            if clear and (x_new not in V): # If we don't hit anything
                V.append(x_new) # Add x_new to our node list
                G.append(TreeNode(x_new, x_nearest)) # Create a new TreeNode object recording the new node and its parent
                if end_region.contains(Point(x_new)): # If goal is reached
                    current_pos = x_new
                    path = [current_pos]
                    # Walk backward from the goal using parents to get back to the start. This gives the path.
                    while current_pos != start_pose:  
                        for ver in G:
                            if ver.pos == current_pos:
                                current_pos = ver.parent
                                break
                        path.insert(0, current_pos) # Build the path list
                    found = 1 # We found the goal
                    break # Break when we've created our path

