# Principles of Autonomy Project, Fall 2022 #
*Team Members: Justin Poh & Corwin Stites*

## Contents ##
1. [Introduction](#Introduction)
2. [Part 1: Activity Planning](#Part-1-Activity-planning)
3. [Part 2: Motion Planning](#Part-2-Motion-Planning)
4. [Part 3: Trajectory Optimization](#Part-3-Trajectory-Optimization)


## Introduction 

The objective of this project was to implement activity planning, motion planning and trajectory optimization for a robot consisting of a Franka arm and mobile base to accomplish pick and place tasks involving a sugar box and a potted meat can. 

The 2 tasks the robot had to accomplish were: 
1. Place the sugar box on a nearby countertop 
2. Place the potted meat can in the red drawer

This writeup begins by describing how the sequence of activities needed to accomplish both tasks is generated. Then, the next section describes how each activity is translated into motion paths of the base and/or arm. Finally, the last section describes how a constrained optimization problem formulation is used to generate an optimized trajectory for the robot arm.

## Part 1: Activity Planning

### Domain Assumptions ###
Our implementation of activity planning makes the following assumptions:
1. We assume that items to be gripped will always be directly accessible (e.g. no need grip an item inside a closed drawer/cabinet). 
2. We assume that the red drawer is the only location that items need to be stored in
3. We assume that once gripped, an item has the same location as the robot arm and the item will not have its own associated location until it is released.
4. We assume that the act of grasping/releasing objects includes moving the arm into the grasp pose. As such, there is no separately defined arm movement action
5. We assume that locations do not have a maximum storage capacity for items

### Key Files and Functions ###

The key files for activity planning can be found in the ```ActivityPlanner``` Folder. They are:
1. ```kitchen.pddl```: Defines the kitchen domain including the main locations, objects and possible actions
2. ```pb1.pddl```: Defines the problem to be solved by the activity planner. including the initial and goal states
3. ```ff_planner.py```: Defines the functions needed to carry out activity planning


### Generating the Activity Plan ###
Our activity plan is generated using an enforced hill climb algorithm implemented by the ```enforced_hill_climb``` function in ```ff_planner.py```. Starting with the initial fact layer, the ```expand_state``` function identifies all feasible actions and the resulting (child) fact layer for each action (accounting for delete effects). The ```calculate_heuristic``` function then calculates the fastforward (FF) heuristic for each child fact layer.

If one of the possible next actions has a lower heuristic value than the incumbent (i.e. the lowest heuristic value found so far), the first action with a lower heuristic value is selected as the next action to take

However, if at least one of the actions has the same heuristic value as the incumbent and none have a lower value, then a plateau has been reached and the ```resolve_plateau``` function resolves the plateau by performing breadth-first search (BFS) to search through child states until a sequence of actions is found that has a lower heuristic value than the incumbent. The ```expand_state``` function is used again here to identify children of each fact layer.
    
Once an action has been selected, the fact layer associated with it is expanded and this process repeats until the goal state is reached.

### Calculating the FF Heuristic ###

```calculate_heuristic``` calculates the FF heuristic based on a relaxed plan graph (RPG) as follows:
1. ```compute_rpg``` computes the relaxed plan graph (RPG) by starting with a given current state and generating a sequence of alternating fact and action layers, ignoring the delete effects of actions. Between fact layers, we carry over facts resulting from no-op from the previous fact layer. To facilitate calculating the FF heuristic, at each fact layer, links are maintained between each new fact (i.e. facts not carried over by the no-op) and the action that added that fact.
2. ```extract_heuristic``` then takes the generated RPG and follows the links between facts and actions to follow the RPG backwards from the final fact layer to the starting fact layer, counting the actions needed along the way to achieve the goal state. The number of actions counted is returned as the heuristic value that informs the enforced hill climb algorithm described above.


### Challenges Faced ###

The key challenge we faced was that our FF heuristic planner was creating connections to facts that were carried over by the "no-op" and actions in the preceding action layer. This was causing us to have way too many fact to action connections and therefore a fast forward heuristic value that was much larger than expected. By adding links only between new facts and actions, we were able to resolve this problem.

## Part 2: Motion Planning

### Overview
Our motion planner implements Rapidly Exploring Random Trees (RRT) for both the base and the arm. The motion of the base is planned in terms of (x position, y position, heading) while the arm is planned in "joint space", which is defined by 7 variables: the joint angles for the 7 joints of the Franka arm.

The key files for this part are:
* The execution engine is defined in ```run_project.py```
* The motion planner is defined in ```MotionPlanner/RRT.py```

### Assumptions ###
Our implementation of the motion planner and execution engine makes the following assumptions:
1. We assume that the **<ins>base is holonomic</ins>** and only needs to either move in a straight line or rotate from one heading to another
2. We assume that the **<ins>kitchen environment is fixed and static </ins>**. Thus, the following values are hardcoded:
    * All base goal locations are hardcoded to have x=0.7
    * To ensure plans around the red drawer are always feasible, base goal locations for interactions with the right-side countertop or drawer are hardcoded to be in front of the stove at (0.7, 0.55)
    * When placing the sugar box down on the countertop, the target offsets from the centerpoint of the countertop are hardcoded
    * The arm opens the drawer by a hard-coded distance of 0.35
    <!-- * When placing the potted meat can down on the countertop, the position is hardcoded to be ???  -->
3. We assume that **<ins>objects are static and always start at the same position</ins>**. Thus, the grasp poses for the sugar box and potted meat can are predetermined and hardcoded 
4. We assume the **<ins>hardcoded park position for the arm </ins>** is stowed enough to not be in collision with anything during robot motion.
5. We assume that we can **<ins>ignore the gripping dynamics of the robot arm </ins>**. Thus:
    * Objects are grasped by attaching them to the end effector and moving them jointly with the end effector until the object is released
    * To open/close the drawer, the arm simply moves by a fixed amount in the x-direction and the drawer position is set to equal the distance that the arm moved

### Motion Planner Implementation ###

To get a motion plan, the execution engine calls the MotionPlanner class method ```plan``` which performs standard RRT. Some notable aspects of our implementation:
1. <ins>Random Sampling</ins>: Random samples for the base or arm are drawn from either the goal region or the full configuration space. Sampling from the goal region is done by setting the lower and upper bound of the goal region as "custom limits" for the relevant joints.
2. <ins>Collision Checking </ins>: Before performing collision checking, our motion planner first interpolates intermediate positions of the base or arm (whichever is being planned) between the nearest node and the new node. This includes interpolating the heading of the robot. Then, for every interpolated position, the following collision checks are performed:
    * Collision between robot body and objects (sugar box and potted meat can). Collisions with an object are ignored if they are grasped
    * Collisions between the robot body and any of the cabinets, countertops and appliances. If the drawer is being opened or closed, collisions with the drawer handle are ignored
    * If an object is being grasped, check collisions between that object and any of the kitchen cabinets, countertops and appliances
3. <ins>Path Retrieval</ins> When retrieving the path, every interpolated point from start to end containing is retreived and returned to the execution engine.

If a path is not found within 10,000 iterations, a ValueError is raised to indicate that no path was found. 

### Integrating the Activity Plan with the Motion Plan ###

The following are the key functions performed by the execution engine that constitute the interfaces between the simulation, the activity plan and the motion plan:
1. ```create_maps```: This function creates a map of all kitchen location and object names into numeric goal positions or grasp poses for the robot arm or base based on their locations in the simulation
2. ```get_activity_plan```: This function generates the activity plan using the activity planner described in part 1
2. ```plan_action```: For each activity in the activity plan, this function uses the motion planenr to do the following:
    * <ins>For a move action</ins>, create a motion path for just the base to reach the goal location
    * <ins>For an open or close action</ins>, create a motion plan to 1) Move the arm to grasp the drawer handle, 2) Open or close the drawer and 3) Move the arm by 0.1m away from the drawer handle
    * <ins>For a grip action</ins>, create a motion plan to move the arm into the required grasp pose and clos the fingers, then move the arm (and object) to the park position
    * <ins>For a placein or placeon action</ins>, calculate the final pose to place the object in the drawer or on the countertop and create a motion plan to move the arm (and object) into that pose, release the object and move the arm back to the park position
3. ```execute_action```: Once motion planning is complete, this function is called on each action in the activity plan to visualize the motion plans geneated by ```plan_action``` in the simulation. In addition, this function also creates and removes any necessary attachments of objects to the arm or to the drawer.

### GIF of Robot Executing the Plan ###

<img src="readme_material/robot-planning-12-10.gif" width="900">

*Note: We recognize that our code needs additional offset factors to properly place the potted meat can in the drawer. However, we ran out of time to implement this and record a new video.*

### Challenges Faced ###

A key challenge we faced was deciding what configuration space to plan the arm motions in. At first, we tried to plan in cartesian coordinates but struggled to convert a set of joint angles (from the random sample) into the resulting end effector pose. dealing with the format of our samples when sampling for our RRT* planner. We ultimately decided it would be easier to plan in joint space instead. 

Another key challenge we faced was figuring out how to perform efficient collision checking between robot links to make sure that the robot could not collide with itself. We started by using pybullet's pairwise link collision checking but found that function to be very slow. Since our existing planner didn't seem to have too many problems with the robot colliding into itself, we decided not to keep trying to implement this aspect of collision checking. As such, this is a significant limitation of our implementation - that there is the possibility that a motion plan for the arm is generated that causes the arm to collide with itself.

## Part 3: Trajectory Optimization
Our formal non-linear optimization problem is formulated as follows:
<br>
$$
\begin{align*}
\text{minimize} \quad & \sum_{i,t} |j_{i+1,t}-j_{i,t}|, \; \forall \;\; i \; \in \; [0,5], \; t \; \; \in \; [t_o, t_f]\\
\text{subject to} \quad & J_{lb_i} \leq j_{i,t} \leq J_{ub_i}, \; \forall \;\; i \; \in \; [0,6], \; t \; \; \in \; [t_o, t_f]\\
& S_{lb_i} \leq j_{i,t_0} \leq S_{ub_i}, \; \forall \;\; i \; \in \; [0,6]\\
& E_{lb_i} \leq j_{i,t_f} \leq E_{ub_i}, \; \forall \;\; i \; \in \; [0,6]\\
& C_{lb_x} \leq x_t \leq C_{ub_x}, \; (x_t, y_t, z_t) = f(j_{0:6,t}) \; \forall \;\; t  \; \in \; [t_o, t_f]\\
& C_{lb_y} \leq y_t \leq C_{ub_y}, \; (x_t, y_t, z_t) = f(j_{0:6,t}) \; \forall \;\; t  \; \in \; [t_o, t_f]\\
& C_{lb_z} \leq z_t \leq C_{ub_z}, \; (x_t, y_t, z_t) = f(j_{0:6,t}) \; \forall \;\; t  \; \in \; [t_o, t_f]\\
& B_{lb_x} \leq x_t \leq B_{ub_x}, \; (x_t, y_t, z_t) = f(j_{0:6,t}) \; \forall \;\; t  \; \in \; [t_o, t_f]\\
& B_{lb_y} \leq y_t \leq B_{ub_y}, \; (x_t, y_t, z_t) = f(j_{0:6,t}) \; \forall \;\; t  \; \in \; [t_o, t_f]\\
& B_{lb_z} \leq z_t \leq B_{ub_z}, \; (x_t, y_t, z_t) = f(j_{0:6,t}) \; \forall \;\; t  \; \in \; [t_o, t_f]\\
& \bigg | \frac{j_{i,t_{\lambda} - j_{i,t_{\lambda-1}}}}{t_{\lambda} - t_{\lambda-1}} \bigg | \leq 2.62, \; \forall \;\; i \; \in \; [0,3], \;  \lambda \; \in \; [0,len(t)]\\
& \bigg | \frac{j_{i,t_{\lambda} - j_{i,t_{\lambda-1}}}}{t_{\lambda} - t_{\lambda-1}} \bigg | \leq 5.25, \; \forall \;\; i \; \in \; [4,7], \;  \lambda \; \in \; [0,len(t)]\\
\end{align*}
$$

### Comparison of RRT-Generated v.s. Optimized Trajectories ###
RRT-Generated Trajectory:
<img src="readme_material/unoptimized_trajectory.gif" width="900">
Optimized Trajectory:
<img src="readme_material/optimized_trajectory.gif" width="900">