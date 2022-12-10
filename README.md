# Principles of Autonomy Project, Fall 2022 #
*Team Members: Justin Poh & Corwin Stites*

## Contents ##
1. [Introduction](#Introduction)
2. [Part 1: Activity Planning](#Part-1-Activity-planning)
3. [Part 2: Motion Planning](#Part-2-Motion-Planning)
4. [Part 3: Trajectory Optimization](#Part-3-Trajectory-Optimization)


## Introduction 

The objective of this project was to implement activity planning, motion planning and trajectory optimization for a robot (the "Franka-Carter" - a Franka arm mounted on a carter mobile base) to enable it to accomplish pick and place tasks involving two objects - a sugar box and a potted meat can. 

Specifically, there were 2 tasks the robot had to accomplish: 
1. Locate the sugar box and place it on a nearby countertop 
2. Locate the potted meat can and place it in the red drawer

The starting position of the robot and objects in the environment is shown in the image below:

<p align="center">
<img src="readme_material/initial_world.png" width="70%">
</p>

This writeup begins by describing the implementation of activity planning where the sequence of activities required to accomplish both tasks is generated. 

The next section describes our implementation of motion planning where the motion paths of the base and arm that are needed to accomplish each activity in the generated activity plan is determined. 

Finally, the last section describes our implementation of trajectory optimization where an optimized trajectory of the robot arm is generated using a non-linear constrained optimization problem formulation.

## Part 1: Activity Planning ##

### Domain Assumptions ###

We have the following types in our domain:

- location: Represents locations in the kitchen.
- item: Represents items in the kitchen.
- arm : Representsthe robotic arm.

We have the following predicates in our domain:

- opened: Records if a location is open or not (this predicate along with closed is used for locations that are drawers).
- closed: Records if a location is closed or not.
- openable: Records if a location can be opened. This differentiates between counters and drawers.
- gripped: Records if an item is being gripped by the robot arm.
- free: Records if the item is not being gripped by the robot arm.
- itemat: Records what location a specific item is at.
- itemin: Records if an item is in an openable location. 
- armat: Records what location the arm is at.
- empty: Records if the arm is holding an item.
- surface: Records if a location is a surface or a cabinet.

We have the following actions in our domain:

- open: This opens a location. It requires the arm to be at the location and empty and for the location to be openable and closed. It makes the location opened and not closed.
- close: This closes a location. It requires the arm to be at the location and empty and for the location to be openable and open. It makes the location closed and not open.
- grip: This causes an item to be gripped by the arm. The arm must be empty, the item must be free, and the arm and item must be at the same location. It causes the arm to be not empty. It causes the item to be not free and gripped. It also causes the item to not be in the location anymore. This is because if the item is gripped, we assume it is in the same location as the arm. This means it is redundant to keep track of both locations.
- placein: This places the object in an openable location. It requires the item to be gripped, the arm at the location, and the location to be opened. This makes the the arm empty. It also makes the item free, not gripped, and makes itemat and itemin the location. We make both itemat and itemin satisified since itemat is the main predicate to keep track of an item's 
- location. We use itemin as a stronger condition to keep track of items in openable locations. 
- placeon: This places the object at a non- openable location. It requires the item to be gripped, the location to be a surface, and the arm at the location. This makes the the arm empty. It also makes the item free, not gripped, and makes itemat the location. 
- move: This moves the arm from one location to another. The arm must be in the start location. It causes the arm to be at the end location and not at the start location.

It is worth noting that the way we have defined the grip action will allow an item to be gripped even if the drawer is closed, which is not a realistic behavior. We could adjust our defined predicates to avoid this problem but have chosen not to do so since the tasks defined in the project do not require an item to be retrieved from the drawer. We are therefore assuming that this definition of actions will not be applied to a problem in which items will need to be retrieved from the drawer.


### Plan Generation ###

We start with an initial fact layer. We expand our actions in the action layer which have their preconditions satisfied by our current fact layer. We then use the action layer to generate new facts from the effects of the actions in the action layer. We then do a union of the new facts with the previous fact layer in order to carry over new facts and facts resulting from no-op. We carry out this process until the goal appears in our fact layer. To keep track of the connections between facts and actions, at each fact layer we create a dictionary. The keys in the dictionary are made up of the facts in the layer while the entries associated with each key are the actions that the fact resulted from. Critically, this is only carried out for facts that were not in the previous fact layer i.e. not facts that were carried over by the no-op. Using this strategy we can follow the RPG backwards from a fact in order to count actions for the fast forward heuristic value. We then use the heuristic value to inform an Enforced Hill Climb algorithm. At each action selection we look for a state with a decreasing heuristic value. If we cannot find a decreasing heuristic value, BFS is used to determine the next action.


### Challenges Faced ###

Previously we were also creating connections to facts that were carried over by the no-op. This was causing us to have way to many fact to action connections as we worked backward through the RPG. This resulted in our fast forward heuristic value being way to high for any given fact. 

## Part 2: Motion Planning

### Overview
In this section, our implementation of motion planning will be described. We begin this section by providing an overview of how our code works and then subsequent sections explain the various parts of our code in more detail. 

For our project, we decided to use Rapidly Exploring Random Trees (RRT) as our motion planning algorithm for both the base and the arm. The configuration space of the base is defined in terms of 3 variables: x position, y position and heading. The configuration of the arm is defined in "joint space" and thus defined by 7 variables - the joint angles for the 7 joints of the Franka arm.

To integrate the activity planner and the motion planner, we also implemented an execution engine. This execution engine essentially performs the following steps:
1. Generate the activity plan using the activity planner described in part 1
2. For each activity in the actvity plan, the execution engine does the following: 
    * Determine what activity needs to be performed 
    * Convert the named goal location for the activity into a numeric goal position
    * Pass the relevant goal position to the motion planner to solve for required motion plan
    * Store the generated motion plan for visualization later
5. Once all activities have been planned, the execution engine then visualizes the entire motion plan

The key files for this part are:
* The execution engine is defined in ```run_project.py``` located in the top-level folder
* The motion planner is defined in ```MotionPlanner/RRT.py```

### Assumptions ###
Our implementation of the motion planner and execution engine makes the following assumptions:
1. We assume that the **<ins>base is holonomic</ins>** and therefore base only needs to either move in a straight line from start to end position or to rotate from one heading to another. Thus, our motion planner only samples (x,y) positions and the heading of the robot is determined based on the direction of travel needed to get from (x_1, y_1) to (x_2, y_2) for each pair of waypoints in the path.
2. We assume that the **<ins>kitchen furniture and appliances will not change dimensions or location </ins>** from those given at the start of this project. Based on this assumption, the following hard-coded values exist in our code:
    * The x-position of all base goal locations is hardcoded to be x=0.7
    * To avoid infeasible plans occurring around the red drawer area, all goal locations with "indigo" in the name have hardcoded base goal locations of (0.7, 0.55)
    * When placing the sugar box down on the countertop, the target x-position is hardcoded to be +0.3m of the center point of the counter top and the target y-position is hardcoded to be +0.3m of the center point of the countertop
    <!-- * When placing the potted meat can down on the countertop, the position is hardcoded to be ???  -->
3. We assume a **<ins>hard-coded goal position to park the arm </ins>** when retrieving or placing objects. This is defined by the parked joint angle for each of the 7 arm joints.
4. We assume that we can **<ins>ignore the gripping dynamics of the robot arm </ins>**. Thus, our code does the following for gripping objects and opening/closing the drawer 
    * When the end effector enters within the goal radius of an object it is trying to grip, the object is simply attached to the arm. The object then moves jointly with the end effector of the arm until the object is released.
    * When the end effector enters the goal radius of the drawer handle, the arm simply moved by a fixed amount in the positive or negative x-direction to open or close the drawer respectively. At each time step, the drawer position is then set to equal the amount that the arm moved in that time step.
5. We assume that the **<ins>drawer has a fixed length that will not change </ins>**. Thus, in our code, the arm opens the drawer by a hard-coded distance of 0.35

### Files and Motion Planner Implemented ###

For our motion planner we decided to use RRT as our motion planner. We first run our PDDL parser to generate an activity plan. This plan is then passed into the ExecutionEngine class in our run_project.py function. The script iterates through each sequential action generated by the planner and plans the motion using our RRT_star script located in the MotionPlanner folder (RRT is without goal biasing is implemented now but RRT* with or without goal biasing may be implemented later time permiting). The motion planner uses get_sample_fn to draw random samples. Once a sample is determined to have the end effector within a radius of our goal, whether that be an object or location, the path to the goal is constructed and returned by the motion planner. These movements are then saved into a dictionary where they are then passed the execute_plan function. This in turn takes the movements and actually executes them in the PyBullet enviornment.

A video of the working planner (without collision checking along path, just collision checking at the start and end points):

<img src="readme_material/robot-planning-12-10.gif" width="900">

### Challenges Faced ###

A key challenge we faced was dealing with the format of our samples when sampling for our RRT* planner. The samples provided from functions within the utils.py file in ss-pybullet gave us a sample in the format of joint angles for each joint along the arm rather than cartesian coordinates of the end effector. We at first tried to find a way to convert this sample format to a cartesian point, but utlimatley, we were able to make the motion planner work using the original format of the sampler output. 
