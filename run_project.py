import numpy as np

def init_parser(problem_file, domain_file):
    return parser

def create_world():
    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=False)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))

    return world

def generate_map():
    pass

if __name__ == "__main__":
    
    problem_file
    domain_file

    parser = init_parser(problem_file, domain_file)
    world = create_world()
    map_obj = generate__map()

    mp = MotionPlanner(world, action_map, location_map)

    act_plan = get_activity_plan(problem_file, domain_file)

    for action in act_plan:
        motion_plan = mp.get_motion_plan(action)
        execute(motion_plan)