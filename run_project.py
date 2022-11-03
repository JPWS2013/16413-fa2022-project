import numpy as np

class ExecutionEngine():
    def __init__(self):
        self.parser = self.init_parser()
        self.world = self.create_world()

        self.location_map, self.action_map

        self.mp = MotionPlanner(world, self.action_map, self.location_map)

    def run():
        act_plan = get_activity_plan(problem_file, domain_file)

        for action in act_plan:
            start_pos, end_pos = get_positions()
            motion_plan = self.mp.solve(start_pos, end_pos)
            self.execute(motion_plan)


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

    def execute(motion_plan):
        pass

if __name__ == "__main__":
    
    problem_file = ''
    domain_file = ''

    engine = ExecutionEngine(problem_file, domain_file)

    

    