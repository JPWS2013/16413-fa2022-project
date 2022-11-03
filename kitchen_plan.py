
problem_file
domain_file

mp = MotionPlanner()

act_plan = get_activity_plan(problem_file, domain_file)

for action in act_plan:
    motion_plan = mp.get_motion_plan(action)
    execute(motion_plan)