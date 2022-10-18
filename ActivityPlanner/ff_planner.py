from pddl_parser.PDDL import PDDL_Parser
from copy import deepcopy
from collections import OrderedDict


def write_action_file(action_list, filename):
	with open(filename, 'w') as fp:
	    for item in action_list:
	        # write each item on a new line
	        fp.write("%s\n" % item)

def ground_actions(parser):
	
	grounded_actions = []

	for action in parser.actions:
		for act in action.groundify(parser.objects, parser.types):
			# print("parameters[0]")
			# print(act.parameters[0])
			# print("parameters[1]")
			# print(act.parameters[1])

			if len(act.parameters) == len(set(act.parameters)):
				grounded_actions.append(act)

			# THis if statement prevents "no-ops" for actions like move
			# if act.parameters[0] != act.parameters[1]:
				


	return grounded_actions

def applicable(state, positive):
	return positive.issubset(state)

def apply(state, positive):
    return state.union(positive)

def compute_rpg(current_state, parser, grounded_actions):

	found_goal = False
	fact_set = current_state
	layers=OrderedDict()
	layers["Fact1"] = (current_state, dict())
	iteration_counter = 1

	while not parser.positive_goals.issubset(fact_set):

		next_action_layer = []

		for action in grounded_actions:
			# if action.name == 'move': 
				# print("Trying action ", action)
				# print("Fact set: ", fact_set)
				# print("Precondiitons: ", action.positive_preconditions)
				# print(" ")
			if applicable(fact_set, action.positive_preconditions):
				# print(action)
				next_action_layer.append(action)

		# for act in next_action_layer:
		# 	print(act)

		# print(layers["Action1"])
		# break

		layers["Action"+str(iteration_counter)]=deepcopy(next_action_layer)

		iteration_counter +=1

		fact_action_map = dict()
		for action in next_action_layer:
			for fact in action.add_effects:
				
				if fact not in fact_action_map.keys():
					fact_action_map[fact]=[action]

				else:
					fact_action_map[fact].append(action)

			fact_set = fact_set.union(action.add_effects)

		# print("For fact layer %i, facts: "%iteration_counter)
		# print(fact_set)
		# print(" ")
		# write_action_file(fact_set, ('Fact Layer '+str(iteration_counter)))


		layers["Fact"+str(iteration_counter)]=(deepcopy(fact_set), deepcopy(fact_action_map))

	# 	# layers.append(deepcopy(fact_set))

	# 	# print(layers)

		# print("Found goal? ", parser.positive_goals.issubset(fact_set))

		if iteration_counter >= 10:
			break

	return layers

def compute_heuristic(current_state, parser, rpg):

	layers_t = list(rpg.keys())
	layers_t.reverse()
	# print(layers_t)
	useable_facts = parser.positive_goals
	counter = 0

	counted_actions = []
	for layer in layers_t:
		counter += 1
		print("Processing layer: ", layer)

		if 'Fact' in layer:
			actions_in_layer = []
			for fact in useable_facts:

				action_list = rpg[layer][1][fact]

				actions_in_layer = actions_in_layer + action_list

				print("procesing fact: ", fact)

				print('Associated actions: ')
				for action in action_list:
					print(action)
			counted_actions = counted_actions + actions_in_layer

			# for action in counted_actions:
			# 	print(action)
		else:
			useable_facts = set()

			# print(actions_in_layer)

			for action in actions_in_layer:
				print(action.positive_preconditions)
				useable_facts = useable_facts.union(set(action.positive_preconditions))

			print("Useable facts: ")
			print(useable_facts)

		print("===========================")

		if counter >=3:
			break










parser = PDDL_Parser()

parser.parse_domain('kitchen.pddl')
parser.parse_problem('pb1.pddl')

print("Initial state: ")
for init_prop in parser.state:
	print(list(init_prop))

print("\nGoal state: ")
for goal in parser.positive_goals:
	print(list(goal))

grounded_actions = ground_actions(parser)

print('===============')

write_action_file(grounded_actions, 'grounded_actions_kitchen.txt')

# for action in grounded_actions:
# 	print(action.name)

rpg = compute_rpg(parser.state, parser, grounded_actions)

heuristic_val = compute_heuristic(parser.state, parser, rpg)

# fact_set, fact_action_map = rpg["Fact2"]

# for fact, actionlist in fact_action_map.items():
# 	print(fact, ":")
# 	for action in actionlist:
# 		print(action)


