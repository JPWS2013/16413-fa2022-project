from pddl_parser.PDDL import PDDL_Parser
from graph import Graph, Edge
from copy import deepcopy


def ground_actions(parser):
	
	grounded_actions = []

	for action in parser.actions:
		for act in action.groundify(parser.objects, parser.types):
			if act.parameters[0] != act.parameters[1]:
				grounded_actions.append(act)

	return grounded_actions

def applicable(state, positive):
	return positive.issubset(state)

def apply(state, positive):
    return state.union(positive)

def calculate_heuristic_value(state, parser, grounded_actions):

	found_goal = False
	fact_set = set(state)
	layers = []
	iteration_counter = 0

	while not parser.positive_goals.issubset(fact_set):

		iteration_counter += 1
		print("Iteration: ", iteration_counter)

		layers.append([action for action in grounded_actions if applicable(fact_set, action.positive_preconditions)])


		for action in layers[-1]:
			fact_set = fact_set.union(action.add_effects)

		# layers.append(deepcopy(fact_set))

		# print(layers)

		if iteration_counter >= 5:
			break

	# print("Layers: ")
	# print(layers)

	# layers.pop()
	relevant_facts = parser.positive_goals
	action_counter = 0
	iteration_counter = 0

	while not parser.state.issubset(relevant_facts):
		iteration_counter += 1 
		print("Iteration: ", iteration_counter)

		print("Current relevant_facts: ", relevant_facts)

		next_relevant_facts = set()

		action_layer = layers.pop()


		for action in action_layer:
			
			if action.add_effects.issubset(relevant_facts):
				print("Action ", action.name, str(action.parameters), " is relevant!")
				action_counter += 1

				next_relevant_facts = next_relevant_facts.union(action.positive_preconditions)

		print("After searching actions, the next relevant facts are: ", next_relevant_facts)

		relevant_facts = deepcopy(next_relevant_facts)

		if iteration_counter >=5:
			break


parser = PDDL_Parser()

parser.parse_domain('boxmoving.pddl')
parser.parse_problem('logistics.pddl')

print("Initial state: ")
print(type(parser.state))
for init_prop in parser.state:
	print(list(init_prop))

print("\nGoal state: ")
for goal in parser.positive_goals:
	print(list(goal))

grounded_actions = ground_actions(parser)


heuristic_value = calculate_heuristic_value(parser.state, parser, grounded_actions)