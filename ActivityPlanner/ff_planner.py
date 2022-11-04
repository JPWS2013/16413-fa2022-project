from pddl_parser.PDDL import PDDL_Parser
from copy import deepcopy
from collections import OrderedDict
import time




def ground_actions(parser):
    
    grounded_actions = []

    for action in parser.actions:
        for act in action.groundify(parser.objects, parser.types):

            # This if statement prevents "no-op"-like behavior for actions like move
            if len(act.parameters) == len(set(act.parameters)):
                grounded_actions.append(act)

    return grounded_actions

def applicable(state, positive):
    return positive.issubset(state)

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
        #   print(act)

        # print(layers["Action1"])
        # break

        layers["Action"+str(iteration_counter)]=next_action_layer

        iteration_counter +=1

        fact_action_map = dict()
        for action in next_action_layer:
            for fact in action.add_effects:
                #If fact is not already present in previous layer (i.e. a new fact is generated)
                if fact not in layers["Fact"+str(iteration_counter-1)][0]:
                    if fact not in fact_action_map.keys():
                        fact_action_map[fact]=[action]

                    else:
                        fact_action_map[fact].append(action)

            fact_set = fact_set.union(action.add_effects)

        # print("For fact layer %i, facts: "%iteration_counter)
        # print(fact_set)
        # print(" ")
        # write_action_file(fact_set, ('Fact Layer '+str(iteration_counter)))


        layers["Fact"+str(iteration_counter)]=(deepcopy(fact_set), fact_action_map)

        # print("Fact set: ")
        # print(fact_set)

        # print("Keys in fact_action_map: ")
        # print(fact_action_map.keys())
        # break
    #   # layers.append(deepcopy(fact_set))

    #   # print(layers)

        # print("Found goal? ", parser.positive_goals.issubset(fact_set))

        if iteration_counter >= 10:
            break

    return layers

def extract_heuristic(current_state, parser, rpg, verbosity=0, show_final_actions=False):

    layers_t = list(rpg.keys())
    layers_t.reverse()
    # print(layers_t)
    useable_facts = set(parser.positive_goals)
    counter = 0

    counted_actions = []
    for layer in layers_t:
        counter += 1
        
        if verbosity>=1:
            print("Processing layer: ", layer)

        if 'Fact' in layer:
            actions_in_layer = []
            found_facts = set()
            for fact in useable_facts:
                
                if fact in rpg[layer][1].keys():
                    found_facts.add(fact)
                    action_list = rpg[layer][1][fact]

                    actions_in_layer = actions_in_layer + action_list

                    if verbosity>=1:
                        print("procesing fact: ", fact)

                    if verbosity==2:
                        print('Associated actions: ')
                        for action in action_list:
                            print(action)

            useable_facts=useable_facts-found_facts
            counted_actions = counted_actions + actions_in_layer

            if verbosity>=1:
                print("Remaining facts in useable_facts: ")
                print(useable_facts)

            # for action in counted_actions:
            #   print(action)
        else:
            # useable_facts = set()

            # print(actions_in_layer)

            for action in actions_in_layer:

                if verbosity==2:
                    print("Adding precondition: ")
                    print(action.positive_preconditions)
                
                useable_facts = useable_facts.union(set(action.positive_preconditions))

            if verbosity>=1:
                print("Useable facts: ")
                print(useable_facts)

        if verbosity>=1:
            print("===========================")

        if counter >=7:
            break
    
    if verbosity>=1:
        print("Number of counted actionss (heuristic value): %i"%len(counted_actions))
    
    if verbosity==2 or show_final_actions==True:
        print("Counted actions: ")
        for action in counted_actions:
            print(action)

    return len(counted_actions)

def calculate_heuristic(state, parser, grounded_actions):
    rpg = compute_rpg(state, parser, grounded_actions)

    return extract_heuristic(state, parser, rpg)

def expand_state(parser, current_state, grounded_actions, h_func, ignore_deletes=False, print_actions=False):
    possible_next_states = []

    for action in grounded_actions:
        if applicable(current_state, action.positive_preconditions):

            # print("Processing action: ", action.name, action.parameters)
            
            next_state=current_state #Make a copy of the current state so you can manipulate it
            next_state = next_state.union(action.add_effects) #Add the add effects

            if not ignore_deletes:
                next_state = next_state - action.del_effects #Remove the delete effects

            # print("Current facts in next_state: ", next_state)
            
            h_val=h_func(next_state, parser, grounded_actions)

            if print_actions:
                print("For action: ", action.name, action.parameters, ", h value is ", h_val)

            # print("Heuristic value for this next state: ", h_val)

            possible_next_states.append((h_val, action, next_state))

    return possible_next_states

def resolve_plateau(possible_next_states, current_h_val, parser, grounded_actions):
    #Resolves the plateau by performing BFS with a Q that starts with all the states that have the same lowest h value
    Q = []
    visited_t = []
    goal_path = None
    final_h_val = None
    final_state = None

    for h_val, action, next_state in possible_next_states:
        if h_val == current_h_val:
            Q.append((current_h_val, [action], next_state))
            visited_t.append(next_state)
    # print("Performing BFS for state: ")
    # for prop in start_state:
    #     print(prop)
    
    while Q: #While the queue is not empty
        h_val, actions_t, state = Q.pop(0) #Grab the next node to expand from the queue
        # print("Expanding state: ")
        # for prop in state:
        #     print(prop)
        
        if h_val<current_h_val: #Check to see if goal state has been reached
            goal_path = actions_t
            final_h_val = h_val
            final_state = state
            break
        
        child_states = expand_state(parser, state, grounded_actions, calculate_heuristic) #Get all the child states 
        
        for next_h_val, next_action, next_state in child_states:
            if next_state not in visited_t: #Check to see if the child node has been visited before
                # print("Action to place on Q: ", next_action.name, next_action.parameters, " with h value ", next_h_val)
                visited_t.append(next_state) #If the node has not been visited, add it to the visited list
                action_list_copy = deepcopy(actions_t)
                action_list_copy.append(next_action)
                Q.append((next_h_val, action_list_copy, next_state)) #And also add it to the end of the queue

    return final_h_val, goal_path, final_state


def enforced_hill_climb(current_state, parser, grounded_actions):

    selected_actions=[]
    iteration_counter = 0
    current_h_val = calculate_heuristic(current_state, parser, grounded_actions)

    while not parser.positive_goals.issubset(current_state):

        iteration_counter+=1

        print("======================================")
        print("Looking for next action for current state:")
        for prop in current_state:
            print(prop)
        print("Current h value: ", current_h_val)
        print("")

        possible_next_states = expand_state(parser, current_state, grounded_actions, calculate_heuristic, print_actions=True)

        h_vals_t, actions_t, next_states_t = zip(*possible_next_states)

        if min(h_vals_t) == current_h_val:
            # If the lowest heuristic value for all child state is the same as the current one, 
            # We've hit a plateau and need to resolve it 
            h_val, best_actions_t, best_next_state = resolve_plateau(possible_next_states, current_h_val, parser, grounded_actions)

            actions_to_print = [str(best_action.name) + str(best_action.parameters) for best_action in best_actions_t]
            print("Selected actions: ", actions_to_print)

            selected_actions = selected_actions + best_actions_t
        

        elif min(h_vals_t) < current_h_val:
            # Otherwise, if the lowest heuristic value for all child states is lower than teh current one,
            # Then just find the first state that hast hat value and use it
            for h_val, action, next_state in possible_next_states:
                print(action.name, action.parameters, " has value: ", h_val)
                if  h_val < current_h_val:
                    print(action.name, action.parameters, " has a better h value!")
                    best_action = action
                    best_next_state = next_state
                    
                    print("Selected action: ", best_action.name, best_action.parameters)
                    selected_actions.append(best_action)
                    
                    break
        else:
            print('ERROR! H VALUE IS INCREASING!!!')
            

        current_h_val = h_val
        current_state = best_next_state

        

        if iteration_counter >=15:
            break

    return selected_actions


if __name__ == '__main__':
    
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

    print("===========================")

    # write_action_file(grounded_actions, 'grounded_actions_kitchen.txt')

    # for action in grounded_actions:
    #   print(action.name)

    start_time = time.time()

    # rpg = compute_rpg(parser.state, parser, grounded_actions)

    # heuristic_val = compute_heuristic(parser.state, parser, rpg)
    selected_actions = enforced_hill_climb(parser.state, parser, grounded_actions)

    plan_duration = time.time() - start_time

    print("------------------------------------------------------------")
    print("------------------------------------------------------------")
    print("")
    print("Final Plan: ")
    for action in selected_actions:
        print(action.name, action.parameters)

    print("Total plannning time: %f seconds"%plan_duration)

    # fact_set, fact_action_map = rpg["Fact2"]

    # for fact, actionlist in fact_action_map.items():
    #   print(fact, ":")
    #   for action in actionlist:
    #       print(action)


