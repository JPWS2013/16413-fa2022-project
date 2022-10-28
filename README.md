## 21OCT22 Update ##

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
