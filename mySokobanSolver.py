'''
IFN680 Sokoban Assignment

The functions and classes defined in this module will be called by a marker script. 
You should complete the functions and classes according to their specified interfaces.

You are not allowed to change the defined interfaces.
That is, changing the formal parameters of a function will break the 
interface and triggers to a fail for the test of your code.
'''


import search
import sokoban
from search import *
from sokoban import *
import copy

def my_team():
    '''
    Return the list of the team members of this assignment submission as a list
    of triplet of the form (student_number, first_name, last_name)
    e.g.  [ (1234567, 'Ada', 'Lovelace'), (1234568, 'Grace', 'Hopper'), (1234569, 'Eva', 'Tardos') ]
    '''
    return [(11710560,'ASWIN','JAYARAMAN'), (10577505,'Phi Long', 'Nguyen')]

#Helper function
def findTaboo(warehouse):
    #Start first by finding all space cell inside warehouse using BFS
    # Directions: left, right, up, down
    walls = warehouse.walls
    worker_loc = warehouse.worker
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    #Worker will always be inside warehouse
    queue = FIFOQueue()
    queue.append(worker_loc)
    inside = set([worker_loc])

    #BFS to find inside space
    while queue:
        x_loc,y_loc = queue.pop()
        for dx,dy in directions:
            x_new_loc = x_loc + dx
            y_new_loc = y_loc + dy
            if ((x_new_loc, y_new_loc) not in inside) and ((x_new_loc, y_new_loc) not in warehouse.walls):
                inside.add((x_new_loc,y_new_loc))
                queue.append((x_new_loc, y_new_loc))

    tabooCoord = []
    def isCorner(x,y):
        #Check if their 2 adjacent space are walls 
        topSpace = (x,y-1) in walls
        botSpace = (x,y+1) in walls
        rightSpace = (x+1,y) in walls
        leftSpace = (x-1,y) in walls
        if (leftSpace and topSpace) or (leftSpace and botSpace) or (rightSpace and topSpace) or (rightSpace and botSpace):
            return True

    #Check for taboo cells
    for (x, y) in inside:  
        #If cell is not checked yet
        if (x,y) not in tabooCoord:
            tabooCheck = []
            #Check corner first (Rule 1) 
            if isCorner(x,y):
                #print("\nCheck Corner:", (x,y))
                if (x,y) not in warehouse.targets:     
                    #print("Taboo Corner:", (x,y))
                    #Add directly to tabooCoord
                    tabooCoord.append((x,y))
                    
                    #Check horizontal
                    for direction in [-1,1]:
                        i = 1
                        while ((x+i*direction,y) not in warehouse.walls):
                            #Check if its a target
                            #print("Check Space:", (x+i*direction,y))
                            if (x+i*direction,y) in warehouse.targets:
                                tabooCheck.clear()
                                break
                            #Check if there's wall top or bottom
                            if (x+i*direction,y+1) in warehouse.walls or (x+i*direction,y-1) in warehouse.walls:
                                #print("Taboo Space:", (x+i*direction,y))
                                tabooCheck.append((x+i*direction,y))
                                i += 1
                            #If there's no wall top and bottom, means box can move up and down -> not taboo
                            else:
                                tabooCheck.clear()
                                break
                    tabooCoord.extend(tabooCheck)
                    #Check vertical
                    for direction in [-1,1]:
                        i = 1
                        while ((x,y+i*direction) not in warehouse.walls):
                            #Check if its a target
                            if (x,y+i*direction) in warehouse.targets:
                                tabooCheck.clear()
                                break
                            #Check if there's wall right or left
                            if (x+1,y+i*direction) in warehouse.walls or (x-1,y+i*direction) in warehouse.walls:
                                tabooCheck.append((x,y+i*direction))
                                i += 1
                            #If there's no wall right and left, means box can move left and right -> not taboo
                            else:
                                tabooCheck.clear()
                                break
                    tabooCoord.extend(tabooCheck)
    return tabooCoord 

def taboo_cells(warehouse):
    '''  
    Identify the taboo cells of a warehouse. A cell inside a warehouse is 
    called 'taboo' if whenever a box get pushed on such a cell then the puzzle 
    becomes unsolvable.  
    When determining the taboo cells, you must ignore all the existing boxes, 
    simply consider the walls and the target cells.  
    Use only the following two rules to determine the taboo cells;
     Rule 1: if a cell is a corner inside the warehouse and not a target, 
             then it is a taboo cell.
     Rule 2: all the cells between two corners inside the warehouse along a 
             wall are taboo if none of these cells is a target.
    
    @param warehouse: a Warehouse object

    @return
       A string representing the puzzle with only the wall cells marked with 
       an '#' and the taboo cells marked with an 'X'.  
       The returned string should NOT have marks for the worker, the targets,
       and the boxes.  
    '''
    ##         "INSERT YOUR CODE HERE"        
    tabooCoord = findTaboo(warehouse)
   
    X,Y = zip(*warehouse.walls) 
    x_size, y_size = 1+max(X), 1+max(Y)
    
    vis = [[" "] * x_size for y in range(y_size)]

    for (x,y) in tabooCoord:
        vis[y][x] = "X"

    for (x,y) in warehouse.walls:
        vis[y][x] = "#"

    return "\n".join(["".join(line) for line in vis])

def manhattan_distance(cell_a, cell_b):
    
    return abs(cell_a[0] - cell_b[0]) + abs(cell_a[1] - cell_b[1])


class SokobanPuzzle(search.Problem):
    '''
    An instance of the class 'SokobanPuzzle' represents a Sokoban puzzle.
    An instance contains information about the walls, the targets, the boxes
    and the worker.

    Your implementation should be fully compatible with the search functions of 
    the provided module 'search.py'. It uses search.Problem as a sub-class. 
    That means, it should have a:
    - self.actions() function
    - self.result() function
    - self.goal_test() function
    See the Problem class in search.py for more details on these functions.
    
    Each instance should have at least the following attributes:
    - self.allow_taboo_push
    - self.macro
    
    When self.allow_taboo_push is set to True, the 'actions' function should 
    return all possible legal moves including those that move a box on a taboo 
    cell. If self.allow_taboo_push is set to False, those moves should not be
    included in the returned list of actions.
    
    If self.macro is set True, the 'actions' function should return 
    macro actions. If self.macro is set False, the 'actions' function should 
    return elementary actions.
    
    
    '''
    
    def __init__(self, warehouse, allow_taboo_push=False, macro=False):
        self.warehouse = warehouse
        self.targets = frozenset(warehouse.targets)
        self.allow_taboo_push = allow_taboo_push
        self.macro = macro
        self.initial = (warehouse.worker, frozenset(warehouse.boxes))
        super().__init__(self.initial)

    def actions(self, state):
        """
        Return the list of actions that can be executed in the given state.
        
        As specified in the header comment of this class, the attributes
        'self.allow_taboo_push' and 'self.macro' should be tested to determine
        what type of list of actions is to be returned.
        """
        if self.macro:
            possible_actions = sokobanMacroProblem(self.warehouse).actions(state)
        else:
            #Implement for elem
            possible_actions = sokobanElemProblem(self.warehouse, self.allow_taboo_push).actions(state)

        return possible_actions

    def result(self, state, action):
        if self.macro:
            return sokobanMacroProblem(self.warehouse).result(state, action)
        else:
            #Implement for elem
            return sokobanElemProblem(self.warehouse, self.allow_taboo_push).result(state, action)


    #Goal test
    def goal_test(self,state):
        worker, boxes = state
        return boxes == self.targets

    
    def h(self, node):
    
        return self.value(node.state)


    def path_cost(self, c, state1, action, state2):
        '''
        This is the path cost we initally used. It makes the path cost
        the movement of the boxes
        '''
        # Counting the path of each time the box is pushed. Used for gathering data displayed on the graphs in the report
        #Boxes as path cost
        # for i in range(1, len(state1)-1):
        #     if state1[i][0] != state2[i][0] or state1[i][1] != state2[i][1]:
        #         return c + 1
        # return c

        # The path cost for the the movements of the worker getting from state1 too state 2
        return c + manhattan_distance(state1[0], state2[0])

    def value(self, state):
        '''
        Calculates the given state's value by finding the distances between
        the boxes and targets that are closest to each other.
    
        @param self: instance of a attribute
    
        @param state: the puzzle's current state
    
        @return
            The sum of distances between boxes and targets.
            Can return False if there is not equal number of targets and boxes
                in the given state.
        '''
    
        worker, boxes = state  # Unpack state tuple (worker position, boxes)
        
        # There must be an equal number of targets and boxes
        assert len(boxes) == len(self.targets)
    
        value = 0
        target_list = list(self.targets)  # Convert frozenset to list
        box_list = list(boxes)  # Convert frozenset to list for easier processing
    
        # Iterate through all the boxes and calculate the distance to each target
        for box in box_list:
            box_x, box_y = box  # Unpack the box tuple
    
            # Find the minimum distance from this box to any target
            min_dist = float('inf')
            for target in target_list:
                target_x, target_y = target  # Unpack the target tuple
                dist = manhattan_distance((box_x, box_y), (target_x, target_y))
                if dist < min_dist:
                    min_dist = dist
            
            value += min_dist
    
        return value

    # def value(self, state):
    #     '''
    #     Calculates the given state's value by finding the distances between
    #     the boxes and targets that are closest to each other.

    #     @param self: instance of a attribute

    #     @param state: the puzzle's current state

    #     @return
    #         The sum of distances between boxes and targets.
    #         Can return False if there is not equal number of targets and boxes
    #             in the given state.

    #     '''

    #     # There must be an equal number of targets and boxes
    #     assert(len(state)-1 == len(self.targets))

    #     value = 0
    #     first = True
    #     dist = 0
    #     boxes = []
    #     boxes = copy.deepcopy(state[1:])
    #     target_list = []
    #     target_list = copy.deepcopy(self.warehouse.targets)
    #     min_dist = 0

    #     box_targets_dist = []
    #     temp_list = []

    #     # Gets all the boxes coordinates, and targets coordinates,
    #     # and finds the boxes distance from all of the targets for the current puzzle
    #     for box in boxes:
    #         # Separate the box's x, y coordinates
    #         box_x = box[0]
    #         box_y = box[1]

    #         for target in target_list:
    #             # Separate the target's x,y coordinates
    #             target_x = target[0]
    #             target_y = target[1]

    #             # Find the diagonal distance (via hypotenus)
    #             dist = math.sqrt((box_x - target_x)**2 + (box_y - target_y)**2)

    #             temp_list += box
    #             temp_list += target
    #             temp_list += [dist,]
    #             box_targets_dist += [temp_list,]
    #             temp_list = []

    #     # Goes through the different distances and finds the shortest distance for each box
    #     while box_targets_dist:
    #         temp_trio = box_targets_dist[0]

    #         for i in range(0, len(box_targets_dist)):
    #             if i == 0:
    #                 min_dist = box_targets_dist[0][4]
    #             if box_targets_dist[i][4] < min_dist:
    #                 min_dist = box_targets_dist[i][4]
    #                 temp_trio = box_targets_dist[i]

    #         # Add the minimal distance to value
    #         value += min_dist

    #         # Loops until the shortest distance for each box is found.
    #         # If boxA has shortest distance to targetB. Remove targetB from the list and BoxA
    #         i = 0
    #         while i < len(box_targets_dist):
    #             trio = box_targets_dist[i]
    #             if (trio[0] == temp_trio[0] and trio[1] == temp_trio[1]) or (trio[2] == temp_trio[2] and trio[3] == temp_trio[3]):
    #                 box_targets_dist.remove(trio)
    #                 i -= 1
    #             i += 1

            
    #         '''
    #         This part was implemented to make a better value for our heuristic
    #         but after testing it, it was determined that the search is faster 
    #         without it
    #         '''
    #         # Finds the box closest to the worker, when found add that distance to value
    #         # min_worker_dist = math.sqrt((boxes[0][0] - state[0][0])**2 + (boxes[0][1] - state[0][1])**2)
    #         # for box in boxes:
    #         #     # Separate the box's x, y coordinates
    #         #     box_x = box[0]
    #         #     box_y = box[1]

    #         #     temp_dist = math.sqrt((box_x - state[0][0])**2 + (box_y - state[0][1])**2)
    #         #     if temp_dist < min_worker_dist:
    #         #         min_worker_dist = temp_dist

    #         # value += min_worker_dist

    #     return value

            


def check_action_seq(warehouse, action_seq):
    '''
    
    Determine if the sequence of actions listed in 'action_seq' is legal or not.
    
    Important notes:
      - a legal sequence of actions does not necessarily solve the puzzle.
      - an action is legal even if it pushes a box onto a taboo cell.
        
    @param warehouse: a valid Warehouse object

    @param action_seq: a sequence of legal actions.
           For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
           
    @return
        The string 'Failure', if one of the action was not successul.
           For example, if the agent tries to push two boxes at the same time,
                        or push one box into a wall, or walk into a wall.
        Otherwise, if all actions were successful, return                 
               A string representing the state of the puzzle after applying
               the sequence of actions.  This must be the same string as the
               string returned by the method  Warehouse.__str__()
    '''
    
    ##         "INSERT YOUR CODE HERE"

    
    # Define movement deltas for each action
    def get_movement_delta(action):
        if action == 'Up':
            return (0, -1)
        elif action == 'Down':
            return (0, 1)
        elif action == 'Left':
            return (-1, 0)
        elif action == 'Right':
            return (1, 0)
        else:
            return None

    # Copy the warehouse to avoid modifying the original object
    current_warehouse = warehouse.copy()
    worker_x, worker_y = current_warehouse.worker
    boxes = set(current_warehouse.boxes)
    walls = set(current_warehouse.walls)

    # Iterate through each action in the sequence
    for action in action_seq:
        # Get movement delta for the current action
        delta = get_movement_delta(action)
        if delta is None:
            return 'Failure'  # Invalid action

        dx, dy = delta

        # Calculate new worker position
        new_worker_x = worker_x + dx
        new_worker_y = worker_y + dy
        new_worker_pos = (new_worker_x, new_worker_y)

        # Check if worker is moving into a wall
        if new_worker_pos in walls:
            return 'Failure'  # Worker cannot move into a wall

        # Check if worker is moving into a box
        if new_worker_pos in boxes:
            # Calculate new box position
            new_box_x = new_worker_x + dx
            new_box_y = new_worker_y + dy
            new_box_pos = (new_box_x, new_box_y)

            # Check if new box position is a wall or another box
            if new_box_pos in walls or new_box_pos in boxes:
                return 'Failure'  # Cannot push box into a wall or another box

            # Move the box to the new position
            boxes.remove(new_worker_pos)
            boxes.add(new_box_pos)

        # Move the worker to the new position
        worker_x, worker_y = new_worker_x, new_worker_y

    # Update warehouse with new worker and box positions
    current_warehouse.worker = (worker_x, worker_y)
    current_warehouse.boxes = list(boxes)

    # Return the updated warehouse state as a string
    return str(current_warehouse)
    # raise NotImplementedError()


def solve_sokoban_elem(warehouse):
    
    
    '''    
    This function should solve using elementary actions 
    the puzzle defined in a file.
    
    @param warehouse: a valid Warehouse object

    @return
        If puzzle cannot be solved return the string 'Impossible'
        If a solution was found, return a list of elementary actions that solves
            the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
            For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
            If the puzzle is already in a goal state, simply return []
            
    '''
   

    # tree=['breadth_first_tree_search','depth_first_tree_search']
    # graph=['breadth_first_graph_search','depth_first_graph_search']
    
    skp = SokobanPuzzle(warehouse)
   
    path = search.astar_graph_search(skp)

    if  path  is None:
        return 'Impossible'
    # Turn list of coordinates into list of strings
    else:
        return path.solution()
    # ''' Astars implemet'''
        
    
def flip_coordinates(coords):
    """Helper function to flip (x, y) to (y, x) and vice versa."""
    return (coords[1], coords[0])
    
def can_go_there(warehouse, dst):
    '''    
    Determine whether the worker can walk to the cell dst=(row,column) 
    without pushing any box.
    
    @param warehouse: a valid Warehouse object

    @return
      True if the worker can walk to cell dst=(row,column) without pushing any box
      False otherwise
    '''
    # Extract state
    worker_loc = warehouse.worker
    boxes = warehouse.boxes  
    walls = warehouse.walls

    #Directions
    directions = {
        'Left': (-1, 0),
        'Right': (1, 0),
        'Up': (0, -1),
        'Down': (0, 1)
    }

    #flip dst back to x,y back internal working in x,y format
    dst = flip_coordinates(dst)

    #If dst is a wall/box, return false
    if (dst in boxes) or (dst in walls):
        return False;

    #Initial Frontier
    frontier = FIFOQueue()
    frontier.append(worker_loc)

    #Visited
    visited = set([worker_loc])

    #Explore while there's frontier
    while frontier:
        #Pop
        current_loc = frontier.pop()

        #Check if reach destination
        if current_loc == dst:
            return True

        #if not destination, explore neighbor cells
        for direction in directions.values():
            neighbor_loc = (current_loc[0] + direction[0], current_loc[1] + direction[1])
            # Check if the neighbor is not wall/box/visited
            if (neighbor_loc not in walls) and (neighbor_loc not in boxes) and (neighbor_loc not in visited):
                visited.add(neighbor_loc)  # Mark the neighbor as visited
                frontier.append(neighbor_loc)  # Add the neighbor to the frontier
    #Return false if found no route
    return False
    
# Helper func for skp 
class sokobanElemProblem(Problem):
    def __init__(self, warehouse, allow_taboo_push=False):
        self.walls = set(warehouse.walls)
        self.targets = frozenset(warehouse.targets)
        self.allow_taboo_push = allow_taboo_push

        # Precompute taboo cells if necessary
        if not self.allow_taboo_push:
            self.taboo_cells = set(findTaboo(warehouse))
        else:
            self.taboo_cells = set()

    def actions(self, state):
        worker, boxes = state
        possible_actions = []
        directions = {
            'Left': (-1, 0),
            'Right': (1, 0),
            'Up': (0, -1),
            'Down': (0, 1)
        }

        for action, (dx, dy) in directions.items():
            new_worker_pos = (worker[0] + dx, worker[1] + dy)

            if new_worker_pos in self.walls:
                continue  # Can't move into a wall
            elif new_worker_pos in boxes:
                # Worker tries to push a box
                new_box_pos = (new_worker_pos[0] + dx, new_worker_pos[1] + dy)

                if new_box_pos in self.walls or new_box_pos in boxes:
                    continue  # Can't push box into a wall or another box
                if not self.allow_taboo_push and new_box_pos in self.taboo_cells:
                    continue  # Can't push box into a taboo cell

                possible_actions.append(action)
            else:
                # Just moving the worker
                possible_actions.append(action)
        return possible_actions

    def result(self, state, action):
        worker, boxes = state
        directions = {
            'Left': (-1, 0),
            'Right': (1, 0),
            'Up': (0, -1),
            'Down': (0, 1)
        }
        dx, dy = directions[action]
        new_worker_pos = (worker[0] + dx, worker[1] + dy)
        new_boxes = set(boxes)

        if new_worker_pos in boxes:
            # Pushing a box
            new_box_pos = (new_worker_pos[0] + dx, new_worker_pos[1] + dy)
            new_boxes.remove(new_worker_pos)
            new_boxes.add(new_box_pos)

        return (new_worker_pos, frozenset(new_boxes))

    def goal_test(self, state):
        _, boxes = state
        return boxes == self.targets

        
# Helper function
class sokobanMacroProblem(Problem):
    def __init__(self, warehouse,allow_taboo_push=False):
        self.warehouse = warehouse
        self.worker = warehouse.worker
        self.boxes = frozenset(warehouse.boxes)  
        self.targets = frozenset(warehouse.targets)  
        self.allow_taboo_push = allow_taboo_push
        
        initial = (self.worker, self.boxes)
        super().__init__(initial)

    #Possible macro actions for the boxes
    def actions(self,state):
        directions = {
        'Left': (-1, 0),
        'Right': (1, 0),
        'Up': (0, -1),
        'Down': (0, 1)
    }
        worker, boxes=  state
        possible_actions = []

        #Check we can push box into taboo space or not
        if self.allow_taboo_push == False:
            tabooSpace = findTaboo(self.warehouse)
        else:
            tabooSpace= []

        #Check which direction boxes can be pushed to
        for box in boxes:
            #Extract action and x/y coord for movement
            for action, (x,y) in directions.items():
                #New position after go "direction"
                box_new_pos = (box[0] + x, box[1] + y)
                #Check new position if its valid
                if (box_new_pos not in self.warehouse.walls) and (box_new_pos not in boxes) and (box_new_pos not in tabooSpace):
                    #New box position is valid, now check if worker can move to position to push
                    worker_push_pos = (box[0] - x, box[1] - y)
                    
                    # Dummy warehouse for updated state
                    dum_warehouse = Warehouse()
                    dum_warehouse.worker = worker
                    dum_warehouse.boxes = list(boxes)
                    dum_warehouse.targets = self.targets
                    dum_warehouse.walls = self.warehouse.walls
               
                    if can_go_there(dum_warehouse,flip_coordinates(worker_push_pos)):
                        #Append the Box and its possible action
                        possible_actions.append((box,action))
        return possible_actions

    #Result state after executing an action
    def result(self, state, action):
        directions = {
        'Left': (-1, 0),
        'Right': (1, 0),
        'Up': (0, -1),
        'Down': (0, 1)
    }
        #Extract
        worker, boxes = state
        box, direction = action
        x,y = directions[direction]

        #Move worker and box
        worker_new_pos = box
        box_new_pos = (box[0] + x, box[1] +y)

        #Clone boxes to update location
        updated_boxes = set(boxes)
        #Remove old box location
        updated_boxes.remove(box)
        #Add updated box location
        updated_boxes.add(box_new_pos)

        return (worker_new_pos,frozenset(updated_boxes))

    #Goal test
    def goal_test(self,state):
        worker, boxes = state
        return boxes == self.targets
        

def solve_sokoban_macro(warehouse):
    '''    
    Solve using macro actions the puzzle defined in the warehouse passed as
    a parameter. A sequence of macro actions should be 
    represented by a list M of the form
            [ ((r1,c1), a1), ((r2,c2), a2), ..., ((rn,cn), an) ]
    For example M = [ ((3,4),'Left') , ((5,2),'Up'), ((12,4),'Down') ] 
    means that the worker first goes the box at row 3 and column 4 and pushes it left,
    then goes to the box at row 5 and column 2 and pushes it up, and finally
    goes the box at row 12 and column 4 and pushes it down.
    
    @param warehouse: a valid Warehouse object

    @return
        If puzzle cannot be solved return the string 'Impossible'
        Otherwise return M a sequence of macro actions that solves the puzzle.
        If the puzzle is already in a goal state, simply return []
    '''
    
    ##         "INSERT YOUR CODE HERE"
     #Create a SokobanPuzzle instance with macro actions enabled
    
    problem = SokobanPuzzle(warehouse, macro=True, allow_taboo_push=False)
    ##Using A* search
    solution = astar_graph_search(problem)

    
    # If no solution was found, return 'Impossible'
    if solution is None:
        return 'Impossible'
    
    # Get the list of actions from the solution
    moves = solution.solution()
    
    ##DEBUG - UNCOMMENT TO DEBUG
    # directions = {
    #     'Left': (-1, 0),
    #     'Right': (1, 0),
    #     'Up': (0, -1),
    #     'Down': (0, 1)
    # }
    
    # # Apply each move and print the warehouse after each move
    # for move in moves:
    #     box, action = move
    #     dx, dy = directions[action]  # Get the direction delta
    
    #     # The worker needs to be on the opposite side of the direction to push the box
    #     worker_push_pos = (box[0] - dx, box[1] - dy)
    
    #     print(f"Move box at {box} {action}")
    #     print(f"Worker needs to be at {worker_push_pos} to push the box")
    #     # Check if the worker can reach the required position
    #     can_reach = can_go_there(warehouse, worker_push_pos)
    #     print(f"Can worker reach {worker_push_pos}? {'Yes' if can_reach else 'No'}")
    
        
    #     # Apply the move to the warehouse
    #     warehouse.worker, warehouse.boxes = problem.result((warehouse.worker, warehouse.boxes), move)
    #     # Print the warehouse state after applying the move
    #     print("Worker: ",warehouse.worker)
    #     print("Boxes: ",warehouse.boxes)
    #     print(warehouse)  # Assuming Warehouse class has __str__ to print the layout
    #     print("\n")
    # ##DEBUG
    # Convert all moves to (y, x) format
    formatted_moves = [((box[1], box[0]), action) for box, action in moves]

    return formatted_moves
        

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




