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
    
    def __init__(self, warehouse, allow_taboo_push=False, macro=True):
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
            #Implement for elem like sokobanMacroProblem class
            
            pass
        return possible_actions

    def result(self, state, action):
        if self.macro:
            return sokobanMacroProblem(self.warehouse).result(state, action)
        else:
            #Implement for elemsokobanMacroProblem class
            pass

    #Goal test
    def goal_test(self,state):
        worker, boxes = state
        return boxes == self.targets
            


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
    # Extract initial state
    initial_worker = warehouse.worker
    initial_boxes = frozenset(warehouse.boxes)  # Use frozenset to make it hashable
    goal_positions = frozenset(warehouse.targets)
    
    # If the boxes are already on the goal positions, return []
    if initial_boxes == goal_positions:
        return []
    
    # Define the possible actions and their effects on the worker's position
    directions = {
        'Left': (-1, 0),
        'Right': (1, 0),
        'Up': (0, -1),
        'Down': (0, 1)
    }

    puzzle = SokobanPuzzle(warehouse, allow_taboo_push=False, macro=False)
    
    # Use BFS for elementary actions to find a solution
    puzzleSolution = breadth_first_graph_search(puzzle)

    if (puzzle.goal_test(puzzleGoalState)):
        return step_move_solution
    elif (puzzleSolution == None or check_action_seq(warehouse,find_action(puzzleSolution)) == "Failure"):
        return 'Impossible'
    else:
        return find_action(puzzleSolution)

# #--------------------------------------------------------------------
# def find_action(goal_node):
#     path = goal_node.path()
#     step_move = []
#     for node in path:
#         if node.action is not None:
#             step_move.append(node.action)
#     return step_move

# # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  

# def manhattan(coor_1, coor_2):
#     return abs(coor_1[0] - coor_2[0]) + abs(coor_1[1] - coor_2[1])

# # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# def get_Distance(start, locations):
#     min_dist = float('inf')
    
#     for coor in locations:
#         dist = manhattan(start, coor)
#         if dist < min_dist:
#             min_dist = dist
            
#     return min_dist

# # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# def get_Heuristic(node):
#     warehouse = node.state
#     try:
#         worker = warehouse.worker
#         boxes = warehouse.boxes
#         targets = warehouse.targets
#     except AttributeError:
#         # Handle the case where warehouse is a tuple
#         worker, boxes, targets = warehouse
    
#     sum = 0
    
#     playerMin = get_Distance(worker, boxes)
#     sum += playerMin
    
#     for box in boxes:
#         boxMin = get_Distance(box, targets)
#         sum += boxMin
        
#     return sum

    
    # BFS using a list-based queue
    
    # If no solution was found, return 'Impossible'
    
    
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
               
                    if can_go_there(dum_warehouse,worker_push_pos):
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
        return frozenset(boxes) == self.targets
        

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
     # Create a SokobanPuzzle instance with macro actions enabled
    problem = SokobanPuzzle(warehouse, macro=True, allow_taboo_push=False)
    ##Using BFS Graph
    solution = breadth_first_graph_search(problem)

    
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
    ##DEBUG
    return moves
