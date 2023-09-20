

"""Psudocode for A* algorithm from textbook

function BEST-FIRST-SEARCH(problem, f) returns a solution node or failure
    node←NODE(STATE=problem.INITIAL)
    frontier←a priority queue ordered by f , with node as an element
    reached←a lookup table, with one entry with key problem.INITIAL and value node
    while not IS-EMPTY(frontier) do
        node←POP(frontier)
        if problem.IS-GOAL(node.STATE) then return node
        for each child in EXPAND(problem, node) do
            s←child.STATE
            if s is not in reached or child.PATH-COST < reached[s].PATH-COST then
                reached[s]←child
                add child to frontier
    return failure

function EXPAND(problem, node) yields nodes
    s←node.STATE
    for each action in problem.ACTIONS(s) do
        s0←problem.RESULT(s, action)
        cost←node.PATH-COST + problem.ACTION-COST(s, action,s0)
        yield NODE(STATE=s0, PARENT=node, ACTION=action, PATH-COST=cost)
"""

from Map import Map_Obj

class Node:
    """Node class for A* algorithm with position, parent, action that caused the pos and cost"""
    
    def __init__(self, position, parent, action, cost):
        self.position = position
        self.parent = parent
        self.action = action
        self.cost = cost
    
    def get_position(self):
        return self.position
    
    def get_parent(self):
        return self.parent
    
    def get_action(self):
        return self.action
    
    def get_cost(self):
        return self.cost
    
    


def find_available_moves(map, current_position):
    """Returns a list of available moves from the current position"""
    available_moves = []
    up_pos = (current_position[0]+1, current_position[1])
    down_pos = (current_position[0]-1, current_position[1])
    left_pos = (current_position[0], current_position[1]-1)
    right_pos = (current_position[0], current_position[1]+1)
    possible_moves = [up_pos, down_pos, left_pos, right_pos]
    for move in possible_moves:
        if map.get_cell_value(move) == 1:
            available_moves.append(move)
    return available_moves


def find_air_distance(position, goal_position):
    """Returns the air distance between two positions"""
    return abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])

def find_start_node(map):
    """Returns the start node"""
    return Node(map.get_start_pos(), None, None, 0)

def a_star_search(function, map):
    node = find_start_node(map)
    frontier = []
    reached = {}
    while frontier:
        frontier = sorted(frontier, key=lambda node: node.get_cost())
        node = frontier.pop()
        node.set_cost(function(node))
        if node.get_position() == map.get_goal_pos():
            return node
        for move in find_available_moves(map, node.get_position()):
            if move not in reached or node.get_cost() < reached[move].get_cost():
                reached[move] = node
                frontier.append(move)


map = Map_Obj(1)
print(map.get_start_pos())
print(map.get_goal_pos())
print(find_air_distance(map.get_start_pos(), map.get_goal_pos()))
print(find_available_moves(map, map.get_start_pos()))