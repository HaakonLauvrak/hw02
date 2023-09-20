import cv2
import numpy as np
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
    
    def __str__(self):
     return "Position: " + str(self.position) + " Parent: " + str(self.parent) + " Action: " + str(self.action) + " Cost: " + str(self.cost)


def find_available_moves(map, node):
    """Returns a list of available moves from the current position"""
    available_moves = []
    current_position = node.get_position()
    up_pos = [current_position[0]+1, current_position[1]]
    down_pos = [current_position[0]-1, current_position[1]]
    left_pos = [current_position[0], current_position[1]-1]
    right_pos = [current_position[0], current_position[1]+1]
    possible_moves = {"up": up_pos, "down": down_pos, "left": left_pos, "right": right_pos}
    for key, value in possible_moves.items():
        if map.get_cell_value(value) > 0:
            available_moves.append(Node(value, node, key, node.get_cost() + map.get_cell_value(value) - find_air_distance(node.get_position(), map.get_goal_pos()) + find_air_distance(value, map.get_goal_pos())))
    return available_moves


def find_air_distance(position, goal_position):
    """Returns the air distance between two positions"""
    return abs(position[0] - goal_position[0]) + abs(position[1] - goal_position[1])

def find_start_node(map):
    """Returns the start node"""
    return Node(map.get_start_pos(), None, None, 0)

def a_star_search(map) -> Node:
    """Finds the shortest path from start node to goal"""
    node = find_start_node(map)
    frontier = []
    for move in find_available_moves(map, node):
        frontier.append(move)
    reached = {}
    while frontier:
        frontier.sort(key=lambda node: node.get_cost(), reverse=True)
        node = frontier.pop()
        map.move_current_pos(node.get_position())
        pil_image = map.show_map()
        frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        frames.append(frame)
        if node.get_position() == map.get_goal_pos():
            return node
        for move in find_available_moves(map, node):
            if ((move.get_position()[0], move.get_position()[1]) not in reached.keys()) or (reached[(move.get_position()[0], move.get_position()[1])].get_cost() > move.get_cost()):
                reached[(move.get_position()[0], move.get_position()[1])] = move
                frontier.append(move)
    return node.get_position(), map.get_goal_pos()

"""Select map"""
map = Map_Obj(4)

"""Run A*"""
frames = []
node = a_star_search(map)
has_parent = True
while has_parent:
    map.set_path(node.get_position())
    if node.get_parent() is None:
        has_parent = False
    else:
        node = node.get_parent()
pil_image = map.show_map()
frame = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
for i in range(0, 30):
    frames.append(frame)


"""Set video properties"""
frame_height, frame_width = frames[0].shape[:2]
size = (frame_width, frame_height)
out = cv2.VideoWriter('task4.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(frames)):
    out.write(frames[i])

out.release()
