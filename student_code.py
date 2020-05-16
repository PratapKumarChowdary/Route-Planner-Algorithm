import math
import heapq

def reconstruct_path(cameFrom, current):
    """Gets a list of nodes which defines the shortest path from start node 
       to goal node
    Args:
        cameFrom (dictionary): For node n, cameFrom[n] is the node immediately preceding it
                               on the cheapest path from start to n currently known.
        current (tuple): Current node or intersection
    Returns:
        list: a list of nodes which defines the shortest path from start node to goal node
    """
    total_path = [current]
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path

def shortest_path(M,start,goal):
    """Finds a path from start node to goal node
    Args:
        M (object): Map
        start (int): Start node or intersection
        goal (int): Goal node or intersection
    Returns:
        if goal node is found:
            list: A list of nodes which defines the shortest path from start node to goal node
        else:
            string: A* not able to find the optimal solution
    """
    
    print("shortest path called")
    
    # A list of discovered nodes that may need to be (re-)expanded.
    # Initially, only the start node is known.
    # Min-Heap is used to store the discovered nodes.
    frontier = []
    heapq.heappush(frontier, (0, start))
    
    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
    # to n currently known.
    cameFrom = {}
    
    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = {node: math.inf for node in M.intersections}
    gScore[start] = 0
    
    # For node n, fScore[n] = gScore[n] + h(n). fScore[n] represents our current best guess as to
    # how short a path from start to finish can be if it goes through n.
    fScore = {node: math.inf for node in M.intersections}
    fScore[start] = heuristic(M.intersections[start], M.intersections[goal])
    

    while frontier:
        # Find the node which is having the lowest fScore[] value in frontier list
        current = heapq.heappop(frontier)[1]
        if current == goal:
            return reconstruct_path(cameFrom, current)

        # Expand the neighbors if current node has any 
        for neighbor in M.roads[current]:
            
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = (
                gScore[current] + 
                edge_weight(M.intersections[current], 
                            M.intersections[neighbor]))

            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                
                fScore[neighbor] = (
                    gScore[neighbor] + 
                    heuristic(M.intersections[neighbor], M.intersections[goal]))
                
                if neighbor not in frontier:
                    heapq.heappush(frontier, (fScore[neighbor], neighbor))

    # Frontier is empty but goal was never reached
    return "A* not able to find the optimal solution"

def heuristic(intersection_coords, goal_coords):
    """Gets a straight line distance between intersection node and goal node 
             using Euclidean Distance formula
    Args:
        intersection (list): Intersection coordinates
        goal_coords (list): Goal node coordinates
    Returns:
        float: A straight line distance between intersection node and goal node
    """
    estimated_distance = math.sqrt(
        math.pow((intersection_coords[0] - goal_coords[0]),2) + 
        math.pow((intersection_coords[1] - goal_coords[1]),2))
    return estimated_distance

def edge_weight(current, neighbor):
    """Gets a edge distance between current node and its neighbor node 
       using Euclidean Distance formula
    Args:
        current (list): Current intersection coordinates
        neighbor (list): Neighbor node coordinates
    Returns:
        float: A edge distance between intersection node and goal node
    """
    edge_value = math.sqrt(
        math.pow((current[0] - neighbor[0]),2) + 
        math.pow((current[1] - neighbor[1]),2))
    return edge_value