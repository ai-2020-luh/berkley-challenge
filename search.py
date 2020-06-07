# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    def dfs(state: {int, int}, visited: [{int, int}] = []):
        if problem.isGoalState(state):
            return []

        # Mark current state as visited
        v = visited + [state]

        for (state, action, cost) in problem.getSuccessors(state):
            # Skip states we've already visited
            if state in visited:
                continue

            # recursive call to dfs, where state has been advanced
            # and the list of visited states was extended with the current one
            result = dfs(state, v)
            if not result is None:
                return [action] + result

        return None

    result = dfs(problem.getStartState())
    return result


def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    frontier = util.Queue()  # frontier is a FIFO Queue
    frontier.push(problem.getStartState())  # add startState to frontier
    visited = []  # list of states already visited (avoid loops)
    tempPath = []
    path = []  # path to the goal (is returned)
    pathToCurrent = util.Queue();  # paths to the states in the frontier
    currState = frontier.pop()  # current state is the first entry in frontier

    while not problem.isGoalState(currState):  # check whether current state is goal state
        if currState not in visited:  # skip visited states
            visited.append(currState)  # add current state to visted list

            successors = problem.getSuccessors(currState)
            for (child, direction, cost) in successors:
                frontier.push(child)  # add the successors of the current state to frontier
                tempPath = path + [direction]
                pathToCurrent.push(tempPath)  # add the paths to the children of the current state to the pathQueue

        currState = frontier.pop()  # set the next frontier entry as current state
        path = pathToCurrent.pop()  # set the path to the current State as path

    return path


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    frontier = util.PriorityQueue()  # frontier is a Priority Queue (if every cost = 1, then FIFO)
    frontier.push(problem.getStartState(), 0)  # add startState and its cost (0) to frontier
    visited = []  # list of states already visited (avoid loops)
    tempPath = []
    path = []  # path to the goal (is returned)
    pathToCurrent = util.PriorityQueue()  # paths to the states in the frontier
    currState = frontier.pop()  # current state is the first entry in frontier

    while not problem.isGoalState(currState):  # check whether current state is goal state
        if currState not in visited:  # skip visited states
            visited.append(currState)  # add current state to visted list

            successors = problem.getSuccessors(currState)
            for (child, direction, cost) in successors:  # iterate through all children
                tempPath = path + [direction]
                childCost = problem.getCostOfActions(tempPath)  # get cost to move to child
                if child not in visited:  # skip visited children
                    frontier.push(child, childCost)  # add child and cost to frontier
                    pathToCurrent.push(tempPath, childCost)  # add path and cost to child to frontier

        currState = frontier.pop()  # set the next frontier entry as current state
        path = pathToCurrent.pop()  # set the path to the current State as path

    return path


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


# Utility class to help structure the A* algorithm
class Checkpoint:
    def __init__(self, cost, path, node):
        self.cost = cost
        self.path = path
        self.node = node


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    # utility method
    def h(node):
        return heuristic(node, problem)

    visited = set()
    queue = util.KeyedPriorityQueue()
    queue.push(
        item=Checkpoint(0, [], problem.getStartState()),
        priority=0 + h(problem.getStartState())
    )

    while not queue.isEmpty():
        current: Checkpoint = queue.pop()

        # if this is the goal node, we can just stop and return the path
        if problem.isGoalState(current.node):
            return current.path

        # we can mark the current node as visited already. There can't be a smaller path from the current to the current
        # since negative weights are not allowed
        visited.add(current.node)

        # Iterate over every child node of current node, that we have not visited already
        for (child, direction, cost) in problem.getSuccessors(current.node):
            if child in visited:
                continue

            costToChild = current.cost + cost
            pathToChild = current.path + [direction]

            queue.update(
                item=Checkpoint(costToChild, pathToChild, child),
                key=lambda i: i.node,
                priority=costToChild + h(child)
            )

    # No path from start to goal exists
    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
