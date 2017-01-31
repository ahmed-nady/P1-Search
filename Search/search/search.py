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
import searchAgents

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
    e = Directions.EAST
    #return [w,w,w,w]
    return []
    return  [w,w,w,w,s,s,e,s,s,w]
    return  [s, s, w, s, w, w, s, w]
	
def graphSearch(problem, frontier):
	print "Start:", problem.getStartState()
  	print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  	print "Start's successors:", problem.getSuccessors(problem.getStartState())
  	
  	explored = []
  	frontier.push([(problem.getStartState(), "Stop" , 0)])
  	
  	while not frontier.isEmpty():
  		# print "frontier: ", frontier.heap
  		path = frontier.pop()
  		# print "path len: ", len(path)
  		# print "path: ", path
  		
  		s = path[len(path) - 1]
  		s = s[0]
  		# print "s: ", s
  		if problem.isGoalState(s):
  			# print "FOUND SOLUTION: ", [x[1] for x in path]
  			return [x[1] for x in path][1:]
  			
  		if s not in explored:
  			explored.append(s)
	  		# print "EXPLORING: ", s
  			
  			for successor in problem.getSuccessors(s):
  				# print "SUCCESSOR: ", successor
  				if successor[0] not in explored:
  					successorPath = path[:]
  					successorPath.append(successor)
  					# print "successorPath: ", successorPath
  					frontier.push(successorPath)
  				# else:
  					# print successor[0], " IS ALREADY EXPLORED!!"
  	
	return []  
	

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
     
	
    #iterative version
    open = util.Stack()
    open.push( (problem.getStartState(), []) )
    closed =[]
    while not open.isEmpty():
        node, actions = open.pop()
        closed.insert(0,node)
        if problem.isGoalState(node):
            return actions 
        for coord, direction, steps in problem.getSuccessors(node):
            if not coord in closed and not open.isIn(coord):
                 open.push((coord, actions + [direction]))

    return []
     
	 
     
def dfsAuxiliary(problem, state, visited):
    visited = visited + [state[0]]
    if problem.isGoalState(state[0]):
        return state[1] 
    for coord, direction, steps in problem.getSuccessors(state[0]):
        if not coord in visited:
            state[0]=coord
            state[1]=state[1]+[direction]
            dfsAuxiliary(problem, state, visited)
            				
    return []
   
def breadthFirstSearch(problem):
  
    open =util.Queue()
    open.enqueue((problem.getStartState(),[]))
    closed =[]
    while not open.isEmpty():
         state,actions = open.dequeue()
         if problem.isGoalState(state):
             print "Goal"
             return actions
         closed.insert(0,state)
         for newState,singleMove,StepCost in problem.getSuccessors(state):
             if not newState in closed and not open.isIn(newState):
                 open.enqueue((newState,actions+[singleMove]))
    return []
             
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    open = util.PriorityQueue()
    open.push((problem.getStartState(),[]),problem.getCostOfActions([]))
    closed =[]
    
    while not open.isEmpty():
       state,actions= open.pop()
       if problem.isGoalState(state):
           print "Goal"
           return actions
       closed.insert(0,state)
       #print state
       for newState,direction,stepCost in problem.getSuccessors(state):
           if not newState in closed and not open.isIn(newState) :
               #h_of_x =heuristic(newState,problem)  
               #print h_of_x  
               #g_of_x = problem.getCostOfAction
               open.push((newState,actions+[direction]),problem.getCostOfActions(actions+[direction]))
           '''if open.isIn(newState):
               cost,path,priority=open.getStateDetails(newState)
               newCost = problem.getCostOfAction(actions+[direction])
               if cost > newCost:
                   open.update((newState,actions+[direction],newCost),priority)''' 
   
           
        
    return [] 
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    open = util.PriorityQueue()
    open.push((problem.getStartState(),[]),heuristic(problem.getStartState(),problem))
    closed =[]
    
    while not open.isEmpty():
       state,actions= open.pop()
       if problem.isGoalState(state):
           print "Goal"
           return actions
       closed.insert(0,state)
       #print state
       for newState,direction,stepCost in problem.getSuccessors(state):
           if not newState in closed and not open.isIn(newState) :
               h_of_x =heuristic(newState,problem)  
               #print h_of_x  
               g_of_x = problem.getCostOfActions(actions+[direction])
               open.push((newState,actions+[direction]),h_of_x+g_of_x)
           
    return [] 

    util.raiseNotDefined()
    
def bestFirstSearch(problem,heuristic=nullHeuristic):
    #(problem.getStartState(),[],0) [] is the list of actions and 0 is solution path from start state to node n
    
   open = util.PriorityQueue()
   open.push((problem.getStartState(),[]),heuristic(problem.getStartState(),problem))
   closed =[]
    
   while not open.isEmpty():
       state,actions= open.pop()
       if problem.isGoalState(state):
           print "Goal"
           return actions
      
       #print state
       for newState,direction,stepCost in problem.getSuccessors(state):
           if not newState in closed and not open.isIn(newState) :
               h_of_x =heuristic(newState,problem)  
               #print h_of_x  
               #g_of_x = problem.getCostOfAction
               open.push((newState,actions+[direction]),h_of_x)
       closed.insert(0,state)
       '''if open.isIn(newState):
               cost,path,priority=open.getStateDetails(newState)
               newCost = problem.getCostOfAction(actions+[direction])
               if cost > newCost:
                   open.update((newState,actions+[direction],newCost),priority)''' 
   
           
        
   return [] 


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
