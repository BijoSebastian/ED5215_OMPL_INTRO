# Author: Bijo Sebastian

from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

#Defining axes and figure for plotting 
fig, ax1 = plt.subplots()
ax1.set(xlabel='X(m)', ylabel='Y(m)') #setting X and Y labels 

def isStateValid(state):
    #Function to check validity of state
    
    #Find if the state lies in obstacle 1 
    if state[0] < 7.0:
        if state[1] < 5.0 and state[1] > 2.0:
            return False
        
    #Find if the state lies in obstacle 2
    if state[0] > 4.0:
        if state[1] > 7.0:
            return False
        
    #If none of the above then state is valid
    return True
 
def plan():
    # create a Real Vector space of dimensionality 2 (XY plane)
    space = ob.RealVectorStateSpace(2)
 
    # set lower and upper bounds to both X, Y
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(10.0)
    space.setBounds(bounds)
 
    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
 
    #Create and set start state
    start = ob.State(space)
    start[0] = 0.0
    start[1] = 0.0
    
    #Create and set goal state
    goal = ob.State(space)
    goal[0] = 0.0
    goal[1] = 10.0
 
    ss.setStartAndGoalStates(start, goal)

    #Plot start and goal point as green and blue 
    ax1.scatter(start[0], start[1], alpha=1.0, s=75, color='green') 
    ax1.scatter(goal[0], goal[1], alpha=1.0, s=75, color='blue') 
    
    #Plot obstacles
    obstacle1 = Rectangle(xy = (0, 2), width = 7.0, height = 3.0, angle = 0.0, edgecolor='k', fc='k', lw=2)
    ax1.add_patch(obstacle1)
    obstacle2 = Rectangle(xy = (4, 7), width = 6.0, height = 3.0, angle = 0.0, edgecolor='k', fc='k', lw=2)
    ax1.add_patch(obstacle2)

    # this will automatically choose a default planner with
    # default parameters
    solved = ss.solve(1.0)
 
    if solved:
        # Try to shorten the path
        #ss.simplifySolution()
        # Get the solution path 
        path = ss.getSolutionPath()
        # Try to have atleast 10 waypoints
        #path.interpolate(10)
        prev_waypoint_state = [0,0]
        for way_point_id in range(path.getStateCount()):
            waypoint_state = path.getState(way_point_id)
            print("Waypoint number:",way_point_id,"[", waypoint_state[0], waypoint_state[1], "]")
            ax1.plot([waypoint_state[0], prev_waypoint_state[0]], [waypoint_state[1], prev_waypoint_state[1]],'r--') 
            ax1.scatter(waypoint_state[0], waypoint_state[1], alpha=1.0, s=25, color='red') 
            prev_waypoint_state = [waypoint_state[0], waypoint_state[1]]
    plt.show()

if __name__ == "__main__":
    plan()