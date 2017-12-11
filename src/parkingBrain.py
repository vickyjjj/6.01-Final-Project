import sys
import math
import os.path
import time

from soar.robot.pioneer import PioneerRobot
from soar.hooks import tkinter_hook, is_gui, sim_completed
from soar.gui.plot_window import PlotWindow
from soar.sim.geometry import normalize_angle_180
from soar.client import brain_path
import lib601.markov as markov
from lib601.dist import *

import beliefGraph
import idealReadings

import sys
sys.path.append("/usr/lib/python3/dist-packages")
import espeak

####################################################################
###
### Preliminaries -- do not change the following code
###
####################################################################

robot = PioneerRobot()

lab_path = os.path.dirname(brain_path)
WORLD_FILE = os.path.join(lab_path,'baseWorld.py')
FORWARD_VELOCITY = 0.2 # meters / second
TIMESTEP_LENGTH = 0.1 # seconds


# Where the robot will be in the world
(x_min, x_max) = (0, 6.08)
robotY = y = 0.5

# Distance and Gain for Wall Following
desired_right = 0.5
Kp,Ka = (10.0,2.)
Kr,Kd = (2.5, -6.2)

# Maximum "good" sonar reading
sonar_max = 1.5

#method to discretize values into boxes of size grid_size
def discretize(value, grid_size, max_bin=float('inf'), value_min = 0):
    return min(int(((value or sonar_max) - value_min)/grid_size), max_bin)

#method to clip x to be within lo and hi limits, inclusive
def clip(x, lo, hi):
    return max(lo, min(x, hi))

####################################################################
###
###          Probabilistic Models -- you may change this code
###
####################################################################

# Number of discrete locations and discrete observations
num_states = 40
num_observations = 12

# compatibility for some students who got the old names
numStates = num_states
numObservations = num_observations
import lib601.dist as dist


def obs_model(s):
    tilt = triangle_dist(ideal[s],num_observations//6, 0, num_observations-1)
    wall = uniform_dist(range(num_observations))
    p = .95
    final_dist = mixture(tilt, wall, p)
    return final_dist

def trans_model(s):
    delta_pos = FORWARD_VELOCITY * robot.direction * TIMESTEP_LENGTH
    width = (x_max - x_min)/(num_states-1)
    delta = delta_pos/width
    p = abs(delta - int(delta))
    if s+int(delta) >= num_states-1:
        new_dist = {num_states-1:1}
##    elif s+int(delta) <= 0:
##        new_dist = {0:1}
    elif robot.direction == 1: 
        new_dist = {clip(s+int(delta), 0, num_states-1):1-p, clip(s+int(delta)+1, 0, num_states-1):p}
    elif robot.direction == -1:
        new_dist = {clip(s+int(delta), 0, num_states-1):1-p, clip(s+int(delta)-1, 0, num_states-1):p}
        
    return DDist(new_dist)


def confident_location(belief):
    s_true = belief.max_prob_elt()
    width = (x_max - x_min)/(num_states-1)
    state_range = int((0.45/2)// width)
    sum_prob = 0
    for i in range(s_true-state_range, state_range+s_true):
        sum_prob += belief.prob(i)
    if sum_prob > 0.75:
        return (s_true, True)
    return (-1, False)  


uniform_init_dist = square_dist(0, num_states)

REAL_ROBOT = True

######################################################################
###
###          Brain Methods -- do not change the following code
###
######################################################################

# Robot's Ideal Readings
#ideal = idealReadings.compute_ideal_readings(WORLD_FILE, x_min, x_max, robotY, num_states, num_observations)
ideal = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5]

def get_parking_spot(ideal):
    avg = sum(ideal)/float(len(ideal))
    i = len(ideal)-1
    while i>0 and ideal[i]>avg:
        i -= 1
    j = i
    while j>0 and ideal[j]<avg:
        j -= 1
    i = j
    while i>0 and ideal[i]>avg:
        i -= 1
    return (i+1+j)/2


def on_load():
    robot.parking_spot = get_parking_spot(ideal)
    robot.confident = False
    robot.rotate = False
    robot.park = False
    robot.back_ward = False
    robot.get_table = True
    robot.get_order = False
    robot.got_order = False
    robot.get_drink = False
    robot.got_drink = False
    robot.deliver = False
    robot.delivered = False
    robot.in_park = False
    robot.table = 1
    robot.bar = 2
    robot.direction = 1
    robot.number_drinks = 0
    if not (hasattr(robot,'g') and robot.g.winfo_exists()):
        robot.g = tkinter_hook(beliefGraph.Grapher(ideal))
        robot.nS = num_states
    if robot.nS != num_states:
        robot.g.destroy()
        robot.g = tkinter_hook(beliefGraph.Grapher(ideal))
        robot.nS = num_states
    robot.hmm = markov.HMM(uniform_init_dist,
                           trans_model,
                           obs_model)
    robot.estimator = robot.hmm.make_state_estimator()
    robot.g.updateDist()
    robot.g.updateBeliefGraph([robot.estimator.belief.prob(s) for s in range(num_states)])
    robot.probMeasures = []
    robot.data = []
    #table = input('Type the table number:')
    #robot.table =int(table)
    print(get_desired_table_state(ideal, 1))
    width = (x_max - x_min)/(num_states-1)
    robot.table_state = get_desired_table_state(ideal, robot.table)
    robot.table_location = robot.table_state * width + x_min
    robot.bar_state = get_desired_table_state(ideal, robot.bar)
    robot.bar_location = robot.bar_state * width + x_min
    robot.position_delta = 0
    
def get_desired_table_state(ideal, table):
    avg = sum(ideal)/float(len(ideal))
    count_rooms = 0
    out_room = 0
    in_room = 0
    for i in range(len(ideal)):
        if ideal[i] > avg and i > 0 and ideal[i-1] < avg:
            in_room = i
            print('in_room %d and table found = %d'%(in_room, count_rooms))
        elif ideal[i] < avg and ideal[i-1] > avg and i > 0:
            out_room = i
            count_rooms += 1
            print('out_room %d and table found = %d'%(out_room, count_rooms))
        elif ideal[i] > avg and i == len(ideal)-1:
            out_room = i
            count_rooms += 1
            print('out_room %d and table found = %d'%(out_room, count_rooms))
        if count_rooms == table:
            return (in_room + out_room-1)/2
    return (in_room + out_room -1)/2

def go_to_table(x, table_location, width):
    robot.direction = (table_location - x)/abs(table_location - x)
    (distance_right, theta) = robot.get_distance_right_and_angle()
    if not theta:
       theta = 0
    e = (desired_right-distance_right)*robot.direction
    ROTATIONAL_VELOCITY = Kp*e - Ka*theta
    if abs(table_location - x) > width*.1:
        robot.fv = FORWARD_VELOCITY * robot.direction
        robot.rv = ROTATIONAL_VELOCITY 
    elif abs(table_location - x) < width*.1:
        robot.fv = 0
        robot.back_ward = False
        robot.rotate = True
        robot.get_order = True
        if robot.got_drink:
            robot.deliver = True          

def park(robot_angle, sonars, table):
    if abs(robot_angle - 3.14/2) > .09 and not robot.park:
        robot.rv = .4
        robot.fv = 0 
    else:
        robot.rotate = False
        robot.park = True
        if robot.park:
            if sonars[4] > .3:
                robot.fv = .4
                robot.rv = 0 
            else:
                robot.fv = 0
                robot.rv = 0
                robot.in_park = True
                if table:
                    if not robot.got_order:
                        
                        message = "How many drinks would you like? Please raise your fingers."
                        os.system('espeak "{}"'.format(message))
                        time.sleep(2)
                        while robot.number_drinks == 0:
                            file = open("refills.txt", "r")
                            robot.number_drinks = int(file.read())
                        message = "Currently fetching" + str(robot.number_drinks) + "drinks."
                        os.system('espeak "{}"'.format(message))
                        time.sleep(2)

                        robot.got_order = True
                        robot.get_order = False
                    elif robot.deliver:
                        message = "Here are your drinks. Enjoy!"
                        os.system('espeak "{}"'.format(message))
                        time.sleep(3)
                        robot.delivered = True
                else:
                    message = "Customers at table " + str(robot.table_state) + " want " + str(robot.number_drinks) + " drinks."
                    os.system('espeak "{}"'.format(message))
                    time.sleep(5) 
                    robot.got_drink = True
                robot.back_ward = True
                
def get_out(robot_angle, sonars):
    try:
        if not robot.rotate and sonars[4] < .7 and robot.in_park and not robot.rotate:
            robot.fv = -.4
            robot.rv = 0
        else:
            if abs(robot_angle) > .09:
                robot.rotate = True
                robot.rv = -.4
                robot.fv = 0
            else:
                robot.rv = 0
                robot.fv = 0
                robot.rotate = False
                robot.in_park = False
    except:
        return
            
def go_to_bar(x, bar_location, width):
    robot.direction = (bar_location - x)/abs(bar_location - x)
    (distance_right, theta) = robot.get_distance_right_and_angle()
    if not theta:
       theta = 0
    e = (desired_right-distance_right)*robot.direction
    ROTATIONAL_VELOCITY = Kp*e - Ka*theta
    if abs(robot.bar_location - x) > width*.1:
        robot.fv = FORWARD_VELOCITY * robot.direction
        robot.rv = ROTATIONAL_VELOCITY
    elif abs(robot.bar_location - x) < width*.1:
        robot.fv = 0
        robot.rotate = True
        robot.get_drink = True

def go_to_start(x, start_location, width):
    robot.direction = (start_location - x)/abs(start_location - x)
    (distance_right, theta) = robot.get_distance_right_and_angle()
    if not theta:
       theta = 0
    e = (desired_right-distance_right)*robot.direction
    ROTATIONAL_VELOCITY = Kp*e - Ka*theta
    if abs(robot.bar_location - x) > width*.1:
        robot.fv = FORWARD_VELOCITY * robot.direction
        robot.rv = ROTATIONAL_VELOCITY #* robot.direction
    elif abs(table_location - x) < width*.1:
        robot.fv = 0
        robot.rv = 0
        print('My job is done!')


def on_step(step_duration):
    sonars = robot.sonars
    (px, py, ptheta) = robot.pose
    width = (x_max - x_min)/(num_states-1)
    if robot.confident:
        if not robot.get_order and not robot.back_ward:
            go_to_table(robot.pose.x+robot.position_delta, robot.table_location, width)
        elif not robot.in_park and robot.get_order: 
            park(robot.pose[2], sonars, True)
        elif robot.in_park and not robot.get_drink: 
            get_out(robot.pose[2], sonars)
        elif robot.got_order and not robot.get_drink:
            go_to_bar(robot.pose.x+robot.position_delta, robot.bar_location, width)
        elif robot.rotate and robot.get_drink:
            park(robot.pose[2], sonars, False)
##        else:
##            print('yuhu')
        elif robot.in_park and robot.got_drink:
            get_out(robot.pose[2], sonars)
        elif robot.got_drink and not robot.delivered:
            go_to_table(robot.pose.x+robot.position_delta, robot.table_location, width)
        elif not robot.in_park and robot.deliver:
            park(robot.pose[2], sonars, True)
        elif robot.delivered and robot.in_park:
            get_out(robot.pose[2], sonars)
        elif robot.delivered and not robot.in_park:
            start_location = 0
            go_to_start(robot.pose.x+robot.position_delta, start_location, width)
        else:
            message = "I'm done."
            os.system('espeak "{}"'.format(message))
        return

           
    # Quality metric.  Important to do this before we update the belief state, because
    # it is always a prediction
    if not REAL_ROBOT:
        parkingSpaceSize = .75
        robotWidth = 0.3
        margin = (parkingSpaceSize - robotWidth) / 2
        robot.probMeasures.append(estimate_quality_measure(robot.estimator.belief,
                                                           x_min, x_max, num_states, margin, px))
        true_state = discretize(px, (x_max - x_min)/num_states, value_min = x_min)
        true_state = clip(true_state, 0, num_states-1)
        n = len(robot.probMeasures)

    # current discretized sonar reading
    left = discretize(sonars[0], sonar_max/num_observations, num_observations-1)
    if not REAL_ROBOT:
        robot.data.append((true_state, ideal[true_state], left))
    # obsProb
    obsProb = sum([robot.estimator.belief.prob(s) * obs_model(s).prob(left)
                   for s in range(num_states)])

    # GRAPHICS
    if robot.g is not None:
        # draw robot's true state
        if not REAL_ROBOT:
            if true_state < num_states:
                robot.g.updateDist()
                robot.g.updateTrueRobot(true_state)
        # update observation model graph
        robot.g.updateObsLabel(left)
        robot.g.updateObsGraph([obs_model(s).prob(left)
                                for s in range(num_states)])

    robot.estimator.update(left)
    (location, robot.confident) = confident_location(robot.estimator.belief)
    if robot.confident:
        robot.position_delta = (location*width+x_min) - robot.pose.x
        print("location", location, "pos x", robot.pose.x, "delta", robot.position_delta)
    # GRAPHICS
    if robot.g is not None:
        # update world drawing
        # update belief graph
        robot.g.updateBeliefGraph([robot.estimator.belief.prob(s)
                                   for s in range(num_states)])
    # DL3 Angle Controller
    (distance_right, theta) = robot.get_distance_right_and_angle()
    if not theta:
       theta = 0
    e = desired_right-distance_right
    ROTATIONAL_VELOCITY = Kp*e - Ka*theta
    robot.fv = FORWARD_VELOCITY * robot.direction
    robot.rv = ROTATIONAL_VELOCITY 
##    robot.rv = 1
def on_shutdown():
    pass

def estimate_quality_measure(belief, x_min, x_max, num_states, delta, true_x):
    min_good = max(true_x - delta, x_min)
    max_good = min(true_x + delta, x_max)
    state_size = (x_max - x_min) / num_states
    min_good_discrete = max(0, discretize(min_good, state_size, value_min = x_min))
    max_good_discrete = min(num_states-1,
                            discretize(max_good, state_size, value_min = x_min)) + 1

    min_good_reconstituted = min_good_discrete * state_size + x_min
    max_good_reconstituted = max_good_discrete * state_size + x_min

    frac_low_bin_in_range = 1 - ((min_good - min_good_reconstituted) / state_size)
    frac_high_bin_in_range = 1 - ((max_good_reconstituted - max_good) / state_size)

    total =  sum(belief.prob(s) for s in range(min_good_discrete+1, max_good_discrete))
    lowP = belief.prob(min_good_discrete) * frac_low_bin_in_range
    highP = belief.prob(max_good_discrete) * frac_high_bin_in_range
    return total + lowP + highP
