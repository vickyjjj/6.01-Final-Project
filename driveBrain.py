import time

from soar.robot.pioneer import PioneerRobot
from soar.gui.plot_window import PlotWindow
from soar.sim.geometry import normalize_angle_180


robot = PioneerRobot()

desired_right = 0.4
forward_velocity = 0.2
rotational_velocity = 0


def on_load():
    robot.distances = []
    robot.rvels = []
    robot.rvels2 = []
    robot.pTh = None
    robot.pTi = None


def on_start():
    robot.fv = forward_velocity


def on_step(step_duration):
    robot.fv = forward_velocity
    sonars = robot.sonars
    (distance_right, theta) = robot.get_distance_right_and_angle()
    print('d_o =', distance_right, ' theta =', theta)
    robot.distances.append(distance_right)

    Kr = 2.5
    Kd = -6.2

    if distance_right != None and theta != None:
        desired_theta = Kd * (distance_right - desired_right)
        rotational_velocity = Kr * (desired_theta - theta)
        robot.rv = rotational_velocity
    else:
        rotational_velocity = 1
        robot.rv = rotational_velocity

    # Record data for plots (do not change below this line)
    robot.rvels.append(rotational_velocity)

    current = time.time()
    dT = current-robot.pTi if robot.pTi is not None else 0.1

    theta = theta if theta is not None else robot.pTh

    if robot.pTh is not None:
        robot.rvels2.append(normalize_angle_180(theta-robot.pTh)/dT)

    robot.pTh = theta
    robot.pTi = current

def on_stop():
    pass

def on_shutdown():
    pass
