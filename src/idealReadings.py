"""
Utility for computing ideal sonar readings
"""
# Updated to use the new Soar's geometry classes
from soar.sim.world import *
from soar.robot.pioneer import PioneerRobot
from soar.sim.geometry import Pose
from lib601 import sonarDist

SONAR_MAX = 1.5

######################################################################
###      Compute ideal readings
######################################################################

def compute_ideal_readings(world_path, xMin, xMax, y, numStates, numObs):
    """
    @param world_path: string naming file to read the world description from
    @param xMin: minimum x coordinate for center of robot
    @param xMax: maximum x coordinate for center of robot
    @param y: constant y coordinate for center of robot
    @param numStates: number of discrete states into which to divide
    the range of x coordinates
    @param numObs: number of discrete observations into which to
    divide the range of good sonar observations, between 0 and C{goodSonarRange}
    @returns: list of C{numStates} values, each of which is between 0
    and C{numObs-1}, which lists the ideal discretized sonar reading
    that the robot would receive if it were at the midpoint of each of
    the x bins.
    """

    xStep = (xMax - xMin) / float(numStates)
    readings = []
    # Start in the middle of the first box
    x = xMin + (xStep / 2.0)
    world_namespace = {}
    exec(open(world_path, 'r').read(), world_namespace)
    world = world_namespace['world']  # Grab the world object
    for ix in range(numStates):
        # left-hand sonar reading assuming we're heading to the right
        sensor_pose = sonarDist.sonarPoses[0].x, sonarDist.sonarPoses[0].y, sonarDist.sonarPoses[0].theta
        readings.append(discrete_sonar(ideal_sonar_reading(Pose(x, y, 0), sensor_pose, world), numObs))
        x += xStep
    return readings
            
def ideal_sonar_reading(robot_pose, sensor_pose, world):
    """
    @param robot_pose: C{util.Pose} representing pose of robot in world
    @param sensor_pose: c{util.Pose} representing pose of sonar sensor
    with respect to the robot
    @param world: C{soarWorld.SoarWorld} representing obstacles in the world
    @returns: length of ideal sonar reading;  if the distance is
    longer than C{sonarDist.sonarMax} or there is no hit at all, then
    C{sonarDist.sonarMax} is returned. 
    """
    # Translate and turn by the robot's pose, then rotate about its center
    origin = robot_pose.transform(sensor_pose).rotate(robot_pose.point(), robot_pose.t)
    sonar_ray = Ray(origin, length=SONAR_MAX, dummy=True)
    # Find all collisions that don't take place with a robot
    collisions = world.find_all_collisions(sonar_ray, eps=1e-3, condition=lambda obj: not isinstance(obj, PioneerRobot))
    if collisions:
        distances = [origin.distance(p) for _, p in collisions]
        distances.sort()
        return distances[0]
    else:
        return SONAR_MAX

def discrete_sonar(d, numBins, sonarMax = None):
    """
    @param d: value of a sonar reading
    @param numBins: number of bins into which to divide the interval
    between 0 and C{sonardist.sonarMax}
    @returns: number of the bin into which this sonar reading should
    fall;  any reading greater than or equal to c{sonarDist.sonarMax}
    is put into bin C{numBins - 1}.
    """
    if not sonarMax:
        sonarMax = SONAR_MAX
    binSize = sonarMax / numBins
    return int(d / binSize)

def inv_discrete_sonar(id, numBins, sonarMax = None):
    if not sonarMax:
        sonarMax = sonarDist.sonarMax
    binSize = sonarMax / numBins
    return id * binSize

# Old name, defined here in case somebody depends on it...
discreteSonarValue = discreteSonar = discrete_sonar
