import math
# import random
# import numpy
# import scipy
from shapely.geometry import LineString

update_interval = 1/15

def reward_function(params):
    return center_line_reward(params)

def attempt_race_line(params):
    """
    the goal of this reward function is to regenerate the racing line.
    As quoted from https://en.wikipedia.org/wiki/Racing_line, the racing line
    is the optimal path around the race course
    Such a path is characterized by:
        - using the full width of the road
        - taking corners from the outside in
        - minimizing turning (relative to the curvature of the path)
            - this allows for you to maintain velocity through manuevers
    In order ot achieve this, we need to:
        - penalize:
            - [x] running off of the road (heavily)
            - [ ] running off the road before the next control update (heavily)
            - [?] rolling over (heavily)
                - need to check if this is monitored by "is_crashed"
            - [ ] skidding
        - reward:
            - low amount of steer
                - should this be relative to current road curvature?
            - [x] speed
            - [ ] movement in direction of goal.
    """
    allWheelsOnTrack:bool = params["all_wheels_on_track"]
    isOffTrack:bool = params["is_offtrack"]
    is_crashed:bool = params["is_crashed"]
    if(not allWheelsOnTrack or isOffTrack or is_crashed):
        return -1000
    elif will_car_run_off_road_before_next_update(params):
        return -1000
    else:
        reward:float = 0.0
        speed:float = params["speed"] # 0.0 to 5.0 m/s
        heading:float = params["heading"] # -180 to 180 degrees relative to x axis of the coordinate system. may need to convert this to relative to the road.
        steering_angle:float = params["steering_angle"] # (right)-30 to 30 (left) relative to the car
        reward = reward + speed/5.0


def will_car_run_off_road_before_next_update(params:map) -> bool:
    # raw inputs
    speed:float = params["speed"] # 0.0 to 5.0 m/s
    heading:float = params["heading"] # -180 to 180 degrees relative to x axis of the coordinate system. may need to convert this to relative to the road.
    steering_angle:float = params["steering_angle"] # (right)-30 to 30 (left) relative to the car
    closest_waypoints = params["closest_waypoints"]
    waypoints = params["waypoints"]
    # generate car's expected future path
    distance_traveled:float = speed * update_interval # m/s * s = m
    corrected_heading = steering_angle + heading
    slope = math.tan(corrected_heading*(math.pi/180))
    scale = math.sqrt(math.pow(distance_traveled, 2)/(1 + math.pow(slope, 2)))
    delta_x = scale
    delta_y = slope * scale
    if abs(heading) > 90:
        delta_x = -1 * delta_x
    if heading < 0:
        delta_y = -1 * delta_y
    car_location = (params['x'], params['y'])
    predicted_car_location = (params['x'] + delta_x, params['y'] + delta_y)
    predicted_car_path = LineString([car_location, predicted_car_location])
    # generate future route for bounds checking
    track_length:float = params["track_length"] # m
    num_waypoints = len(params["waypoints"])
    inter_waypoint_distance = track_length/num_waypoints
    forward_waypoint_count = math.ceil(distance_traveled/inter_waypoint_distance)
    forwardRange = forward_waypoint_count * (-1 if params["is_reversed"] else 1)
    upcomingWayPoints = spliceWithLoop(waypoints, closest_waypoints[0], closest_waypoints[0] + forwardRange)
    upcomingCenterLine = LineString(upcomingWayPoints)
    track_width = params["track_width"]
    upcomingWayPointsLeft = upcomingCenterLine.offset_curve(0.5 * track_width)
    upcomingWayPointsRight = upcomingCenterLine.offset_curve(-0.5 * track_width)
    return predicted_car_path.intersects(upcomingWayPointsLeft) or predicted_car_path.intersects(upcomingWayPointsRight)
    



def is_car_going_wide_on_turn(params:map):
    # raw inputs
    # generate future route for curvature checking
    # if curvature is high enough reward for going wide.
    return 0

    
def spliceWithLoop(array: list, start: int, stop: int) -> list:
    arrayLen = len(array)
    if (start < arrayLen and stop < arrayLen):
        return array[start:stop]
    if (start < arrayLen and stop == arrayLen):
        return array[start:stop]
    if (start < arrayLen and stop > arrayLen):
        result = array[start:]
        stop = stop-arrayLen
        while stop > arrayLen:
            result = result + array
            stop = stop - arrayLen
        result = result + array[:stop]
        return result 

def center_line_reward(params):
    '''
    Example of rewarding the agent to follow center line
    '''
    
    # Read input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    
    # Calculate 3 markers that are at varying distances away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width
    
    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        reward = 1.0
    elif distance_from_center <= marker_2:
        reward = 0.5
    elif distance_from_center <= marker_3:
        reward = 0.1
    else:
        reward = 1e-3  # likely crashed/ close to off track
    
    return float(reward)