import math
import random
import numpy
import scipy
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points
import shapely

update_interval = 1/15

def reward_function(params):
    """
    This reward function was designed to be run with the following parameters:
     - racetype: time trial
     - track: re:Invent 2018 Counterclockwise
     - sensor: Camera
     - Action space type: continuous
     - action space speed: 0 to 2 m/s
     - steering angle -30 to 30 degrees
     - Learning algo: ppo
     - hyperparams
        - gradient descent batch size = 64
        - entropy 0.01
        - discount 0.90
        - loss Huber
        - learning rate 0.0003
        - number of exp eps: 10
        - number of epochs: 10
    """
    log_route_info(params)
    return attempt_race_line_reward(params)

def attempt_race_line_reward(params):
    """
    the goal of this reward function is to regenerate the racing line on the fly.
    As quoted from https://en.wikipedia.org/wiki/Racing_line, the racing line
    is the optimal path around the race course
    Such a path is characterized by:
        - using the full width of the road
        - taking corners from the outside in
        - minimizing turning (relative to the curvature of the path)
            - this allows for the driver to maintain velocity through manuevers
    In order to achieve this, we need to:
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
    log_current_status(params)
    allWheelsOnTrack:bool = params["all_wheels_on_track"]
    isOffTrack:bool = params["is_offtrack"]
    is_crashed:bool = params["is_crashed"]
    if(not allWheelsOnTrack or isOffTrack or is_crashed):
        # print(f"Car crashed or ran off track")
        return 1e-3
    elif will_car_run_off_road_before_next_update(params):
        # print(f"Car is projected to run off track")
        return 1e-3
    else:
        reward:float = 0.0
        speedReward: float = reward_speed(params)
        reward = reward + speedReward
        reward = reward + location_reward(params)
        return reward 

def location_reward(params:map) -> float:
    reward:float = 0.0
    return reward 
def reward_speed(params:map) -> float:
    reward:float = 0.0
    speed:float = params["speed"] # 0.0 to 5.0 m/s
    reward = reward + speed/5.0
    return reward 
def will_car_run_off_road_before_next_update(params:map) -> bool:
    # raw inputs
    closest_waypoints = params["closest_waypoints"]
    waypoints = params["waypoints"]
    # generate car's expected future path
    predicted_car_path = get_projected_car_path(params)
    # generate future route for bounds checking
    forwardRange = get_expected_future_waypoint_advancement(params)
    upcomingWayPointsLeft, upcomingCenterLine, upcomingWayPointsRight = get_track_section(params, forwardRange)
    return predicted_car_path.intersects(upcomingWayPointsLeft) or predicted_car_path.intersects(upcomingWayPointsRight)
def reward_speed_depending_on_upcoming(params: map) -> float:
    speed:float = params["speed"] # 0.0 to 5.0 m/s
    heading:float = params["heading"] # -180 to 180 degrees relative to x axis of the coordinate system. may need to convert this to relative to the road.
    steering_angle:float = params["steering_angle"] # (right)-30 to 30 (left) relative to the car
    x = params['x']
    y = params['y']
    cutOffSpeed = 200.0
    # generate future route
    secondsToLookForward:float = 1.0
    predicted_car_path = get_projected_car_path(params, secondsToLookForward)
    upcomingTrack = get_expected_future_waypoint_advancement(params, secondsToLookForward)
    upcomingWayPointsLeft, upcomingCenterLine, upcomingWayPointsRight = get_track_section(params, upcomingTrack)
    rightIntersects = predicted_car_path.intersection(upcomingWayPointsRight)
    if len(rightIntersects.geoms) > 0:
        nearest = list(nearest_points(rightIntersects, Point(x, y)))[0]
        dist = nearest.distance(Point(x,y))
        cutOffSpeed = min(cutOffSpeed, dist/secondsToLookForward)
    leftIntersects = predicted_car_path.intersection(upcomingWayPointsLeft)
    if len(leftIntersects.geoms) > 0:
        nearest = list(nearest_points(leftIntersects, Point(x,y)))[0]
        dist = nearest.distance(Point(x,y))
        cutOffSpeed = min(cutOffSpeed, dist/secondsToLookForward)
    if speed < cutOffSpeed:
        return speed/5.0
    else:
        return 1e-3
def log_route_info(params:map):
    if params['progress'] < 0.01:
        print(f"params:{params}")
def log_current_status(params:map):
    speed:float = params["speed"] # 0.0 to 5.0 m/s
    heading:float = params["heading"] # -180 to 180 degrees relative to x axis of the coordinate system. may need to convert this to relative to the road.
    steering_angle:float = params["steering_angle"] # (right)-30 to 30 (left) relative to the car
    # print(f"Car's current location: {(params['x'], params['y'])} and progress: {params['progress']}")
    # print(f"Car's current speed: {speed}, heading: {heading}, and steering: {steering_angle}")
    # print(f"Car's closest waypoints: {params['closest_waypoints']}")
def is_car_going_wide_on_turn(params:map):
    # raw inputs
    # generate future route for curvature checking
    # if curvature is high enough reward for going wide.
    # otherwise return 0
    closest_waypoints = params["closest_waypoints"]
    waypoints = params["waypoints"]
    return 0
def get_expected_future_waypoint_advancement(params:map, time_delta:float = update_interval) -> int:
    speed:float = params["speed"] # 0.0 to 5.0 m/s
    track_length:float = params["track_length"] # m
    num_waypoints = len(params["waypoints"])
    inter_waypoint_distance = track_length/num_waypoints
    distance_traveled:float = speed * time_delta # m/s * s = m
    forward_waypoint_count = math.ceil(distance_traveled/inter_waypoint_distance)
    forwardRange = min(forward_waypoint_count + 1, 2)
    #min(forward_waypoint_count * (-1 if params["is_reversed"] else 1), 2)
    return forwardRange
def get_track_section(params:map, forward_range: int, reverse_range: int = 0) -> tuple:
    closest_waypoints = params["closest_waypoints"]
    waypoints = params["waypoints"]
    upcomingWayPoints = spliceWithLoop(waypoints, closest_waypoints[0] - reverse_range, closest_waypoints[0] + forward_range)
    upcomingCenterLine = LineString(upcomingWayPoints)
    track_width = params["track_width"]
    upcomingWayPointsLeft = upcomingCenterLine.parallel_offset(0.5 * track_width, "left")
    upcomingWayPointsRight = upcomingCenterLine.parallel_offset(0.5 * track_width, "right")
    return (upcomingWayPointsLeft, upcomingCenterLine, upcomingWayPointsRight)
def get_projected_car_path(params:map, time_delta:float = update_interval) -> LineString:
    speed:float = params["speed"] # 0.0 to 5.0 m/s
    heading:float = params["heading"] # -180 to 180 degrees relative to x axis of the coordinate system. may need to convert this to relative to the road.
    steering_angle:float = params["steering_angle"] # (right)-30 to 30 (left) relative to the car
    x = params['x']
    y = params['y']
    # generate car's expected future path
    distance_traveled:float = speed * time_delta # m/s * s = m
    corrected_heading = steering_angle + heading
    slope = math.tan(corrected_heading*(math.pi/180))
    scale = math.sqrt(math.pow(distance_traveled, 2)/(1 + math.pow(slope, 2)))
    delta_x = scale
    delta_y = slope * scale
    if abs(heading) > 90:
        delta_x = -1 * delta_x
    if heading < 0:
        delta_y = -1 * delta_y
    car_location = (x, y)
    predicted_car_location = (x + delta_x, y + delta_y)
    predicted_car_path = LineString([car_location, predicted_car_location])
    # print(f"given current position of {car_location}, heading of {heading}, speed: {speed}, and steering: {steering_angle}, the predicted car path is {predicted_car_path}")
    return predicted_car_path

def spliceWithLoop(array: list, start: int, stop: int) -> list:
    # print(f"Indexing start: {start}, stop: {stop}, from: {list}")
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