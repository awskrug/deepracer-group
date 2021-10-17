# tt-center

import math


MAX_SIGHT = 1.0

MAX_REWARD = 2.0
MIN_REWARD = 0.001


def reward_function(params):
    reward = 0.0

    reward += score_steering(params)

    reward += score_speed(params)

    return float(reward)


def get_distance(coor1, coor2):
    return math.sqrt(
        (coor1[0] - coor2[0]) * (coor1[0] - coor2[0])
        + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1])
    )


def get_radians(coor1, coor2):
    return math.atan2((coor2[1] - coor1[1]), (coor2[0] - coor1[0]))


def get_degrees(coor1, coor2):
    return math.degrees(get_radians(coor1, coor2))


def get_diff_radians(angle1, angle2):
    diff = (angle1 - angle2) % (2.0 * math.pi)

    if diff >= math.pi:
        diff -= 2.0 * math.pi

    return diff


def get_diff_degrees(angle1, angle2):
    return math.degrees(get_diff_radians(angle1, angle2))


def get_distance_list(car, waypoints):
    dist_list = []
    min_dist = float("inf")
    min_idx = -1

    for i, waypoint in enumerate(waypoints):
        dist = get_distance(car, waypoint)
        if dist < min_dist:
            min_dist = dist
            min_idx = i
        dist_list.append(dist)

    return dist_list, min_dist, min_idx, len(waypoints)


def get_waypoints(params):
    # waypoints are always provided in counter clock wise order
    if params["is_reversed"]:  # driving clock wise.
        return list(reversed(params["waypoints"]))
    else:  # driving counter clock wise.
        return params["waypoints"]


def up_sample(waypoints, factor=20):
    p = waypoints
    n = len(p)

    return [
        [
            i / factor * p[int((j + 1) % n)][0] + (1 - i / factor) * p[j][0],
            i / factor * p[int((j + 1) % n)][1] + (1 - i / factor) * p[j][1],
        ]
        for j in range(n)
        for i in range(factor)
    ]


def draw_ray(params, waypoints, sight=1.0):
    car = [params["x"], params["y"]]

    track_width = params["track_width"]

    dist_list, _, min_idx, length = get_distance_list(car, waypoints)

    target_dist = track_width * sight

    target_idx = min_idx

    for i in range(5, int(length * 0.1)):
        index = (min_idx + i) % length
        target_idx = index
        if dist_list[index] >= target_dist:
            break

    return waypoints[target_idx], target_idx, min_idx


def score_steering(params):
    car = [params["x"], params["y"]]

    # track [center, shortcut]
    waypoints = up_sample(get_waypoints(params), 10)

    # draw ray to canter line
    target, _, _ = draw_ray(params, waypoints, MAX_SIGHT)

    # target angle
    target_angle = get_radians(car, target)

    heading = params["heading"]
    steering = params["steering_angle"]

    # target steering
    target_steering = get_diff_degrees(target_angle, math.radians(heading))
    target_steering = max(-30, min(30, target_steering))

    # diff steering
    diff = abs(steering - target_steering)

    score = 1.1 - (diff / 60)

    return max(min(score, MAX_REWARD), MIN_REWARD)


def score_speed(params):
    speed = params["speed"]

    max_speed = 2.5

    score = speed / max_speed

    return max(min(score, MAX_REWARD), MIN_REWARD)
