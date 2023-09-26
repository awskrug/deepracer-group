import math

# https://github.com/dgnzlz/Capstone_AWS_DeepRacer

VERBOSE = False

ENABLED_DISTANCE_REWARD = True
ENABLED_SPEED_REWARD = True
ENABLED_STEPS_REWARD = True
ENABLED_FINISH_REWARD = False

REWARD_FOR_FASTEST_TIME = 50
STANDARD_TIME = 30  # seconds (time that is easily done by model)
FASTEST_TIME = 20  # seconds (best time of 1st place on the track)


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = 0  # None
        self.verbose = verbose

    def reward_function(self, params):
        # Import package (needed for heading)
        # import math

        ################## HELPER FUNCTIONS ###################

        def get_distance(coor1, coor2):
            return math.sqrt((coor1[0] - coor2[0]) * (coor1[0] - coor2[0]) + (coor1[1] - coor2[1]) * (coor1[1] - coor2[1]))

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

        def detect_bot(params):
            car = [params["x"], params["y"]]

            heading = math.radians(params["heading"])
            track_width = params["track_width"]
            is_reversed = params["is_reversed"]

            objects_location = params["objects_location"]
            objects_left_of_center = params["objects_left_of_center"]

            warned = False
            is_inner = False

            bot_idx = -1
            bot_dist = float("inf")

            for i, location in enumerate(objects_location):
                dist = get_distance(car, location)

                angle = get_radians(car, location)

                diff = abs(get_diff_degrees(heading, angle))

                if dist < track_width and diff < 120:
                    warned = True

                    if dist < bot_dist:
                        bot_idx = i
                        bot_dist = dist

            if warned:
                if is_reversed:
                    if objects_left_of_center[bot_idx] == False:
                        is_inner = True
                else:
                    if objects_left_of_center[bot_idx]:
                        is_inner = True

            return warned, is_inner, bot_dist

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):
            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(
                    x1=racing_coords[i][0],
                    x2=car_coords[0],
                    y1=racing_coords[i][1],
                    y2=car_coords[1],
                )
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            # Calculate the distances between 2 closest racing points
            a = abs(
                dist_2_points(
                    x1=closest_coords[0],
                    x2=second_closest_coords[0],
                    y1=closest_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Distances between car and closest and second closest racing point
            b = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=closest_coords[0],
                    y1=car_coords[1],
                    y2=closest_coords[1],
                )
            )
            c = abs(
                dist_2_points(
                    x1=car_coords[0],
                    x2=second_closest_coords[0],
                    y1=car_coords[1],
                    y2=second_closest_coords[1],
                )
            )

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2 * (a**2) * (b**2) + 2 * (a**2) * (c**2) - (b**4) + 2 * (b**2) * (c**2) - (c**4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
            # Virtually set the car more into the heading direction
            heading_vector = [
                math.cos(math.radians(heading)),
                math.sin(math.radians(heading)),
            ]
            new_car_coords = [
                car_coords[0] + heading_vector[0],
                car_coords[1] + heading_vector[1],
            ]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=closest_coords[0],
                y1=new_car_coords[1],
                y2=closest_coords[1],
            )
            distance_second_closest_coords_new = dist_2_points(
                x1=new_car_coords[0],
                x2=second_closest_coords[0],
                y1=new_car_coords[1],
                y2=second_closest_coords[1],
            )

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):
            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [
            [-3.69279, -0.00512, 1.5, 0.12227],
            [-3.73767, -0.18145, 1.51028, 0.12048],
            [-3.7485, -0.36504, 1.54722, 0.11886],
            [-3.72482, -0.55235, 1.61504, 0.1169],
            [-3.66679, -0.73948, 1.70629, 0.11482],
            [-3.57535, -0.92231, 1.81751, 0.11248],
            [-3.4523, -1.09695, 1.95273, 0.1094],
            [-3.30035, -1.26019, 2.12545, 0.10493],
            [-3.12304, -1.41009, 2.36613, 0.09812],
            [-2.92462, -1.54634, 2.63453, 0.09136],
            [-2.70861, -1.6695, 3.0061, 0.08272],
            [-2.47842, -1.78127, 3.54283, 0.07223],
            [-2.2371, -1.88418, 4.0, 0.06559],
            [-1.98657, -1.98116, 3.63072, 0.07399],
            [-1.72705, -2.07624, 3.16518, 0.08732],
            [-1.46516, -2.17137, 2.85201, 0.0977],
            [-1.20335, -2.26503, 2.61529, 0.10632],
            [-0.94135, -2.35296, 2.48899, 0.11104],
            [-0.67861, -2.43119, 2.43916, 0.11239],
            [-0.41503, -2.49579, 2.43916, 0.11126],
            [-0.15081, -2.54293, 2.43916, 0.11004],
            [0.11357, -2.56911, 2.43916, 0.10892],
            [0.37727, -2.57097, 2.43916, 0.10811],
            [0.63917, -2.54634, 2.43916, 0.10785],
            [0.89795, -2.49423, 2.44926, 0.10778],
            [1.15221, -2.4146, 2.51135, 0.1061],
            [1.40063, -2.3083, 2.62549, 0.10292],
            [1.64207, -2.17702, 2.80971, 0.09781],
            [1.87586, -2.02342, 2.96718, 0.09427],
            [2.10124, -1.8495, 3.14602, 0.09049],
            [2.31766, -1.6574, 3.2946, 0.08783],
            [2.52445, -1.44887, 3.16583, 0.09277],
            [2.72087, -1.22551, 2.95469, 0.10067],
            [2.90605, -0.98882, 2.79629, 0.10747],
            [3.07892, -0.74029, 2.61177, 0.11591],
            [3.23748, -0.48102, 2.47149, 0.12297],
            [3.37893, -0.2124, 2.31312, 0.13124],
            [3.49955, 0.06263, 2.19461, 0.13684],
            [3.59549, 0.34032, 2.02671, 0.14496],
            [3.66467, 0.61712, 1.8886, 0.15107],
            [3.70518, 0.8898, 1.73087, 0.15927],
            [3.71644, 1.15551, 1.73087, 0.15365],
            [3.69764, 1.41136, 1.73087, 0.14822],
            [3.64891, 1.65477, 1.73087, 0.14342],
            [3.56879, 1.88212, 1.73087, 0.13927],
            [3.45638, 2.08922, 1.73087, 0.13614],
            [3.30982, 2.2695, 1.78298, 0.13031],
            [3.1347, 2.42155, 1.83851, 0.12615],
            [2.93558, 2.54408, 1.86412, 0.12542],
            [2.716, 2.63454, 1.93403, 0.12279],
            [2.48051, 2.69184, 1.98641, 0.12201],
            [2.23349, 2.71446, 2.08941, 0.11872],
            [1.97988, 2.70305, 2.21263, 0.11473],
            [1.72418, 2.65936, 2.35382, 0.11021],
            [1.47008, 2.58594, 2.5128, 0.10526],
            [1.22047, 2.48583, 2.69745, 0.0997],
            [0.97734, 2.36245, 2.93889, 0.09277],
            [0.74182, 2.21974, 3.05296, 0.0902],
            [0.5137, 2.06263, 2.68217, 0.10327],
            [0.29156, 1.89606, 2.68217, 0.10352],
            [0.07656, 1.74197, 2.68217, 0.09862],
            [-0.14177, 1.59729, 2.68217, 0.09765],
            [-0.36473, 1.46495, 2.68217, 0.09667],
            [-0.594, 1.34849, 2.68217, 0.09588],
            [-0.83151, 1.25212, 3.0462, 0.08414],
            [-1.07539, 1.17133, 3.43692, 0.07475],
            [-1.32427, 1.10292, 2.99098, 0.0863],
            [-1.57692, 1.0439, 2.49667, 0.10392],
            [-1.83207, 0.99121, 2.17286, 0.11991],
            [-2.08849, 0.9417, 1.95937, 0.13328],
            [-2.33476, 0.89179, 1.78311, 0.14092],
            [-2.57379, 0.8338, 1.64392, 0.14962],
            [-2.80065, 0.76312, 1.56819, 0.15152],
            [-3.01084, 0.67663, 1.5, 0.15153],
            [-3.2003, 0.57271, 1.5, 0.14406],
            [-3.36598, 0.45145, 1.5, 0.13687],
            [-3.50492, 0.31363, 1.5, 0.13047],
            [-3.61436, 0.16066, 1.5, 0.12539],
        ]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        # all_wheels_on_track = params["all_wheels_on_track"]
        x = params["x"]
        y = params["y"]
        # distance_from_center = params["distance_from_center"]
        # is_left_of_center = params["is_left_of_center"]
        heading = params["heading"]
        progress = params["progress"]
        steps = params["steps"]
        speed = params["speed"]
        # steering_angle = params["steering_angle"]
        track_width = params["track_width"]
        # waypoints = params["waypoints"]
        # closest_waypoints = params["closest_waypoints"]
        is_offtrack = params["is_offtrack"]
        is_reversed = params["is_reversed"]

        # closest_objects = params["closest_objects"]

        ############### OPTIMAL X,Y,SPEED,TIME ################

        track = racing_track

        # if closest_objects:
        #     warned, is_inner, _ = detect_bot(params)

        #     if warned:
        #         if is_inner:
        #             track = outer_track
        #         else:
        #             track = inner_track

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = track[closest_index]
        optimals_second = track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1
        MIN_REWARD = 1e-2

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 3
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(MIN_REWARD, 1 - (dist / (track_width * 0.5)))
        if ENABLED_DISTANCE_REWARD:
            reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 3
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        if ENABLED_SPEED_REWARD:
            reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1.5
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        times_list = [row[3] for row in track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(
                MIN_REWARD,
                (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) / (STANDARD_TIME - FASTEST_TIME)) * (steps_prediction - (STANDARD_TIME * 15 + 1)),
            )
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        if ENABLED_STEPS_REWARD:
            reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = MIN_REWARD
        else:
            reward += 1.1 - (direction_diff / 30)

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = MIN_REWARD

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        # REWARD_FOR_FASTEST_TIME = 300
        # STANDARD_TIME = 50  # seconds (time that is easily done by model)
        # FASTEST_TIME = 20  # seconds (best time of 1st place on the track)
        if progress > 99.8:
            finish_reward = max(
                MIN_REWARD,
                (-REWARD_FOR_FASTEST_TIME / (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15),
            )
        else:
            finish_reward = 0
        if ENABLED_FINISH_REWARD:
            reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack == True:
            reward = MIN_REWARD

        ####################### VERBOSE #######################
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward(verbose=VERBOSE)  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)


def get_test_params():
    return {
        "x": 0.7,
        "y": 1.05,
        "speed": 2.0,
        "heading": 160.0,
        "track_width": 0.45,
        "is_reversed": False,
        "steering_angle": 0.0,
        "steps": 200,
        "progress": 99.9,
        "is_offtrack": False,
        "waypoints": [
            [0.75, -0.7],
            [1.0, 0.0],
            [0.7, 0.52],
            [0.58, 0.7],
            [0.48, 0.8],
            [0.15, 0.95],
            [-0.1, 1.0],
            [-0.7, 0.75],
            [-0.9, 0.25],
            [-0.9, -0.55],
        ],
    }


def test_reward():
    params = get_test_params()

    reward = reward_function(params)

    print("test_reward: {}".format(reward))


if VERBOSE:
    test_reward()
