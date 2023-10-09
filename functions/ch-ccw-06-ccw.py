import math

# https://github.com/dgnzlz/Capstone_AWS_DeepRacer

REWARD_FOR_FASTEST_TIME = 150
STANDARD_TIME = 12  # seconds (time that is easily done by model)
FASTEST_TIME = 8  # seconds (best time of 1st place on the track)


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
        racing_track = []

        racing_track_ccw = [
            [-3.56843, -0.03983, 1.48416, 0.11578],
            [-3.59688, -0.20816, 1.53745, 0.11104],
            [-3.60089, -0.38002, 1.59301, 0.10791],
            [-3.58124, -0.55396, 1.64714, 0.10627],
            [-3.53802, -0.72862, 1.68052, 0.10707],
            [-3.47104, -0.90261, 1.62333, 0.11485],
            [-3.37835, -1.07415, 1.62333, 0.12011],
            [-3.25687, -1.24056, 1.62333, 0.12692],
            [-3.10183, -1.3971, 1.62333, 0.13572],
            [-2.90808, -1.53465, 2.41973, 0.0982],
            [-2.69644, -1.66016, 2.75121, 0.08944],
            [-2.47023, -1.77499, 3.14332, 0.08071],
            [-2.23192, -1.88084, 3.76362, 0.06929],
            [-1.98371, -1.98026, 3.84247, 0.06959],
            [-1.72705, -2.07624, 3.55993, 0.07697],
            [-1.46499, -2.16606, 3.30306, 0.08387],
            [-1.20266, -2.24662, 3.07394, 0.08927],
            [-0.94015, -2.31679, 2.85471, 0.09519],
            [-0.67761, -2.37518, 2.66792, 0.10081],
            [-0.41524, -2.42024, 2.41856, 0.11007],
            [-0.15337, -2.45033, 2.17695, 0.12108],
            [0.10759, -2.4635, 2.17695, 0.12003],
            [0.36709, -2.45773, 2.17695, 0.11923],
            [0.62418, -2.42926, 2.17695, 0.11882],
            [0.8772, -2.37298, 2.30341, 0.11253],
            [1.12539, -2.29175, 2.43065, 0.10744],
            [1.36815, -2.18779, 2.56264, 0.10305],
            [1.60496, -2.06297, 2.68391, 0.09974],
            [1.83526, -1.91876, 2.7849, 0.09757],
            [2.05836, -1.75622, 2.88264, 0.09576],
            [2.2735, -1.57643, 2.97386, 0.09428],
            [2.47983, -1.38049, 3.03472, 0.09376],
            [2.67628, -1.16937, 2.98294, 0.09667],
            [2.86147, -0.94408, 2.87653, 0.10139],
            [3.03393, -0.70591, 2.7769, 0.10589],
            [3.19179, -0.45635, 2.61456, 0.11294],
            [3.33275, -0.19715, 2.45908, 0.11998],
            [3.4542, 0.06943, 2.29287, 0.12776],
            [3.55395, 0.34054, 2.12907, 0.13568],
            [3.62925, 0.61316, 1.95528, 0.14465],
            [3.67792, 0.88404, 1.79754, 0.15311],
            [3.698, 1.14991, 1.647, 0.16189],
            [3.68781, 1.40747, 1.3882, 0.18568],
            [3.64532, 1.65299, 1.3882, 0.17949],
            [3.56879, 1.88212, 1.3882, 0.17402],
            [3.45638, 2.08922, 1.3882, 0.16974],
            [3.30101, 2.25997, 1.49122, 0.15481],
            [3.1153, 2.39442, 1.59708, 0.14356],
            [2.90802, 2.49393, 1.70855, 0.13458],
            [2.68563, 2.56035, 1.81943, 0.12756],
            [2.45296, 2.59549, 1.93871, 0.12137],
            [2.21379, 2.60133, 2.08202, 0.11491],
            [1.97109, 2.58035, 2.22833, 0.10932],
            [1.72708, 2.53476, 2.39325, 0.10372],
            [1.48342, 2.46691, 2.59309, 0.09754],
            [1.24138, 2.37938, 2.83263, 0.09086],
            [1.00183, 2.27492, 3.12045, 0.08375],
            [0.76535, 2.15632, 3.51668, 0.07523],
            [0.53219, 2.02665, 3.9, 0.06841],
            [0.30229, 1.88886, 2.85203, 0.09398],
            [0.07525, 1.74563, 2.29234, 0.1171],
            [-0.14384, 1.60284, 2.29234, 0.11408],
            [-0.36592, 1.46798, 2.29234, 0.11334],
            [-0.594, 1.34849, 2.29234, 0.11232],
            [-0.83151, 1.25212, 2.77204, 0.09247],
            [-1.07547, 1.17153, 3.14115, 0.08179],
            [-1.32445, 1.10336, 3.70076, 0.06975],
            [-1.57707, 1.04429, 3.08611, 0.08407],
            [-1.83207, 0.99121, 2.59465, 0.10039],
            [-2.0878, 0.94144, 2.26236, 0.11516],
            [-2.32962, 0.88639, 2.01587, 0.12303],
            [-2.56087, 0.8212, 1.79136, 0.13412],
            [-2.77821, 0.74326, 1.3, 0.17761],
            [-2.97835, 0.65101, 1.3, 0.16952],
            [-3.1584, 0.54382, 1.3, 0.16118],
            [-3.31506, 0.42135, 1.3, 0.15296],
            [-3.43114, 0.27831, 1.36967, 0.1345],
            [-3.51404, 0.12317, 1.43158, 0.12287],
        ]

        racing_track_cw = [
            [-3.58984, -0.37244, 1.45365, 0.11995],
            [-3.58212, -0.20115, 1.38131, 0.12413],
            [-3.55204, -0.03341, 1.3, 0.13109],
            [-3.49817, 0.12931, 1.3, 0.13185],
            [-3.41785, 0.28482, 1.3, 0.13464],
            [-3.30644, 0.42948, 1.3, 0.14045],
            [-3.15649, 0.55572, 1.69274, 0.1158],
            [-2.98187, 0.66645, 1.84829, 0.11187],
            [-2.78511, 0.76055, 2.01293, 0.10835],
            [-2.56862, 0.8372, 2.2279, 0.10309],
            [-2.33532, 0.89667, 2.50046, 0.09629],
            [-2.08826, 0.94019, 2.75637, 0.09101],
            [-1.82714, 0.96894, 2.80063, 0.0938],
            [-1.57128, 1.01329, 2.72722, 0.09522],
            [-1.32074, 1.07273, 2.63561, 0.0977],
            [-1.07564, 1.14695, 2.63561, 0.09717],
            [-0.83622, 1.23611, 2.63561, 0.09693],
            [-0.60299, 1.34088, 2.63561, 0.09701],
            [-0.3767, 1.46242, 3.1502, 0.08154],
            [-0.15548, 1.59578, 3.33254, 0.07751],
            [0.06146, 1.73889, 3.07596, 0.08449],
            [0.2748, 1.89, 2.85308, 0.09163],
            [0.50676, 2.04072, 2.6628, 0.10389],
            [0.74241, 2.17843, 2.49142, 0.10955],
            [0.98152, 2.30112, 2.34095, 0.1148],
            [1.22366, 2.40678, 2.20818, 0.11964],
            [1.46817, 2.49348, 2.0801, 0.12472],
            [1.71412, 2.55931, 1.95617, 0.13016],
            [1.96024, 2.60242, 1.81063, 0.138],
            [2.20483, 2.6211, 1.67803, 0.14619],
            [2.44574, 2.61353, 1.55903, 0.1546],
            [2.68019, 2.57786, 1.42898, 0.16596],
            [2.9043, 2.51133, 1.31828, 0.17733],
            [3.11283, 2.41145, 1.31828, 0.17539],
            [3.29893, 2.27617, 1.31828, 0.17453],
            [3.45246, 2.1031, 1.31828, 0.17549],
            [3.55899, 1.892, 1.7082, 0.13842],
            [3.63291, 1.66146, 1.83685, 0.1318],
            [3.67532, 1.41554, 1.98924, 0.12545],
            [3.68769, 1.15784, 2.15397, 0.11977],
            [3.67162, 0.89155, 2.32872, 0.11456],
            [3.62883, 0.61949, 2.45267, 0.11229],
            [3.56002, 0.34423, 2.56264, 0.11072],
            [3.46607, 0.06877, 2.65758, 0.10952],
            [3.34842, -0.20317, 2.75949, 0.10737],
            [3.20975, -0.46756, 2.81855, 0.10592],
            [3.05271, -0.72089, 2.85612, 0.10436],
            [2.87987, -0.96068, 2.8624, 0.10327],
            [2.69353, -1.18535, 2.84781, 0.1025],
            [2.49564, -1.394, 2.78291, 0.10333],
            [2.28773, -1.58598, 2.71787, 0.10412],
            [2.07115, -1.7611, 2.5887, 0.10759],
            [1.84705, -1.91932, 2.46383, 0.11134],
            [1.61616, -2.06006, 2.29915, 0.11761],
            [1.37915, -2.1828, 2.14418, 0.12448],
            [1.13642, -2.28601, 2.14418, 0.12301],
            [0.8884, -2.368, 2.14418, 0.12183],
            [0.63544, -2.42589, 2.14418, 0.12102],
            [0.37814, -2.45614, 2.20131, 0.11769],
            [0.11792, -2.4599, 2.61913, 0.09936],
            [-0.14374, -2.44459, 2.85379, 0.09185],
            [-0.40626, -2.41286, 3.07016, 0.08613],
            [-0.66925, -2.36657, 3.33426, 0.08009],
            [-0.93242, -2.3076, 3.61702, 0.07456],
            [-1.19557, -2.23757, 3.89183, 0.06997],
            [-1.4586, -2.15774, 3.72102, 0.07387],
            [-1.72122, -2.06935, 3.11097, 0.08907],
            [-1.97892, -1.97498, 2.67287, 0.10267],
            [-2.22762, -1.87709, 2.36341, 0.11309],
            [-2.46668, -1.77276, 1.59153, 0.16389],
            [-2.69448, -1.6592, 1.59153, 0.15993],
            [-2.90817, -1.53408, 1.59153, 0.1556],
            [-3.10443, -1.39596, 1.59153, 0.15079],
            [-3.2617, -1.23774, 1.61921, 0.13778],
            [-3.38318, -1.06939, 1.61609, 0.12846],
            [-3.47374, -0.89624, 1.59501, 0.12251],
            [-3.53685, -0.72121, 1.57244, 0.11832],
            [-3.57502, -0.54617, 1.51669, 0.11812],
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
        steering_angle = params["steering_angle"]
        track_width = params["track_width"]
        # waypoints = params["waypoints"]
        # closest_waypoints = params["closest_waypoints"]
        is_offtrack = params["is_offtrack"]
        is_reversed = params["is_reversed"]

        # closest_objects = params["closest_objects"]

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # track = racing_track

        if is_reversed:
            track = racing_track_cw
        else:
            track = racing_track_ccw

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
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30 or abs(steering_angle) > 20:
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
        if progress > 99.5:
            finish_reward = max(
                MIN_REWARD,
                (-REWARD_FOR_FASTEST_TIME / (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15),
            )
        else:
            finish_reward = 0
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


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
