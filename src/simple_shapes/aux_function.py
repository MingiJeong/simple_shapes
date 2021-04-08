import math
import numpy as np

def distance_calculator(x1, y1, x2, y2):
    """
    function to return Euclidean distance between two coordinates (x1,y1) and (x2,y2)
    """
    return math.sqrt((x2 - x1)**2 + (y2-y1)**2)


def regular_polygon(side_number, side_length, dir_counter_clock=True):
    """
    function to return coordinates of vertices on the regular polygon
    """

    # unit radius
    radius = 1

    # using cosine 2 rule
    init_side_length = math.sqrt(2*(radius**2) - 2*(radius**2)*math.cos(2*math.pi/side_number))

    # we want the the polygon's side length as input target; thus multiply this ratio
    ratio = side_length/init_side_length

    
    # find coordinate list
    coordinate_list = []
    for i in range(side_number):
        coordinate_list.append((radius * ratio * math.cos(2*math.pi * i/side_number), radius * ratio * math.sin(2*math.pi * i/side_number)))
    
    # append the starting point at the end to make the robot return home
    coordinate_list.append(coordinate_list[0])
    
    # clock-wise rotation coordinates
    if dir_counter_clock is False:
        coordinate_list = coordinate_list[::-1] # reverse the cooridnate list

    print("======================================================")
    print("side number: {}".format(side_number))
    print("side length: {}".format(side_length))
    print("counter-clock direction?: {}".format(dir_counter_clock))
    print("double check side by coordinate: {}".format(distance_calculator(coordinate_list[0][0], 
                                                coordinate_list[0][1], coordinate_list[1][0], coordinate_list[1][1])))
    
    print("coordinate list: {}".format(coordinate_list))
    print("======================================================")

    return coordinate_list


def check_vector_angle(vec1, vec2):
    """
    check angle between two vectors by using vec 1 as base.
    - arg: vec1, vec2 format np.array([x,y]) 1x2 
    - return: angle between the vectors abs 0 to pi (+: if coincide with check vector, -: if not)
    """

    # inner dot calculation
    inner_dot = np.dot(vec1, vec2)

    # vector length calculation
    vec1_len = np.linalg.norm(vec1)
    vec2_len = np.linalg.norm(vec2)

    # between angle calculation
    between_angle = math.acos(round(np.asscalar(inner_dot / (vec1_len * vec2_len)),6))

    # check vector (assuming "between angle" is correct, we get rotation vector of vec1 based on that angle)
    # length normalization and magnitude reflection
    check_vector = [vec1[0] * math.cos(between_angle) - vec1[1] * math.sin(between_angle),
                        vec1[0] * math.sin(between_angle) + vec1[1] * math.cos(between_angle)]
    check_vector_normalized = [check_vector[i] / vec1_len for i in range(len(check_vector))]
    vec2_normalized = [vec2[i] / vec2_len for i in range(len(check_vector))]

    # check if the vector is the same as what we want
    # note that 0.1 is just a threshhold for comparison
    if abs(check_vector_normalized[0] - vec2_normalized[0]) < 0.1 and abs(check_vector_normalized[1] - vec2_normalized[1]) < 0.1:
        return between_angle
    return -between_angle
