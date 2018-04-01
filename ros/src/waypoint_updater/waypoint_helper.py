#!/usr/bin/env python

import math
import tf

'''
Waypoints helper methods
'''

''' Computes the distance between two waypoints in a list along the piecewise linear arc
    connecting all waypoints between the two '''
def linear_arc_distance(waypoints, wp1, wp2):
    dist = 0
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
    for i in range(wp1, wp2+1):
        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
        wp1 = i
    return dist

''' Computes the direct/aerial distance between two positions described by x,y,z '''
def direct_position_distance(pos1, pos2):
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)


def get_yaw_angle(pose):
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    return yaw


def transform_waypoint_x_axis_to_pose_x_axis(pose, waypoint):
    yaw = get_yaw_angle(pose)

    # translation
    translation_x = waypoint.pose.pose.position.x - pose.position.x
    translation_y = waypoint.pose.pose.position.y - pose.position.y

    # rotation
    x = math.cos(-yaw) * translation_x - math.sin(-yaw) * translation_y

    return x


def waypoint_ahead(pose, waypoint):
    rotated_x = transform_waypoint_x_axis_to_pose_x_axis(pose, waypoint)
    return rotated_x > 0


def get_cyclic_range_indices_normal(min_index, current_index, max_index, count):
    indices_count_to_end = max_index - current_index
    if indices_count_to_end >= count:
        return [(current_index, current_index + count + 1)]
    else:
        return [(current_index, max_index + 1), (min_index, min_index + count - indices_count_to_end)]


def get_cyclic_range_indices_reversed(min_index, current_index, max_index, count):
    indices_count_to_start = current_index - min_index
    if indices_count_to_start >= count:
        return [(current_index, current_index - count - 1)]
    else:
        return [(current_index, min_index - 1), (max_index, max_index - count + indices_count_to_start)]


def get_cyclic_range_indices(min_index, current_index, max_index, count, reverse_search=False):
    if reverse_search:
        return get_cyclic_range_indices_reversed(min_index, current_index, max_index, count)
    else:
        return get_cyclic_range_indices_normal(min_index, current_index, max_index, count)


def get_cyclic_range(min_index, current_index, max_index, count, reverse_search=False):
    range_indices_list = get_cyclic_range_indices(min_index, current_index, max_index, count, reverse_search)

    step = -1 if reverse_search else 1

    range_list = []
    for index_tuple in range_indices_list:
        range_list += list(range(index_tuple[0], index_tuple[1], step))

    return range_list


def get_closest_waypoint_index(waypoints, current_pose, current_waypoint_index, base_waypoints_num, reverse_search=False):
    # waypoints are cyclic
    waypoints_range = get_cyclic_range(0, current_waypoint_index, base_waypoints_num - 1, base_waypoints_num, reverse_search)

    min_distance = direct_position_distance(current_pose.position, waypoints[current_waypoint_index].pose.pose.position)

    # optimization - waypoints are sorted - start search from current waypoint and end search when gap is growing
    for i in waypoints_range:
        distance = direct_position_distance(current_pose.position, waypoints[i].pose.pose.position)
        # should continue only while the distance trend is down
        if distance < min_distance:
            min_distance = distance
            current_waypoint_index = i
        else:
            continue

    return current_waypoint_index


def get_closest_waypoint_ahead_index(waypoints, current_pose, current_waypoint_index
                                     , base_waypoints_num, reverse_search=False):
    return get_closest_waypoint_before_or_ahead_index(waypoints, current_pose, current_waypoint_index
                                                      , base_waypoints_num, 1, reverse_search)


def get_closest_waypoint_before_index(waypoints, current_pose, current_waypoint_index
                                      , base_waypoints_num, reverse_search=False):
    return get_closest_waypoint_before_or_ahead_index(waypoints, current_pose, current_waypoint_index
                                                      , base_waypoints_num, -1, reverse_search)


def get_closest_waypoint_before_or_ahead_index(waypoints, current_pose, current_waypoint_index, base_waypoints_num
                                               , index_search_addition, reverse_search=False):

    current_waypoint_index = get_closest_waypoint_index(waypoints, current_pose, current_waypoint_index
                                                        , base_waypoints_num, reverse_search)

    # min distance waypoint might be behind or ahead of the current position
    if not waypoint_ahead(current_pose, waypoints[current_waypoint_index]):
        current_waypoint_index += index_search_addition

    return current_waypoint_index % base_waypoints_num