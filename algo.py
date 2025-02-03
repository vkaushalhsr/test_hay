#!/usr/bin/env python3
# General imports

import traceback
import os
import threading
from datetime import datetime
import pytz
import math
import signal
import subprocess

# ROS python imports
import rospy
import roslaunch
import tf

# ROS msgs imports
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from map_msgs.msg import OccupancyGridUpdate
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, Image
from std_srvs.srv import Empty

# Image Processing imports
import numpy as np
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

ROBOT = rospy.get_param("/haystack/robot", "VIOLET")
DEBUG = False
MANUAL = False
if ROBOT == "PURPLE":
    print(f"ROBOT purple")
    REEMAN = True
if ROBOT == "VIOLET":
    print(f"ROBOT is Violet")
    REEMAN = False
LIVE_IMAGE_UPDATE = True

# ROS Topics
MAP_TOPIC = "/map"
GLOBAL_COST_MAP_TOPIC = "/move_base/global_costmap/costmap"
GLOBAL_COST_MAP_UPDATE_TOPIC = "/move_base/global_costmap/costmap_updates"
PERSON_DETECTED_TOPIC = "/is_person_detected"
MAP_CAMERA_FILTER_TOPIC = "/map_camera_filter"
MAP_DISINFECTED_TOPIC = "/map_disinfected"
CURRENT_FREE_SPACE_TOPIC = "/map_valid_free_space"
ROBOT_TRACE_TOPIC = "/robot_trace"
DISINFECT_OBJECT_DISTANCE = "/disinfect_object_distance"
VELOCITY_TOPIC = "/cmd_vel"
PATTERN_PATH_TOPIC = "/path"
POSE_MARKER_TOPIC = "/nodes_marker"
DISINFECTED_OBJECT_MARKER_TOPIC = "/disinfected_object_marker"
COVERAGE_MARKER_TOPIC = "/coverage_marker"
DISINFECT_OBJECT_MARKER_TOPIC = "/disinfect_object_marker"
SHORTEST_PATH_TOPIC = "/connecting_path"
GET_PLAN_SERVICE = '/move_base/NavfnROS/make_plan'
CLEAR_COSTMAP_SERVICE = "/move_base/clear_costmaps"
GET_PLAN_TOPIC = "/get_plan"
SCAN_TOPIC = "/scan_filtered"
CAMERA_DEPTH_TOPIC = "/camera/depth/image_rect_raw"
GOAL_TOPIC = "coverage/goal"
TIMEZONE_TOPIC = "/haystack/timezone"

# ROS PARAMS
COVERAGE_STATE_PARAM = "/coverage/state"
DISINFECT_STATE_PARAM = "/disinfect_status"
COVERAGE_IMAGE_NAME = "/coverage/image_name"
COVERAGE_PERCENTAGE = "/coverage/percentage"
DISINFECTION_PERCENTAGE = "/coverage/disinfection_percentage"
LAMP_STATE = "/lamp_state"
MODE_PARAM = "/haystack/mode"
DETECTED_OBJECT_PARAM = "/coverage/detected_objects"
DEVICE_STATUS = "/haystack/device_status"
LAUNCH_STATUS = "/haystack/launch_status"
COVERAGE_START_TIME_PARAM = "/coverage/start_time"

# ROS Frame
GLOBAL_FRAME = "/map"
ROBOT_FRAME = "/base_link"

# General Parameters


IMAGE_PATH = "/haystack_disinfect_report/images/"

# Map Parameters
OBSTACLE_THRESHOLD = 100
ROBOT_OBSTACLE_THRESHOLD = 99
FREE_SPACE_THRESHOLD = 0
UNIDENTIFIED_THRESHOLD = -1
ROBOT_ROTATION_RADIUS = 0.45
ROBOT_PATTERN_RADIUS = 0.40
ROBOT_LINEAR_RADIUS = 0.30
ROBOT_CENTER_OFFSET_FROM_CAMERA = 0.13
RATE = 10
ROBOT_ROTATION_CHECK = 2
INSCRIBED_INFLATED_OBSTACLE = 99

# initialization
IDLE_WAIT_TIME = 20.0
INITIALIZATION_DISTANCE = 0.7
INITIALIZATION_SAFETY_CHECK = 3
INITIALIZATION_RETRY = 5
ROTATION_DEGREE = 360
ROTATIONAL_VELOCITY = 20

# Exploration
K_MEAN_CLUSTER_COUNT = 5
MINIMUM_EXPLORATION_NODES = 2
EXPLORATION_LOW_COST_SEARCH_STEP = 6

# Coverage
if REEMAN:
    PATTERN_STEP_DISTANCE = 0.60
else:
    PATTERN_STEP_DISTANCE = 0.70
COVERAGE_FREE_SPACE_DILATION = 4
DISINFECTION_COVERAGE_RADIUS = 1.0
DISINFECTION_BLIND_SPOT_RADIUS = 0.3
DISINFECT_OBJECT_THRESHOLD = 3
MINIMUM_COVERAGE_NODES = 4
COVERAGE_THRESHOLD = 99.8
COVERAGE_CONNECTED_COMPONENT_THRESHOLD = 99.8
ROTATION_NEAR_OBJECT = True
COVERAGE_MAX_VELOCITY = 0.08
GOAL_WAIT_DELAY = False
DISINFECTION_WAIT_FLAG = True
SMART_DISINFECTION = True
DISINFECT_OBJECT_WAIT_TIME = 20
DISINFECTION_WAIT_TIME_MULTIPLIER = 10
COVERAGE_GOAL_WAIT_TIME = 3.0
ROTATION_SAFETY_CHECK = True
SMART_DISINFECT_DISTANCE = 0.5
SUCCESS_PERCENTAGE = 95

# Inplace
INPLACE_DURATION = {0: 120, 1: 300, 2: 600, 3: 900}

# Linear Moment
MAX_DEGREE = 45
TRUST_LIMIT = 10
ROBOT_RADIUS = 0.30
LIDAR_OBSTACLE_DETECTION_DISTANCE = 0.50
LIDAR_OBSTACLE_DETECTION_FORWARD_DISTANCE = 0.70

# Report Parameters
CROP_OFFSET = 15
REPORT_IMAGE_WIDTH = 250
REPORT_IMAGE_HEIGHT = 250
OBSTACLE_COLOR = (0, 0, 0)  # black
ROBOT_TRACE_COLOR = (33, 209, 27)  # green
ROBOT_BODY_COLOR = (20, 211, 214)  # light yellow
ROBOT_DIRECTION_COLOR = (255, 0, 0)  # blue
COVERED_COLOR = (163, 230, 163)  # greenish
DISINFECTED_OBSTACLE_COLOR = (75, 120, 90)  # dark green
DISINFECTED_COLOR = (193, 240, 193)  # light green

if DEBUG:
    IMAGE_PATH = "/home/hariharan/"
    DISINFECT_NAVIGATION_LAUNCH_PATH = ["/home/hariharan/turtlebot/src/exploration/launch/turtlebot_nav.launch"]
    IDLE_WAIT_TIME = 1.0
    COVERAGE_MAX_VELOCITY = 0.12

if REEMAN:
    GET_PLAN_SERVICE = '/move_base/make_plan'
    SCAN_TOPIC = "/scan"
    ROBOT_RADIUS = 0.0
    if not DEBUG:
        IDLE_WAIT_TIME = 40.0
        DISINFECT_NAVIGATION_LAUNCH_PATH = roslaunch.rlutil.resolve_launch_arguments(['yoyo_bringup',
                                                                                      'navigation.launch'])
    ROTATION_SAFETY_CHECK = False
elif not DEBUG:
    DISINFECT_NAVIGATION_LAUNCH_PATH = roslaunch.rlutil.resolve_launch_arguments(
        ['haystack', 'navigator_sanitize.launch'])

output = subprocess.check_output("timedatectl show --property=Timezone --value", shell=True, text=True)
current_time_zone = output.strip("\n")
TIMEZONE = pytz.timezone(rospy.get_param(TIMEZONE_TOPIC, current_time_zone))
logfile_name = IMAGE_PATH + datetime.now(TIMEZONE).strftime("%Y_%m_%d_%H_%M_%S") + '.log'


def log_data(data):
    logfile = open(logfile_name, "a")
    logfile.write(data + "\n")
    logfile.close()


def node_to_radius_degree(node):
    radius_theta_data = {}
    radius_values = []
    matrix_size = node.shape[0]
    center_node = (int(matrix_size / 2), int(matrix_size / 2))
    for i in range(0, 360):
        radius_values.append([])

    for row in range(0, matrix_size):
        for column in range(0, matrix_size):
            radius = round(math.sqrt(
                (row - center_node[0]) * (row - center_node[0]) +
                (column - center_node[1]) * (column - center_node[1])), 4)
            degree = int(get_relative_angle(center_node, (column, row)))
            if degree < 0:
                degree = 360 + degree
            radius_values[degree].append(radius)
            radius_theta_data[str(degree) + " " + str(radius)] = (row, column)
            if not AIA.running:
                return np.array([], dtype=np.uint16), np.array([], dtype=np.uint16)

    return radius_values, radius_theta_data


def dilated_area(size, nodes, layer_size, rect, zero_value):
    image_map = np.zeros((size[0], size[1]), np.uint8)
    image_map[nodes] = 255
    if rect:
        filter_matrix = cv2.getStructuringElement(cv2.MORPH_RECT,
                                                  (int(layer_size + 1),
                                                   int(layer_size + 1)))
    else:
        filter_matrix = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                  (int(layer_size + 1),
                                                   int(layer_size + 1)))

    image_dilated_map = cv2.dilate(image_map, filter_matrix, iterations=1)
    image_dilated_map[np.where(image_dilated_map == 0)] = zero_value
    return image_dilated_map


def line_detection(size, nodes):
    image = np.zeros((size[0], size[1]), np.uint8)
    image[nodes] = 255
    threshold = 100
    done = 0
    angle = 0
    while AIA.running:
        original_image = np.copy(image)
        lines = cv2.HoughLinesP(image, rho=1, theta=1 * np.pi / 180, threshold=threshold, maxLineGap=10)
        threshold -= 10
        print(lines)
        log_data(str(lines))
        if lines is not None:
            for line in lines:
                angle = get_relative_angle((line[0][2], line[0][3]), (line[0][0], line[0][1]))
                cv2.line(original_image, (line[0][0], line[0][1]), (line[0][2], line[0][3]),
                         (0, 123, 255), 3, cv2.LINE_AA)
                print(angle, threshold)
                log_data(str(angle) + " " + str(threshold))
                done = 1
                break
        if done == 1 and threshold > 5:
            break
        if threshold < 5:
            print("Reached lower threshold")
            log_data("Reached lower threshold")
            break
    return angle


def get_node(start, distance, degree):
    start_y = (start[0] - AIA.map_origin[0]) * AIA.map_resolution + \
              (AIA.map_resolution / 2)
    start_x = (start[1] - AIA.map_origin[1]) * AIA.map_resolution + \
              (AIA.map_resolution / 2)
    x = math.cos(math.radians(degree)) * distance
    y = math.sin(math.radians(degree)) * distance
    node_x = int(round(((start_y + y - (AIA.map_resolution / 2)) / AIA.map_resolution) + AIA.map_origin[0]))
    node_y = int(round(((start_x + x - (AIA.map_resolution / 2)) / AIA.map_resolution) + AIA.map_origin[1]))
    return node_x, node_y


def get_contours(size, nodes):
    image = np.zeros((size[0], size[1]), np.uint8)
    image[nodes] = 255
    kernel = np.ones((3, 3), np.uint8)
    # img_dilation = cv2.dilate(image, kernel, iterations=1)
    # img_erosion = cv2.erode(img_dilation, kernel, iterations=1)

    contours, heir = cv2.findContours(image=image, mode=cv2.RETR_TREE,
                                      method=cv2.CHAIN_APPROX_SIMPLE)
    connected_components = []
    edge_connected_components = []
    # print("len of contour = " + str(len(contours)))
    # log_data("len of contour = " + str(len(contours)))
    # end_time = datetime.now(TIMEZONE)
    # date_time = end_time.strftime("%Y_%m_%d_%H_%M_%S_")
    # cv2.imwrite(IMAGE_PATH + date_time + "Before_Contours.png", image)
    total_area = np.zeros((size[0], size[1]), np.uint8)
    for i in range(0, len(contours)):
        edge_contour_image = np.ones((size[0], size[1]), np.uint8)
        cv2.drawContours(edge_contour_image, contours, i, (255, 255, 255), 1)
        edge_contour_np = np.where((edge_contour_image == image) == True)
        contour_image = np.ones((size[0], size[1]), np.uint8)
        cv2.drawContours(contour_image, contours, i, (255, 255, 255), cv2.FILLED)
        # cv2.imwrite(IMAGE_PATH + date_time + str(i) + "_contour.png", contour_image)
        # contour_image = cv2.erode(contour_image, kernel, iterations=1)
        contour_np = np.where((contour_image == image) == True)
        already_selected_np = np.where(
            (dilated_area((size[0], size[1]), contour_np,
                          0, False, 1) == total_area) == True)
        already_selected_map = dilated_area((size[0], size[1]), already_selected_np, 0, False, 0)
        contour_np = np.where((dilated_area((size[0], size[1]), contour_np, 0, False, 0) ==
                               already_selected_map) == False)
        # cv2.imwrite(IMAGE_PATH + date_time + str(i) + "_erode_contour.png", contour_image)
        if len(contour_np[0]) > 0:
            contour_map = dilated_area((size[0], size[1]),
                                       contour_np, 0, False, 0)
            edge_contour_map = dilated_area((size[0], size[1]),
                                            edge_contour_np, 0, False, 0)
            # contour_map = cv2.dilate(contour_map, kernel, iterations=1)
            temp = np.logical_or(total_area, contour_map)
            total_area = temp.astype('uint8') * 255
            # cv2.imwrite(IMAGE_PATH + date_time + str(i) + "_selected_contour.png", contour_map)
            # cv2.imwrite(IMAGE_PATH + date_time + str(i) + "_selected_edge_contour.png", edge_contour_map)

            connected_components.append(contour_map)
            edge_connected_components.append(edge_contour_map)

    return connected_components, edge_connected_components


def get_relative_angle(starting, goal):
    intersection = (starting[0], goal[1])
    # print(starting, goal)
    x_difference = goal[0] - starting[0]
    y_difference = goal[1] - starting[1]
    angle = 0
    if x_difference != 0 and y_difference != 0:
        starting_intersection = math.sqrt((starting[0] - intersection[0]) * (starting[0] - intersection[0]) +
                                          (starting[1] - intersection[1]) * (starting[1] - intersection[1]))
        goal_intersection = math.sqrt((goal[0] - intersection[0]) * (goal[0] - intersection[0]) +
                                      (goal[1] - intersection[1]) * (goal[1] - intersection[1]))

        angle = math.degrees(math.atan(goal_intersection / starting_intersection))
        # print(starting_intersection, goal_intersection, angle)

    if x_difference > 0 and y_difference > 0:
        angle = 90 - angle
    elif x_difference > 0 > y_difference:
        angle = -90 + angle
    elif x_difference < 0 < y_difference:
        angle = 90 + angle
    elif x_difference < 0 and y_difference < 0:
        angle = -90 - angle
    elif x_difference > 0 and y_difference == 0:
        angle = 0
    elif x_difference < 0 and y_difference == 0:
        angle = 180
    elif x_difference == 0 and y_difference > 0:
        angle = 90
    elif x_difference == 0 and y_difference < 0:
        angle = -90
    # print(angle)
    return angle


def create_marker(marker_type, frame_id, x, y, direction, color):
    robot_marker = Marker()
    robot_marker.header.frame_id = "map"
    robot_marker.header.stamp = rospy.Time.now()
    robot_marker.id = frame_id
    robot_marker.type = marker_type
    robot_marker.pose.position.x = x
    robot_marker.pose.position.y = y
    robot_marker.pose.position.z = 0.5
    quaternion = quaternion_from_euler(0, 0, math.radians(direction))
    robot_marker.pose.orientation.x = quaternion[0]
    robot_marker.pose.orientation.y = quaternion[1]
    robot_marker.pose.orientation.z = quaternion[2]
    robot_marker.pose.orientation.w = quaternion[3]
    # robot_marker.points.x = 0
    # robot_marker.points.y = 0
    robot_marker.scale.x = 0.1
    robot_marker.scale.y = 0.1
    robot_marker.scale.z = 0.1
    robot_marker.color.a = color[0]
    robot_marker.color.r = color[1]
    robot_marker.color.g = color[2]
    robot_marker.color.b = color[3]
    robot_marker.lifetime = rospy.Duration.from_sec(100)
    return robot_marker


def create_pose_stamp(x, y, direction):
    angle = math.radians(direction)
    quaternion = quaternion_from_euler(0, 0, angle)
    pose_stamp = PoseStamped()
    pose_stamp.header.frame_id = "map"
    pose_stamp.header.stamp = rospy.Time.now()
    # pose_stamp.id = frame_id
    pose_stamp.pose.position.z = 0
    pose_stamp.pose.orientation.x = quaternion[0]
    pose_stamp.pose.orientation.y = quaternion[1]
    pose_stamp.pose.orientation.z = quaternion[2]
    pose_stamp.pose.orientation.w = quaternion[3]
    pose_stamp.header.stamp = rospy.Time.now()
    pose_stamp.pose.position.x = x
    pose_stamp.pose.position.y = y
    return pose_stamp


class Node:
    """node with properties of g, h, coordinate and parent node"""

    def __init__(self, G=0, H=0, coordinate=None, parent=None):
        self.G = G
        self.H = H
        self.F = G + H
        self.parent = parent
        self.coordinate = coordinate

    def reset_f(self):
        self.F = self.G + self.H


def hcost(node_coordinate, goal):
    dx = abs(node_coordinate[0] - goal[0])
    dy = abs(node_coordinate[1] - goal[1])
    hcost = dx + dy
    return hcost


def gcost(fixed_node, update_node_coordinate):
    dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
    dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
    gc = math.hypot(dx, dy)  # gc = move from fixed_node to update_node
    gcost = fixed_node.G + gc  # gcost = move from start point to update_node
    return gcost


def node_to_coordinate(node_list):
    # convert node list into coordinate list and array
    coordinate_list = [node.coordinate for node in node_list]
    return coordinate_list


def find_path(open_list, closed_list, goal):
    # searching for the path, update open and closed list
    # obstacle = obstacle and boundary
    flag = len(open_list)
    for i in range(flag):
        node = open_list[0]
        open_coordinate_list = [node.coordinate for node in open_list]
        closed_coordinate_list = [node.coordinate for node in closed_list]
        temp = AIA.find_neighbor(node, closed_coordinate_list)
        for element in temp:
            if element in closed_list:
                continue
            elif element in open_coordinate_list:
                # if node in open list, update g value
                ind = open_coordinate_list.index(element)
                new_g = gcost(node, element)
                if new_g <= open_list[ind].G:
                    open_list[ind].G = new_g
                    open_list[ind].reset_f()
                    open_list[ind].parent = node
            else:  # new coordinate, create corresponding node
                ele_node = Node(coordinate=element, parent=node,
                                G=gcost(node, element), H=hcost(element, goal))
                open_list.append(ele_node)
        open_list.remove(node)
        closed_list.append(node)
        open_list.sort(key=lambda x: x.F)
    return open_list, closed_list


def check_node_coincide(close_ls1, closed_ls2):
    """
    :param close_ls1: node closed list for searching from start
    :param closed_ls2: node closed list for searching from end
    :return: intersect node list for above two
    """
    # check if node in close_ls1 intersect with node in closed_ls2
    cl1 = node_to_coordinate(close_ls1)
    cl2 = node_to_coordinate(closed_ls2)
    intersect_ls = [node for node in cl1 if node in cl2]
    return intersect_ls


def find_node_index(coordinate, node_list):
    # find node index in the node list via its coordinate
    ind = 0
    for node in node_list:
        if node.coordinate == coordinate:
            target_node = node
            ind = node_list.index(target_node)
            break
    return ind


def get_path(org_list, goal_list, coordinate):
    # get path from start to end
    path_org: list = []
    path_goal: list = []
    ind = find_node_index(coordinate, org_list)
    node = org_list[ind]
    while node != org_list[0]:
        path_org.append(node.coordinate)
        node = node.parent
    path_org.append(org_list[0].coordinate)
    ind = find_node_index(coordinate, goal_list)
    node = goal_list[ind]
    while node != goal_list[0]:
        path_goal.append(node.coordinate)
        node = node.parent
    path_goal.append(goal_list[0].coordinate)
    path_org.reverse()
    path = path_org + path_goal
    path = np.array(path)
    return path


def draw_control(org_closed, goal_closed, flag):
    """
    control the plot process, evaluate if the searching finished
    flag == 0 : draw the searching process and plot path
    flag == 1 or 2 : start or end is blocked, draw the border line
    """
    stop_loop = 0  # stop sign for the searching
    org_closed_ls = node_to_coordinate(org_closed)
    goal_closed_ls = node_to_coordinate(goal_closed)
    path = []
    if flag == 0:
        node_intersect = check_node_coincide(org_closed, goal_closed)
        if node_intersect:  # a path is find
            path = get_path(org_closed, goal_closed, node_intersect[0])
            stop_loop = 1
            print('Path found!')
            log_data('Path found!')
    elif flag == 1:  # start point blocked first
        stop_loop = 1
        print('There is no path to the goal! Start point is blocked!')
        log_data('There is no path to the goal! Start point is blocked!')
    elif flag == 2:  # end point blocked first
        stop_loop = 1
        print('There is no path to the goal! End point is blocked!')
        log_data('There is no path to the goal! End point is blocked!')
    return stop_loop, path


def searching_control(start, end):
    """manage the searching process, start searching from two side"""
    # initial origin node and end node
    print("Inside A star Path Finding Algorithms")
    log_data("Inside A star Path Finding Algorithms")
    origin = Node(coordinate=start, H=hcost(start, end))
    goal = Node(coordinate=end, H=hcost(end, start))
    # list for searching from origin to goal
    origin_open: list = [origin]
    origin_close: list = []
    # list for searching from goal to origin
    goal_open = [goal]
    goal_close: list = []
    # initial target
    target_goal = end
    # flag = 0 (not blocked) 1 (start point blocked) 2 (end point blocked)
    flag = 0  # init flag
    path = []
    while True:
        # searching from start to end
        origin_open, origin_close = \
            find_path(origin_open, origin_close, target_goal)
        if not origin_open:  # no path condition
            flag = 1  # origin node is blocked
            draw_control(origin_close, goal_close, flag)
            break
        # update target for searching from end to start
        target_origin = min(origin_open, key=lambda x: x.F).coordinate

        # searching from end to start
        goal_open, goal_close = \
            find_path(goal_open, goal_close, target_origin)
        if not goal_open:  # no path condition
            flag = 2  # goal is blocked
            draw_control(origin_close, goal_close, flag)
            break
        # update target for searching from start to end
        target_goal = min(goal_open, key=lambda x: x.F).coordinate

        # continue searching, draw the process
        stop_sign, path = draw_control(origin_close, goal_close, flag)
        if stop_sign:
            break
    final_path = []
    path_value = Path()
    path_value.header.frame_id = "map"
    for n in range(0, len(path)):
        y = (path[n][0] - AIA.map_origin[0]) * AIA.map_resolution + (AIA.map_resolution / 2)
        x = (path[n][1] - AIA.map_origin[1]) * AIA.map_resolution + (AIA.map_resolution / 2)
        final_path.append((path[n][0], path[n][1]))
        path_value.poses.append(create_pose_stamp(x, y, 0))
        AIA.connect_path_pub.publish(path_value)
    return final_path


class AdvanceIntelligenceAlgorithm:
    def __init__(self):
        self.last_disinfected_point = None
        self.object_detected_nodes = {}
        self.object_detected = []
        self.disinfected_obstacle_np = (np.array([], dtype='int64'), np.array([], dtype='int64'))
        self.raw_covered_node_np = (np.array([], dtype='int64'), np.array([], dtype='int64'))
        self.raw_disinfected_node_np = (np.array([], dtype='int64'), np.array([], dtype='int64'))
        self.object_detection_np = (np.array([], dtype='int64'), np.array([], dtype='int64'))
        self.disinfected_object_detection_np = (np.array([], dtype='int64'), np.array([], dtype='int64'))
        self.disinfect_map_update_check = 0
        self.ignored_covered_map = None
        self.uncovered_node_np = None
        self.obstacle_map = None
        self.absolute_map = None
        self.object_detection_lock = True
        self.update_disinfect_map = True
        self.robot_safe = True
        self.disinfect_type = 0
        self.obstacle_detected_count = 0
        self.trust_safe = 0
        self.inscribed_radius = None
        self.mark_trace = False
        self.image_robot_trace = []
        self.init_image_robot_trace = []
        self.image_robot_trace_poses = []
        self.robot_path_np = (np.array([], dtype='int64'), np.array([], dtype='int64'))
        self.init_robot_path_np = (np.array([], dtype='int64'), np.array([], dtype='int64'))
        self.robot_travelled_path = []
        self.inflation_radius = None
        self.cost_scaling_factor = None
        self.pattern_threshold = None
        self.path_threshold = None
        self.rotation_safety_threshold = None
        self.map_origin = None
        self.map_msg = None
        self.global_map_msg = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_x = None
        self.map_y = None
        self.map = None
        self.demo_mode = False
        self.disinfect_obstacle_nodes = []
        self.disinfected_area_nodes = []
        self.disinfected_object_node = []
        self.inflation_cost_values = []
        self.rotatable_path_points = []
        self.report_image = None
        self.robot_node = None
        self.percentage = 0
        self.starting_radian = 0
        self.starting_position = (0, 0)
        self.robot_orientation = None
        self.robot_position = None
        self.robot_angle = None
        self.robot_radian = None
        self.running = True
        self.got_map = False
        self.filter_matrix = None
        self.starting_node = None
        self.end_time = None
        self.detected_object = []
        self.exploration_image = True
        self.explored_points = []
        self.image_path_nodes = []
        self.disinfected_free_space = []
        self.disinfected_nodes = []
        self.disinfect_object_node = []
        self.disinfected_percentage = 0
        self.sensor_check = {"LIDAR": False, "CAMERA": False}
        self.rate = rospy.Rate(RATE)
        self.listener = tf.TransformListener()
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.image_generation_start_time = rospy.Time.now().to_sec()
        self.map_camera_filter_pub = rospy.Publisher(MAP_CAMERA_FILTER_TOPIC, OccupancyGrid, queue_size=10)
        self.pose_marker_pub = rospy.Publisher(POSE_MARKER_TOPIC, Marker, queue_size=10)
        self.disinfected_object_marker_pub = rospy.Publisher(DISINFECTED_OBJECT_MARKER_TOPIC, MarkerArray,
                                                             queue_size=10)
        self.coverage_marker_pub = rospy.Publisher(COVERAGE_MARKER_TOPIC, MarkerArray,
                                                   queue_size=10)
        self.path_pub = rospy.Publisher(PATTERN_PATH_TOPIC, Path, queue_size=10)
        self.velocity_pub = rospy.Publisher(VELOCITY_TOPIC, Twist, queue_size=10)
        self.connect_path_pub = rospy.Publisher(SHORTEST_PATH_TOPIC, Path, queue_size=10)
        self.map_disinfected_pub = rospy.Publisher(MAP_DISINFECTED_TOPIC, OccupancyGrid, queue_size=10)
        self.robot_trace_pub = rospy.Publisher(ROBOT_TRACE_TOPIC, Path, queue_size=10)
        self.disinfect_object_marker_pub = rospy.Publisher(DISINFECT_OBJECT_MARKER_TOPIC, MarkerArray, queue_size=10)
        self.disinfect_navigation_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.disinfect_navigation_uuid)
        self.disinfect_navigation_uuid = roslaunch.parent.ROSLaunchParent(self.disinfect_navigation_uuid,
                                                                          DISINFECT_NAVIGATION_LAUNCH_PATH)
        self.get_plan = rospy.ServiceProxy(GET_PLAN_SERVICE, GetPlan)
        self.clear_costmap = rospy.ServiceProxy(CLEAR_COSTMAP_SERVICE, Empty)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction, )
        self.disinfect_update_obj = threading.Thread(target=self.disinfect_map_thread)
        self.map_processing_obj = threading.Thread(target=self.map_processing_thread)
        self.generate_Image_obj = threading.Thread(target=self.generate_image_thread)
        self.start_time = datetime.now(TIMEZONE)
        rospy.set_param(COVERAGE_START_TIME_PARAM, self.start_time.strftime("%H:%M:%S"))
        self.move_base_active = False
        self.initialization_done = False
        self.coverage_done = False
        self.send_goal_enable = True
        self.disinfect_object_wait_time_enabled = True
        self.cancel_step_goal = False
        self.state = "IDLE"
        self.velocity_plan_retries = 0
        rospy.Subscriber(DISINFECT_OBJECT_DISTANCE, String, self.disinfect_object_callback, queue_size=1)
        rospy.Subscriber(PERSON_DETECTED_TOPIC, Bool, self.person_detected_callback)
        rospy.Subscriber(MAP_TOPIC, OccupancyGrid, self.map_callback)
        rospy.Subscriber(GLOBAL_COST_MAP_TOPIC, OccupancyGrid, self.global_cost_map_callback)
        rospy.Subscriber(GET_PLAN_TOPIC, String, self.get_plan_callback)
        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self.goal_callback)
        if not REEMAN:
            rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scan_callback)
            self.depth_image_sub = rospy.Subscriber(CAMERA_DEPTH_TOPIC, Image, self.depth_camera_callback)
        else:
            rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scan_reeman_callback)

    def signal_handler(self, *args):
        # print("USER_CTRL_C")
        # log_data("USER_CTRL_C")
        self.running = False
        rospy.set_param(COVERAGE_STATE_PARAM, "USER_CTRL_C")
        self.state = "USER_CTRL_C"
        rospy.signal_shutdown("shutdown")

    def get_plan_callback(self, data):
        searching_control(self.robot_node, self.starting_node)

    def goal_callback(self, msg):
        try:
            goal_node_x = int((msg.pose.position.y / self.map_resolution) + self.map_origin[0])
            goal_node_y = int((msg.pose.position.x / self.map_resolution) + self.map_origin[0])
            self.velocity_plan_retries = 0
            self.adv_velocity_to_goal((goal_node_x, goal_node_y), 0.1, ignore_obstacle=False)
        except:
            log_data(f"{traceback.format_exc()}")

    def global_cost_map_callback(self, map_msg):
        # print("global costmap callback")
        self.global_map_msg = map_msg
        if not self.map_processing_obj.is_alive():
            self.map_processing_obj = threading.Thread(target=self.map_processing_thread)
            self.map_processing_obj.start()

    def map_processing_thread(self):
        if self.global_map_msg is not None:
            map_msg = self.global_map_msg
            global_cost_map_data = map_msg.data
            global_cost_map_data = np.array(global_cost_map_data)
            if self.map is not None:
                global_cost_map_data[np.where(self.map == UNIDENTIFIED_THRESHOLD)] = UNIDENTIFIED_THRESHOLD
                map_msg.data = tuple(global_cost_map_data)
                self.filter_matrix = np.reshape(np.array(global_cost_map_data), (self.map_width, self.map_height))
                try:
                    self.map_camera_filter_pub.publish(map_msg)
                except rospy.ROSException:
                    pass
                self.got_map = True

    def map_callback(self, map_msg):
        self.map_msg = map_msg
        self.map_resolution = round(map_msg.info.resolution, 2)
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_x = map_msg.info.origin.position.x
        self.map_y = map_msg.info.origin.position.y
        self.map = np.array(map_msg.data)
        if self.map_origin is None:
            self.map_origin = (int(round(abs(self.map_x) / self.map_resolution)),
                               int(round(abs(self.map_y) / self.map_resolution)))

    def scan_reeman_callback(self, data):
        scan_data = np.array(data.ranges)

        scan_data[np.isinf(scan_data)] = 0
        scan_data = np.round(scan_data, decimals=2)
        obstacle_detected = False

        forward_backward_safe_distance = 0.3
        left_right_first_half_distance = 0.27
        left_right_second_half_distance = 0.15

        left_first_half = scan_data[28:100]
        self.left_first_half = left_first_half[left_first_half > 0]
        left_second_half = scan_data[101:500]
        self.left_second_half = left_second_half[left_second_half > 0]
        left_third_half = scan_data[501:600]
        self.left_third_half = left_third_half[left_third_half > 0]
        front = scan_data[601:966]
        self.front = front[front > 0]
        right_third_half = scan_data[967:1067]
        self.right_third_half = right_third_half[right_third_half > 0]
        right_second_half = scan_data[1068:1367]
        self.right_second_half = right_second_half[right_second_half > 0]
        right_first_half = scan_data[1368:1428]
        self.right_first_half = right_first_half[right_first_half > 0]

        front_value = np.argwhere(np.logical_and(self.front <= forward_backward_safe_distance,
                                                 self.front >= ROBOT_RADIUS))

        left_first_half_value = np.argwhere(np.logical_and(self.left_first_half <= left_right_first_half_distance,
                                                           self.left_first_half >= left_right_second_half_distance,
                                                           self.left_first_half >= ROBOT_RADIUS))
        left_second_half_value = np.argwhere(np.logical_and(self.left_second_half <= 0.28,
                                                            self.left_second_half >= left_right_second_half_distance,
                                                            self.left_second_half >= ROBOT_RADIUS))
        left_third_half_value = np.argwhere(np.logical_and(self.left_third_half <= 0.25,
                                                           self.left_third_half >= left_right_second_half_distance,
                                                           self.left_third_half >= ROBOT_RADIUS))

        right_first_half_value = np.argwhere(np.logical_and(self.right_first_half <= left_right_first_half_distance,
                                                            self.right_first_half >= left_right_second_half_distance,
                                                            self.right_first_half >= ROBOT_RADIUS))
        right_second_half_value = np.argwhere(np.logical_and(self.right_second_half <= 0.28,
                                                             self.right_second_half >= left_right_second_half_distance,
                                                             self.right_second_half >= ROBOT_RADIUS))
        right_third_half_value = np.argwhere(np.logical_and(self.right_third_half <= 0.25,
                                                            self.right_third_half >= left_right_second_half_distance,
                                                            self.right_third_half >= ROBOT_RADIUS))

        if (front_value.size or left_first_half_value.size or left_second_half_value.size or
            left_third_half_value.size or right_first_half_value.size or right_second_half_value.size or
            right_third_half_value.size) > 0:
            obstacle_detected = True

        if obstacle_detected:
            self.robot_safe = False
            if self.demo_mode:
                rospy.set_param("/haystack/obstacle", True)
            self.trust_safe = 0
        else:
            self.trust_safe += 1
            if self.trust_safe > TRUST_LIMIT:
                self.robot_safe = True
                if self.demo_mode:
                    rospy.set_param("/haystack/obstacle", False)
                self.trust_safe = 0

    def scan_callback(self, data):
        self.sensor_check["LIDAR"] = True
        scan_data = np.array(data.ranges)

        scan_data[np.isinf(scan_data)] = 0
        scan_data = np.round(scan_data, decimals=2)

        back = np.append(scan_data[(720 - int(MAX_DEGREE / 2) * 2):], scan_data[:(0 + int(MAX_DEGREE / 2) * 2)])
        left = scan_data[180 - int(MAX_DEGREE / 2) * 2:180 + int(MAX_DEGREE / 2) * 2]
        front = scan_data[360 - int(MAX_DEGREE / 2) * 2:360 + int(MAX_DEGREE / 2) * 2]
        right = scan_data[540 - int(MAX_DEGREE / 2) * 2:540 + int(MAX_DEGREE / 2) * 2]
        obstacle_detected = 0
        back_value = np.argwhere(np.logical_and(back <= LIDAR_OBSTACLE_DETECTION_DISTANCE, back >= ROBOT_RADIUS))
        if back_value.size > 0:
            if len(back_value[0]):
                obstacle_detected += 1

        left_value = np.argwhere(np.logical_and(left <= LIDAR_OBSTACLE_DETECTION_DISTANCE, left >= ROBOT_RADIUS))
        if left_value.size > 0:
            if len(left_value[0]):
                obstacle_detected += 1

        right_value = np.argwhere(np.logical_and(right <= LIDAR_OBSTACLE_DETECTION_DISTANCE, right >= ROBOT_RADIUS))
        if right_value.size > 0:
            if len(right_value[0]):
                obstacle_detected += 1

        front_value = np.argwhere(np.logical_and(front <= LIDAR_OBSTACLE_DETECTION_FORWARD_DISTANCE,
                                                 front >= ROBOT_RADIUS))
        if front_value.size > 0:
            if len(front_value[0]):
                obstacle_detected += 1

        if obstacle_detected > 0:
            self.robot_safe = False
            if self.demo_mode:
                rospy.set_param("/haystack/obstacle", True)
            self.trust_safe = 0
        else:
            self.trust_safe += 1
            if self.trust_safe > TRUST_LIMIT:
                self.robot_safe = True
                if self.demo_mode:
                    rospy.set_param("/haystack/obstacle", False)
                self.trust_safe = 0

    def depth_camera_callback(self, data):
        self.sensor_check["CAMERA"] = True

    def person_detected_callback(self, status):
        if status.data and self.running and self.state != "IDLE" and not self.demo_mode:
            # print("PERSON_DETECTED")
            # log_data("PERSON_DETECTED")
            rospy.set_param(COVERAGE_STATE_PARAM, "PERSON_DETECTED")
            rospy.set_param(DISINFECT_STATE_PARAM, "PERSON_DETECTED")
            rospy.set_param(LAMP_STATE, False)
            self.state = "PERSON_DETECTED"
            self.running = False

    def disinfect_object_callback(self, data):
        if self.object_detection_lock:
            self.object_detection_lock = False
            try:
                distance, angle, object_type = data.data.split(",")
                distance = round(float(distance), 2)
                angle = -int(float(angle))
                object_type = object_type.upper()
                print("disinfect object data - ", distance, angle, object_type)
                log_data("disinfect object data - " + str((distance, angle, object_type)))
                if distance != 0 and distance < 6:
                    x = round(math.sin(math.radians(abs(angle))) * distance, 2)
                    y = round(math.cos(math.radians(abs(angle))) * distance, 2)
                    robot_pov_angle = math.atan((x + ROBOT_CENTER_OFFSET_FROM_CAMERA) / y)
                    robot_pov_distance = y / math.cos(robot_pov_angle)
                    if angle:
                        angle = int(math.degrees(robot_pov_angle)) * (angle / abs(angle))
                    else:
                        angle = int(math.degrees(robot_pov_angle))
                    # print(angle, robot_pov_distance)
                    if self.move_base_active:
                        # print(self.robot_angle)
                        obstacle_angle = self.robot_angle + angle
                        if obstacle_angle > 180:
                            obstacle_angle = obstacle_angle - 360
                        if obstacle_angle < -180:
                            obstacle_angle = 360 + obstacle_angle
                        node = get_node(self.robot_node, robot_pov_distance, obstacle_angle)
                        self.object_detection_np = (np.append(self.object_detection_np[0], node[0]),
                                                    np.append(self.object_detection_np[1], node[1]))
                        self.object_detected_nodes[node] = object_type
                        if object_type not in self.object_detected:
                            self.object_detected.append(object_type)
                            rospy.set_param(DETECTED_OBJECT_PARAM, ",".join(self.object_detected))
                        y = (node[0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
                        x = (node[1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)
                        self.detected_object.append(create_marker(1, len(self.detected_object) + 1, x, y, 0, (1, 1, 0, 0)))
                        node_marker_arr = MarkerArray()
                        node_marker_arr.markers = self.detected_object
                        try:
                            self.disinfect_object_marker_pub.publish(node_marker_arr)
                        except rospy.ROSException:
                            pass
            except:
                print("Disinfect object error")
                print(traceback.format_exc())
            rospy.sleep(duration=1.0)
            self.object_detection_lock = True

    def generate_image_thread(self):
        while self.running and LIVE_IMAGE_UPDATE:
            self.generate_image()
            rospy.sleep(duration=5.0)

    def get_robot_tf(self):
        try:
            self.robot_position, self.robot_orientation = self.listener.lookupTransform(GLOBAL_FRAME, ROBOT_FRAME,
                                                                                        rospy.Time(0))

            orientation = self.robot_orientation
            orientation_list = [orientation[0], orientation[1], orientation[2], orientation[3]]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            self.robot_radian = round(yaw, 4)
            # degree = [math.degrees(x) for x in (roll, pitch, yaw)]
            self.robot_angle = int(math.degrees(yaw))
            robot_node_x = int((self.robot_position[1] / self.map_resolution) + self.map_origin[0])
            robot_node_y = int((self.robot_position[0] / self.map_resolution) + self.map_origin[0])
            self.robot_node = (robot_node_x, robot_node_y)
            if self.robot_node not in self.image_robot_trace and self.mark_trace:
                self.image_robot_trace.append(self.robot_node)
                self.image_robot_trace_poses.append(create_pose_stamp(self.robot_position[0], self.robot_position[1], 0))
                self.robot_path_np = (np.append(self.robot_path_np[0], self.robot_node[0]),
                                      np.append(self.robot_path_np[1], self.robot_node[1]))
            elif self.robot_node not in self.init_image_robot_trace:
                self.init_image_robot_trace.append(self.robot_node)
                self.init_robot_path_np = (np.append(self.init_robot_path_np[0], self.robot_node[0]),
                                           np.append(self.init_robot_path_np[1], self.robot_node[1]))
            try:
                self.pose_marker_pub.publish(create_marker(1,
                                                           50, self.robot_position[0], self.robot_position[1], 0, (0.5, 0.5, 1, 0)))
            except rospy.ROSException:
                pass
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf error")
            log_data("tf error")


    def get_path_plan(self, start, goal, only_length=False):
        print("Inside Get Plan")
        log_data("Inside Get Plan")
        start_y = (start[0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
        start_x = (start[1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)
        goal_y = (goal[0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
        goal_x = (goal[1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)
        req = GetPlan()
        req.start = create_pose_stamp(start_x, start_y, 0)
        req.goal = create_pose_stamp(goal_x, goal_y, 0)
        req.tolerance = 0.1
        path = []
        retry = 0
        while len(path) == 0 and retry < 3:
            # print("Getting path plan")
            # log_data("Getting path plan")
            try:
                resp = self.get_plan(req.start, req.goal, req.tolerance)
                # print("Got plan")
                # log_data("Got plan")
                # print("len of plan = ", len(resp.plan.poses))
                if only_length:
                    path = len(resp.plan.poses)
                    break
                path_value = Path()
                path_value.header.frame_id = "map"
                for pose in resp.plan.poses:
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    node_x = int((y / self.map_resolution) + self.map_origin[0])
                    node_y = int((x / self.map_resolution) + self.map_origin[0])
                    path.append((node_x, node_y))
                    path_value.poses.append(create_pose_stamp(x, y, 0))
                self.connect_path_pub.publish(path_value)
                # print("publishing path")
                # log_data("publishing path")
            except rospy.ROSException:
                print("Error get Plan from MoveBase")
                log_data("Error get Plan from MoveBase")
                print(traceback.format_exc())
                log_data(str(traceback.format_exc()))
            retry += 1
            # rospy.sleep(duration=1.0)
        return path

    def disinfect_map_thread(self):
        map_msg = self.map_msg
        disinfect_node_cell = int(round(DISINFECTION_COVERAGE_RADIUS / self.map_resolution))
        coverage_node_cell = int(round(PATTERN_STEP_DISTANCE / self.map_resolution))
        self.obstacle_map = np.zeros((self.map_width, self.map_height), np.uint8)
        self.ignored_covered_map = np.zeros((self.map_width, self.map_height), np.uint8)
        while self.running:
            if self.update_disinfect_map:
                try:
                    # print("Updating the Disinfected Map")
                    log_data("Updating the Disinfected Map")
                    path_value = Path()
                    path_value.header.frame_id = "map"
                    path_value.poses = self.image_robot_trace_poses
                    try:
                        self.robot_trace_pub.publish(path_value)
                    except rospy.ROSException:
                        pass
                    reachable_free_spaces, edges_free_spaces = self.get_free_space_contour()
                    free_space_np = np.where(reachable_free_spaces == 255)
                    map_data = self.filter_matrix.copy()
                    obstacle_np = np.where((map_data <= OBSTACLE_THRESHOLD) & (map_data >= ROBOT_OBSTACLE_THRESHOLD))
                    if REEMAN:
                        self.obstacle_map = dilated_area((self.map_width, self.map_height),
                                                         obstacle_np, 0, False, 0)
                    else:
                        obstacle_inflated_nodes_map = dilated_area((self.map_width, self.map_height),
                                                                   obstacle_np, 0, False, 0)
                        temp = np.logical_or(self.obstacle_map, obstacle_inflated_nodes_map)
                        self.obstacle_map = temp.astype('uint8') * 255
                    free_space_map = dilated_area((self.map_width, self.map_height),
                                                  free_space_np,
                                                  0, False, 1)
                    raw_covered_node_map = dilated_area((self.map_width, self.map_height), self.robot_path_np,
                                                        coverage_node_cell * 3 / 2, False, 0)
                    temp = np.logical_or(raw_covered_node_map, self.ignored_covered_map)
                    raw_covered_node_map = temp.astype('uint8') * 255
                    raw_covered_node_np = np.where(raw_covered_node_map == 255)
                    covered_free_np = np.where((free_space_map == raw_covered_node_map) == True)
                    covered_obstacle_np = np.where((free_space_map == self.obstacle_map) == True)
                    covered_map = np.zeros((self.map_width, self.map_height), np.uint8)
                    covered_map[covered_free_np] = 255
                    covered_map[covered_obstacle_np] = 255
                    covered_nodes_np = np.where(covered_map == 255)
                    raw_disinfected_node_map = dilated_area((self.map_width, self.map_height), self.robot_path_np,
                                                            disinfect_node_cell * 3 / 2, False, 3)
                    raw_disinfected_node_np = np.where(raw_disinfected_node_map == 255)
                    disinfected_obstacle_np = np.where((self.obstacle_map == raw_disinfected_node_map) == True)
                    disinfected_free_space_np = np.where((free_space_map == raw_disinfected_node_map) == True)
                    self.uncovered_node_np = np.where(
                        (dilated_area((self.map_width, self.map_height), covered_nodes_np,
                                      0, False, 1) == free_space_map) == False)
                    map_data = np.ones((self.map_width, self.map_height), np.uint8) * -1
                    map_data[free_space_np] = 1
                    map_data[disinfected_free_space_np] = -2
                    map_data[covered_nodes_np] = -50
                    map_data[np.where(self.obstacle_map == 255)] = OBSTACLE_THRESHOLD
                    map_data[disinfected_obstacle_np] = -20
                    self.absolute_map = map_data.copy()
                    map_msg.data = tuple(map_data.flatten())
                    try:
                        self.map_disinfected_pub.publish(map_msg)
                    except rospy.ROSException:
                        pass
                    if self.disinfect_type < 16:
                        try:
                            self.disinfected_percentage = int(
                                len(disinfected_free_space_np[0]) * 100 / len(free_space_np[0]))
                            rospy.set_param(DISINFECTION_PERCENTAGE, self.disinfected_percentage)
                        except:
                            print("Error in Disinfection Percentage Calculation")
                            log_data("Error in Disinfection Percentage Calculation")
                            print(traceback.format_exc())
                            log_data(str(traceback.format_exc()))
                            continue
                        try:
                            self.percentage = len(covered_nodes_np[0]) * 100 / len(free_space_np[0])
                            rospy.set_param(COVERAGE_PERCENTAGE, self.percentage)
                        except:
                            print("Error in Coverage Percentage Calculation")
                            log_data("Error in Coverage Percentage Calculation")
                            print(traceback.format_exc())
                            log_data(str(traceback.format_exc()))
                            continue

                    self.raw_disinfected_node_np = raw_disinfected_node_np
                    self.raw_covered_node_np = raw_covered_node_np
                    self.disinfected_obstacle_np = disinfected_obstacle_np
                    if self.disinfect_map_update_check > 0:
                        self.disinfect_map_update_check -= 1
                except:
                    print("Error in Trace Robot Pose Thread")
                    log_data("Error in Trace Robot Pose Thread")
                    print("len of x and y = ", len(self.robot_path_np[0]), len(self.robot_path_np[1]))
                    log_data("len of x and y = " + str(len(self.robot_path_np[0])) + " " +
                             str(len(self.robot_path_np[1])))
                    print(traceback.format_exc())
                    log_data(str(traceback.format_exc()))

        if not LIVE_IMAGE_UPDATE:
            self.generate_image()
        print("Robot Pose Monitor Thread exited")
        log_data("Robot Pose Monitor Thread exited")

    def inplace(self):
        self.mark_trace = True
        self.get_robot_tf()
        rospy.set_param(COVERAGE_STATE_PARAM, "RUNNING")
        self.state = "RUNNING"
        rospy.set_param(LAMP_STATE, True)
        duration = INPLACE_DURATION[0] * RATE
        if 15 < self.disinfect_type < 32:
            print("Rotation")
            log_data("Rotation")
            duration = INPLACE_DURATION[self.disinfect_type - 16] * RATE
        if 31 < self.disinfect_type < 48:
            print("Standby")
            log_data("Standby")
            duration = INPLACE_DURATION[self.disinfect_type - 32] * RATE
        percentage = 0
        percentage_pending = True
        while percentage_pending and self.running:
            if percentage <= duration:
                self.percentage = int(percentage * 100 / duration)
                rospy.set_param(COVERAGE_PERCENTAGE, self.percentage)
            else:
                percentage_pending = False
            if 15 < self.disinfect_type < 32:
                self.publish_velocity(0, math.radians(ROTATIONAL_VELOCITY))
            self.rate.sleep()
            percentage += 1
        if self.running:
            self.state = "RETURN_STARTING_POSITION"
            rospy.set_param(COVERAGE_STATE_PARAM, "RETURN_STARTING_POSITION")
            self.rotation_goal(self.starting_radian)
        rospy.set_param(LAMP_STATE, False)
        rospy.set_param("/haystack/joystick/state", "RELEASED")
        self.running = False
        self.report_generation()

    def movement(self):
        rospy.set_param(COVERAGE_STATE_PARAM, "RUNNING")
        self.state = "RUNNING"
        counter = 0
        total_count = 40
        movement_distance = 1.15
        if 47 < self.disinfect_type < 64:
            print("Fixed Movement Mode")
            log_data("Fixed Movement Mode")
            total_count = (self.disinfect_type - 47) * 10
        elif 63 < self.disinfect_type < 80:
            print("Demo Mode")
            log_data("Demo Mode")
            total_count = 40
            self.demo_mode = True
        x = math.cos(self.robot_radian) * movement_distance
        y = math.sin(self.robot_radian) * movement_distance
        init_y = self.robot_position[1] + y
        init_x = self.robot_position[0] + x
        init_node_x = int(((self.robot_position[1] + y) / self.map_resolution) + self.map_origin[0])
        init_node_y = int(((self.robot_position[0] + x) / self.map_resolution) + self.map_origin[0])
        init_node = (init_node_x, init_node_y)
        start_node = self.robot_node
        desired_angle_goal = math.atan2(init_y - self.robot_position[1],
                                        init_x - self.robot_position[0])
        if not self.demo_mode:
            rospy.set_param(LAMP_STATE, True)
            self.mark_trace = True
            self.get_robot_tf()
        toggle = 0
        while counter < total_count and self.running:
            if toggle == 0:
                goal_node = init_node
                toggle = 1
            else:
                goal_node = start_node
                toggle = 0
            if self.demo_mode:
                self.velocity_to_goal(goal_node, 0.1, ignore_obstacle=True, rotation_constriant=False,
                                      record_path=True)
            else:
                self.velocity_to_goal(goal_node, 0.1, ignore_obstacle=True, rotation_constriant=False,
                                      record_path=True)
            if self.running:
                rospy.sleep(duration=2)
            if self.demo_mode:
                rospy.set_param(LAMP_STATE, True)
                self.mark_trace = True
                self.get_robot_tf()
                self.rotate(528)
            else:
                self.rotate(178)
            if self.running:
                rospy.sleep(duration=2)
            if self.demo_mode:
                rospy.set_param(LAMP_STATE, False)
                self.mark_trace = False
            counter += 1
            self.percentage = int((counter / total_count) * 100)
            rospy.set_param(COVERAGE_PERCENTAGE, self.percentage)
            print("counter = ", counter)
            log_data("counter = " + str(counter))
        if self.running:
            self.state = "RETURN_STARTING_POSITION"
            rospy.set_param(COVERAGE_STATE_PARAM, "RETURN_STARTING_POSITION")
            self.rotation_goal(self.starting_radian)
        rospy.set_param(LAMP_STATE, False)
        self.running = False
        self.report_generation()

    def autonomous(self):
        while self.running:
            try:
                if not self.initialization_done and self.running:
                    rospy.set_param(COVERAGE_STATE_PARAM, "INITIALIZATION")
                    self.state = "INITIALIZATION"
                    if not self.initialization() and self.running:
                        os.system("rosnode kill /move_base")
                        self.got_map = False
                        self.client.wait_for_server(timeout=rospy.Duration(15))
                        rospy.sleep(duration=3.0)
                        if not self.initialization() and self.running:
                            rospy.set_param(COVERAGE_STATE_PARAM, "INITIALIZATION_ERROR")
                            rospy.set_param(DISINFECT_STATE_PARAM, "INITIALIZATION_ERROR")
                            self.state = "INITIALIZATION_ERROR"
                            print("Initialization Error")
                            log_data("Initialization Error")
                            self.running = False
                    else:
                        self.initialization_done = True
                        print("Initialization done")
                        log_data("Initialization done")
                elif self.initialization_done and not self.coverage_done and self.running:
                    self.coverage()
                elif self.coverage_done and self.running:
                    if self.running:
                        self.mark_trace = False
                        print("Return to starting position")
                        log_data("Return to starting position")
                        rospy.set_param(COVERAGE_STATE_PARAM, "RETURN_STARTING_POSITION")
                        self.state = "RETURN_STARTING_POSITION"
                        self.velocity_plan_retries = 0
                        if not self.adv_velocity_to_goal(self.starting_node, 0.1, ignore_obstacle=False):
                            print("Retrying with Astar Path planning")
                            log_data("Retrying with Astar Path planning")
                            if self.adv_velocity_to_goal(self.starting_node, 0.1, ignore_obstacle=True):
                                self.rotation_goal(self.starting_radian)
                        else:
                            self.rotation_goal(self.starting_radian)
                        self.running = False
            except:
                print("Error in Advance Intelligence Thread")
                log_data("Error in Advance Intelligence Thread")
                print(traceback.format_exc())
                log_data(str(traceback.format_exc()))
                self.running = False
        rospy.sleep(duration=0.1)
        self.running = False
        print("Exited Algo Thread")
        log_data("Exited Algo Thread")
        self.report_generation()

    def run(self):
        self.disinfect_type = rospy.get_param('/disinfect_type', 0)
        print("Disinfect Type = ", self.disinfect_type)
        log_data("Disinfect Type = " + str(self.disinfect_type))
        if self.disinfect_type > 63:
            self.demo_mode = True
        rospy.set_param(COVERAGE_STATE_PARAM, "IDLE")
        self.state = "IDLE"
        print("Idle waiting for ", IDLE_WAIT_TIME, " seconds...")
        log_data("Idle waiting for " + str(IDLE_WAIT_TIME) + " seconds...")
        idle_counter = 0
        if REEMAN:
            os.system("rosnode kill /mobile_base_nodelet_manager")
        else:
            os.system("rosnode kill /controller_spawner /motor_node")
        try:
            while idle_counter < IDLE_WAIT_TIME and self.running:
                print(str(idle_counter))
                log_data(str(idle_counter))
                rospy.sleep(duration=1.0)
                idle_counter += 1
        except rospy.ROSException:
            pass
        if not REEMAN and not DEBUG and self.running:
            if any(not value for value in self.sensor_check.values()):
                missed_devices = ""
                for key, value in self.sensor_check.items():
                    if not value:
                        print(key + " is not available")
                        missed_devices += ", " + key
                rospy.set_param(DEVICE_STATUS, "SENSOR ERROR in " + missed_devices[2:])
                rospy.set_param(LAUNCH_STATUS, False)
                print(f'Sensor Data Error in {missed_devices[2:]}')
                log_data(f'Sensor Data Error in {missed_devices[2:]}')
                rospy.set_param(COVERAGE_STATE_PARAM, "INITIALIZATION_ERROR")
                rospy.set_param(DISINFECT_STATE_PARAM, "INITIALIZATION_ERROR")
                self.state = "INITIALIZATION_ERROR"
                rospy.set_param("/haystack/mode_switcher", "DISINFECTION_ALGO")
                rospy.set_param(MODE_PARAM, "IDLE")
                return
            self.depth_image_sub.unregister()
            rospy.sleep(duration=3.0)
        if self.running:
            if not DEBUG:
                self.disinfect_navigation_uuid.start()
            print("Waiting for Move Base...")
            log_data("Waiting for Move Base...")
            self.move_base_active = self.client.wait_for_server(timeout=rospy.Duration(30))
            if (not self.move_base_active and not self.got_map) and self.state != "PERSON_DETECTED":
                print("MoveBase Not Detected...")
                log_data("MoveBase Not Detected...")
                rospy.set_param(COVERAGE_STATE_PARAM, "INITIALIZATION_ERROR")
                rospy.set_param(DISINFECT_STATE_PARAM, "INITIALIZATION_ERROR")
                self.state = "INITIALIZATION_ERROR"
                rospy.set_param("/haystack/mode_switcher", "DISINFECTION_ALGO")
                rospy.set_param(MODE_PARAM, "IDLE")
                return
            # rospy.sleep(duration=2.0)
            self.listener.waitForTransform(GLOBAL_FRAME, ROBOT_FRAME, rospy.Time(0), rospy.Duration(4))
            self.get_robot_tf()
            # self.starting_node = self.robot_node
            self.disinfect_update_obj.start()
            self.generate_Image_obj.start()
            self.cost_scaling_factor = rospy.get_param("/move_base/global_costmap/inflation_layer/cost_scaling_factor",
                                                       5)
            self.inflation_radius = rospy.get_param("/move_base/global_costmap/inflation_layer/inflation_radius", 0.60)
            self.inscribed_radius = rospy.get_param("/move_base/global_costmap/robot_radius", 0.25)
            print(self.cost_scaling_factor, self.inscribed_radius, self.inflation_radius, self.map_resolution)
            for distance in range(int(round((self.inflation_radius * 100) / (self.map_resolution * 100))),
                                  int(round((self.inscribed_radius * 100) / (self.map_resolution * 100))) - 1, -1):
                print(distance * self.map_resolution)
                cost = int(round(math.exp(-1.0 * self.cost_scaling_factor * ((distance * self.map_resolution) -
                                                                             self.inscribed_radius)) * (
                                         INSCRIBED_INFLATED_OBSTACLE - 1)))
                self.inflation_cost_values.append(cost)
            self.inflation_cost_values = self.inflation_cost_values[::-1]
            print("Total Cost = ", self.inflation_cost_values)
            log_data("Total Cost = " + str(self.inflation_cost_values))
            self.rotation_safety_threshold = int(((ROBOT_ROTATION_RADIUS * 100) - (self.inscribed_radius * 100)) /
                                                 (self.map_resolution * 100)) - 1
            self.path_threshold = int(((ROBOT_LINEAR_RADIUS * 100) - (self.inscribed_radius * 100)) /
                                      (self.map_resolution * 100)) - 1
            self.pattern_threshold = int(((ROBOT_PATTERN_RADIUS * 100) - (self.inscribed_radius * 100)) /
                                         (self.map_resolution * 100)) - 1
            print("Rotation Cost = ", self.inflation_cost_values[self.rotation_safety_threshold])
            print("Path Cost = ", self.inflation_cost_values[self.path_threshold])
            print("Pattern Cost = ", self.inflation_cost_values[self.pattern_threshold])
            log_data("Rotation Cost = " + str(self.inflation_cost_values[self.rotation_safety_threshold]))
            log_data("Path Cost = " + str(self.inflation_cost_values[self.path_threshold]))
            log_data("Pattern Cost = " + str(self.inflation_cost_values[self.pattern_threshold]))
        else:
            self.running = False
            return
        print(f"Starting node cost = {self.filter_matrix[self.robot_node[0]][self.robot_node[1]]}")
        log_data(f"Starting node cost = {self.filter_matrix[self.robot_node[0]][self.robot_node[1]]}")
        # self.starting_position = (self.robot_position[0], self.robot_position[1])
        robot_node_x = int((self.starting_position[1] / self.map_resolution) + self.map_origin[0])
        robot_node_y = int((self.starting_position[0] / self.map_resolution) + self.map_origin[0])
        self.starting_node = (robot_node_x, robot_node_y)
        # self.starting_radian = self.robot_radian
        print(f"Starting position {self.starting_node}, {self.starting_position}, {self.starting_radian}")
        log_data(f"Starting position {self.starting_node}, {self.starting_position}, {self.starting_radian}")
        while self.absolute_map is None and self.running:
            print("Waiting for absolute map")
            log_data("Waiting for absolute map")
            rospy.sleep(duration=1.0)

        if self.disinfect_type < 16:
            self.autonomous()
        elif 15 < self.disinfect_type < 48:
            self.inplace()
        elif 47 < self.disinfect_type < 80:
            self.movement()

    def robot_rotation_check(self, node):
        print("Checking Rotation Safety")
        if not self.running:
            return 0
        if not ROTATION_SAFETY_CHECK:
            return 0
        rotation_area = dilated_area((self.map_width, self.map_height),
                                     node, ROBOT_ROTATION_CHECK * 2, False, 0)
        rotation_area_np = np.where(rotation_area == 255)
        safe_check = np.where(self.filter_matrix[rotation_area_np] >
                              self.inflation_cost_values[self.rotation_safety_threshold])
        if len(safe_check[0]) == 0:
            if self.absolute_map is not None:
                second_safe_check = np.where((self.absolute_map[rotation_area_np] == -20) &
                                             (self.absolute_map[rotation_area_np] == 100))
                if len(second_safe_check[0]) == 0:
                    # print("Safe to rotate in absolute map")
                    # log_data("Safe to rotate in absolute map")
                    if node not in self.rotatable_path_points:
                        self.rotatable_path_points.append(node)
                    return 0
                else:
                    print("Not Safe to rotate in absolute map")
                    log_data("Not Safe to rotate in absolute map")
                    return 1
            else:
                # print("Safe to rotate")
                # log_data("Safe to rotate")
                if node not in self.rotatable_path_points:
                    self.rotatable_path_points.append(node)
                return 0
        else:
            print("Not safe to rotate!!")
            log_data("Not safe to rotate!!")
            return 1

    def rotate_inplace(self):
        print("Inplace rotation started")
        log_data("Inplace rotation started")
        self.get_robot_tf()
        start_z_position = self.robot_angle
        if start_z_position < 0:
            start_z_position = 360 + start_z_position
        # print("starting angle = ", start_z_position)
        # log_data("starting angle = " + str(start_z_position))
        current_degree = 0
        while current_degree <= ROTATION_DEGREE and self.running:
            velocity = Twist()
            velocity.linear.x = 0.0
            velocity.angular.z = math.radians(ROTATIONAL_VELOCITY)
            try:
                self.velocity_pub.publish(velocity)
                self.rate.sleep()
            except rospy.ROSException:
                pass
            self.get_robot_tf()
            current_z_position = self.robot_angle
            if current_z_position < 0:
                current_z_position = 360 + current_z_position
            diff = abs(int(current_z_position) - int(start_z_position))
            if diff > 300:
                diff = 360 - diff
            start_z_position = current_z_position
            current_degree += int(diff)
            log_data(str(int(current_degree)) + "Degree completed")
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        try:
            self.velocity_pub.publish(velocity)
            self.rate.sleep()
        except rospy.ROSException:
            pass

    def initialization(self):
        print("Inside Initialization")
        log_data("Inside Initialization")
        initialization_retry = INITIALIZATION_RETRY
        initialization_status = False
        while initialization_retry > 0 and not initialization_status and self.running:
            self.get_robot_tf()
            x = math.cos(self.robot_radian) * INITIALIZATION_DISTANCE
            y = math.sin(self.robot_radian) * INITIALIZATION_DISTANCE
            # print(self.robot_position)
            # print(self.robot_position[0] + x, self.robot_position[1] + y)
            init_node_x = int(((self.robot_position[1] + y) / self.map_resolution) + self.map_origin[1])
            init_node_y = int(((self.robot_position[0] + x) / self.map_resolution) + self.map_origin[0])
            init_node = (init_node_x, init_node_y)
            try:
                self.pose_marker_pub.publish(create_marker(1, 0, self.robot_position[0] + x,
                                                           self.robot_position[1] + y, 0, (1, 0.5, 0, 0)))
            except rospy.ROSException:
                pass
            if (not FREE_SPACE_THRESHOLD <= self.filter_matrix[init_node[0]][init_node[1]] <
                    self.inflation_cost_values[self.rotation_safety_threshold] or
                    self.filter_matrix[init_node[0]][init_node[1]] == UNIDENTIFIED_THRESHOLD):
                init_node = self.get_nearby_low_region(init_node, (COVERAGE_FREE_SPACE_DILATION * 2),
                                                       False, False)
            if (FREE_SPACE_THRESHOLD <= self.filter_matrix[init_node[0]][init_node[1]] <
                    self.inflation_cost_values[self.rotation_safety_threshold]):
                rotation_check = self.robot_rotation_check(init_node)
                if rotation_check == 0:
                    y = (init_node[0] - self.map_origin[0]) * self.map_resolution + \
                        (self.map_resolution / 2)
                    x = (init_node[1] - self.map_origin[1]) * self.map_resolution + \
                        (self.map_resolution / 2)
                    desired_angle_goal = math.atan2(y - self.robot_position[1],
                                                    x - self.robot_position[0])
                    if self.velocity_to_goal(init_node, 0.15, ignore_obstacle=False,
                                             rotation_constriant=True, record_path=True):
                        self.rotation_goal(desired_angle_goal)
                        self.rotate_inplace()
                        initialization_status = True
                    else:
                        print("Cant reach the goal")
                        log_data("Cant reach the goal")
                        initialization_retry -= 1
                else:
                    print("Not safe to rotate")
                    log_data("Not safe to rotate")
                    initialization_retry -= 1
            else:
                print("No Free Space to Move")
                log_data("No Free Space to Move")
                initialization_retry -= 1
                print("Initialization Retry = ", initialization_retry)
                log_data("Initialization Retry = " + str(initialization_retry))
            rospy.sleep(duration=1.0)
        return initialization_status

    def get_free_space_contour(self):
        start_update = rospy.Time.now().to_sec()
        initial_free_space = np.where((self.filter_matrix < ROBOT_OBSTACLE_THRESHOLD) &
                                      (self.filter_matrix >= FREE_SPACE_THRESHOLD))
        # print("getting free space - ", rospy.Time.now().to_sec() - start_update)
        connected_components, edged_connected_components = get_contours((self.map_width, self.map_height),
                                                                        initial_free_space)
        # print("got countour - ", rospy.Time.now().to_sec() - start_update)
        # print("len of free spaces = ", len(initial_free_space[0]))
        # log_data("len of free spaces = " + str(len(initial_free_space[0])))
        # print("len of edge free spaces = ", len(edged_connected_components))
        # log_data("len of edge free spaces = " + str(len(edged_connected_components)))
        # print("len of connected components = ", len(connected_components))
        # log_data("len of connected components = " + str(len(connected_components)))
        #
        # print("Starting position = ", self.robot_node)
        # log_data("Starting position = " + str(self.robot_node))
        reachable_free_spaces = np.zeros((self.map_width, self.map_height), np.uint8)
        edges_free_spaces = np.zeros((self.map_width, self.map_height), np.uint8)
        for i in range(0, len(connected_components)):
            if connected_components[i][self.robot_node[0]][self.robot_node[1]] == 255:
                temp = np.logical_or(reachable_free_spaces, connected_components[i])
                reachable_free_spaces = temp.astype('uint8') * 255
                temp = np.logical_or(edges_free_spaces, connected_components[i])
                edges_free_spaces = temp.astype('uint8') * 255

                # print("robot node available in contour = ", i)
                break
            else:
                # print("robot node not available in contour = ", i)
                pass
        for i in range(0, len(connected_components)):
            if connected_components[i][self.starting_node[0]][self.starting_node[1]] == 255:
                temp = np.logical_or(reachable_free_spaces, connected_components[i])
                reachable_free_spaces = temp.astype('uint8') * 255
                temp = np.logical_or(edges_free_spaces, connected_components[i])
                edges_free_spaces = temp.astype('uint8') * 255
                # print("starting node available in contour = ", i)
                break
            else:
                # print("starting node not available in contour = ", i)
                pass
        return reachable_free_spaces, edges_free_spaces

    def coverage(self):
        print("Inside Coverage")
        log_data("Inside Coverage")
        self.mark_trace = True
        self.get_robot_tf()
        rospy.set_param(COVERAGE_STATE_PARAM, "RUNNING")
        rospy.set_param(LAMP_STATE, True)
        self.state = "RUNNING"
        obstacle_nodes = np.where(self.filter_matrix == OBSTACLE_THRESHOLD)
        aligned_direction = line_detection((self.map_width, self.map_height), obstacle_nodes)
        if MANUAL:
            while self.running and MANUAL:
                rospy.sleep(duration=1.0)
        else:
            self.path_planning(aligned_direction)
        self.coverage_done = True

    def path_planning(self, aligned_direction):
        reach_starting_position = False
        aligned_direction = int(aligned_direction)
        direction_value = [aligned_direction]
        if aligned_direction - 90 > -180:
            direction_value.append(aligned_direction - 90)
        else:
            direction_value.append(aligned_direction - 90 + 360)
        if aligned_direction + 90 < 180:
            direction_value.append(aligned_direction + 90)
        else:
            direction_value.append(aligned_direction + 90 - 360)
        if aligned_direction <= 0:
            direction_value.append(aligned_direction + 180)
        else:
            direction_value.append(aligned_direction - 180)

        direction_value = sorted(direction_value)
        direction_value = direction_value[::-1]
        dynamic_direction_value = direction_value.copy()
        for direction in direction_value:
            if abs(direction - self.robot_angle) < 45:
                print("robot facing direction = ", direction)
                break
            else:
                dynamic_direction_value.remove(direction)
                dynamic_direction_value.append(direction)
        print("Direction values = ", direction_value)
        print("Direction values = " + str(direction_value))
        current_node = self.robot_node
        y = (current_node[0] - self.map_origin[0]) * self.map_resolution + \
            (self.map_resolution / 2)
        x = (current_node[1] - self.map_origin[1]) * self.map_resolution + \
            (self.map_resolution / 2)
        try:
            self.pose_marker_pub.publish(create_marker(1, 0, x, y, 0, (1, 0.5, 0, 0)))
        except rospy.ROSException:
            pass
        disinfect_node_cell = int(round(DISINFECTION_COVERAGE_RADIUS / self.map_resolution))
        running = True
        ret = True
        disinfect_wait_time_multiplier = rospy.get_param("/haystack/disinfect_wait_time_multiplier",
                                                         DISINFECTION_WAIT_TIME_MULTIPLIER)
        wait_time = int(disinfect_wait_time_multiplier) * self.disinfect_type

        while running and self.running:
            print("Checking.....")
            log_data("Checking.....")
            skip = False
            distance = 0
            if self.last_disinfected_point is not None:
                distance = round(math.sqrt(
                    (self.robot_position[0] - self.last_disinfected_point[0]) * (
                            self.robot_position[0] - self.last_disinfected_point[0]) +
                    (self.robot_position[1] - self.last_disinfected_point[1]) * (
                            self.robot_position[1] - self.last_disinfected_point[1])), 2)
                print("Distance from last disinfected point = ", distance)
            if SMART_DISINFECTION:
                current_disinfected_map = dilated_area((self.map_width, self.map_height),
                                                       (np.array([self.robot_node[0]]),
                                                        np.array([self.robot_node[1]])),
                                                       disinfect_node_cell * 2, False, 0)
                current_disinfected_map[self.disinfected_object_detection_np] = 0
                object_map = dilated_area((self.map_width, self.map_height), self.object_detection_np,
                                          1, False, 1)
                disinfected_object_map = dilated_area((self.map_width, self.map_height),
                                                      self.disinfected_object_detection_np,
                                                      1, False, 0)
                disinfected_object_map[np.where(current_disinfected_map == 255)] = 255
                matched_points = np.where((current_disinfected_map == object_map) == True)
                if len(matched_points[0]) > 0:
                    coordinates = list(zip(matched_points[0], matched_points[1]))
                    objects = np.array([self.object_detected_nodes[key] for key in coordinates
                                        if key in self.object_detected_nodes])
                    if len(objects) != 0:
                        unique_objects, counts = np.unique(objects, return_counts=True)
                        major_object = unique_objects[np.argmax(counts)]
                        if distance > SMART_DISINFECT_DISTANCE or self.last_disinfected_point is None:
                            self.last_disinfected_point = (self.robot_position[0], self.robot_position[1])
                            print("Object Detected disinfecting")
                            log_data("Object Detected disinfecting")
                            rotation_check = self.robot_rotation_check(self.robot_node)
                            skip = True
                            if rotation_check == 0:
                                rospy.set_param(COVERAGE_STATE_PARAM, f'ROTATING NEAR {major_object}')
                                self.rotate_inplace()
                                rospy.set_param(COVERAGE_STATE_PARAM, "RUNNING")
                            else:
                                print("Cant Rotate standing in same place")
                                log_data("Cant Rotate standing in same place")
                                counter = 0
                                rospy.set_param(COVERAGE_STATE_PARAM, f'STANDING NEAR {major_object}')
                                while counter < DISINFECT_OBJECT_WAIT_TIME and self.running:
                                    print(str(counter))
                                    log_data(str(counter))
                                    rospy.sleep(duration=1.0)
                                    counter += 1
                                rospy.set_param(COVERAGE_STATE_PARAM, "RUNNING")
                        self.disinfected_object_detection_np = np.where(disinfected_object_map == 255)

            if ret and not skip and DISINFECTION_WAIT_FLAG:
                counter = 0
                if distance > SMART_DISINFECT_DISTANCE or self.last_disinfected_point is None:
                    self.last_disinfected_point = (self.robot_position[0], self.robot_position[1])
                    print("Waiting for disinfection..", wait_time)
                    log_data("Waiting for disinfection.." + str(wait_time))
                    rospy.set_param(COVERAGE_STATE_PARAM, "DISINFECTING THE SURROUNDINGS")
                    while counter < wait_time and self.running:
                        print(str(counter))
                        log_data(str(counter))
                        rospy.sleep(duration=1.0)
                        counter += 1
                    rospy.set_param(COVERAGE_STATE_PARAM, "RUNNING")
            free_node_direction_check = 0
            current_direction_value = dynamic_direction_value.copy()
            for direction in current_direction_value:
                direction_tolerance = []
                done = 0
                for i in [0, -15, 15, -30, 30, -45, 45]:
                    temp = direction + i
                    if temp > 180:
                        temp = temp - 360
                    if temp < -180:
                        temp = 360 + 180
                    direction_tolerance.append(temp)
                for tolerance in direction_tolerance:
                    temp = [get_node(current_node, PATTERN_STEP_DISTANCE / 4, tolerance),
                            get_node(current_node, PATTERN_STEP_DISTANCE / 2, tolerance),
                            get_node(current_node, PATTERN_STEP_DISTANCE * 3 / 4, tolerance),
                            get_node(current_node, PATTERN_STEP_DISTANCE, tolerance)]
                    inbetween_nodes = [temp[0]]
                    for nodes in temp[1:]:
                        if (FREE_SPACE_THRESHOLD <= self.filter_matrix[nodes[0]][nodes[1]] <=
                            self.inflation_cost_values[self.pattern_threshold]) and \
                                self.absolute_map[nodes[0]][nodes[1]] != -20:
                            inbetween_nodes.append(nodes)
                        else:
                            break
                    while len(inbetween_nodes) > 2 and self.running:
                        next_node = inbetween_nodes.pop()
                        if self.absolute_map[next_node[0]][next_node[1]] != -50:

                            free_node_direction_check = 1
                            y = (current_node[0] - self.map_origin[0]) * self.map_resolution + \
                                (self.map_resolution / 2)
                            x = (current_node[1] - self.map_origin[1]) * self.map_resolution + \
                                (self.map_resolution / 2)
                            try:
                                self.pose_marker_pub.publish(create_marker(1, 0, x, y, 0, (1, 0.5, 0, 0)))
                            except rospy.ROSException:
                                pass
                            current_node = next_node
                            done = 1
                            break
                    if done:
                        break
                if done:
                    break
                dynamic_direction_value.remove(direction)
                dynamic_direction_value.append(direction)
            if free_node_direction_check and self.running:
                print("Found Direction Goal")
                if self.send_goal_enable:
                    self.get_robot_tf()
                    goal_y = (current_node[0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
                    goal_x = (current_node[1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)
                    current_distance = abs(math.sqrt(((goal_x - self.robot_position[0]) ** 2) +
                                                     ((goal_y - self.robot_position[1]) ** 2)))
                    if ((FREE_SPACE_THRESHOLD <= self.filter_matrix[current_node[0]][current_node[1]] <=
                         self.inflation_cost_values[self.rotation_safety_threshold]) and
                            (FREE_SPACE_THRESHOLD <= self.filter_matrix[self.robot_node[0]][self.robot_node[1]] <=
                             self.inflation_cost_values[self.rotation_safety_threshold]) and
                            current_distance < 1):
                        ret = self.send_path_goal(current_node, COVERAGE_FREE_SPACE_DILATION, True,
                                                  max_velocity=COVERAGE_MAX_VELOCITY)
                    else:
                        ret = self.send_path_goal(current_node, COVERAGE_FREE_SPACE_DILATION, False,
                                                  max_velocity=COVERAGE_MAX_VELOCITY)

            elif (free_node_direction_check == 0 and self.running and
                  self.percentage < COVERAGE_CONNECTED_COMPONENT_THRESHOLD):
                print("Finding uncovered area")
                log_data("Finding uncovered area")
                self.get_robot_tf()
                self.disinfect_map_update_check = 1
                while self.disinfect_map_update_check != 0:
                    self.rate.sleep()
                print("len of uncovered area = " + str(len(self.uncovered_node_np[0])))
                log_data("len of uncovered area = " + str(len(self.uncovered_node_np[0])))
                if not len(self.uncovered_node_np[0]) == 0:
                    connected_components, edged_connected_components = get_contours(
                        (self.map_width, self.map_height), self.uncovered_node_np)
                    print("Len of connected component = " + str(len(connected_components)))
                    log_data("Len of connected component = " + str(len(connected_components)))
                    large_area = []
                    count = 1
                    for i in range(0, len(connected_components)):
                        nodes = np.where(connected_components[i] == 255)
                        center_node = (nodes[0][int(len(nodes[0]) / 2)], nodes[1][int(len(nodes[0]) / 2)])
                        if len(nodes[0]) < MINIMUM_COVERAGE_NODES:
                            print("Len of sub connected component = " + str(len(nodes[0])))
                            log_data("Len of sub connected component = " + str(len(nodes[0])))
                            temp = np.logical_or(connected_components[i], self.ignored_covered_map)
                            self.ignored_covered_map = temp.astype('uint8') * 255
                        else:
                            print("Len of sub connected component = " + str(len(nodes[0])))
                            log_data("Len of sub connected component = " + str(len(nodes[0])))
                            nearby_low_cost_region = self.get_nearby_low_region(center_node,
                                                                                (3 * 3),
                                                                                True, True)
                            len_of_path = self.get_path_plan(self.robot_node, nearby_low_cost_region, only_length=True)
                            y = (nearby_low_cost_region[0] - self.map_origin[0]) * self.map_resolution + \
                                (self.map_resolution / 2)
                            x = (nearby_low_cost_region[1] - self.map_origin[1]) * self.map_resolution + \
                                (self.map_resolution / 2)
                            try:
                                self.pose_marker_pub.publish(create_marker(1, count, x, y, 0, (1, 0.5, 0, 0)))
                                count += 1
                            except rospy.ROSException:
                                pass
                            # print("len of plan = " + str(len_of_path))
                            # log_data("len of plan = " + str(len_of_path))
                            if len_of_path > 0:
                                target_distance = len_of_path
                                print("len of path = " + str(target_distance))
                                log_data("len of path = " + str(target_distance))
                                large_area.append((edged_connected_components[i], connected_components[i],
                                                   target_distance))
                            else:
                                print("No plan available for the area")
                                log_data("No plan available for the area")

                    n = len(large_area)
                    for i in range(n):
                        for j in range(0, n - i - 1):
                            if large_area[j][2] > large_area[j + 1][2]:
                                large_area[j], large_area[j + 1] = large_area[j + 1], large_area[j]

                    for edge_map, area_map, distance in large_area:
                        edge = np.where(edge_map == 255)
                        print("Len of sub edge connected component = " + str(len(edge[0])))
                        log_data("Len of sub edge connected component = " + str(len(edge[0])))
                        min_distance_coverage_goal = None
                        min_distance = None
                        for i in range(0, len(edge[0]), math.ceil(len(edge[0]) / MINIMUM_COVERAGE_NODES)):
                            goal = (edge[0][i], edge[1][i])
                            distance = round(math.sqrt(
                                (self.robot_node[0] - goal[0]) * (self.robot_node[0] - goal[0]) +
                                (self.robot_node[1] - goal[1]) * (self.robot_node[1] - goal[1])), 2)
                            if min_distance is None:
                                min_distance = distance
                                min_distance_coverage_goal = goal
                            elif min_distance > distance:
                                min_distance = distance
                                min_distance_coverage_goal = goal
                        if self.send_goal_enable:
                            if self.send_path_goal(min_distance_coverage_goal, COVERAGE_FREE_SPACE_DILATION,
                                                   False, max_velocity=COVERAGE_MAX_VELOCITY):
                                ret = True
                                current_node = min_distance_coverage_goal
                            else:
                                ret = False
                                current_node = self.robot_node
                            break

                    if len(large_area) == 0:
                        print("No Large Area found")
                        log_data("No Large Area found")
                        if self.percentage < 95:
                            print("Trying to go to starting position to check possibility")
                            log_data("Trying to go to starting position to check possibility")
                            if not self.adv_velocity_to_goal(self.starting_node, 0.08, ignore_obstacle=False):
                                print("Retrying with Astar Path planning")
                                log_data("Retrying with Astar Path planning")
                                ret = self.adv_velocity_to_goal(self.starting_node, 0.08, ignore_obstacle=True)

                                if not ret:
                                    print("No Plan Available for the blocked area")
                                    log_data("No Plan Available for the blocked area")
                                    running = False
                            else:
                                ret = True
                                current_node = self.robot_node
                            if reach_starting_position:
                                break
                            else:
                                reach_starting_position = True
                            # self.move_to_rotatable_place()
                            # self.clear_costmap()
                            # rospy.sleep(duration=1.0)
                        else:
                            running = False

                else:
                    print("All area covered")
                    log_data("All area covered")
                    running = False
            elif free_node_direction_check == 0 and self.running:
                running = False
            print("Coverage Percentage = ", self.percentage)
            if self.percentage >= COVERAGE_THRESHOLD:
                running = False

    def send_path_goal(self, goal, dilation, linear_velocity_method, max_velocity=0.05):
        # goal = self.get_nearby_low_region(goal, (dilation * 2), True, True)
        # y = (goal[0] - self.map_origin[0]) * self.map_resolution + \
        #     (self.map_resolution / 2)
        # x = (goal[1] - self.map_origin[1]) * self.map_resolution + \
        #     (self.map_resolution / 2)
        # try:
        #     self.pose_marker_pub.publish(create_marker(1, 35, x, y, 0, (1, 0.5, 1, 0)))
        # except:
        #     pass
        self.velocity_plan_retries = 0
        return self.adv_velocity_to_goal(goal, max_velocity, ignore_obstacle=False)
        # if not linear_velocity_method:
        #     self.velocity_plan_retries = 0
        #     return self.adv_velocity_to_goal(goal, max_velocity, ignore_obstacle=False, stop_if_covered=True)
        # else:
        #     if not self.velocity_to_goal(goal, max_velocity, ignore_obstacle=False, rotation_constriant=True):
        #         self.velocity_plan_retries = 0
        #         return self.adv_velocity_to_goal(goal, max_velocity, ignore_obstacle=False, stop_if_covered=True)
        #     return True

    def get_nearby_low_region(self, node, layer, full_check, check_uncovered):
        print("Finding nearby low cost node")
        log_data("Finding nearby low cost node")
        try:
            if not full_check:
                if (FREE_SPACE_THRESHOLD <= self.filter_matrix[node[0]][node[1]] <=
                        self.inflation_cost_values[self.rotation_safety_threshold]):
                    return node
            dilated_map = dilated_area((self.map_width, self.map_height),
                                       (np.array([node[0]]),
                                        np.array([node[1]])),
                                       layer, False, 0)
            x, y = np.where(dilated_map == 255)
            nearby_free_space = list(zip(x, y))
            nearby_free_space = sorted([*set(nearby_free_space)])
            min_value = OBSTACLE_THRESHOLD
            min_point = None
            for i in nearby_free_space:
                if FREE_SPACE_THRESHOLD <= self.filter_matrix[i[0]][i[1]] < min_value:
                    if check_uncovered:
                        if self.absolute_map[i[0]][i[1]] != -50:
                            min_value = self.filter_matrix[i[0]][i[1]]
                            min_point = i
                    else:
                        min_value = self.filter_matrix[i[0]][i[1]]
                        min_point = i
            if min_point is not None:
                return min_point
            else:
                return node
        except:
            print(traceback.format_exc())
            log_data(str(traceback.format_exc()))
            return node

    def publish_velocity(self, linear, angular):
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        try:
            self.velocity_pub.publish(velocity)
        except rospy.ROSException:
            pass

    def rotate(self, target_degree):
        self.get_robot_tf()
        start_z_position = self.robot_angle
        if start_z_position < 0:
            start_z_position = 360 + start_z_position
        current_degree = 0
        while current_degree <= target_degree and self.running:
            self.get_robot_tf()
            if self.robot_safe:
                self.publish_velocity(0, math.radians(ROTATIONAL_VELOCITY))
                self.rate.sleep()
                current_z_position = self.robot_angle
                if current_z_position < 0:
                    current_z_position = 360 + current_z_position
                diff = abs(int(current_z_position) - int(start_z_position))
                if diff > 300:
                    diff = 360 - diff
                start_z_position = current_z_position
                current_degree += int(diff)
                print(int(current_degree), "Degree completed")
            else:
                print("robot_unsafe")
                self.rate.sleep()
        self.publish_velocity(0, 0)

    def rotation_goal(self, target_angle):
        print("Inside Rotation Goal")
        log_data("Inside Rotation Goal")
        self.get_robot_tf()
        rotation_check = self.robot_rotation_check(self.robot_node)
        if rotation_check != 0:
            if self.state == "RETURN_STARTING_POSITION":
                distance = round(math.sqrt(
                    (self.robot_node[0] - self.starting_node[0]) * (self.robot_node[0] - self.starting_node[0]) +
                    (self.robot_node[1] - self.starting_node[1]) * (self.robot_node[1] - self.starting_node[1])), 2)
                if distance > 5:
                    return
        while self.running:
            self.get_robot_tf()
            angular = target_angle - self.robot_radian
            if angular < -3.14:
                angular = 6.28 + angular
            elif angular > 3.14:
                angular = angular - 6.28
            if abs(angular) > 0.261799:
                angular = angular / abs(angular) * 0.261799
            if abs(angular) > 0.0872665:
                self.publish_velocity(0, angular)
                self.rate.sleep()
            else:
                self.publish_velocity(0, 0)
                print("Rotation Completed")
                log_data("Rotation Completed")
                break
        self.publish_velocity(0, 0)

    def find_neighbor(self, node, closed):
        # generate neighbors in certain condition
        neighbor: list = []
        # generate neighbors in certain condition
        try:
            current_node_map = dilated_area((self.map_width, self.map_height),
                                            (np.array([node.coordinate[0]]), np.array([node.coordinate[1]])),
                                            3, False, 0)
            travelled_path_np = (np.append(self.robot_path_np[0], self.init_robot_path_np[0]),
                                 np.append(self.robot_path_np[1], self.init_robot_path_np[1]))
            travelled_path_map = dilated_area((self.map_width, self.map_height),travelled_path_np, 2, False, 1)
            nearby_node_np = np.where((current_node_map == travelled_path_map) == True)
            neighbor = [(nearby_node_np[0][x], nearby_node_np[1][x])
                        for x in range(len(nearby_node_np[0]))]
            if node.coordinate in neighbor:
                neighbor.remove(node.coordinate)
            neighbor = [x for x in neighbor if x not in closed]
        except:
            log_data(traceback.format_exc())

        return neighbor


    def move_to_rotatable_place(self):
        ret = False
        print("Inside Retracing Path")
        log_data("Inside Retracing Path")
        if len(self.robot_travelled_path) > 14:
            path = self.robot_travelled_path[-1:-14:-1]
            self.robot_travelled_path = self.robot_travelled_path[:-14]
        else:
            path = self.robot_travelled_path
            self.robot_travelled_path = []
        for n in range(0, len(path)):
            self.get_robot_tf()
            y = (path[n][0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
            x = (path[n][1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)

            distance = abs(math.sqrt(((x - self.robot_position[0]) ** 2) +
                                     ((y - self.robot_position[1]) ** 2)))
            if distance > 0.3 or n == len(path) - 1:
                rotation_check = self.robot_rotation_check(self.robot_node)
                goal_node = path[n]
                if rotation_check != 0:
                    self.velocity_to_goal(goal_node, 0.1, ignore_obstacle=True, rotation_constriant=False,
                                          record_path=False)
                else:
                    print("Reached Safe Point")
                    log_data("Reached Safe Point")
                    ret = True
                    break
        return ret

    def velocity_to_goal(self, goal_node, max_velocity, ignore_obstacle, rotation_constriant, record_path=True):
        print("Inside velocity")
        log_data("Inside velocity")
        wait = True
        if self.absolute_map is not None:
            if self.absolute_map[goal_node[0]][goal_node[1]] == -50:
                max_velocity = 0.1
                wait = False
        reverse = False
        goal_y = (goal_node[0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
        goal_x = (goal_node[1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)
        try:
            self.pose_marker_pub.publish(create_marker(1,
                                                       0, goal_x, goal_y, 0, (1, 0.5, 0, 0)))
        except rospy.ROSException:
            pass
        inc_x = goal_x - self.robot_position[0]
        inc_y = goal_y - self.robot_position[1]
        angle_to_goal = round(math.atan2(inc_y, inc_x), 4)
        angular = angle_to_goal - self.robot_radian
        linear = 0.01
        rotation_check = self.robot_rotation_check(self.robot_node)
        if rotation_check != 0:
            goal_node = self.get_nearby_low_region(goal_node, (2 * 2), True, False)
            # print("goal angle = " + str(angle_to_goal))
            # log_data("goal angle = " + str(angle_to_goal))
            # print("robot angle = " + str(self.robot_radian))
            # log_data("robot angle = " + str(self.robot_radian))
            # print("angular difference = " + str(angular))
            # log_data("angular difference = " + str(angular))
            if self.robot_angle <= 0:
                opp_robot_degree = 180 + self.robot_angle
            elif self.robot_angle > 0:
                opp_robot_degree = self.robot_angle - 180
            opp_robot_radian = round(math.radians(opp_robot_degree), 4)
            # if ((3.927 >= abs(angular) >= 3.1416 or 3.1416 >= abs(angular) >= 2.3562) and
            #         round(abs(abs(angle_to_goal) - abs(opp_robot_radian)), 4) <= 0.7854):
            # if ((4.1888 >= abs(angular) >= 3.1416 or 3.1416 >= abs(angular) >= 2.0944) and
            #         round(abs(abs(angle_to_goal) - abs(opp_robot_radian)), 4) <= 1.0472):
            #     print("Enabling reverse velocity")
            #     log_data("Enabling reverse velocity")
            #     reverse = True
            # elif ((6.2832 > abs(angular) >= 5.4978 or 0 <= abs(angular) <= 0.7854) and
            #       round(abs(abs(angle_to_goal) - abs(self.robot_radian)), 4) <= 0.7854):
            if ((6.2832 > abs(angular) >= 5.236 or 0 <= abs(angular) <= 1.0472) and
                    round(abs(abs(angle_to_goal) - abs(self.robot_radian)), 4) <= 1.0472):
                print("Facing Same Direction")
                log_data("Facing Same Direction")
            else:
                if not rotation_constriant:
                    print("Can't rotate but Proceeding..")
                    log_data("Can't rotate but Proceeding..")
                    if ((4.7124 >= abs(angular) >= 3.1416 or 3.1416 >= abs(angular) >= 1.5708) and
                            round(abs(abs(angle_to_goal) - abs(opp_robot_radian)), 4) <= 1.5708):
                        reverse = True
                else:
                    print("Can't rotate re-routing..")
                    log_data("Can't rotate re-routing..")
                    self.move_to_rotatable_place()
                    return False
        self.update_disinfect_map = True
        while self.running and not self.cancel_step_goal:
            self.get_robot_tf()
            if self.demo_mode or self.disinfect_type > 47:
                if not self.robot_safe:
                    print("Obstacle detected")
                    log_data("Obstacle detected")
                    # if self.disinfect_type > 47:
                    #     self.obstacle_detected_count += 1
                    # if self.obstacle_detected_count > 100:
                    #     self.running = False
                    self.rate.sleep()
                    continue
            self.obstacle_detected_count = 0
            if record_path:
                if len(self.robot_travelled_path) == 0:
                    self.robot_travelled_path.append((self.robot_node[0], self.robot_node[1]))
                elif self.robot_travelled_path[-1] != (self.robot_node[0], self.robot_node[1]):
                    self.robot_travelled_path.append((self.robot_node[0], self.robot_node[1]))
            if not (FREE_SPACE_THRESHOLD <= self.filter_matrix[goal_node[0]][goal_node[1]] <
                    ROBOT_OBSTACLE_THRESHOLD) and not ignore_obstacle:
                print("goal inside obstacle")
                log_data("goal inside obstacle")
                self.publish_velocity(0, 0)
                break
            if not reverse:
                inc_x = goal_x - self.robot_position[0]
                inc_y = goal_y - self.robot_position[1]

            else:
                inc_x = self.robot_position[0] - goal_x
                inc_y = self.robot_position[1] - goal_y

            angle_to_goal = round(math.atan2(inc_y, inc_x), 4)
            angular = round(angle_to_goal - self.robot_radian, 4)
            current_distance = abs(math.sqrt(((goal_x - self.robot_position[0]) ** 2) +
                                             ((goal_y - self.robot_position[1]) ** 2)))
            max_linear = current_distance / 2
            if max_linear > max_velocity:
                max_linear = max_velocity
            if angular < -3.14:
                angular = 6.28 + angular
            elif angular > 3.14:
                angular = angular - 6.28
            if abs(angular) > 0.349066:  # 0.261799
                angular = angular / abs(angular) * 0.349066  # 0.174533
                linear = 0.0
            elif max_linear > linear:
                linear += 0.01
            elif max_linear < linear:
                linear = max_linear
            if abs(angular) < 0.0872665:  # 0.174533
                angular = 0

            if current_distance > 0.1:
                if not reverse:
                    self.publish_velocity(linear, angular)
                else:
                    self.publish_velocity(-linear, angular)
                self.rate.sleep()
            else:
                self.publish_velocity(0, 0)
                print("Path completed")
                break
        self.publish_velocity(0, 0)
        if self.state == "RUNNING" and GOAL_WAIT_DELAY and wait:
            rospy.sleep(duration=COVERAGE_GOAL_WAIT_TIME)
        self.update_disinfect_map = True
        return True

    def adv_velocity_to_goal(self, goal_node, max_velocity, ignore_obstacle=False):
        print("Inside Velocity Goal")
        log_data("Inside Velocity Goal")
        end_goal = goal_node
        # print(goal_node, self.robot_node)
        self.get_robot_tf()
        goal_y = (goal_node[0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
        goal_x = (goal_node[1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)
        self.pose_marker_pub.publish(create_marker(1,
                                                   40, goal_x, goal_y, 0, (1, 0.5, 1, 0)))
        goal_node = self.get_nearby_low_region(goal_node, (COVERAGE_FREE_SPACE_DILATION * 2), False, False)
        print(f"before goal position {(goal_x, goal_y)}, {end_goal}")
        log_data(f"before goal position {(goal_x, goal_y)}, {end_goal}")
        y = (goal_node[0] - self.map_origin[0]) * self.map_resolution + \
            (self.map_resolution / 2)
        x = (goal_node[1] - self.map_origin[1]) * self.map_resolution + \
            (self.map_resolution / 2)
        try:
            self.pose_marker_pub.publish(create_marker(1, 35, x, y, 0, (1, 0.5, 0.5, 0)))
        except:
            pass
        print(f"after goal position {(x, y)}, {goal_node}")
        log_data(f"after goal position {(x, y)}, {goal_node}")
        if ignore_obstacle:
            path = searching_control(self.robot_node, goal_node)
        else:
            path = self.get_path_plan(self.robot_node, goal_node, only_length=False)

        print("Len of the Plan = ", len(path))
        log_data("Len of the Plan = " + str(len(path)))
        # print(path)
        if len(path) > 0:
            path_np = (np.array(path)[:, 0], np.array(path)[:, 1])
        else:
            path_np = []
        ret_val = True
        if len(path) == 0:
            print("No Plan available")
            log_data("No Plan available")
            ret_val = False
        goal_point_captured = False
        for n in range(0, len(path)):
            self.get_robot_tf()
            if not (FREE_SPACE_THRESHOLD <= self.filter_matrix[end_goal[0]][end_goal[1]] <
                    ROBOT_OBSTACLE_THRESHOLD) and not ignore_obstacle:
                print("End goal inside obstacle")
                log_data("End goal inside obstacle")
                self.publish_velocity(0, 0)
                ret_val = False
                break
            y = (path[n][0] - self.map_origin[0]) * self.map_resolution + (self.map_resolution / 2)
            x = (path[n][1] - self.map_origin[1]) * self.map_resolution + (self.map_resolution / 2)

            distance = abs(math.sqrt(((x - self.robot_position[0]) ** 2) +
                                     ((y - self.robot_position[1]) ** 2)))
            if distance > 0.3 and not goal_point_captured:
                goal_node = path[n]
                goal_point_captured = True
            if distance > 0.40 or n == len(path) - 1:
                if not goal_point_captured:
                    goal_node = path[n]
                # print("node cost = ", self.filter_matrix[path[n][0]][path[n][1]])
                path_check = np.where(self.filter_matrix[path_np] >= ROBOT_OBSTACLE_THRESHOLD)
                if len(path_check[0]) > 3 and not ignore_obstacle:
                    print("Path inside obstacle")
                    log_data("Path inside obstacle")
                    if not n == len(path) - 1:
                        print("Replanting...")
                        log_data("Replanting...")
                        if self.velocity_plan_retries < 3:
                            self.velocity_plan_retries += 1
                            ret_val = self.adv_velocity_to_goal(end_goal, max_velocity, ignore_obstacle=False)
                        else:
                            ret_val = False
                    break

                if not self.velocity_to_goal(goal_node, max_velocity, ignore_obstacle=ignore_obstacle,
                                             rotation_constriant=(not ignore_obstacle), record_path=True):
                    self.velocity_plan_retries = 0
                    ret_val = self.adv_velocity_to_goal(end_goal, max_velocity, ignore_obstacle=ignore_obstacle)
                    break
                goal_point_captured = False
            if self.cancel_step_goal or not self.running:
                print("Entire path covered")
                log_data("Entire path covered")
                self.cancel_step_goal = False
                self.publish_velocity(0, 0)
                break
        print("Return status = " + str(ret_val))
        log_data("Return status = " + str(ret_val))

        return ret_val

    def generate_image(self):
        # print("Inside Image Generation")
        # log_data("Inside Image Generation")
        try:
            temp = self.filter_matrix.copy()
            u_n = np.where(self.filter_matrix == UNIDENTIFIED_THRESHOLD)
            f_n = np.where(
                (self.filter_matrix >= FREE_SPACE_THRESHOLD) & (self.filter_matrix <= ROBOT_OBSTACLE_THRESHOLD))

            r_o_n = np.where(self.filter_matrix == OBSTACLE_THRESHOLD)
            o_map = dilated_area((self.map_width, self.map_height), r_o_n, 4, False, 0)
            o_n = np.where(o_map == 255)
            temp[f_n] = 255
            temp[self.raw_disinfected_node_np] = 150
            temp[self.raw_covered_node_np] = 130
            temp[u_n] = 100
            temp[o_n] = 0
            d_o_map = dilated_area((self.map_width, self.map_height), self.disinfected_obstacle_np,
                                   0, False, 3)
            d_o_n = np.where((d_o_map == o_map) == True)
            temp[d_o_n] = 170
            report_image = np.zeros([self.map_width, self.map_height, 3], np.uint8)
            report_image[:, :, 0] = temp
            report_image[:, :, 1] = temp
            report_image[:, :, 2] = temp
            report_image[np.where((report_image == [150, 150, 150]).all(axis=2))] = list(DISINFECTED_COLOR)
            report_image[np.where((report_image == [170, 170, 170]).all(axis=2))] = list(
                DISINFECTED_OBSTACLE_COLOR)
            report_image[np.where((report_image == [130, 130, 130]).all(axis=2))] = list(COVERED_COLOR)
            report_image[self.robot_path_np] = ROBOT_TRACE_COLOR
            start_angle = self.robot_angle
            if start_angle < 0:
                start_angle = 360 + start_angle
            start_point = get_node(self.robot_node, 0.5, start_angle)
            cv2.circle(report_image, (self.robot_node[1], self.robot_node[0]), radius=7, color=ROBOT_BODY_COLOR,
                       thickness=-1)
            cv2.arrowedLine(report_image, (self.robot_node[1], self.robot_node[0]), (start_point[1], start_point[0]),
                            ROBOT_DIRECTION_COLOR, 3, tipLength=1)
            report_image = cv2.flip(report_image, 1)
            report_image = cv2.rotate(report_image, cv2.ROTATE_90_CLOCKWISE)
            obstacle_x, obstacle_y = np.where(np.all(report_image == list(OBSTACLE_COLOR), axis=2) |
                                              np.all(report_image == list(DISINFECTED_OBSTACLE_COLOR), axis=2))
            if len(obstacle_x) > 0:
                top, bottom = np.min(obstacle_y) - CROP_OFFSET, np.max(obstacle_y) + CROP_OFFSET
                left, right = np.min(obstacle_x) - CROP_OFFSET, np.max(obstacle_x) + CROP_OFFSET
                if top < 0:
                    top = 0
                if bottom > self.map_width:
                    bottom = self.map_width
                if left < 0:
                    left = 0
                if right > self.map_height:
                    right = self.map_height

                lower_diff = abs(left - top)
                upper_diff = abs(right - bottom)
                if lower_diff > upper_diff:
                    right += int(abs(lower_diff - upper_diff) / 2)
                    left -= int(abs(lower_diff - upper_diff) / 2)
                else:
                    bottom += int(abs(lower_diff - upper_diff) / 2)
                    top -= int(abs(lower_diff - upper_diff) / 2)
                if top < 0:
                    top = 0
                if bottom > self.map_width:
                    bottom = self.map_width
                if left < 0:
                    left = 0
                if right > self.map_height:
                    right = self.map_height
                report_image = report_image[left:right, top:bottom]
                report_image = cv2.resize(report_image, (self.map_width, self.map_height))
                report_image = cv2.resize(report_image, (REPORT_IMAGE_WIDTH, REPORT_IMAGE_HEIGHT))
                cv2.imwrite(IMAGE_PATH + "RUNNING.png", report_image, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])
            self.report_image = report_image.copy()
        except:
            print("Error in Image Generation")
            log_data("Error in Image Generation")
            print(traceback.format_exc())
            log_data(str(traceback.format_exc()))

    def report_generation(self):
        print("Inside Report Generation")
        log_data("Inside Report Generation")
        if self.report_image is None:
            self.report_image = np.full((REPORT_IMAGE_WIDTH, REPORT_IMAGE_HEIGHT, 3), (100, 100, 100), np.uint8)
        self.end_time = datetime.now(TIMEZONE)
        time_difference = self.end_time - self.start_time
        date_time = self.start_time.strftime("%Y_%m_%d_%H_%M")
        duration_minutes = int(time_difference.total_seconds() / 60)
        percentage = self.percentage
        if self.disinfect_type < 16:
            percentage = self.disinfected_percentage
        report_name = date_time + "_" + str(duration_minutes) + "_" + str(percentage)
        rospy.set_param(COVERAGE_IMAGE_NAME, report_name)
        cv2.imwrite(IMAGE_PATH + report_name + ".png", self.report_image, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])
        print("Image saved at ", IMAGE_PATH + report_name + ".png")
        log_data("Image saved at " + IMAGE_PATH + report_name + ".png")
        if not DEBUG:
            if os.path.exists(IMAGE_PATH + "RUNNING.png"):
                os.remove(IMAGE_PATH + "RUNNING.png")
        rospy.set_param(LAMP_STATE, False)
        print("Algorithm Exit Status = ", self.state)
        log_data("Algorithm Exit Status = " + str(self.state))
        if self.state != "PERSON_DETECTED" and self.state != "USER_CTRL_C" and self.state != "INITIALIZATION_ERROR":
            if percentage > SUCCESS_PERCENTAGE:
                self.state = "COVERAGE_DONE"
                rospy.set_param(COVERAGE_STATE_PARAM, "COVERAGE_DONE")
                rospy.set_param(DISINFECT_STATE_PARAM, "COVERAGE_DONE")
            else:
                self.state = "COVERAGE_INCOMPLETE"
                rospy.set_param(COVERAGE_STATE_PARAM, "COVERAGE_INCOMPLETE")
                rospy.set_param(DISINFECT_STATE_PARAM, "COVERAGE_INCOMPLETE")

        print("Algorithm Exit Status = ", self.state)
        log_data("Algorithm Exit Status = " + str(self.state))

        if self.state == "COVERAGE_DONE" or self.state == "COVERAGE_INCOMPLETE" or self.state == "INITIALIZATION_ERROR":
            rospy.set_param("/haystack/mode_switcher", "DISINFECTION_ALGO")
            rospy.set_param(MODE_PARAM, "IDLE")


if __name__ == '__main__':
    rospy.init_node("Intelligence_Coverage")
    print("Advance Intelligence Exploration Coverage Entered")
    log_data("Advance Intelligence Exploration Coverage Entered")
    AIA = AdvanceIntelligenceAlgorithm()
    AIA.run()
    if AIA.disinfect_update_obj.is_alive():
        AIA.disinfect_update_obj.join()
    if AIA.generate_Image_obj.is_alive():
        AIA.generate_Image_obj.join()
    AIA.disinfect_navigation_uuid.shutdown()
    print("Advance Intelligence Exploration Coverage Exited")
    log_data("Advance Intelligence Exploration Coverage Exited")