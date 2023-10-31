#!/usr/bin/env python3
from argparse import Action
import rclpy
import copy
from rclpy import qos
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from copy import deepcopy
import action_msgs
import yaml
import math
import sys
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, InteractiveMarkerUpdate
from interactive_markers import InteractiveMarkerServer, MenuHandler
from geometry_msgs.msg import Point, Quaternion, Pose
from std_srvs.srv import Trigger
import numpy as np
from scipy.spatial.transform import Rotation as R

#WAYPOINT_PATH = '/home/ros2_ws/src/waypoint_manager2/config/waypoints/tsudanuma2-3.yaml'
# WAYPOINT_PATH = '/home/ros2_ws/src/waypoint_manager2/config/waypoints/tsukuba_check_area_way.yaml'
# WAYPOINT_PATH = '/home/ros2_ws/src/waypoint_manager2/config/waypoints/tsukuba_all_way.yaml'
WAYPOINT_PATH = '/home/ros2_ws/src/waypoint_manager2/config/waypoints/tsudanuma_gaisyu_res01_way.yaml'
# WAYPOINT_SAVE_PATH = '/home/ros2_ws/src/waypoint_manager2/config/waypoints/test_output.yaml'
WP_FEEDBACK_VISIBLE = True
OVERWRITE = True
TIME_PERIOD = 0.1

menu_handler = MenuHandler()
h_first_entry = 0
h_mode_last = 0
radius_mode_last = 0
trafic_mode_last = 0

marker_pos = 0

class waypoint_behavior:
    def __init__(self):
        pass
    
    def white_line_service(self):
        pass

class route_manager:
    def __init__(self):
        self.marker_array = MarkerArray()

    def makeRoute(self, i, position1, position2):
        route = Marker()
        route.header.frame_id = 'map'
        route.ns = "routes" + str(i)
        route.id = i
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.pose.position.x = 0.0
        route.pose.position.y = 0.0
        route.pose.position.z = 0.0
        route.scale.x = route.scale.y = route.scale.z = 0.03
        route.color.r = 1.0
        route.color.g = 1.0
        route.color.b = 0.0
        route.color.a = 1.0
        route.points.append(position1)
        route.points.append(position2)
        route.lifetime = rclpy.duration.Duration(seconds = 0.0).to_msg()
        self.marker_array.markers.append(route)

    def updateRoute(self, config):
        waypoints = config['waypoint_server']['waypoints']
        for i in range(len(waypoints)-1):
            position1 = Point(x=float(waypoints[i]['position']['x']), y=float(waypoints[i]['position']['y']), z=0.0)
            position2 = Point(x=float(waypoints[i+1]['position']['x']), y=float(waypoints[i+1]['position']['y']), z=0.0)
            self.makeRoute(i, position1, position2)

class Trafic_light_client(Node):
    def __init__(self):
        super().__init__('Trafic_light_client')
        self.client = self.create_client(Trigger, '/trafic_light')
    
    def send_request(self):
        req = Trigger.Request()
        self.future = self.client.call_async(req)

class waypoint_manager2_node(Node):
    def __init__(self):
        super().__init__('waypoint_manager2_node')

        self.action_client = ActionClient(
            self,                     
            NavigateToPose,                
            '/navigate_to_pose' 
        )
        self.config = {}
        self.load_waypoints()

        self.send_wp_trigger_service = self.create_service(Trigger, 'waypoint_manager2/send_wp', self.send_wp_callback)
        self.save_wp_service = self.create_service(Trigger, 'waypoint_manager2/save_wp', self.save_wp_callback)
        self.next_wp_service = self.create_service(Trigger, 'waypoint_manager2/next_wp', self.next_wp_callback)
        self.route_pub = self.create_publisher(MarkerArray, 'waypoint_manager2/routes', 10)
        self.update_pub = self.create_publisher(InteractiveMarkerUpdate, 'waypoint_manager2/update', 10)

        # subscribe amcl pose
        qos_profile = qos.qos_profile_sensor_data
        self.amcl_pos_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, qos_profile)

        self.time_period = TIME_PERIOD
        self.tmr = self.create_timer(self.time_period, self.callback)

        self.server = InteractiveMarkerServer(self, 'waypoint_manager2')

        # self.goal_msg = FollowWaypoints.Goal()
        self.goal_msg = NavigateToPose.Goal()
        self.resend_wp_flag = False
        self.goal_handle = None

        self.route_manager = route_manager()
        self.initMenu()

        self.num_wp = 0

        self.old_x, self.old_y = 1, 1

        # action status
        self.sum_nav_time = 0
        self.sum_rec_num = 0
        self.distance = 10

        # stop_wp
        self.reject_next_wp = False
        self.next_wp_flag = False

        self.current_waypoint = 0
        self.old_number_of_recoveries = 0
        self.failed_count = 0

        # for calculating distance with amcl_pose
        self.amcl_pos = PoseWithCovarianceStamped()
        self.amcl_distance = 0
        self.amcl_status = False

        # for max amcl and action
        self.goal_ = False
        self.first = True
        self.goal_status = False

        # waypoint color
        self.color_flag = False

        # trafic mode
        self.trafic_flag = False
        self.trafic_light_client = Trafic_light_client()

    def callback(self):
        self.route_manager.marker_array = MarkerArray()
        self.route_manager.updateRoute(self.config)
        self.route_pub.publish(self.route_manager.marker_array)

    def send_wp_callback(self, request, response):
        if self.resend_wp_flag:
            pass
        self.send_goal()
        response.success = True
        self.resend_wp_flag = True
        return response

    def save_wp_callback(self, request, response):
        self.save_waypoints()
        response.success = True
        return response

    def next_wp_callback(self, request, response):
        self.next_wp()
        self.reject_next_wp = False
        response.success = True
        return response

    def load_waypoints(self):
        with open(WAYPOINT_PATH, 'r') as yml:
            self.config = yaml.safe_load(yml)
    
    def save_waypoints(self):
        if OVERWRITE:
            with open(WAYPOINT_PATH, 'w') as yml:
                yaml.dump(self.config, yml)
        else:
            with open(WAYPOINT_SAVE_PATH, 'w') as yml:
                yaml.dump(self.config, yml)

    def processFeedback(self, feedback):
        p = feedback.pose.position
        o = feedback.pose.orientation
        if WP_FEEDBACK_VISIBLE:
            print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')
        
    def amcl_callback(self, msg):
        self.amcl_pos = copy.deepcopy(msg)
        
    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.CYLINDER
        marker.scale.x = msg.scale
        marker.scale.y = msg.scale
        marker.scale.z = 0.01
        if self.color_flag == 'Stop_wp':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6
        elif self.color_flag == 'Trafic_wp':
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.6
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
        return marker

    def makeArrow(self, orientation, msg):
        marker = Marker()

        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = msg.scale / 2
        marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        return marker

    def deepCb(self, feedback):
        self.get_logger().info('The deep sub-menu has been found.')

    def modeCb(self, feedback):
        global h_mode_last
        menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)
        h_mode_last = feedback.menu_entry_id

        # menu_entry_id: Stop_ON > 6, Stop_OFF > 7
        waypoints = self.config['waypoint_server']['waypoints']
        if 'properties' not in waypoints[int(feedback.marker_name)]:
            waypoints[int(feedback.marker_name)]['properties'] = {}
        if feedback.menu_entry_id == 6:
            waypoints[int(feedback.marker_name)]['properties'].update(Stop_wp = 'Stop_ON')
        elif feedback.menu_entry_id == 7:
            waypoints[int(feedback.marker_name)]['properties'].update(Stop_wp = 'Stop_OFF')

        menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)

        menu_handler.reApply(self.server)
        self.apply_wp()
        self.server.applyChanges()

    def modeCb_radius(self, feedback):
        global radius_mode_last
        menu_handler.setCheckState(radius_mode_last, MenuHandler.UNCHECKED)
        radius_mode_last = feedback.menu_entry_id

        # menu_entry_id: 0.5 -> 9, 0.75 -> 10, 1.0 -> 11, 1.5 -> 12
        waypoints = self.config['waypoint_server']['waypoints']
        if 'properties' not in waypoints[int(feedback.marker_name)]:
            waypoints[int(feedback.marker_name)]['properties'] = {}
        if feedback.menu_entry_id == 9:
            waypoints[int(feedback.marker_name)]['properties'].update(goal_radius = 0.5)
        elif feedback.menu_entry_id == 10:
            waypoints[int(feedback.marker_name)]['properties'].update(goal_radius = 0.75)
        elif feedback.menu_entry_id == 11:
            waypoints[int(feedback.marker_name)]['properties'].update(goal_radius = 1.0)
        elif feedback.menu_entry_id == 12:
            waypoints[int(feedback.marker_name)]['properties'].update(goal_radius = 1.5)

        menu_handler.setCheckState(radius_mode_last, MenuHandler.CHECKED)
        menu_handler.reApply(self.server)

        self.server.clear()
        self.apply_wp()
        self.server.applyChanges()

    def modeCb_trafic(self, feedback):
        global trafic_mode_last
        menu_handler.setCheckState(trafic_mode_last, MenuHandler.UNCHECKED)
        trafic_mode_last = feedback.menu_entry_id

        # menu_entry_id: Stop_ON > 14, Stop_OFF > 15
        waypoints = self.config['waypoint_server']['waypoints']
        if 'properties' not in waypoints[int(feedback.marker_name)]:
            waypoints[int(feedback.marker_name)]['properties'] = {}
        if feedback.menu_entry_id == 14:
            waypoints[int(feedback.marker_name)]['properties'].update(Trafic_wp = 'Trafic_ON')
        elif feedback.menu_entry_id == 15:
            waypoints[int(feedback.marker_name)]['properties'].update(Trafic_wp = 'Trafic_OFF')

        menu_handler.setCheckState(trafic_mode_last, MenuHandler.CHECKED)

        menu_handler.reApply(self.server)
        self.apply_wp()
        self.server.applyChanges()

    def initMenu(self):
        global h_first_entry, h_mode_last, radius_mode_last, trafic_mode_last
        
        h_first_entry = menu_handler.insert('insert_wp', callback=self.insert_callback)
        delete_entry = menu_handler.insert('delete_wp', callback=self.delete_callback)
        save_entry = menu_handler.insert('save_wp', callback=self.menu_save_wp)
        start_entry = menu_handler.insert('start_wp_nav', callback=self.start_wp)

        sub_menu_handle = menu_handler.insert('Stop_wp')
        for i in range(2):
            if i == 0:
                s = 'Stop_ON'
            else:
                s = 'Stop_OFF'
            h_mode_last = menu_handler.insert(s, parent=sub_menu_handle, callback=self.modeCb)
            menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)
        # check the very last entry
        menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)

        sub_menu_handle_radius = menu_handler.insert('Goal_diameter')
        for i in range(4):
            if i == 0:
                s = '0.5'
            if i == 1:
                s = '0.75'
            if i == 2:
                s = '1.0'
            if i == 3:
                s = '1.5'
            radius_mode_last = menu_handler.insert(s, parent=sub_menu_handle_radius, callback=self.modeCb_radius)
            menu_handler.setCheckState(radius_mode_last, MenuHandler.UNCHECKED)
        # check the very last entry
        menu_handler.setCheckState(radius_mode_last, MenuHandler.CHECKED)

        sub_menu_handle = menu_handler.insert('Trafic_wp')
        for i in range(2):
            if i == 0:
                s = 'Trafic_ON'
            else:
                s = 'Trafic_OFF'
            trafic_mode_last = menu_handler.insert(s, parent=sub_menu_handle, callback=self.modeCb_trafic)
            menu_handler.setCheckState(trafic_mode_last, MenuHandler.UNCHECKED)
        # check the very last entry
        menu_handler.setCheckState(trafic_mode_last, MenuHandler.CHECKED)

    def start_wp(self, feedback):
        self.server.clear()
        self.apply_wp()
        self.server.applyChanges()
        self.send_goal()

    def start_wp_goal(self):
        # self.server.clear()
        self.set_next_wp()
        self.server.applyChanges()
        self.send_goal()

    def menu_save_wp(self, feedback):
        self.save_waypoints()

    def insert_callback(self, feedback):
        p = feedback.pose.position
        print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')

        # register insert point
        i = int(feedback.marker_name)
        waypoints = self.config['waypoint_server']['waypoints']
        waypoints.insert(i+1, copy.deepcopy(waypoints[i]))

        waypoints[i+1]['position']['x'] = p.x + 0.3
        waypoints[i+1]['position']['y'] = p.y 

        # initialize server
        self.server.clear()

        # recreate interactive_marker
        self.apply_wp()

        # apply change
        self.server.applyChanges()

    def delete_callback(self, feedback):
        p = feedback.pose.position
        print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')

        # register insert point
        i = int(feedback.marker_name)
        waypoints = self.config['waypoint_server']['waypoints']
        del waypoints[i]

        # initialize server
        self.server.clear()

        # recreate interactive_marker
        self.apply_wp()

        # apply change
        self.server.applyChanges()

    def normalizeQuaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def makeMovingMarker(self, i, position, orientation):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.pose.position = position
        int_marker.pose.orientation = copy.deepcopy(orientation)

        waypoints = self.config['waypoint_server']['waypoints']
        self.color_flag = False

        if 'properties' in waypoints[i]:
            if 'goal_radius' in waypoints[i]['properties']:
                int_marker.scale = float(waypoints[i]['properties']['goal_radius'])
        else:
            int_marker.scale = 1.0

        if 'properties' in waypoints[i]:
            if 'Stop_wp' in waypoints[i]['properties']:
                if waypoints[i]['properties']['Stop_wp'] == 'Stop_ON':
                    self.color_flag = 'Stop_wp'
        else:
            self.color_flag = False

        if 'properties' in waypoints[i]:
            if 'Trafic_wp' in waypoints[i]['properties']:
                if waypoints[i]['properties']['Trafic_wp'] == 'Trafic_ON':
                    self.color_flag = 'Trafic_wp'
        else:
            self.color_flag = False

        int_marker.name = str(i)
        int_marker.description = 'waypoints' + str(i)

        # waypoints control
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        self.normalizeQuaternion(control.orientation)
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.orientation_mode = InteractiveMarkerControl.INHERIT
        int_marker.controls.append(copy.deepcopy(control))

        # arrow control
        arrow_control = InteractiveMarkerControl()
        arrow_control.name = 'arrow' + str(i)
        arrow_control.orientation.w = 1.0
        arrow_control.orientation.x = 0.0
        arrow_control.orientation.y = 1.0
        arrow_control.orientation.z = 0.0
        self.normalizeQuaternion(arrow_control.orientation)
        arrow_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        arrow_control.orientation_mode = InteractiveMarkerControl.INHERIT
        int_marker.controls.append(copy.deepcopy(arrow_control))

        # make a box which also moves in the plane
        control.markers.append(self.makeBox(int_marker))
        control.always_visible = True
        int_marker.controls.append(control)

        # make a arrow
        arrow_control.markers.append(self.makeArrow(orientation, int_marker))
        arrow_control.always_visible = True
        int_marker.controls.append(arrow_control)

        # make a menu
        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        int_marker.controls.append(menu_control)

        # we want to use our special callback function
        self.server.insert(int_marker, feedback_callback=self.processFeedback)
        menu_handler.apply(self.server, int_marker.name)

        # set different callback for POSE_UPDATE feedback
        self.server.setCallback(int_marker.name, self.alignMarker, InteractiveMarkerFeedback.POSE_UPDATE)
    
    def alignMarker(self, feedback):
        pose = feedback.pose
        pose.position.z = 0.0

        i = int(feedback.marker_name)
        arrow_name = 'arrow' + str(i)
        waypoints = self.config['waypoint_server']['waypoints']
        waypoints[i]['position']['x'] = feedback.pose.position.x
        waypoints[i]['position']['y'] = feedback.pose.position.y

        # convert q to euler
        x, y, z = self.euler_from_quaternion(pose.orientation)
        waypoints[i]['euler_angles']['x'] = copy.deepcopy(float(x))
        waypoints[i]['euler_angles']['y'] = copy.deepcopy(float(y))
        waypoints[i]['euler_angles']['z'] = copy.deepcopy(float(z)) 
        print(x, y, z)

        self.server.setPose(feedback.marker_name, pose)
        # self.server.clear()
        # self.apply_wp()
        self.server.applyChanges()
    
    def calc_direction(self, x, y):
        dx = x - self.old_x
        dy = y - self.old_y
        angle = math.atan2(dy, dx)
        quat = self.quaternion_from_euler(0, 3.14, -angle)
        euler = [0, 0, -angle]
        self.old_x = x
        self.old_y = y
        return quat, euler

    def quaternion_from_euler(self, roll, pitch, yaw):
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        q = r.as_quat()
        return q
  
    def euler_from_quaternion(self, quaternion):
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        e = r.as_euler('xyz', degrees=False)
        return e[0], e[1], e[2]

    def apply_wp(self):
        pose_ = PoseStamped()

        waypoints = self.config['waypoint_server']['waypoints']
        for i in range(len(waypoints)):
            pose_.header.frame_id = "map"
            pose_.pose.position.x = float(waypoints[i]['position']['x'])
            pose_.pose.position.y = float(waypoints[i]['position']['y'])
            pose_.pose.position.z = -0.01
            euler = waypoints[i]['euler_angles']
            q = self.quaternion_from_euler(float(euler['x']), float(euler['y']), float(euler['z']))
            pose_.pose.orientation.x = q[0]
            pose_.pose.orientation.y = q[1]
            pose_.pose.orientation.z = q[2]
            pose_.pose.orientation.w = q[3]

            if i == self.current_waypoint:
                self.goal_msg.pose = copy.deepcopy(pose_)

            # create marker
            position = Point(x=float(waypoints[i]['position']['x']), y=float(waypoints[i]['position']['y']), z=0.0)
            orientation = copy.deepcopy(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
            self.makeMovingMarker(i, position, orientation)

    def set_next_wp(self):
        self.goal_msg = NavigateToPose.Goal()
        pose_ = PoseStamped()

        waypoints = self.config['waypoint_server']['waypoints']
        # for i in range(len(waypoints)):
        pose_.header.frame_id = "map"
        pose_.pose.position.x = float(waypoints[self.current_waypoint]['position']['x'])
        pose_.pose.position.y = float(waypoints[self.current_waypoint]['position']['y'])
        pose_.pose.position.z = -0.01
        euler = waypoints[self.current_waypoint]['euler_angles']
        q = self.quaternion_from_euler(float(euler['x']), float(euler['y']), float(euler['z']))
        pose_.pose.orientation.x = q[0]
        pose_.pose.orientation.y = q[1]
        pose_.pose.orientation.z = q[2]
        pose_.pose.orientation.w = q[3]

            # if i == self.current_waypoint:
        self.goal_msg.pose = copy.deepcopy(pose_)
    
    def send_goal(self):
        self.future = self.action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    def feedback_callback(self, feedback):
        print("navigation_time :", str(feedback.feedback.navigation_time.sec) + '.' + str(feedback.feedback.navigation_time.nanosec))
        print("number_of_recoveries :", feedback.feedback.number_of_recoveries)
        print("distance_remaining :", feedback.feedback.distance_remaining)
        self.distance = feedback.feedback.distance_remaining
        self.nav_time = feedback.feedback.navigation_time
        self.is_reached_goal()

    def response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
           return

        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')

            if self.trafic_flag:
                self.trafic_light_client.send_request()
                self.trafic_flag = False

            if (not self.goal_) and self.first:
                self.start_wp_goal()
                self.goal_ = False
                self.first = False
        else:
            self.get_logger().info('Goal failed!')
            self.failed_count += 1

    def calc_distance_with_amcl(self):
        self.amcl_distance = math.sqrt((self.goal_msg.pose.pose.position.x - self.amcl_pos.pose.pose.position.x)**2 + (self.goal_msg.pose.pose.position.y - self.amcl_pos.pose.pose.position.y)**2)

    def is_reached_goal(self):
        GOAL_RADIUS = 0.5
        self.reject_next_wp = False
        self.next_wp_flag = False
        self.amcl_status = False

        waypoints = self.config['waypoint_server']['waypoints']

        # set goal_radius
        if 'properties' in waypoints[self.current_waypoint]:
            if 'goal_radius' in waypoints[self.current_waypoint]['properties']:
                GOAL_RADIUS = copy.deepcopy(float(waypoints[self.current_waypoint]['properties']['goal_radius']) / 2)
        else:
            GOAL_RADIUS = 0.5

        if 'properties' in waypoints[self.current_waypoint]:
            if 'Stop_wp' in waypoints[self.current_waypoint]['properties']:
                if waypoints[self.current_waypoint]['properties']['Stop_wp'] == 'Stop_ON':
                    self.reject_next_wp = True
                    self.goal_ = True

        if 'properties' in waypoints[self.current_waypoint]:
            if 'Trafic_wp' in waypoints[self.current_waypoint]['properties']:
                if waypoints[self.current_waypoint]['properties']['Trafic_wp'] == 'Trafic_ON':
                    self.reject_next_wp = True
                    self.goal_ = True
                    self.trafic_flag = True

        self.calc_distance_with_amcl()

        if self.amcl_distance > self.distance:
            self.amcl_status = True
            self.distance = copy.deepcopy(self.amcl_distance)

        print("distance_amcl :", self.amcl_distance)
        print( 'current_waypoint: ', self.current_waypoint)

        if self.nav_time.sec >= 3.0 or self.distance > GOAL_RADIUS + 0.5:
            self.next_wp_flag = True
        
        print('reject_next_wp: ' + str(self.reject_next_wp) + '\n' + 'next_wp_flag: ' + str(self.next_wp_flag) + '\n' + 'bigger_amcl_distance: ' + str(self.amcl_status) + '\n')

        # check stop_wp
        if self.reject_next_wp:
            pass
        else:
            if self.distance <= GOAL_RADIUS + 0.25 and self.next_wp_flag and self.current_waypoint < len(waypoints) - 1:
                self.next_wp_flag = False
                self.next_wp()

    def next_wp(self):
        self.current_waypoint += 1
        self.old_number_of_recoveries = 0
        self.set_next_wp()
        self.send_goal()
        self.first = True

def main(args=None):
    rclpy.init(args=args)

    action_client = waypoint_manager2_node()

    action_client.apply_wp()

    action_client.server.applyChanges()
    rclpy.spin(action_client)

    # action_client.server.shutdown()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()