#!/usr/bin/env python3
from argparse import Action
import rclpy
import copy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import yaml
import math
import sys
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from interactive_markers import InteractiveMarkerServer
from geometry_msgs.msg import Point, Quaternion
from std_srvs.srv import Trigger
import numpy as np

WAYPOINT_PATH = '/home/fmasa/ros2_ws/src/waypoint_manager2/config/waypoints/test.yaml'
WAYPOINT_SAVE_PATH = '/home/fmasa/ros2_ws/src/waypoint_manager2/config/waypoints/test_output.yaml'
WP_FEEDBACK_VISIBLE = True
OVERWRITE = True
TIME_PERIOD = 0.1

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
        route.color.r = 0.0
        route.color.g = 0.0
        route.color.b = 1.0
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

class waypoint_manager2_node(Node):
    def __init__(self):
        super().__init__('waypoint_manager2_node')

        self.action_client = ActionClient(
            self,                     
            FollowWaypoints,                
            'FollowWaypoints' 
        )
        self.config = {}
        self.load_waypoints()

        self.send_wp_trigger_service = self.create_service(Trigger, 'waypoint_manager2/send_wp', self.send_wp_callback)
        self.save_wp_service = self.create_service(Trigger, 'waypoint_manager2/save_wp', self.save_wp_callback)
        self.route_pub = self.create_publisher(MarkerArray, 'waypoint_manager2/routes', 10)

        self.time_period = TIME_PERIOD
        self.tmr = self.create_timer(self.time_period, self.callback)

        self.server = InteractiveMarkerServer(self, 'waypoint_manager2')

        self.goal_msg = FollowWaypoints.Goal()
        self.resend_wp_flag = False
        self.goal_handle = None

        self.route_manager = route_manager()

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
        if WP_FEEDBACK_VISIBLE:
            print(f'{feedback.marker_name} is now at {p.x}, {p.y}, {p.z}')

    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.CYLINDER
        marker.scale.x = msg.scale
        marker.scale.y = msg.scale
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.6
        return marker

    def makeArrow(self, orientation):
        marker = Marker()

        marker.type = Marker.ARROW
        marker.scale.x = 0.5
        marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.x = orientation.x
        marker.pose.orientation.y = orientation.y
        marker.pose.orientation.z = orientation.z
        marker.pose.orientation.w = orientation.w
        return marker

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
        # int_marker.pose.orientation = orientation
        int_marker.scale = 1.0

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
        # control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        int_marker.controls.append(copy.deepcopy(control))

        # arrow control
        arrow_control = InteractiveMarkerControl()
        arrow_control.orientation.w = 1.0
        arrow_control.orientation.x = 0.0
        arrow_control.orientation.y = 1.0
        arrow_control.orientation.z = 0.0
        self.normalizeQuaternion(arrow_control.orientation)
        arrow_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # arrow_control.orientation_mode = InteractiveMarkerControl.INHERIT
        int_marker.controls.append(copy.deepcopy(arrow_control))

        # make a box which also moves in the plane
        control.markers.append(self.makeBox(int_marker))
        control.always_visible = True
        int_marker.controls.append(control)

        # make a arrow
        arrow_control.markers.append(self.makeArrow(orientation))
        arrow_control.always_visible = True
        int_marker.controls.append(arrow_control)

        # we want to use our special callback function
        self.server.insert(int_marker, feedback_callback=self.processFeedback)

        # set different callback for POSE_UPDATE feedback
        self.server.setCallback(int_marker.name, self.alignMarker, InteractiveMarkerFeedback.POSE_UPDATE)
    
    def alignMarker(self, feedback):
        pose = feedback.pose
        pose.position.z = 0.0

        i = int(feedback.marker_name)
        waypoints = self.config['waypoint_server']['waypoints']
        waypoints[i]['position']['x'] = feedback.pose.position.x
        waypoints[i]['position']['y'] = feedback.pose.position.y

        # convert q to euler
        x, y, z = self.euler_from_quaternion(feedback.pose.orientation)
        waypoints[i]['euler_angles']['x'] = float(z) #convert miss
        waypoints[i]['euler_angles']['y'] = float(y)
        waypoints[i]['euler_angles']['z'] = float(z)  

        if WP_FEEDBACK_VISIBLE:
            self.get_logger().info(
                # f'{feedback.marker_name}: aligning position = {feedback.pose.position.x}, '
                # f'{feedback.pose.position.y}, {feedback.pose.position.z} to '
                # f'{pose.position.x}, {pose.position.y}, {pose.position.z}'
                # f'{feedback.pose.orientation.x}, {feedback.pose.orientation.y}, {feedback.pose.orientation.z}, {feedback.pose.orientation.w}'
                f'{float(x)}, {float(y)}, {float(z)}'
            )

        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def apply_wp(self):
        self.goal_msg = FollowWaypoints.Goal()
        pose_ = PoseStamped()

        waypoints = self.config['waypoint_server']['waypoints']
        for i in range(len(waypoints)):
            # print(waypoints[i]['position']['x'])
            pose_.header.frame_id = "map"
            pose_.pose.position.x = float(waypoints[i]['position']['x'])
            pose_.pose.position.y = float(waypoints[i]['position']['y'])
            pose_.pose.position.z = -0.01
            euler = waypoints[i]['euler_angles']
            # q = self.quaternion_from_euler(float(euler['x']), float(euler['y']), float(euler['z']))
            # q = self.quaternion_from_euler(0.0, 0.0, float(euler['z']))
            q = self.quaternion_from_euler(float(euler['x']), 0.0, 0.0)
            pose_.pose.orientation.x = q[0]
            pose_.pose.orientation.y = q[1]
            pose_.pose.orientation.z = q[2]
            pose_.pose.orientation.w = q[3]
            self.goal_msg.poses.append(deepcopy(pose_))

            # create marker
            # q = self.quaternion_from_euler(0.0, 0.0, 0.0)
            position = Point(x=float(waypoints[i]['position']['x']), y=float(waypoints[i]['position']['y']), z=0.0)
            orientation = Quaternion(x=q[0], y=q[1], w=q[2], z=q[3])
            self.makeMovingMarker(i, position, orientation)
    
    def send_goal(self):
        self.action_client.wait_for_server()

        # print(len(self.goal_msg.poses))
        self.future = self.action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    def feedback_callback(self, feedback):
        print("feed back :", feedback.feedback.current_waypoint)

    def response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
           return

        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        try:
            print("missed waypoints :", result.missed_waypoints[0])
        except:
            pass
        # self.server.shutdown()
        # rclpy.shutdown()

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