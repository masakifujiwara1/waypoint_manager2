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
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

WAYPOINT_PATH = '/home/fmasa/ros2_ws/src/waypoint_manager2/config/waypoints/test.yaml'
WAYPOINT_SAVE_PATH = '/home/fmasa/ros2_ws/src/waypoint_manager2/config/waypoints/test_output.yaml'
WP_FEEDBACK_VISIBLE = True
OVERWRITE = True

class waypoint_behavior:
    def __init__(self):
        pass
    
    def white_line_service(self):
        pass

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
        self.marker_array = MarkerArray()

        self.send_wp_trigger_service = self.create_service(Trigger, 'waypoint_manager2/send_wp', self.send_wp_callback)
        self.save_wp_service = self.create_service(Trigger, 'waypoint_manager2/save_wp', self.save_wp_callback)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoint_manager2/waypoints', 10)

        self.time_period = 0.01
        # self.tmr = self.create_timer(self.time_period, self.callback)

        self.server = InteractiveMarkerServer(self, 'marker_control')

        self.goal_msg = FollowWaypoints.Goal()


    def callback(self):
        pass

    def send_wp_callback(self, request, response):
        self.send_goal()
        response.success = True
        return response

    def save_wp_callback(self, request, response):
        self.save_waypoints()
        response.success = True
        return response

    def load_waypoints(self):
        with open(WAYPOINT_PATH, 'r') as yml:
            self.config = yaml.safe_load(yml)
        
        # print(len(self.config['waypoint_server']['waypoints']))
    
    def save_waypoints(self):
        if OVERWRITE:
            with open(WAYPOINT_PATH, 'w') as yml:
                yaml.dump(self.config, yml)
        else:
            with open(WAYPOINT_PATH, 'w') as yml:
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

    # def makeBoxControl(self, msg):
    #     control = InteractiveMarkerControl()
    #     control.always_visible = True
    #     control.markers.append(self.makeBox(msg))
    #     msg.controls.append(control)
    #     return control
    
    def normalizeQuaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def makeMovingMarker(self, position, i):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.pose.position = position
        int_marker.scale = 1.0

        int_marker.name = str(i)
        int_marker.description = 'waypoints' + str(i)

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

        # make a box which also moves in the plane
        control.markers.append(self.makeBox(int_marker))
        control.always_visible = True
        int_marker.controls.append(control)

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

        if WP_FEEDBACK_VISIBLE:
            self.get_logger().info(
                f'{feedback.marker_name}: aligning position = {feedback.pose.position.x}, '
                f'{feedback.pose.position.y}, {feedback.pose.position.z} to '
                f'{pose.position.x}, {pose.position.y}, {pose.position.z}'
            )

        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
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

    def vizualize_wp(self):
        # self.goal_msg = FollowWaypoints.Goal()
        pose_ = PoseStamped()

        waypoints = self.config['waypoint_server']['waypoints']
        for i in range(len(waypoints)):
            # print(waypoints[i]['position']['x'])
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
            self.goal_msg.poses.append(deepcopy(pose_))

            # create marker
            position = Point(x=float(waypoints[i]['position']['x']), y=float(waypoints[i]['position']['y']), z=0.0)
            self.makeMovingMarker(position, i)
    
    def send_goal(self):
        self.action_client.wait_for_server()  

        self.future = self.action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.response_callback)

    def feedback_callback(self, feedback):
        print("feed back :", feedback.feedback.current_waypoint)

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
           return

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        print("missed waypoints :", result.missed_waypoints[0])
        # self.server.shutdown()
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = waypoint_manager2_node()

    action_client.vizualize_wp()
    # action_client.send_goal()

    action_client.server.applyChanges()
    rclpy.spin(action_client)

    # action_client.server.shutdown()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()