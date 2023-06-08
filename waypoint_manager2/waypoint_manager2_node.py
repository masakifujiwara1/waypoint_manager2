from argparse import Action
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import yaml
import math
from visualization_msgs.msg import Marker, MarkerArray

WAYPOINT_PATH = '/home/fmasa/ros2_ws/src/waypoint_manager2/config/waypoints/test.yaml'

class waypoint_behavior:
    def __init__(self):
        pass
    
    def white_line_service(self):
        pass

class waypoint_manager2_node(Node):
    def __init__(self):
        super().__init__('rotate_turtle')

        self.action_client = ActionClient(
            self,                     
            FollowWaypoints,                
            'FollowWaypoints' 
        )
        self.config = {}
        self.load_waypoints()
        self.marker_array = MarkerArray()

        self.waypoint_pub = self.create_publisher(MarkerArray, 'waypoint_manager2/waypoints', 10)
        self.time_period = 1.0
        self.tmr = self.create_timer(self.time_period, self.callback)

    def callback(self):
        self.waypoint_pub.publish(self.marker_array)

    def load_waypoints(self):
        with open(WAYPOINT_PATH, 'r') as yml:
            self.config = yaml.safe_load(yml)
        
        # print(len(self.config['waypoint_server']['waypoints']))

    def visualization_waypoints(self):
        pass

    def create_arrow(self):
        pass

    def create_cylinder(self, i, x, y, z):
        cylinder = Marker()
        cylinder.header.frame_id = 'map'
        cylinder.ns = "waypoints" + str(i)
        cylinder.id = i
        cylinder.type = Marker.CYLINDER
        cylinder.action = Marker.ADD
        cylinder.pose.position.x = x
        cylinder.pose.position.y = y
        cylinder.pose.position.z = z
        cylinder.scale.x = cylinder.scale.y = 1.0
        cylinder.scale.z = 0.01
        cylinder.color.r = 0.0
        cylinder.color.g = 1.0
        cylinder.color.b = 0.0
        cylinder.color.a = 0.6
        cylinder.lifetime = rclpy.duration.Duration(seconds = 0.0).to_msg()
        self.marker_array.markers.append(cylinder)

    def create_txt(self, i, x, y, z):
        text = Marker()
        text.header.frame_id = 'map'
        text.ns = "texts" + str(i)
        text.id = i
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x + 0.02
        text.pose.position.y = y + 0.02
        text.pose.position.z = z + 0.02
        text.text = 'waypoints_' + str(i)
        text.scale.z = 0.3
        text.color.r = 0.0
        text.color.g = 0.0
        text.color.b = 0.0
        text.color.a = 1.0
        text.lifetime = rclpy.duration.Duration(seconds = 0.0).to_msg()
        self.marker_array.markers.append(text)

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

    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
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
            goal_msg.poses.append(deepcopy(pose_))

            self.create_cylinder(i, pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z)
            self.create_txt(i, pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z)

        self.action_client.wait_for_server()  

        self.future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
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
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = waypoint_manager2_node()

    action_client.send_goal()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()