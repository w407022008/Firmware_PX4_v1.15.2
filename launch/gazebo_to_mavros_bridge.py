#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math


class GazeboToMavrosBridge:
    def __init__(self):
        rospy.init_node('gazebo_to_mavros_bridge', anonymous=True)
        
        # Get parameters
        self.uav_name = rospy.get_param('~uav_name', '')
        self.yaw_offset = rospy.get_param('~offset_yaw', 0.0)
        self.reset_origin = rospy.get_param('~reset_origin', False)
        
        # Origin offset
        self.reset_origin_x = 0.0
        self.reset_origin_y = 0.0
        self.reset_origin_z = 0.0
        
        # Current position
        self.cur_pos_x = 0.0
        self.cur_pos_y = 0.0
        self.cur_pos_z = 0.0
        self.cur_pos_time = rospy.Time()
        
        # Publishers
        self.vision_pose_pub = rospy.Publisher(
            f"{self.uav_name}/mavros/vision_pose/pose", 
            PoseStamped, 
            queue_size=10
        )
        
        # Subscriber for Gazebo ground truth
        self.gazebo_sub = rospy.Subscriber(
            f"{self.uav_name}/gazebo/ground_truth/odometry", 
            Odometry, 
            self.gazebo_callback
        )
        
        rospy.loginfo("[gazebo_to_mavros_bridge] Node initialized")
        rospy.loginfo(f"[gazebo_to_mavros_bridge] uav_name: {self.uav_name}")
        rospy.loginfo(f"[gazebo_to_mavros_bridge] offset_yaw: {self.yaw_offset}")
        rospy.loginfo(f"[gazebo_to_mavros_bridge] reset_origin: {self.reset_origin}")

    @staticmethod
    def quaternion_to_euler(w, x, y, z):
        """
        Convert quaternion to euler angles
        """
        euler = [0.0, 0.0, 0.0]
        euler[0] = math.atan2(2.0 * (z * y + w * x), 1.0 - 2.0 * (x * x + y * y))  # roll
        euler[1] = math.asin(2.0 * (y * w - z * x))  # pitch
        euler[2] = math.atan2(2.0 * (z * w + x * y), 1.0 - 2.0 * (y * y + z * z))  # yaw
        return euler

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert euler angles to quaternion
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return w, x, y, z

    def apply_yaw_offset(self, w, x, y, z):
        """
        Apply yaw offset to quaternion
        """
        if abs(self.yaw_offset) <= 0.001:
            return w, x, y, z
        
        euler = self.quaternion_to_euler(w, x, y, z)
        euler[2] += self.yaw_offset
        return self.euler_to_quaternion(euler[0], euler[1], euler[2])

    def gazebo_callback(self, msg):
        """
        Callback for Gazebo ground truth odometry messages
        """
        if msg.header.frame_id == "world":
            # Create pose stamped message
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = msg.pose.pose.position.x - self.reset_origin_x
            pose_msg.pose.position.y = msg.pose.pose.position.y - self.reset_origin_y
            pose_msg.pose.position.z = msg.pose.pose.position.z - self.reset_origin_z

            # Apply yaw offset to orientation
            w, x, y, z = self.apply_yaw_offset(
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            )
            
            pose_msg.pose.orientation.w = w
            pose_msg.pose.orientation.x = x
            pose_msg.pose.orientation.y = y
            pose_msg.pose.orientation.z = z
        
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = "world"
            
            # Publish the pose
            self.vision_pose_pub.publish(pose_msg)
            
            # Update current position
            self.cur_pos_time = msg.header.stamp
            self.cur_pos_x = msg.pose.pose.position.x
            self.cur_pos_y = msg.pose.pose.position.y
            self.cur_pos_z = msg.pose.pose.position.z
            
        else:
            rospy.logwarn("Wrong Gazebo ground truth frame id. Expected 'world', got '%s'", msg.header.frame_id)


def main():
    try:
        bridge = GazeboToMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()