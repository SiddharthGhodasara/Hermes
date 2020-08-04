#!/usr/bin/env python

import rospy
import roslib
import time
import math
import tf2_ros
from robot_controller.msg import velocities 
import geometry_msgs
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry


class Base():
	def vel_callback(self,vel):
		self.odom_broadcaster_ = tf2_ros.TransformBroadcaster()
		self.odom_trans = geometry_msgs.msg.TransformStamped()
		self.odom= Odometry()		

		self.current_time= rospy.Time.now().to_sec()
		#print(self.current_time)
		#print(self.last_vel_time_)
		self.linear_velocity_x_ = vel.linear_x
		self.linear_velocity_y_ = vel.linear_y
		self.angular_velocity_z_ = vel.angular_z
		
		self.vel_dt_ = (self.current_time - self.last_vel_time_)
		#print(self.vel_dt_)
                self.last_vel_time_ = self.current_time
		#rospy.Time.now().to_sec()

		delta_heading = self.angular_velocity_z_ * self.vel_dt_ #radians
		delta_x = (self.linear_velocity_x_ * math.cos(self.heading_) - self.linear_velocity_y_ * math.sin(self.heading_)) * self.vel_dt_ #m
		delta_y = (self.linear_velocity_x_ * math.sin(self.heading_) + self.linear_velocity_y_ * math.cos(self.heading_)) * self.vel_dt_ #m	

		#calculate current position of the robot
		self.x_pos_ += delta_x
		self.y_pos_ += delta_y
		self.heading_ += delta_heading	

		#calculate robot's heading in quaternion angle
                #ROS has a function to calculate yaw in quaternion angle
                odom_quat=quaternion_from_euler(0,0,self.heading_)
		#print(odom_quat)
		self.odom_trans.header.frame_id = "odom"
		self.odom_trans.child_frame_id = "base_link"
		#robot's position in x,y, and z
		self.odom_trans.transform.translation.x = self.x_pos_
		self.odom_trans.transform.translation.y = self.y_pos_
		self.odom_trans.transform.translation.z = 0.0

		#robot's heading in quaternion
		self.odom_trans.transform.rotation.x = odom_quat[0]
		self.odom_trans.transform.rotation.y = odom_quat[1]
		self.odom_trans.transform.rotation.z = odom_quat[2]
		self.odom_trans.transform.rotation.w = odom_quat[3]
		self.odom_trans.header.stamp = self.current_time

		#publish robot's tf using odom_trans object
		self.odom_broadcaster_.sendTransform(self.odom_trans)
		
		self.odom.header.stamp = self.current_time
		self.odom.header.frame_id = "odom"
		self.odom.child_frame_id = "base_link"

		#robot's position in x,y, and z
		self.odom.pose.pose.position.x = self.x_pos_
		self.odom.pose.pose.position.y = self.y_pos_
		self.odom.pose.pose.position.z = 0.0

		#robot's heading in quaternion
		self.odom.pose.pose.orientation.x = odom_quat[0]
		self.odom.pose.pose.orientation.y = odom_quat[1]
		self.odom.pose.pose.orientation.z = odom_quat[2]
		self.odom.pose.pose.orientation.w = odom_quat[3]
		self.odom.pose.covariance[0] = 0.001
		self.odom.pose.covariance[7] = 0.001
		self.odom.pose.covariance[35] = 0.001

		#linear speed from encoders
		self.odom.twist.twist.linear.x = self.linear_velocity_x_
		self.odom.twist.twist.linear.y = self.linear_velocity_y_
		self.odom.twist.twist.linear.z = 0.0

		#angular speed from encoders
		self.odom.twist.twist.angular.x = 0.0
		self.odom.twist.twist.angular.y = 0.0
		self.odom.twist.twist.angular.z = self.angular_velocity_z_
		self.odom.twist.covariance[0] = 0.0001
		self.odom.twist.covariance[7] = 0.0001
		self.odom.twist.covariance[35] = 0.0001
		#print(self.odom)
		self.odom_publisher_.publish(self.odom)
		


	def __init__(self):

		#print("init")
		self.linear_velocity_x_=0
		self.linear_velocity_y_=0
		self.angular_velocity_z_=0
		self.last_vel_time_=0
		self.current_time=0
		self.vel_dt_=0
		self.x_pos_=0
		self.y_pos_=0
		self.heading_=0
		rospy.init_node('base_python',anonymous=True)
		self.odom_publisher_ = rospy.Publisher("odom", Odometry, queue_size=50 )
		self.velocity_subscriber_ = rospy.Subscriber("vel_raw", velocities, self.vel_callback)
		



if __name__ == '__main__':
	try:
		s=Base()
		#print(s)
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Base_node terminated")
	


