#!/usr/bin/env python

import tf
import math
import md25
import rospy
from math import sin, cos, pi
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class WheelsDataMessenger(object):
	def __init__(self, i2c_address=0x58, wheelbase_diameter_mm=1000.0, wheel_diameter_mm=100.0, encoder_counts_per_rev=360):
		self.odometry_publisher = rospy.Publisher('odom', Odometry, queue_size=50)
		self.odometry_broadcaster = tf.TransformBroadcaster()

		# Do not add {make the node anonymous}, only one instance at a time is required
		rospy.init_node('wheels_data_messenger', anonymous=False)

		self.rate = rospy.Rate(5)
		self.md25 = md25.MD25(i2c_address)

		self.x = self.y = self.th = 0.0

		self.vx = 0.1
		self.vy = -0.1
		self.vth = 0.1

		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

		self.wheelbase_diameter_in_m = wheelbase_diameter_mm / 1000.0
		self.wheel_diameter_m = wheel_diameter_mm / 1000.0
		self.encoder_counts_per_rev = float(encoder_counts_per_rev)
		self.encoder_counts_to_distance_multiplier = self.wheel_diameter_m * math.pi / self.encoder_counts_per_rev

		rospy.on_shutdown(self.shutdown)

	def run(self):
		while not rospy.is_shutdown():
			self.publish_odometry()
			self.rate.sleep()

	def publish_odometry(self):
		self.current_time = rospy.Time.now()

		# update robot velocities
		self.update_velocities(*self.get_encoder_counts_in_m())

		# Computer odometry using the velocites of the robot
		dt = (self.current_time - self.last_time).to_sec()
		delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
		delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
		delta_th = self.vth * dt

		self.x += delta_x
		self.y += delta_y
		self.th += delta_th

		# Since all odometry is 6DOF create quaternion from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)

		# Publish the transform over tf
		self.odometry_broadcaster.sendTransform(
				(self.x, self.y, 0.0),
				odom_quat,
				self.current_time,
				"base_link",
				"odom"
			)

		# Setup odometry header
		odom = Odometry()
		odom.header.stamp = self.current_time
		odom.header.frame_id = "odom"

		# Setup odometry position
		odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))

		# Setup odometry velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

		# Publish message
		self.odometry_publisher.publish(odom)

		# Update time
		self.last_time = self.current_time
		self.rate.sleep()

	def subscribe_to_teleop_controls(self):
		topic_name = "/cmd_vel_mux/input/teleop"
		topic_type = "geometry_msgs/Twist"

		rospy.Subscriber(topic_name, Twist, self.teleop_callback)

	def teleop_callback(self, data):
		speed = data.linear.x
		rotation = data.angular.z

		left_wheel_speed = (speed * self.wheelbase_diameter_in_m)/2 + speed
		right_wheel_speed = (speed * 2) - left_wheel_speed

		self.set_wheel_speeds(left_wheel_speed, right_wheel_speed)
		

	def update_velocities(self, left_encoder_counts_in_m, right_encoder_counts_in_m):
		right_left = (right_encoder_counts_in_m - left_encoder_counts_in_m)
		amplitude = (right_encoder_counts_in_m + left_encoder_counts_in_m) * 0.5
		fraction = right_left / self.wheelbase_diameter_in_m

		# Assuming theta is the previous yaw value
		self.vx = amplitude * cos(self.th + (fraction/2.0))
		self.vy = amplitude * sin(self.th + (fraction/2.0))
		self.vth = fraction

	def get_encoder_counts_in_m(self):
		encoder_counts = self.md25.get_encoder_counts()
		left_counts = encoder_counts["left_count"]
		right_counts = encoder_counts["right_count"]

		# convert encoder counts to millimeters
		left_distance_travelled_m = left_counts * self.encoder_counts_to_distance_multiplier
		right_distance_travelled_m = right_counts * self.encoder_counts_to_distance_multiplier

		return (left_distance_travelled_m, right_distance_travelled_m)

	def set_wheel_speeds(self, left_speed, right_speed):
		self.md25.set_wheel_speeds(left_speed, right_speed)

	def shutdown(self):
		self.set_wheel_speeds(0,0)


if __name__ == "__init__":
	msger = WheelsDataMessenger()
	msger.subscribe_to_teleop_controls()
	msger.run()

	try:
		pass
	except rospy.ROSInterruptException:
		pass
else:
	msger = WheelsDataMessenger()
	msger.subscribe_to_teleop_controls()
	msger.run()