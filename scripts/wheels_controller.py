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

		# Do not add make the node anonymous, there should only one instance
		# running at a time
		rospy.init_node('wheels_data_messenger', anonymous=False)

		self.rate = rospy.Rate(5)
		self.md25 = md25.MD25(i2c_address)

		self.x, self.y, self.th = 0.0

		self.vx = 0.1
		self.vy = -0.1
		self.vth = 0.1

		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

		self.wheelbase_diameter_in_m = wheelbase_diameter_mm / 1000.0
		self.wheel_diameter_m = wheel_diameter_mm / 1000.0
		self.encoder_counts_per_rev = float(encoder_counts_per_rev)
		self.encoder_counts_to_distance_multiplier = self.wheel_diameter_m * math.pi / self.encoder_counts_per_rev

	def run(self):
		while not rospy.is_shutdown():
			self.publish_odometry()
			self.rate.sleep()

	def publish_odometry(self):
		self.current_time = rospy.Time.now()

		# update robot velocities
		self.update_velocities(*self.get_encoder_counts_in_m())

		# Computer odometry using the velocites of the robot
		dt = (current_time - last_time).to_sec()
		delta_x = (vx * cos(th) - vy * sin(th)) * dt
		delta_y = (vx * sin(th) + vy * cos(th)) * dt
		delta_th = vth * dt

		self.x += delta_x
		self.y += delta_y
		self.th += delta_th

		# Since all odometry is 6DOF create quaternion from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0,0,th)

		# Publish the transform over tf
		self.odometry_broadcaster.sendTransfrom(
				(self.x, self.y, 0.0),
				odom_quat,
				current_time,
				"base_link",
				"odom"
			)

		# Setup odometry header
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"

		# Setup odometry position
		odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))

		# Setup odometry velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

		# Publish message
		self.odometry_publisher.publish(odom)

		# Update time
		last_time = current_time
		self.rate.sleep()

	def set_wheel_speed_callback(self):
		pass

	def update_velocities(self, left_encoder_counts_in_m, right_encoder_counts_in_m):
		right_left = (right_encoder_counts_in_m - left_encoder_counts_in_m)
		amplitude = (right_encoder_counts_in_m + left_encoder_counts_in_m) * 0.5
		fraction = right_left / self.wheelbase_diameter_in_m

		# Assuming theta is the previous yaw value
		self.vx = amplitude * cos(self.th + (fraction/2.0))
		self.vy = amplitude * sin(self.th + (fraction/2.0))
		self.vth = fraction

	def get_encoder_counts_in_m(self):
		encoder_counts = self.get_encoder_counts()
		left_counts = encoder_counts["left_counts"]
		right_counts = encoder_counts["right_counts"]

		# convert encoder counts to millimeters
		left_distance_travelled_m = left_counts * self.encoder_counts_to_distance_multiplier
		right_distance_travelled_m = right_counts * self.encoder_counts_to_distance_multiplier

		return (left_distance_travelled_m, right_distance_travelled_m)

	def set_wheel_speeds(self, left_speed, right_speed):
		self.md25.set_wheel_speeds(left_speed, right_speed)


if __name__ == "__init__":
	try:
		msger = WheelsDataMessenger()
		msger.run()
	except rospy.ROSInterruptException:
		pass