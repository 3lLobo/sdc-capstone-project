#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from twist_controller import Controller


class DBWNode(object):
	def __init__(self):
		rospy.init_node('dbw_node')

		vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
		fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
		brake_deadband = rospy.get_param('~brake_deadband', .1)
		decel_limit = rospy.get_param('~decel_limit', -5.)
		accel_limit = rospy.get_param('~accel_limit', 1.)
		wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
		wheel_base = rospy.get_param('~wheel_base', 2.8498)
		steer_ratio = rospy.get_param('~steer_ratio', 14.8)
		max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
		max_steer_angle = rospy.get_param('~max_steer_angle', 8.) # angle in radians

		min_speed = 0.0
		self.brake_max_torque = abs(decel_limit*vehicle_mass*wheel_radius)
		self.brake_deadband_perc = abs(brake_deadband/decel_limit)
		self.brake_deadband = brake_deadband

		self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

		self.reset() # reset class variables

		self.controller = Controller(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)		

		rospy.Subscriber('/current_velocity', TwistStamped, callback=self.current_velocity_cb, queue_size=1)
		rospy.Subscriber('/twist_cmd', TwistStamped, callback=self.twist_cmd_cb, queue_size=1)
		rospy.Subscriber('/vehicle/dbw_enabled', Bool, callback=self.dbw_enabled_cb, queue_size=1)
		self.loop()

	def reset(self):
		self.linear_velocity = None
		self.angular_velocity = None
		self.current_linear = None
		self.pose = None
		self.final_waypoints = None
		self.dbw_enabled = None
		self.time_elapsed = 0.0
		self.previous_time = rospy.get_time()

		self.previous_linear = None
		self.decel = 0.0

	#Callback functions
	def twist_cmd_cb(self, msg):
		self.linear_velocity = msg.twist.linear.x
		self.angular_velocity = msg.twist.angular.z

	def dbw_enabled_cb(self, msg):
		self.dbw_enabled = msg.data
		
	def current_velocity_cb(self, msg):
		self.current_linear = msg.twist.linear.x

	def pose_cb(self, msg):
		pass
		self.pose = [msg.pose.position.x, msg.pose.position.y]

	def waypoints_cb(self, msg):
		pass
		self.final_waypoints = [[msg.waypoints[0].pose.pose.position.x,msg.waypoints[0].pose.pose.position.y],[msg.waypoints[5].pose.pose.position.x,msg.waypoints[5].pose.pose.position.y]]

	"""
	Primary Logic for drive-by-wire node.

	This logic will call the Controller object with the current state information (speed, etc) to obtain throttle, brake, and steering commands.

	If DBW is enabled, then the values for throttle, braking, and steering are published.

	If DBW becomes disabled, then all values are reset.

	"""
	def loop(self):
		rate = rospy.Rate(5) # 50Hz
		while not rospy.is_shutdown():
			if self.dbw_enabled:
				if self.previous_time:
					self.time_elapsed = rospy.get_time() - self.previous_time  
				else:
					self.time_elapsed = 0.0
				self.previous_time = rospy.get_time()
				if self.previous_linear and self.time_elapsed > 0.0 :
				
					self.decel = (self.previous_linear-self.current_linear)/self.time_elapsed  
				else:
					self.decel = 0.0
				self.previous_linear = self.current_linear
				throttle, brake, steer = self.controller.control(self.linear_velocity, self.angular_velocity, self.current_linear, self.pose, self.final_waypoints, self.dbw_enabled, self.time_elapsed)
				if throttle > 0.0 or (brake < self.brake_deadband_perc and self.current_linear > 0.5): 
					brake = 0.0
				else: 
					brake = self.brake_max_torque*brake
				
				rospy.loginfo('linear: %s, current: %s', self.linear_velocity, self.current_linear)
				rospy.loginfo('angular: %s, decel: %s, time: %s', self.angular_velocity, self.decel, self.previous_time)
				rospy.loginfo('throttle: %s, brake: %s, steer: %s', throttle, brake, steer)

				self.publish(throttle, brake, steer)
			else: 
				self.reset()
			rate.sleep()
			

	def publish(self, throttle, brake, steer):
		stecmd = SteeringCmd()
		stecmd.enable = True
		stecmd.steering_wheel_angle_cmd = steer
		self.steer_pub.publish(stecmd)
	
		if throttle > 0.0:
			thrcmd = ThrottleCmd()
			thrcmd.enable = True
			thrcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
			thrcmd.pedal_cmd = throttle
			self.throttle_pub.publish(thrcmd)
		else:
			bracmd = BrakeCmd()
			bracmd.enable = True
			bracmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
			bracmd.pedal_cmd = brake
			self.brake_pub.publish(bracmd)

if __name__ == '__main__':
	DBWNode()
