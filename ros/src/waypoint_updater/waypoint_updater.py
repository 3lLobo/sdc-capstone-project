#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32
from copy import deepcopy
from scipy.interpolate import CubicSpline
import math
import tf

LOOKAHEAD_WPS = 50 


STOP_DISTANCE = 2.0 # in meters - distance to decide to stop or not

ACCELERATION = 1.0 



class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')
		self.base_waypoints = None
		self.final_waypoints = None
		self.current_pose = None
		self.next_waypoint_index = None
		self.light_wp = None
		self.max_speed = None
		self.slow_dist = None

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
		self.wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
		rospy.Subscriber('/current_velocity', TwistStamped, callback=self.current_velocity_cb, queue_size=1)
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)

		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		self.loop()

	def pose_cb(self, msg):
		self.current_pose = msg.pose
	
	def waypoints_cb(self, wps):
		self.base_waypoints = wps.waypoints
		if self.base_waypoints:
			self.wp_sub.unregister()
			self.wp_sub = None
			self.max_speed = self.get_waypoint_velocity(self.base_waypoints[0])
			self.slow_dist = self.max_speed*4

	def current_velocity_cb(self, msg):
		self.current_velocity = msg.twist.linear.x

	def traffic_cb(self, msg):

		self.light_wp = msg.data

	def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
		pass
		
""" Publish the final_waypoints """
	def publish(self):

		final_waypoints_msg = Lane()	#create a lane
		final_waypoints_msg.header.stamp = rospy.Time.now()	# set time stamp

		self.set_final_waypoints()	#update
		self.set_final_waypoints_speed()	#set waypoint
		final_waypoints_msg.waypoints = self.final_waypoints

		self.final_waypoints_pub.publish(final_waypoints_msg)


		""" Find the next waypoint, set its speed and publish """
	def loop(self):
		# Update freaquency = 5Hz
		rate = rospy.Rate(5)
		
		while not rospy.is_shutdown(): 		#check if ROS is running
			is_initialized = self.base_waypoints and self.current_pose and self.light_wp
			
			if is_initialized:	
				start_idx = self.closest_waypoint(self.current_pose.position)
				self.next_waypoint_index = self.ahead_waypoint(start_idx)
				self.publish()
			rate.sleep()

""" Get the next LOOKAHEAD_WPS waypoints """
	def set_final_waypoints(self):

		self.final_waypoints = deepcopy(self.base_waypoints[self.next_waypoint_index: self.next_waypoint_index + LOOKAHEAD_WPS])
		rem_points = LOOKAHEAD_WPS - len(self.final_waypoints)
		if rem_points > 0: # If wraparound occurs, grab waypoints from beginning of base_waypoints. 
			self.final_waypoints = self.final_waypoints + deepcopy(self.base_waypoints[0:rem_points])
	
""" Set the waypoints speed """
	def set_final_waypoints_speed(self):

		distnc = self.distance(self.base_waypoints, self.next_waypoint_index, abs(self.light_wp))

		no_red_light_detected = self.light_wp < 0
		if no_red_light_detected:
			speed = self.current_velocity
			target_speed = self.max_speed

			# Reduce speed aproching TL
			reduced_speed = False
			if reduced_speed and distnc < self.slow_dist:
 				target_speed = self.max_speed / 1.8
			

			for wp in self.final_waypoints:
				if speed > target_speed:				
					speed = min(target_speed, speed+ACCELERATION)
				else:
					speed = max(target_speed, speed-ACCELERATION)
				self.set_waypoint_velocity(wp, speed) 
			
			return

		# stop the vehilce
		if distnc <= self.slow_dist: 
			speed = max(self.current_velocity,max(distnc*0.2,0.25))
			if distnc > STOP_DISTANCE:
				slow_down = speed / (distnc - STOP_DISTANCE)
				rospy.logwarn("Slowing down velocity: %f",slow_down)
			else:
				speed = 0
				slow_down = 0
				rospy.logwarn("vehicle stopped")			
			for wp in self.final_waypoints:
				speed = max(0, speed - slow_down)
				self.set_waypoint_velocity(wp, speed) #Accelelerate to top speed

		else: 
			# drive at max speed.
			speed = self.current_velocity
			if speed < self.max_speed:
			for wp in self.final_waypoints:
				if speed > self.max_speed:				
					speed = min(self.max_speed, speed+ACCELERATION)
				else:
					speed = max(self.max_speed, speed-ACCELERATION)
				self.set_waypoint_velocity(wp, speed) 
	

	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x


	def set_waypoint_velocity(self, waypoint, velocity):
		waypoint.twist.twist.linear.x = velocity


	def distance(self, waypoints, wp1, wp2):
		distnc = 0.0
		wp3 = -1
		#if wraparound occurs
		if wp2 < wp1: 
			wp3 = wp2
			wp2 = len(waypoints)-1
			
		for i in xrange(wp1,wp2):
			dbp = self.distance_between_points(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
			distnc += dbp
		for i in xrange(-1,wp3):
			distnc += self.distance_between_points(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
		return distnc


	def distance_between_points(self, a, b):
		dbp = (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2
		distnc = math.sqrt(dbp)
		return distnc

		
	def closest_waypoint(self, position):
		a = xrange(len(self.base_waypoints))
		return min( a , key = lambda p: self.distance_between_points(position, self.base_waypoints[p].pose.pose.position))


	def get_euler_yaw(self):	# get yaw angle
		quaternion = (
			self.current_pose.orientation.x,
			self.current_pose.orientation.y,
			self.current_pose.orientation.z,
			self.current_pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		return euler[2]


	def ahead_waypoint(self, wp_idx):
		next_idx = wp_idx

		map_wp_x = self.base_waypoints[wp_idx].pose.pose.position.x
		map_wp_y = self.base_waypoints[wp_idx].pose.pose.position.y

		x = self.current_pose.position.x	# current positon
		y = self.current_pose.position.y
	
		yaw = self.get_euler_yaw()	# yaw angle
 
		# check if wp is behind the vehicle
		localize_x = (map_wp_x - x) * math.cos(yaw) + (map_wp_y - y) * math.sin(yaw)
		if localize_x < 0.0: 
			next_idx = next_idx + 1

		self.ahead_waypoint_index = next_idx
		return next_idx

if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
