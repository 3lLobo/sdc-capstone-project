#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from light_classification.tl_classifierob import TLClassifierOB
import math
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
	def __init__(self):
		rospy.init_node('tl_detector')

		self.pose = None
		self.waypoints = None
		self.camera_image = None
		self.lights = []
		self.slps = []
                self.listener = None
              
		sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
		self.waypoints = rospy.wait_for_message('/base_waypoints', Lane).waypoints # get base_waypoints

		sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)

		config_string = rospy.get_param("/traffic_light_config")
		self.config = yaml.load(config_string)

                self.model_type = rospy.get_param("model_type")

		# List of positions that correspond to the line to stop in front of a given intersection. Convert to more appropriate format
		for slp in self.config['stop_line_positions']:
			tl = TrafficLight()
			tl.pose.pose.position.x = slp[0]
			tl.pose.pose.position.y = slp[1]
			tl.pose.pose.position.z = 0
			self.slps.append( self.get_closest_waypoint(tl.pose.pose.position) )

		sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

		self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

		self.bridge = CvBridge()
                if self.model_type == "CNN" :   
					self.light_classifier = TLClassifier()
                else:
                    self.light_classifier = TLClassifierOB()

                    
		self.listener = tf.TransformListener()
		self.state = TrafficLight.UNKNOWN
		self.last_state = TrafficLight.UNKNOWN
		self.last_wp = -1
		self.state_count = 0

		rospy.spin()

	def pose_cb(self, msg):
		self.pose = msg

	def waypoints_cb(self, waypoints):
		self.waypoints = waypoints.waypoints

	def traffic_cb(self, msg):
		self.lights = msg.lights

	def image_cb(self, msg):

		self.has_image = True
		self.camera_image = msg
		if not self.waypoints: pass
		elif not self.lights: pass
		elif not self.pose: pass

		light_wp, state = self.process_traffic_lights()
		if state == TrafficLight.UNKNOWN:
			state_str = 'Green/Yellow'
		else:
			state_str = 'Red'

		rospy.logwarn("Light wp: %s,  Light state: %s",light_wp, state_str)

		if self.state != state:
			self.state_count = 0
			self.state = state
		elif self.state_count >= STATE_COUNT_THRESHOLD:
			self.last_state = self.state
			light_wp = light_wp if state == TrafficLight.RED else (-light_wp)
			self.last_wp = light_wp
			self.upcoming_red_light_pub.publish(Int32(light_wp))
		else:
			self.upcoming_red_light_pub.publish(Int32(self.last_wp))
		self.state_count += 1

	def get_closest_waypoint(self, position):
		val1 = xrange(len(self.waypoints))
		val2 = key = lambda p: self.distance_between_points(position, self.waypoints[p].pose.pose.position)
		minval = min(val1, val2)

		return minval

	def distance_between_points(self, a, b):
		dist = math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2 )
		return dist

	def project_to_image_plane(self, point_in_world):

		fx = self.config['camera_info']['focal_length_x']
		fy = self.config['camera_info']['focal_length_y']
		image_width = self.config['camera_info']['image_width']
		image_height = self.config['camera_info']['image_height']

		trans = None
		try:
			now = rospy.Time.now()
			self.listener.waitForTransform("/base_link",
				  "/world", now, rospy.Duration(1.0))
			(trans, rot) = self.listener.lookupTransform("/base_link",
				  "/world", now)

		except (tf.Exception, tf.LookupException, tf.ConnectivityException):
			rospy.logerr("Failed to find camera to map transform")

		x = 0
		y = 0

		return (x, y)

	def get_light_state(self, tl):

		if(not self.has_image):
            		self.prev_light_loc = None
            		return TrafficLight.RED

		if hasattr(self.camera_image, 'encoding'):
		    self.attribute = self.camera_image.encoding
		    if self.camera_image.encoding == '8UC3':
				self.camera_image.encoding = "rgb8"
		else:
		    self.camera_image.encoding = 'rgb8'
		cv_img = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

                if (self.listener is not None):
		        x, y = self.project_to_image_plane(tl.pose.pose.position)

		return self.light_classifier.get_classification(cv_img)

	def process_traffic_lights(self):

		light = None
                if (self.waypoints is None):
                        return -1, TrafficLight.UNKNOWN
                
		if(self.pose):
			car_position = self.get_closest_waypoint(self.pose.pose.position)

			func = lambda p: self.slps[p]-car_position if self.slps[p] >= car_position else len(self.waypoints) + self.slps[p]-car_position
			index = min(xrange(len(self.slps)), key = func)
			light_wp = self.slps[index]
			light = self.lights[index]
		if light:
			state = self.get_light_state(light)
			return light_wp, state
		return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
	try:
		TLDetector()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start traffic node.')
