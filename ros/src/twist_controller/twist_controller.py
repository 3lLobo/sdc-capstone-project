from rospy import logwarn
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from math import atan2, sin, cos

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
	def __init__(self, wb, steer, mspeed, max_lat_accel, max_steer):
		self.max_angle = max_steer
		self.yaw_controller = YawController(wb, steer, mspeed, max_lat_accel, max_steer)
		self.set_controllers()

	def control(self,lin_vel, ang_vel, current_linear, pose, way_points, dbw_enabled, elapsed_time):
	#Reset if drive by wire is not enabled and return 0's
		if not dbw_enabled:
			self.set_controllers()
			return 0.0, 0.0, 0.0

	# CTE for throttle
		cte = lin_vel - current_linear
		brake = self.pid_brake.step(-cte, elapsed_time)
		throttle = self.pid_throttle.step(cte, elapsed_time)
		steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_linear)

	# Low pass filter
		steer = self.lowpass_steer.filt(steer)
		return throttle, brake, steer

	def set_controllers(self):

		self.pid_throttle = PID(0.35,0.0,0.0,0.0,1.0)
		self.pid_brake = PID(0.3,0.0,0.0,0.0,1.0)
		self.lowpass_steer = LowPassFilter(0.2,1.0) #Low pass filter
