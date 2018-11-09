#!/usr/bin/env python 
import rospy
import math
import actionlib

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import SegmentObstacle
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped

from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback


#front x +
#left y +
class narrow_path:
	def __init__(self):
		rospy.init_node('narrow_path', anonymous=True)
				
		self.pub = rospy.Publisher('ackermann', AckermannDriveStamped, queue_size=10)		

		self.server = actionlib.SimpleActionServer('narrow_path', MissionPlannerAction, execute_cb=self.execute_cb, auto_start=False)
		self.server.start()
		self.result = MissionPlannerResult()
	
		self.wayPoint = Point(1,0,0)
		
		self.count = [0]
		self.control_factor = rospy.get_param("/narrow_path/control_factor", 104)
		self.right_steer_scale = rospy.get_param("/narrow_path/right_steer_scale", 2.0)	
		self.throttle = rospy.get_param("/narrow_path/throttle", 0)
		self.stop_count= rospy.get_param("/narrow_path/stop_count", 100)
		self.finish_flag = False
	
	def updateParam(self):
		self.control_factor = rospy.get_param("/narrow_path/control_factor")
		self.right_steer_scale = rospy.get_param("/narrow_path/right_steer_scale")	
		print("control_factor: " + str(self.control_factor))	
		print("right_steer_scale: " + str(self.right_steer_scale))
	
	def execute_cb(self, goal):
		self.sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_cb)

		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if(self.finish_flag == True):
				self.server.set_succeeded(result)	
			r.sleep()

	def obstacles_cb(self, data):
		self.updateParam()
	
		x_center = 0
		y_center = 0
		#find wayPoint 
		for segment_data in data.segments:
			x_center = x_center + segment_data.first_point.x
			x_center = x_center + segment_data.last_point.x
			y_center = y_center + segment_data.first_point.y
			y_center = y_center + segment_data.last_point.y
		if (x_center == 0):
			self.count[0] = self.count[0] + 1
			print("mission count : %d", self.count[0])
			acker_data = AckermannDriveStamped()
			acker_data.drive.speed = self.throttle 
			acker_data.drive.steering_angle = 0	 
			print("speed : " + str(acker_data.drive.speed))
			print("steering : " + str(acker_data.drive.steering_angle))
			self.pub.publish(acker_data) 
			
				     
		else:
			#self.count[0] = 0;
			x_center = x_center/len(data.segments)
			y_center = y_center/len(data.segments)
			self.wayPoint = Point(x_center,y_center,0)
               
			print(self.wayPoint)
			#if detect segment, up start signal and start mission
			self.start_signal = 1
			print("during narrow mission")
			print("#######################################################")
			acker_data = AckermannDriveStamped()
			acker_data.drive.speed = self.throttle		
			steer_angle = math.atan(self.wayPoint.y/self.wayPoint.x)
			acker_data.drive.steering_angle = int(-(self.control_factor*steer_angle)/math.pi)
			if (acker_data.drive.steering_angle > 0):
				acker_data.drive.steering_angle = int(acker_data.drive.steering_angle/self.right_steer_scale)
			if (acker_data.drive.steering_angle > 26):
				acker_data.drive.steering_angle = 26
			elif (acker_data.drive.steering_angle < -26):
				acker_data.drive.steering_angle = -26
			print("speed : " + str(acker_data.drive.speed))
			print("steering : " + str(acker_data.drive.steering_angle))

			if(self.count[0] > self.stop_count):
				self.finish_flag = True
			self.pub.publish(acker_data)
		
		
if __name__ == '__main__':
	try:
		narrow_mission = narrow_path()
		
		rospy.spin()
	except rospy.ROSInterruptException:
		print(error)
		pass			
				

