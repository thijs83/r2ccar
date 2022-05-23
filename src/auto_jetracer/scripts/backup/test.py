#!/usr/bin/env python3

import rospy
from getkey import getkey
from std_msgs.msg import Float32
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped



class aprildrive:
	def __init__(self):
		self.pub = rospy.Publisher('pose_tag', PoseWithCovarianceStamped, queue_size=8)

		
		#Setup topics publishing and nodes
		sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback, queue_size=2)
		rospy.init_node('convert', anonymous=False)

		#Print contol hints
		print("ROS node is ready")

		rospy.spin()
		


	def callback(self,DetectionArray):
		if DetectionArray.detections:
			x = DetectionArray.detections[0].pose.pose.pose.position.x #left-right
			y = DetectionArray.detections[0].pose.pose.pose.position.y #up-down
			z = DetectionArray.detections[0].pose.pose.pose.position.z #futher-closer
			
			initpose = PoseWithCovarianceStamped()
			initpose.header.stamp = rospy.get_rostime()
			initpose.header.frame_id = "map"
			initpose.pose.pose.position.x = z
			initpose.pose.pose.position.y = -x
			initpose.pose.pose.position.z = 0

			initpose.pose.pose.orientation.x = DetectionArray.detections[0].pose.pose.pose.orientation.x
			initpose.pose.pose.orientation.y = DetectionArray.detections[0].pose.pose.pose.orientation.y
			initpose.pose.pose.orientation.z = DetectionArray.detections[0].pose.pose.pose.orientation.z 
			initpose.pose.pose.orientation.w = DetectionArray.detections[0].pose.pose.pose.orientation.w
			
			self.pub.publish(initpose)
			

if __name__ == '__main__':
	
	try:
		print('starting')
		drive = aprildrive()

	except rospy.ROSInterruptException:
		drive.pub_throttle.publish(0)
