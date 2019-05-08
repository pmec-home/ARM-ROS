#!/usr/bin/env python
# license removed for brevity
import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray

# Read the joint position publish by the moveit on /joint_states and publishes
# on arm/cmd_position with a more lean format

def radiansToDegrees(radians):
	return radians * 180 / math.pi

def callback(data):
	position = Int16MultiArray()
	position.data = [0, 0, 0, 0, 0, 0]
	for i in range(0, 6):
		position.data[i] = round(radiansToDegrees(data.position[i])*10)
	pubPosition.publish(position)
	#rate.sleep()

pubPosition = rospy.Publisher('arm/cmd_position', Int16MultiArray, queue_size=1)
rospy.init_node('converter_rosserial', anonymous=True)
rospy.Subscriber("joint_states", JointState, callback, queue_size=1)
rate = rospy.Rate(5)

if __name__ == '__main__':
	try:
		print("Running")
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
