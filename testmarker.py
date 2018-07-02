
import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import math

rospy.init_node('register')

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)


markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

for k in range(30):
	for i in range(100):
	#while not rospy.is_shutdown():

		marker = Marker()
		marker.header.frame_id = "/neck"
		marker.id = i
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		#marker.lifetime = rospy.Duration(0.5)
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = math.cos(i+k*10 / 50.0)
		marker.pose.position.y = math.cos(i+k*10 / 40.0) 
		marker.pose.position.z = math.cos(i+k*10 / 30.0) 

		# We add the new marker to the MarkerArray, removing the oldest
		# marker from it when necessary
		if(count > MARKERS_MAX):
			markerArray.markers.pop(0)

		markerArray.markers.append(marker)

		rospy.sleep(0.01)

	# Renumber the marker IDs
	id = 0

	for m in markerArray.markers:
		#m.id = id
		id += 1

	# Publish the MarkerArray
	publisher.publish(markerArray)


