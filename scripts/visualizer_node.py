import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


class Visualizer:
    def __init__(self, node_name='visualizer'):
        rospy.init_node(node_name)

        # init self.pose_array a an empty list
        self.pose_array = []
        self.marker_type = rospy.get_param('~marker_type', 'sphere')

        # Subscriber
        self.pose_sub = rospy.Subscriber('/pose_topic', PoseStamped, self.pose_cb)

        # Publisher
        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)

    def pose_cb(self, data):
        self.pose_array.append(np.array([data.pose.position.x,
                                         data.pose.position.y,
                                         data.pose.position.z]))
        if self.marker_type == 'line':
            self.publish_continuous_markers()
        else:
            self.publish_discrete_markers()

    def publish_continuous_markers(self):
        '''
        Doc: https://wiki.ros.org/rviz/DisplayTypes/Marker 
        '''
        marker_array = MarkerArray()

        for i in range(len(self.pose_array)-1):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.seq = i
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            line_head = [self.pose_array[i][0], self.pose_array[i][1], self.pose_array[i][2]]
            line_tail = [self.pose_array[i+1][0], self.pose_array[i+1][1], self.pose_array[i+1][2]]
            marker.points = [Point(line_head[0], line_head[1], line_head[2]),
                             Point(line_tail[0], line_tail[1], line_tail[2])]

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.3

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Add the marker to the MarkerArray
            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def publish_discrete_markers(self):
        marker_array = MarkerArray()

        for i in range(len(self.pose_array)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.seq = i
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = self.pose_array[i][0]
            marker.pose.position.y = self.pose_array[i][1]
            marker.pose.position.z = self.pose_array[i][2]

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Add the marker to the MarkerArray
            marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    visualizer = Visualizer()

    rospy.spin()
