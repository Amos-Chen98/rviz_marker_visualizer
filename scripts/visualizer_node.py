import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


class Visualizer:
    def __init__(self, node_name='visualizer'):
        rospy.init_node(node_name)

        # init self.pose_array a an empty list
        self.marker_type = rospy.get_param('~marker_type', 'sphere')
        self.marker_array = MarkerArray()
        self.marker_id = 0
        self.current_pose = None
        self.last_pose = None

        # Subscriber
        self.pose_sub = rospy.Subscriber('/pose_topic', PoseStamped, self.pose_cb)

        # Publisher
        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)

    def pose_cb(self, data):
        if self.current_pose is not None:
            self.last_pose = self.current_pose
        self.current_pose = data.pose

        if self.marker_type == 'line':
            self.publish_continuous_markers()
        else:
            self.publish_discrete_markers()

    def publish_continuous_markers(self):
        '''
        Doc: https://wiki.ros.org/rviz/DisplayTypes/Marker 
        '''
        if self.last_pose is None:
            return

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.seq = self.marker_id
        marker.id = self.marker_id
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.points = [Point(self.last_pose.position.x,
                               self.last_pose.position.y,
                               self.last_pose.position.z),
                         Point(self.current_pose.position.x,
                               self.current_pose.position.y,
                               self.current_pose.position.z)]

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add the marker to the MarkerArray
        self.marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_pub.publish(self.marker_array)

        self.marker_id += 1

    def publish_discrete_markers(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.seq = self.marker_id
        marker.id = self.marker_id
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.current_pose.position.x
        marker.pose.position.y = self.current_pose.position.y
        marker.pose.position.z = self.current_pose.position.z

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add the marker to the MarkerArray
        self.marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_pub.publish(self.marker_array)

        self.marker_id += 1


if __name__ == '__main__':
    visualizer = Visualizer()

    rospy.spin()
