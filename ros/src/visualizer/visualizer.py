#!/usr/bin/env python

import rospy
from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import MarkerArray, Marker

class Visualizer(object):
    def __init__(self):
        rospy.init_node('visualizer')

        # subscribers
        rospy.Subscriber('/base_waypoints', Lane, self.cb_base_waypoints, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.cb_final_waypoints, queue_size=1)

        # publish for visualization by rviz
        self.pub_track = rospy.Publisher('/visualizer/track', Path, queue_size=1, latch=True)
        self.pub_path = rospy.Publisher('/visualizer/path', MarkerArray, queue_size=1)

        # some variables
        self.path = MarkerArray()
        self.top_speed = rospy.get_param('~/waypoint_loader/velocity', 45.0) / 3.6  # convert to m/s

        # main loop
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_path.publish(self.path)
            rate.sleep()

    def cb_base_waypoints(self, msg):
        track = Path()
        track.header.frame_id = "world"

        for wp in msg.waypoints:
            wp_pos = wp.pose.pose.position
            track.poses.append(self.pose(wp_pos.x, wp_pos.y, -2))

        self.pub_track.publish(track)

    def cb_final_waypoints(self, msg):
        self.path.markers = []

        for idx, wp in enumerate(msg.waypoints):
            self.path.markers.append(self.marker_path(idx, wp.pose.pose.position, wp.twist.twist.linear.x))

    def pose(self, pos_x = 0, pos_y = 0, pos_z = 0, rot_x = 0, rot_y = 0, rot_z = 0, rot_w = 0):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = pos_x
        pose.pose.position.y = pos_y
        pose.pose.position.z = pos_z
        pose.pose.orientation.x = rot_x
        pose.pose.orientation.y = rot_y
        pose.pose.orientation.z = rot_z
        pose.pose.orientation.w = rot_w
        return pose

    def marker_path(self, id, position, speed):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = "path"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = -1
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0

        marker.color.r = 0.0
        marker.color.g = 0.2 + 0.8 * (speed / self.top_speed)
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

if __name__ == '__main__':
    try:
        Visualizer()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visualizer node.')