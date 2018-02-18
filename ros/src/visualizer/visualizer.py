#!/usr/bin/env python

import rospy
import yaml

from styx_msgs.msg import Lane, Waypoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker

from scipy.spatial.kdtree import KDTree
import numpy as np
import math

class Visualizer(object):
    def __init__(self):
        rospy.init_node('visualizer')

        # subscribers
        rospy.Subscriber('/base_waypoints', Lane, self.cb_base_waypoints, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.cb_final_waypoints, queue_size=1)

        # publish for visualization by rviz
        self.pub_track = rospy.Publisher('/visualizer/track', Path, queue_size=1, latch=True)
        self.pub_path = rospy.Publisher('/visualizer/path', MarkerArray, queue_size=1)
        self.pub_stop_lines = rospy.Publisher('/visualizer/stop_lines', Marker, queue_size=1, latch=True)

        # some variables
        self.path = MarkerArray()
        self.top_speed = rospy.get_param('~/waypoint_loader/velocity', 45.0) / 3.6  # convert to m/s

        self.waypoints = None
        self.kdtree_wp = None
        
        # main loop
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_path.publish(self.path)
            rate.sleep()

    def cb_base_waypoints(self, msg):
        # publish the track itself
        track = Path()
        track.header.frame_id = "world"

        for wp in msg.waypoints:
            wp_pos = wp.pose.pose.position
            track.poses.append(self.pose(wp_pos.x, wp_pos.y, -2))

        self.pub_track.publish(track)

        # save the waypoints in a kdtree for nearest waypoint lookups
        self.waypoints = list(map(lambda w : [w.pose.pose.position.x, w.pose.pose.position.y], msg.waypoints))
        self.kdtree_wp = KDTree(self.waypoints)

        # handle stop lines now the waypoints are known
        self.publish_stop_lines()

    def cb_final_waypoints(self, msg):
        self.path.markers = []

        for idx, wp in enumerate(msg.waypoints):
            self.path.markers.append(self.marker_path(idx, wp.pose.pose.position, wp.twist.twist.linear.x))

    def publish_stop_lines(self):
        # load the locations of the stop lines from the ros-param
        config_string = rospy.get_param('/traffic_light_config')
        config_yaml = yaml.load(config_string)

        marker = self.marker_line_list('stop_lines', 0)

        for stop_line in config_yaml['stop_line_positions']:
            # add a line segment perpendicular to the track at the nearest waypoint
            wp = self.track_nearest_waypoint(stop_line[0], stop_line[1])

            normal_x, normal_y = self.track_normal_vector(wp)

            # add line segment
            point = Point()
            point.x = stop_line[0] + (normal_x * 10)
            point.y = stop_line[1] + (normal_y * 10)
            marker.points.append(point)

            point = Point()
            point.x = stop_line[0] - (normal_x * 10)
            point.y = stop_line[1] - (normal_y * 10)
            marker.points.append(point)

        self.pub_stop_lines.publish(marker)


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

    def marker_line_list(self, ns, id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.ns = ns
        marker.id = id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale = Vector3(1.0, 1.0, 1.0)

        marker.color.r = 1.0
        marker.color.a = 1.0

        return marker

    def track_nearest_waypoint(self, x, y):
        _, n = self.kdtree_wp.query(np.array([[x, y]]))
        return n[0]

    def track_normal_vector(self, wp):
        wp_next = (wp + 1) % len(self.waypoints)

        # compute normal vector
        normal_x = -(self.waypoints[wp_next][1] - self.waypoints[wp][1])
        normal_y = self.waypoints[wp_next][0] - self.waypoints[wp][0]

        # make sure the normal vector is of unit size
        size = math.sqrt((normal_x * normal_x) + (normal_y * normal_y))
        normal_x /= size
        normal_y /= size

        return normal_x, normal_y



if __name__ == '__main__':
    try:
        Visualizer()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visualizer node.')