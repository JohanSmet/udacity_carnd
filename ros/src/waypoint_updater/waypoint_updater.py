#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf.transformations

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []

        rospy.spin()

    def pose_cb(self, msg):
        if len(self.base_waypoints) == 0:
            return

        wp_idx = self.next_waypoint(msg.pose.position, msg.pose.orientation)
        rospy.loginfo('pose_cb: next waypoint = {}'.format(wp_idx));

        final_waypoints = Lane()
        for idx in range(wp_idx, wp_idx + LOOKAHEAD_WPS):
            final_waypoints.waypoints.append(self.base_waypoints[idx % len(self.base_waypoints)])
        self.final_waypoints_pub.publish(final_waypoints)

    def waypoints_cb(self, waypoints):
        # save waypoints for later use
        self.base_waypoints = waypoints.waypoints
        rospy.loginfo('waypoints_cb: received {} waypoints'.format(len(self.base_waypoints)))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
    
    def closest_waypoint(self, position):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        closest_wp = 0
        min_dist = dl(position, self.base_waypoints[closest_wp].pose.pose.position)

        for i in range(1, len(self.base_waypoints)):
            dist = dl(position, self.base_waypoints[i].pose.pose.position)
            if dist < min_dist:
                min_dist = dist
                closest_wp = i
        
        return closest_wp

    def next_waypoint(self, position, orientation):
        ros_np = lambda o: [o.x, o.y, o.z, o.w]

        closest_wp = self.closest_waypoint(position)
        position_wp = self.base_waypoints[closest_wp].pose.pose.position

        heading_car, _, _ = tf.transformations.euler_from_quaternion(ros_np(orientation))
        heading_wp = math.atan2(position_wp.y - position.y, position_wp.x - position.x)
        
        angle = math.fabs(heading_car - heading_wp)
        angle = min(2 * math.pi - angle, angle)

        if (angle > math.pi / 4.0):
            closest_wp = (closest_wp + 1) % len(self.base_waypoints)
        
        return closest_wp



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
