#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

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
ACCELERATION = 0.25


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []
        self.tl_waypoint = -1
        self.pose = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(10)   # 10Hz
        while not rospy.is_shutdown():
            self.publish_update()
            rate.sleep()

    def publish_update(self):
        # don't do anything until all necessary data is present
        if len(self.base_waypoints) == 0 or self.pose == None:
            return
        
        # get the next waypoint up ahead the car
        wp_idx = self.next_waypoint(self.pose.position, self.pose.orientation)

        # build list of upcoming waypoints
        final_waypoints = Lane()
        final_waypoints.header.stamp = rospy.Time.now()

        for idx in range(wp_idx, min(wp_idx + LOOKAHEAD_WPS, len(self.base_waypoints))):
            wp = Waypoint()
            wp.pose.pose.position.x = self.base_waypoints[idx].pose.pose.position.x
            wp.pose.pose.position.y = self.base_waypoints[idx].pose.pose.position.y
            wp.twist.twist.linear.x = self.base_waypoints[idx].twist.twist.linear.x
            final_waypoints.waypoints.append(wp)

        # stop at red traffic lights
        if self.tl_waypoint != -1:
            # the center of the car should stop a few meters before the stop-line
            stop_wp = self.earlier_waypoint(self.tl_waypoint, 3)
            stop_idx = max(stop_wp - wp_idx, 0)

            for i in range(stop_idx, LOOKAHEAD_WPS):
                self.set_waypoint_velocity(final_waypoints.waypoints, i, 0)

            prev_pos = final_waypoints.waypoints[stop_idx].pose.pose.position
            vel = 0

            stop_idx = stop_idx - 1
            while stop_idx >= 0:
                dist = self.distance_points(prev_pos, final_waypoints.waypoints[stop_idx].pose.pose.position)
                vel = math.sqrt(vel**2 + (2 * ACCELERATION * dist))
                vel = min(vel, self.get_waypoint_velocity(final_waypoints.waypoints[stop_idx]))
                self.set_waypoint_velocity(final_waypoints.waypoints, stop_idx, vel)
                stop_idx = stop_idx - 1

        # publish
        self.final_waypoints_pub.publish(final_waypoints)

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        # save waypoints for later use
        self.base_waypoints = waypoints.waypoints
        rospy.logdebug('waypoints_cb: received {} waypoints'.format(len(self.base_waypoints)))

    def traffic_cb(self, msg):
        rospy.logdebug('traffic_cb: red light at {}'.format(msg.data))
        self.tl_waypoint = msg.data

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

    def distance_points(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2  + (p1.z-p2.z)**2)

    def distance_waypoints(self, wp1, wp2):
        return self.distance_points(wp1.pose.pose.position, wp2.pose.pose.position)

    def kph2mps(self, kph):
        return kph / 3.6
    
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

        # use the axis/angle formula to get a heading from the quaternion 
        #   (assuming the axis always point straight up)
        heading_car = math.acos(orientation.w)

        heading_wp = math.atan2(position_wp.y - position.y, position_wp.x - position.x)
        
        angle = math.fabs(heading_car - heading_wp)
        angle = min(2 * math.pi - angle, angle)

        if (angle > math.pi / 4.0):
            closest_wp = (closest_wp + 1) % len(self.base_waypoints)
        
        return closest_wp

    def earlier_waypoint(self, wp, target_dist):
        result_wp = wp
        dist = 0

        while target_dist > dist:
            prev_wp = result_wp
            result_wp = (result_wp - 1) % len(self.base_waypoints)
            dist += self.distance_waypoints(self.base_waypoints[prev_wp], self.base_waypoints[result_wp])

        return result_wp



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
