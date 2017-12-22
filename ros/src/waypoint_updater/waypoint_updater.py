#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32

import math, sys
import numpy as np


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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
PUBLISHING_RATE = 4 # Publishing frequency (Hz)
MAX_SPEED_LIMIT = 10 # max speed limit
BRAKE_WPS = 50. # number of waypoints before traffic light to brake

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
        
        # Subscriber for /base_waypoint and /current_post
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb,queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb,queue_size=1)
        
        # Subscriber for /traffic_waypoint
        # self.traffic_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb,queue_size=1)   
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb,queue_size=1) 
                        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        

        self.current_pose   = None  # current coords of vehicle                
        self.base_waypoints = None  # list of base waypoints               
        self.queue_wp       = None  # waypoints to publish                
        self.next_waypoint  = None  # index of next waypoint                
        self.stop_waypoint  = None  # stop line index for the nearest light                
        self.next_basewp    = None  # the next waypoint index to retrieve from base                
        self.destination    = None  # the final waypoint in the list                
        self.num_base_wp    = 0     # the number of points in the base list                
        self.msg_seq_num    = 0     # sequence number of published message                
        self.velocity_drop  = 60.   # distance to begin reducing velocity                
        self.VELOCITY_MAX   = 5.554 # mps Carla max of 20 km/h (updated by waypoints_cb)                
        self.LOOKAHEAD_WPS  = 25    # Number of waypoints we will publish.                
        self.prev_state     = None  # previous traffic light state                
        self.halt           = False # shut down                
        self.replan         = True  # when a light changes, update velocity                
        self.loop           = True  # loop around the test site (updated by waypoints_cb)
      
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # It is not a mandatory requirement to implement obstacle callback for capstone projects.  
        # further implementation will be considered. 
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
