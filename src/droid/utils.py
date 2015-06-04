#!/usr/bin/python

import numpy as np
from shapely.geometry import *
from geopy.distance import vincenty

import rospy
from mavros.msg import Waypoint

def point_as_primitive(point):
    return ( float(point.x), float(point.y) )

def location_distance(from_point, to_point):
    p1 = tuple( np.array(from_point) )
    p2 = tuple( np.array(to_point) )

    return vincenty( p1, p2 ).m

def point_at(distance, from_point, to_point, precision=30):
    from_point = Point( from_point )
    to_point = Point( to_point )

    line = LineString( [from_point, to_point] )

    target_point = from_point
    last_distance = location_distance( target_point, to_point )

    for value in np.linspace(0, 1, precision):
        temp_point = line.interpolate( value , normalized=True )

        distance_to_goal = location_distance( temp_point, to_point )

        if distance_to_goal < distance:
            break

        if distance < distance_to_goal < last_distance:
            target_point = temp_point
            last_distance = distance_to_goal

    return target_point

def create_waypoint(x, y, z = 5, wp_type = Waypoint.NAV_WAYPOINT):
    point = Waypoint()

    point.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    point.command = wp_type
    point.is_current = False
    point.autocontinue = True

    point.param1 = 0
    point.param2 = 0
    point.param3 = 0
    point.param4 = 0

    point.x_lat = x
    point.y_long = y
    point.z_alt = z

    return point

def log_success(status, msg, *args):
    if status:
        rospy.loginfo("[SUCC] " + (msg % args))
    else:
        rospy.loginfo("[FAIL] " + (msg % args))
