#!/usr/bin/python

import rospy

from mavros.msg import RCIn
from mavros.msg import OverrideRCIn

from geometry_msgs.msg import Twist

from numpy import interp

cmd_publisher = None

MAX_ANGLE = 1972
MIN_ANGLE = 1111
MID_ANGLE = 1522

WEIGHT = 0.85

MAX_LINEAR = 1931 * WEIGHT
MIN_LINEAR = 1111 + ( 1111 * ( 1 - WEIGHT ) )
MID_LINEAR = 1517

def handler_rc(data):
    cmd = OverrideRCIn()
    # cmd.channels = data.channels

    linear = data.linear.x
    angular = data.angular.z

    if linear >= 0:
        linear = int( interp( linear, [0, 1], [ MID_LINEAR, MAX_LINEAR ] ) )
    else:
        linear = int( interp( abs( linear ), [0, 1], [ MID_LINEAR, MIN_LINEAR ] ) )

    angular = int( interp( angular, [-1, 1], [ MAX_ANGLE, MIN_ANGLE ] ) )

    cmd.channels[0] = angular # angular
    cmd.channels[2] = linear # linear

    cmd_publisher.publish( cmd )

def run():
    global cmd_publisher

    rospy.init_node("rc_teleop")

    # Get config
    robot_id = rospy.get_param("~robot", 0)

    last_time = rospy.Time.now()

    # Get data from Keyboard
    rospy.Subscriber("/cmd_vel", Twist, handler_rc)

    # Push data to robot
    # cmd_topic = "/robot_%i/mavros/rc/override" % robot_id
    cmd_topic = "/mavros/rc/override"
    cmd_publisher = rospy.Publisher(cmd_topic, OverrideRCIn, queue_size=10)

    # rate = rospy.Rate(10)

    # while is not rospy.is_shutdown():
        # if last_time - ros:
            # pass
        # rate.sleep()

    rospy.spin()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException, e:
        raise e
