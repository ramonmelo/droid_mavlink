#!/usr/bin/python

import rospy

from mavros.msg import RCIn
from mavros.msg import OverrideRCIn

cmd_publisher = None

# last_time = None

def handler_rc(data):
    # global last_time

    cmd = OverrideRCIn()
    cmd.channels = data.channels

    cmd_publisher.publish( cmd )

    # last_time = rospy.Time.now()

def run():
    global cmd_publisher
    global last_time

    rospy.init_node("rc_teleop")

    # Get config
    robot_id = rospy.get_param("~robot", 0)

    last_time = rospy.Time.now()

    # Get data from RC
    rospy.Subscriber("/mavros/rc/in", RCIn, handler_rc)

    # Push data to robot
    cmd_topic = "/robot_%i/mavros/rc/override" % robot_id
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
