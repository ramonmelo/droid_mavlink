#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from droid_controller.msg import *

from droid_controller.srv import *

from droid import Droid

import utils

class DroidRover(Droid):

    TOPIC_OBSTACLE = 'droid/obstacle'

    def __init__(self):
        super(DroidRover, self).__init__()
        self.type = Droid.MAV_TYPE.rover
        self.mode_wait = Droid.MAV_MODE.hold

        self.has_obstacle = False

        # Droid topics
        rospy.Subscriber( DroidRover.TOPIC_OBSTACLE, Bool, self.handler_topic_obstacle )

    # Data Handlers

    def handler_topic_obstacle(self, data):
        self.has_obstacle = data.data

    # Service Handlers

    def handler_service_mission(self, data):
        res = self.service_mav_mission_push( data.waypoints )
        self.change_mode( mode = Droid.MODE.auto )

        return DroidMissionResponse( res.success )

    # Controller

    def run(self):
        rospy.loginfo("Init Droid Rover")
        super(DroidRover, self).run()

    def prepare(self):
        self.arm()

    def think(self):
        super(DroidRover, self).think()

    # Behaviors

    def behavior_auto(self):
        if self.mode != Droid.MODE.auto or self.state.mode != Droid.MAV_MODE.auto:
            self.service_mav_mission_pull()
            self.arm()
            self.change_mode(mode = Droid.MODE.auto, mav_mode = Droid.MAV_MODE.auto )

        ##########

        if self.has_obstacle:
            self.change_mode(mode = Droid.MODE.wait, mav_mode = Droid.MAV_MODE.hold )

            if self.mission:
                current_mission = self.mission.waypoints

                for i, wp in enumerate( current_mission ):
                    if wp.is_current:
                        break

                mission_slice = current_mission[i:]

                for droid_id in self.droid_known_data.keys():
                    droid = self.droid_known_data[ droid_id ]

                    if droid.ready and droid.type == Droid.MAV_TYPE.copter:
                        self.send_mission( droid_id, mission_slice )
                        break
                else:
                    rospy.logwarn("No help droid found")

            rospy.logwarn("Returning to HOME")
            self.change_mode(mode = Droid.MODE.wait, mav_mode = Droid.MAV_MODE.rtl )


