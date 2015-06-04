#!/usr/bin/python

import rospy
from mavros.msg import Waypoint, OverrideRCIn
from droid_controller.msg import *

from std_srvs.srv import *
from droid_controller.srv import *

from droid import Droid

import utils

class DroidCopter(Droid):

    SERVICE_INIT = "init"

    MISSION_ALTITUDE = 5

    def __init__(self):
        super(DroidCopter, self).__init__()
        self.type = Droid.MAV_TYPE.copter
        self.mode_wait = Droid.MAV_MODE.loiter
        rospy.Service( DroidCopter.SERVICE_INIT, Empty, self.handler_service_init )

    # Service Handlers

    def handler_service_init(self, data):
        self.init_mission()
        return EmptyResponse()

    def handler_service_mission(self, data):
        mission = data.waypoints
        for wp in mission:
            wp.is_current = False

        self.init_mission( mission )

        return DroidMissionResponse( True )

    # Controller

    def run(self):
        rospy.loginfo("Init Droid Copter")
        super(DroidCopter, self).run()

    def prepare(self):
        pass
        # rospy.sleep(20)
        # self.set_param( 'RTL_ALT', self.MISSION_ALTITUDE * 100 )

    def think(self):
        super(DroidCopter, self).think()
        # self.verify_battery()

    # Utils

    def verify_battery(self):
        data = self.battery

        if data is not None and data.remaining < 0.3:
            rospy.logwarn("Low Battery")
            self.change_mode( Droid.MODE.battery, Droid.MAV_MODE.rtl )

    def init_mission(self, mission = []):
        rospy.loginfo("Initializing...")

        # First WAIT
        self.mode = Droid.MODE.wait

        init_mission = []

        ready = (self.state.armed and self.rel_alt > 0)

        # Resquest mode LOITER
        self.change_mode( mav_mode = Droid.MAV_MODE.loiter )
        # rospy.sleep(1)

        if not ready:
            # Request to ARM the Quad
            self.arm()
            rospy.sleep(2)

            # Create the Initial mission
            # With the home position and take off waypoint
            lat, lon = self.gps.latitude, self.gps.longitude

            rospy.loginfo("Create start mission")
            wp_home = utils.create_waypoint( x=lat, y=lon )
            wp = utils.create_waypoint( x=lat, y=lon, z=DroidCopter.MISSION_ALTITUDE, wp_type = Waypoint.NAV_TAKEOFF )

            init_mission.extend( [wp_home, wp] )

        # Send the mission
        init_mission.extend( mission )

        res = self.service_mav_mission_push( init_mission )
        utils.log_success( res.success, "Mission sended" )

        rospy.sleep(2)

        # Request mode AUTO
        self.change_mode( mav_mode = Droid.MAV_MODE.auto )

        if not ready:
            # Send RC commands to init the AUTO mission
            cmd = OverrideRCIn()

            cmd.channels = [ cmd.CHAN_NOCHANGE ] * 8
            cmd.channels[2] = 1700

            self.topic_rc_pub.publish( cmd )
            rospy.sleep(2)
            # Release RC commands
            cmd.channels[2] = cmd.CHAN_RELEASE
            self.topic_rc_pub.publish( cmd )

            rospy.sleep(2)

        # Wait until reach the desired inital altitude
        if not mission:
            while self.rel_alt < DroidCopter.MISSION_ALTITUDE and not rospy.is_shutdown():
                rospy.sleep(1)

    # Behaviors

    def behavior_auto(self):
        self.change_mode(mode = Droid.MODE.auto, mav_mode = Droid.MAV_MODE.auto )
