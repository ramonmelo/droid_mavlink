#!/usr/bin/python

import rospy

from std_msgs.msg import Int16, Bool, Float64
from droid_controller.msg import *
from sensor_msgs.msg import NavSatFix
from mavros.msg import *

from droid_controller.srv import *
from mavros.srv import *
from std_srvs.srv import *

from collections import namedtuple

import utils

class Droid(object):
    """Controller for Droid"""

    SERVICE_FOLLOW = 'follow'
    SERVICE_WAIT = 'wait'
    SERVICE_GOTO = 'goto'
    SERVICE_AUTO = 'auto'
    SERVICE_MISSION = 'mission'
    SERVICE_MAV_MISSION_GOTO = 'mavros/mission/goto'
    SERVICE_MAV_MISSION_PUSH = 'mavros/mission/push'
    SERVICE_MAV_MISSION_PULL = 'mavros/mission/pull'
    SERVICE_MAV_MODE = 'mavros/set_mode'
    SERVICE_MAV_ARM = 'mavros/cmd/arming'
    SERVICE_MAV_PARAM_SET = 'mavros/param/set'
    SERVICE_MAV_PARAM_GET = 'mavros/param/get'

    TOPIC_COMM = '/droid/info'
    TOPIC_GPS = 'mavros/global_position/global'
    TOPIC_ALT = 'mavros/global_position/rel_alt'
    TOPIC_STATE = 'mavros/state'
    TOPIC_MISSION_STATE = 'mavros/mission/waypoints'
    TOPIC_RC_OVERRIDE = 'mavros/rc/override'
    TOPIC_BATTERY = 'mavros/battery'

    MAV_TYPE = namedtuple("DroidType", "unknown copter rover")("UNKNOWN", "COPTER", "ROVER")

    MODE = namedtuple("DroidMode", "wait follow goto auto battery")("WAIT", "FOLLOW", "GOTO", "AUTO", "BATTERY")
    MAV_MODE = namedtuple("DroidMavMode", "loiter land auto manual learning guided rtl hold")("LOITER", "LAND", "AUTO", "MANUAL", "LEARNING", "GUIDED", "RTL", "HOLD")

    def __init__(self):
        self.id = rospy.get_param("~id")
        self.prefix = rospy.get_param("~prefix")
        self.gps = None
        self.state = None
        self.mission = None
        self.battery = None
        self.rel_alt = 0
        self.ready = False
        self.type = Droid.MAV_TYPE.unknown
        self.droid_known_data = {}

        self.goto_once = False

        self.mode = Droid.MODE.wait
        self.mode_wait = Droid.MAV_MODE.guided
        self.mode_data = {}

        # Comunication Topic
        self.topic_comm_pub = rospy.Publisher( Droid.TOPIC_COMM, DroidInfo, queue_size=10 )
        rospy.Subscriber( Droid.TOPIC_COMM, DroidInfo, self.handler_topic_comm )

        # Droid services
        rospy.Service( Droid.SERVICE_WAIT, Empty, self.handler_service_wait )
        rospy.Service( Droid.SERVICE_AUTO, Empty, self.handler_service_auto )
        rospy.Service( Droid.SERVICE_FOLLOW, DroidFollow, self.handler_service_follow )
        rospy.Service( Droid.SERVICE_GOTO, DroidGoto, self.handler_service_goto )
        rospy.Service( Droid.SERVICE_MISSION, DroidMission, self.handler_service_mission )

        # Mavros data
        rospy.Subscriber( Droid.TOPIC_GPS, NavSatFix, self.handler_topic_gps )
        rospy.Subscriber( Droid.TOPIC_STATE, State, self.handler_topic_state )
        rospy.Subscriber( Droid.TOPIC_MISSION_STATE, WaypointList, self.handler_topic_mission )
        rospy.Subscriber( Droid.TOPIC_ALT, Float64, self.handler_topic_alt )
        rospy.Subscriber( Droid.TOPIC_BATTERY, BatteryStatus, self.handler_topic_battery )

        # Mavros topic
        self.topic_rc_pub = rospy.Publisher( Droid.TOPIC_RC_OVERRIDE, OverrideRCIn, queue_size = 10 )

        # Mavros service
        rospy.wait_for_service( Droid.SERVICE_MAV_MISSION_GOTO )
        self.service_mav_goto = rospy.ServiceProxy( Droid.SERVICE_MAV_MISSION_GOTO, WaypointGOTO )

        rospy.wait_for_service( Droid.SERVICE_MAV_MISSION_PUSH )
        self.service_mav_mission_push = rospy.ServiceProxy( Droid.SERVICE_MAV_MISSION_PUSH, WaypointPush )

        rospy.wait_for_service( Droid.SERVICE_MAV_MISSION_PULL )
        self.service_mav_mission_pull = rospy.ServiceProxy( Droid.SERVICE_MAV_MISSION_PULL, WaypointPull )

        rospy.wait_for_service( Droid.SERVICE_MAV_MODE )
        self.service_mav_mode = rospy.ServiceProxy( Droid.SERVICE_MAV_MODE, SetMode )

        rospy.wait_for_service( Droid.SERVICE_MAV_ARM )
        self.service_mav_arm = rospy.ServiceProxy( Droid.SERVICE_MAV_ARM, CommandBool )

        rospy.wait_for_service( Droid.SERVICE_MAV_PARAM_SET )
        self.service_mav_param_set = rospy.ServiceProxy( Droid.SERVICE_MAV_PARAM_SET, ParamSet )

        rospy.wait_for_service( Droid.SERVICE_MAV_PARAM_GET )
        self.service_mav_param_get = rospy.ServiceProxy( Droid.SERVICE_MAV_PARAM_GET, ParamGet )

    # Data Handlers

    def handler_topic_comm(self, data):
        if data.id != self.id:
            self.droid_known_data[ data.id ] = data

    def handler_topic_gps(self, data):
        self.gps = data

    def handler_topic_state(self, data):
        self.state = data

    def handler_topic_mission(self, data):
        self.mission = data

    def handler_topic_alt(self, data):
        self.rel_alt = data.data

    def handler_topic_battery(self, data):
        self.battery = data

    # Service Handlers

    def handler_service_wait(self, data):
        self.change_mode( Droid.MODE.wait, self.mode_wait )

        return EmptyResponse()

    def handler_service_follow(self, data):
        self.change_mode( Droid.MODE.follow )
        self.mode_data = data
        self.follow_data = None

        return DroidFollowResponse()

    def handler_service_goto(self, data):
        self.change_mode( Droid.MODE.goto )
        self.mode_data = data
        self.goto_once = False

        return DroidGotoResponse()

    def handler_service_auto(self, data):
        self.change_mode( Droid.MODE.auto )

        return EmptyResponse()

    def handler_service_mission(self, data):
        raise Exception("Not implemented")

    # Update information data
    def update_conn_topic(self):
        try:
            info = DroidInfo()

            info.header.stamp = rospy.Time.now()
            info.id = self.id
            info.gps = self.gps
            info.state = self.state
            info.ready = self.ready
            info.type = self.type

            self.topic_comm_pub.publish( info )
        except Exception, e:
            rospy.logerr(e)

    # Controller
    def run(self):
        rate = rospy.Rate(2)

        self.check()
        self.prepare()

        rospy.loginfo("Droid ready...")
        while not rospy.is_shutdown():
            self.update_conn_topic()
            self.think()

            rate.sleep()

    def check(self):
        while (self.gps == None or self.state == None) and not rospy.is_shutdown():
            rospy.loginfo("Wait DATA...")
            rospy.sleep(1)

        gps_status = self.gps.status

        while (gps_status.status == gps_status.STATUS_NO_FIX) and not rospy.is_shutdown():
            rospy.loginfo("Wait GPS Fix...")
            gps_status = self.gps.status
            rospy.sleep(1)

        self.ready = True

    def prepare(self):
        raise Exception("Not implemented")

    def think(self):
        if self.mode == Droid.MODE.wait:
            # self.change_mode( Droid.MODE.wait, self.mode_wait )
            self.change_mode( Droid.MODE.wait )

        if self.mode == Droid.MODE.follow:
            self.behavior_follow()

        if self.mode == Droid.MODE.goto:
            self.behavior_goto()

        if (self.mode == Droid.MODE.auto) or (self.state.mode == Droid.MAV_MODE.auto):
            self.behavior_auto()

        rospy.loginfo("Droid #%i in %s:%s mode", self.id, self.mode, self.state.mode)

    # Utils

    def arm(self, to = True):
        res = self.service_mav_arm( value = to )
        utils.log_success( res.success, "Arm" )

    def change_mode(self, mode = None, mav_mode = None):
        if mode:
            self.mode = mode

        if mav_mode and self.state.mode != mav_mode:
            res = self.service_mav_mode( custom_mode = mav_mode )
            utils.log_success( res.success, "Change to mode %s" % mav_mode )

    def set_param(self, name, value):
        int_value = 0
        real_value = 0.0

        if type(value) is int:
            int_value = value
        elif type(value) is float:
            real_value = value

        res = self.service_mav_param_set( param_id = name, integer = int_value, real = real_value )

        return res.success

    def send_mission(self, droid_id, mission):
        service_addr = "/%s_%i/%s" % ( self.prefix, droid_id, Droid.SERVICE_MISSION )

        rospy.wait_for_service( service_addr )
        send_mission_proxy = rospy.ServiceProxy( service_addr, DroidMission )

        send_mission_proxy( mission )

    def get_param(self, name):
        value = 0

        res = self.service_mav_param_get( param_id = name )

        if res.success:
            if res.integer != 0:
                value = res.integer
            elif res.real != 0.0:
                value = res.real

        return value

    # Behaviors

    def behavior_follow(self):
        # data = self.mode_data

        if not self.follow_data:
            self.follow_data = self.mode_data

        data = self.follow_data

        ##############

        robot_target_id = data.id
        distance_to_robot = data.distance if data.distance != 0 else data.DEFAULT_DISTANCE

        ##############

        robot_data = self.droid_known_data[ robot_target_id ]

        robot_target_position = ( robot_data.gps.latitude, robot_data.gps.longitude )
        robot_self_position = ( self.gps.latitude, self.gps.longitude )

        robot_desired_position = utils.point_at( distance_to_robot, robot_self_position, robot_target_position )

        ###############

        self.change_mode( Droid.MODE.follow, Droid.MAV_MODE.guided )
        # self.change_mode( mav_mode =  )

        ###############

        wp = utils.create_waypoint( *utils.point_as_primitive( robot_desired_position ) )

        self.service_mav_goto( wp )

    def behavior_goto(self):
        data = self.mode_data

        ###############

        if not self.goto_once:
            self.change_mode( mav_mode = Droid.MAV_MODE.guided )

            ###############

            wp = utils.create_waypoint( data.waypoint.x_lat, data.waypoint.y_long, data.waypoint.z_alt )

            self.service_mav_goto( wp )

            self.goto_once = True

    def behavior_auto(self):
        raise Exception("Not implemented")
