#!/usr/bin/python

import rospy

from droid.droid_copter import DroidCopter
from droid.droid_rover import DroidRover

from mavros.srv import ParamGet

sw_type = {
    10: DroidCopter,
    20: DroidRover
}

def run():
    rospy.init_node("droid_node", anonymous=True)

    get_param_service = 'mavros/param/get'
    rospy.wait_for_service( get_param_service )
    get_param_service = rospy.ServiceProxy( get_param_service, ParamGet )

    sw_type_res = None

    while True and not rospy.is_shutdown():
        rospy.sleep(5)

        sw_type_res = get_param_service( param_id = 'SYSID_SW_TYPE' )

        if sw_type_res is not None and sw_type_res.success:
            break

    droid = sw_type[ sw_type_res.integer ]()
    droid.run()

    rospy.spin()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException, e:
        raise e
