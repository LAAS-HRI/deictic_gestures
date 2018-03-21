#!/usr/bin/env python

import rospy
import tf
from std_srvs.srv import *
from geometry_msgs.msg import *
from nao_interaction_msgs.srv import *
from deictic_gestures.srv import *

LOOK_AT_MAX_SPEED = 0.6

class LookAtSrv(object):
    def __init__(self):
        rospy.loginfo("waiting for service /naoqi_driver/tracker/look_at")
        rospy.wait_for_service("/naoqi_driver/tracker/look_at")
        rospy.loginfo("waiting for service /naoqi_driver/tracker/stop_tracker")
        rospy.wait_for_service("/naoqi_driver/tracker/stop_tracker")
        self.services_proxy = {
            "look_at": rospy.ServiceProxy("naoqi_driver/tracker/look_at", TrackerLookAt),
            "stop_tracker": rospy.ServiceProxy("/naoqi_driver/tracker/stop_tracker", Empty)}

        self.services = {"look_at": rospy.Service('/deictic_gestures/look_at', LookAt,
                                                   self.handle_look_at)}

        self.tfListener = tf.TransformListener()
        self.parameters = {"fixed_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint"),
                           "look_at_max_speed": rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)}

        self.publishers = {
            "result_point": rospy.Publisher('/deictic_gestures/looking_point', geometry_msgs.msg.PointStamped, queue_size=5)}

    def handle_look_at(self, req):
        # First version using naoqi
        self.parameters["look_at_max_speed"] = rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)
        try:
            self.tfListener.waitForTransform("/torso", req.point.header.frame_id, rospy.Time(0), rospy.Duration(2.0))
            (translation, rotation) = self.tfListener.lookupTransform('/torso', req.point.header.frame_id, rospy.Time(0))
            req.point.point.x += translation[0]
            req.point.point.y += translation[1]
            req.point.point.z += translation[2]
            self.services_proxy["stop_tracker"]()
            target = Point(req.point.point.x, req.point.point.y, req.point.point.z)
            self.services_proxy["look_at"](target, 0, LOOK_AT_MAX_SPEED, 0)
            self.publishers["result_point"].publish(req.point)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[look_at_srv] Exception occurred :" + str(e))
            return False

if __name__ == '__main__':
    rospy.init_node('look_at_srv')
    srv = LookAtSrv()
    rospy.spin()
    exit(0)
