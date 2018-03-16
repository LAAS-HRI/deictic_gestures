#!/usr/bin/env python

import rospy
import tf
from std_srvs.srv import *
from pointing_srv.srv import *
from geometry_msgs.msg import *
from nao_interaction_msgs.srv import *
from deictic_gestures.srv import *

LOOK_AT_MAX_SPEED = 0.6

class PointAtSrv(object):
    def __init__(self):
        rospy.loginfo("waiting for service /naoqi_driver/tracker/look_at")
        rospy.wait_for_service("/naoqi_driver/tracker/look_at")
        rospy.loginfo("waiting for service /naoqi_driver/tracker/stop_tracker")
        rospy.wait_for_service("/naoqi_driver/tracker/stop_tracker")
        self.services_proxy = {
            "look_at": rospy.ServiceProxy("naoqi_driver/tracker/look_at", nao_interaction_msgs.srv.PointAt),
            "stop_tracker": rospy.ServiceProxy("/naoqi_driver/tracker/stop_tracker", std_srvs.srv.Empty)}

        self.services = {"point_at": rospy.Service('/deictic_gestures/look_at', deictic_gestures.srv.PointAt,
                                                   self.handle_look_at)}

        self.tfListener = tf.TransformListener()
        self.parameters = {"fixed_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint"),
                           "look_at_max_speed" : rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)}

        self.publishers = {
            "result_point": rospy.Publisher('/deictic_gestures/looking_point', geometry_msgs.msg.PointStamped, queue_size=5)}

    def handle_look_at(self, req):
        # First version using naoqi
        self.parameters["look_at_max_speed"] = rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)
        try:
            self.tfListener.waitForTransform("/torso", req.header.frame_id, rospy.Time(0), rospy.Duration(2.0))
            (translation, rotation) = self.tfListener.lookupTransform('/torso', req.header.frame_id, rospy.Time(0))
            req.target.point.position.x += translation[0]
            req.target.point.position.y += translation[1]
            req.target.point.position.z += translation[2]
            self.services_proxy["stop_tracker"]()
            self.services_proxy["point_at"]([req.point.position.x, req.point.position.y, req.point.position.z], 0, LOOK_AT_MAX_SPEED, 0)
            self.publishers["result_point"].publish(req.target)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[point_at_srv] Exception occured :" + str(e))
            return False

if __name__ == '__main__':
    srv = PointAtSrv()
    rospy.spin()
    exit(0)
