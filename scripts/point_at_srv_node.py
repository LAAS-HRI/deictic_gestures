#!/usr/bin/env python

import rospy
import tf
from std_srvs.srv import *
from pointing_srv.srv import *
from geometry_msgs.msg import *
from nao_interaction_msgs.srv import *
from deictic_gestures.srv import *

POINT_AT_MAX_SPEED = 0.6

class PointAtSrv(object):
    def __init__(self):
        rospy.loginfo("waiting for service /naoqi_driver/tracker/point_at")
        rospy.wait_for_service("/naoqi_driver/tracker/point_at")
        self.services_proxy = {
            "point_at": rospy.ServiceProxy("naoqi_driver/tracker/point_at", nao_interaction_msgs.srv.PointAt)}

        self.services = {"point_at": rospy.Service('/deictic_gestures/point_at', deictic_gestures.srv.PointAt,
                                                   self.handle_point_at)}

        self.tfListener = tf.TransformListener()
        self.parameters = {"fixed_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint"),
                           "point_at_max_speed": rospy.get_param("point_at_max_speed", POINT_AT_MAX_SPEED)}

        self.publishers = {
            "result_point": rospy.Publisher('/deictic_gestures/pointing_point', geometry_msgs.msg.PointStamped, queue_size=5)}

    def handle_point_at(self, req):
        # First version using naoqi
        self.parameters["point_at_max_speed"] = rospy.get_param("point_at_max_speed", POINT_AT_MAX_SPEED)
        try:
            self.tfListener.waitForTransform("/torso", req.target.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            (translation, rotation) = self.tfListener.lookupTransform('/torso', req.target.header.frame_id,
                                                                      rospy.Time(0))
            req.target.point.position.x += translation[0]
            req.target.point.position.y += translation[1]
            req.target.point.position.z += translation[2]
            effector = "LArm" if req.point.position.y > 0.0 else "RArm"
            self.services_proxy["point_at"](effector, [req.target.point.position.x, req.target.point.position.y,
                                                       req.target.point.position.z], 0, POINT_AT_MAX_SPPED)
            self.publishers["result_point"].publish(req.target)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[point_at_srv] Exception occured :" + str(e))
            return False


if __name__ == '__main__':
    srv = PointAtSrv()
    rospy.spin()
    exit(0)
