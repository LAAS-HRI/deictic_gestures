#!/usr/bin/env python

import rospy
import tf
from std_srvs.srv import *
from geometry_msgs.msg import *
from nao_interaction_msgs.srv import *
from deictic_gestures.srv import *

POINT_AT_MAX_SPEED = 0.6

class PointAtSrv(object):
    def __init__(self):
        rospy.loginfo("waiting for service /naoqi_driver/tracker/point_at")
        rospy.wait_for_service("/naoqi_driver/tracker/point_at")
        self.services_proxy = {
            "point_at": rospy.ServiceProxy("naoqi_driver/tracker/point_at", TrackerPointAt)}

        self.services = {"point_at": rospy.Service('/deictic_gestures/point_at', PointAt,
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
            self.tfListener.waitForTransform("/torso", req.point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            (translation, rotation) = self.tfListener.lookupTransform('/torso', req.point.header.frame_id,
                                                                      rospy.Time(0))
            req.point.point.x += translation[0]
            req.point.point.y += translation[1]
            req.point.point.z += translation[2]
            effector = "LArm" if req.point.point.y > 0.0 else "RArm"
            target = Point(req.point.point.x, req.point.point.y, req.point.point.z)
            self.services_proxy["point_at"](effector, target, 0, POINT_AT_MAX_SPEED)
            self.publishers["result_point"].publish(req.point)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[point_at_srv] Exception occured :" + str(e))
            return False


if __name__ == '__main__':
    rospy.init_node('point_at_srv')
    srv = PointAtSrv()
    rospy.spin()
    exit(0)
