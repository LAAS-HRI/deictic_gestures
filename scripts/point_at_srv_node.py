#!/usr/bin/env python

import rospy
import tf
import sys
import time
import argparse
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped, Point
from nao_interaction_msgs.srv import TrackerPointAt
from deictic_gestures.srv import PointAt
import underworlds
from underworlds.types import Situation

POINT_AT_MAX_SPEED = 0.6

class PointAtSrv(object):
    def __init__(self, ctx, world):
        self.world = ctx.worlds[world]
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
            "result_point": rospy.Publisher('/deictic_gestures/pointing_point', PointStamped, queue_size=5)}

        self.log_pub = {"situation_log": rospy.Publisher("look_at_srv/log", String, queue_size=5)}

        self.current_situations_map = {}

    def start_n1_situation(self, timeline, predicate, subject_name, isevent=False):
        description = predicate + "(" + subject_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub["situation_log"].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_n1_situation(self, timeline, predicate, subject_name):
        description = predicate + "(" + subject_name + ")"
        sit = self.current_situations_map[description]
        self.log_pub["situation_log"].publish("END " + description)
        try:
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[robot_monitor] Exception occurred : " + str(e))

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
            self.start_n1_situation(self.world.timeline, "moving", "robot")
            self.services_proxy["point_at"](effector, target, 0, POINT_AT_MAX_SPEED)
            self.end_n1_situation(self.world.timeline, "moving", "robot")
            self.publishers["result_point"].publish(req.point)
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[point_at_srv] Exception occured :" + str(e))
            return False


if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description="Handle look at")
    parser.add_argument("world", help="The world where to write the situation associated to moving")
    args = parser.parse_args()

    rospy.init_node('point_at_srv')
    with underworlds.Context("uwds_database_ros_bridge") as ctx:  # Here we connect to the server
        PointAtSrv(ctx, args.world)
        rospy.spin()
        exit(0)
