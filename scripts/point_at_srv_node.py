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

        self.log_pub = {"isPointingAt": rospy.Publisher("predicates_log/pointingat", String, queue_size=5),
                        "isMoving": rospy.Publisher("predicates_log/moving", String, queue_size=5)}

        self.current_situations_map = {}

    def start_predicate(self, timeline, predicate, subject_name, object_name=None, isevent=False):
        if object_name is None:
            description = predicate + "(" + subject_name + ")"
        else:
            description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub[predicate].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_predicate(self, timeline, predicate, subject_name, object_name=None):
        if object_name is None:
            description = predicate + "(" + subject_name + ")"
        else:
            description = predicate + "(" + subject_name + "," + object_name + ")"
        try:
            sit = self.current_situations_map[description]
            self.log_pub[predicate].publish("END " + description)
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[point_at_srv] Exception occurred : " + str(e))

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
            self.start_predicate(self.world.timeline, "isMoving", "robot")
            self.start_predicate(self.world.timeline, "isPointingAt", "robot", object_name=req.point.header.frame_id)
            self.services_proxy["point_at"](effector, target, 0, POINT_AT_MAX_SPEED)
            self.end_predicate(self.world.timeline, "isPointingAt", "robot", object_name=req.point.header.frame_id)
            self.end_predicate(self.world.timeline, "isMoving", "robot")
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
