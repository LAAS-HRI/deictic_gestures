#!/usr/bin/env python

import rospy
import tf
import sys
import numpy
import time
import argparse
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from deictic_gestures.srv import PointAt
from naoqi import ALProxy
import underworlds
from underworlds.types import Situation
from underworlds.helpers.transformations import translation_matrix, quaternion_matrix

POINT_AT_MAX_SPEED = 0.7


def transformation_matrix(t, q):
    translation_mat = translation_matrix(t)
    rotation_mat = quaternion_matrix(q)
    return numpy.dot(translation_mat, rotation_mat)

class PointAtSrv(object):
    def __init__(self, ctx, world, nao_ip, nao_port):
        self.world = ctx.worlds[world]
        self.nao_ip = nao_ip
        self.nao_port = nao_port
        self.tracker = ALProxy("ALTracker", nao_ip, nao_port)

        self.services = {"point_at": rospy.Service('/deictic_gestures/point_at', PointAt,
                                                   self.handle_point_at)}

        self.tfListener = tf.TransformListener()
        self.parameters = {"fixed_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint"),
                           "point_at_max_speed": rospy.get_param("point_at_max_speed", POINT_AT_MAX_SPEED)}

        self.publishers = {
            "result_point": rospy.Publisher('/deictic_gestures/pointing_point_result', PointStamped, queue_size=5),
            "input_point": rospy.Publisher('/deictic_gestures/pointing_point_input', PointStamped, queue_size=5)}

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
            self.publishers["input_point"].publish(req.point)
            if self.tfListener.canTransform("/torso", req.point.header.frame_id, rospy.Time()):
                (translation, rotation) = self.tfListener.lookupTransform("/base_link", req.point.header.frame_id,
                                                                          rospy.Time())
                t = transformation_matrix(translation, rotation)
                p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
                new_p = numpy.dot(t, p)
                effector = "LArm" if new_p[1, 0] > 0.0 else "RArm"
                self.start_predicate(self.world.timeline, "isMoving", "robot")
                self.start_predicate(self.world.timeline, "isPointingAt", "robot", object_name=req.point.header.frame_id)
                try:
                    self.tracker.pointAt(effector,[new_p[0, 0], new_p[1, 0], new_p[2, 0]], 0, POINT_AT_MAX_SPEED)
                except Exception:
                    self.tracker = ALProxy("ALTracker", self.nao_ip, self.nao_port)
                    self.tracker.pointAt(effector, [new_p[0, 0], new_p[1, 0], new_p[2, 0]], 0, POINT_AT_MAX_SPEED)
                self.end_predicate(self.world.timeline, "isPointingAt", "robot", object_name=req.point.header.frame_id)
                self.end_predicate(self.world.timeline, "isMoving", "robot")
                return True
            return False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[point_at_srv] Exception occured :" + str(e))
            return False


if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description="Handle look at")
    parser.add_argument("world", help="The world where to write the situation associated to moving")
    parser.add_argument("--nao_ip", default="mummer-eth0.laas.fr", help="The robot IP")
    parser.add_argument("--nao_port", default="9559", help="The robot port")
    args = parser.parse_args()

    rospy.init_node('point_at_srv')
    with underworlds.Context("point_at_srv") as ctx:  # Here we connect to the server
        PointAtSrv(ctx, args.world, args.nao_ip, int(args.nao_port))
        rospy.spin()
        exit(0)
