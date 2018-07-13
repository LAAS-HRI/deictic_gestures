#!/usr/bin/env python

import rospy
import tf
import sys
import numpy
import time
import math
import argparse
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from deictic_gestures.srv import PointAt, CanLookAt
from naoqi import ALProxy
import underworlds
from underworlds.types import Situation
from tf.transformations import translation_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler

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
        self.motion = ALProxy("ALMotion", nao_ip, nao_port)

        self.services = {"point_at": rospy.Service('/deictic_gestures/point_at', PointAt, self.handle_point_at),
                         "can_point_at": rospy.Service('/deictic_gestures/can_point_at', CanLookAt, self.handle_can_point_at)}

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
        timeline.update(sit)
        self.log_pub[predicate].publish("START " + description)
        return sit.id

    def end_predicate(self, timeline, predicate, subject_name, object_name=None):
        if object_name is None:
            description = predicate + "(" + subject_name + ")"
        else:
            description = predicate + "(" + subject_name + "," + object_name + ")"
        try:
            sit = self.current_situations_map[description]
            timeline.end(sit)
            self.log_pub[predicate].publish("END " + description)
        except Exception as e:
            rospy.logwarn("[point_at_srv] Exception occurred : " + str(e))

    def handle_can_point_at(self, req):
        if self.tfListener.canTransform("/torso", req.point.header.frame_id, rospy.Time()):
            (translation, rotation) = self.tfListener.lookupTransform("/base_link", req.point.header.frame_id,
                                                                      rospy.Time())
            t = transformation_matrix(translation, rotation)
            p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
            new_p = numpy.dot(t, p)

            yaw = math.atan2(new_p[1], new_p[0])
            if abs(yaw) > math.pi / 2:
                rot = yaw - math.pi / 2 if yaw > math.pi / 2 else yaw + math.pi / 2
                if rot> 0 : rot += 0.1
                else: rot -= 0.1
                return False, rot
            else:
                return True, 0

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
                yaw = math.atan2(new_p[1], new_p[0])

                if abs(yaw) > math.pi / 2:
                    rot = yaw - math.pi / 2 if yaw > math.pi / 2 else yaw + math.pi / 2
                    return False

                effector = "LArm" if new_p[1, 0] > 0.0 else "RArm"
                wrist_effector = "LWristYaw" if new_p[1, 0] > 0.0 else "RWristYaw"
                self.start_predicate(self.world.timeline, "isMoving", "robot")
                self.start_predicate(self.world.timeline, "isPointingAt", "robot", object_name=req.point.header.frame_id)
                try:
                    self.tracker.pointAt(effector,[new_p[0, 0], new_p[1, 0], new_p[2, 0]], 0, POINT_AT_MAX_SPEED)
                except Exception:
                    self.tracker = ALProxy("ALTracker", self.nao_ip, self.nao_port)
                    self.tracker.pointAt(effector, [new_p[0, 0], new_p[1, 0], new_p[2, 0]], 0, POINT_AT_MAX_SPEED)

                wrist_angle = -1.57 if wrist_effector == "LWristYaw" else 1.57
                try:
                    self.motion.setAngles([wrist_effector], [wrist_angle])
                except Exception:
                    self.motion = ALProxy("ALMotion", self.nao_ip, self.nao_port)
                    self.motion.setAngles([wrist_effector], [wrist_angle])
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
