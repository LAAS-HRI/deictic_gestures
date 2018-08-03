#!/usr/bin/env python

import time
import rospy
import tf
import sys
import argparse
import math
import numpy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from naoqi import ALProxy
from deictic_gestures.srv import LookAt, CanLookAt
from std_srvs.srv import SetBool
import underworlds
from underworlds.types import Situation
from underworlds.helpers.transformations import translation_matrix, quaternion_matrix

LOOK_AT_MAX_SPEED = 0.17
LOOK_AT_SPEED = 0.06
LOOK_AT_MAX_ANGLE = 90
MIN_DIST_MOVE = 0.1

def transformation_matrix(t, q):
    translation_mat = translation_matrix(t)
    rotation_mat = quaternion_matrix(q)
    return numpy.dot(translation_mat, rotation_mat)


class LookAtSrv(object):
    def __init__(self, ctx, world, nao_ip, nao_port):
        self.world = ctx.worlds[world]
        self.nao_ip = nao_ip
        self.nao_port = nao_port
        self.tracker = ALProxy("ALTracker", nao_ip, nao_port)
        self.motion = ALProxy("ALMotion", nao_ip, nao_port)

        self.services = {"look_at": rospy.Service('/deictic_gestures/look_at', LookAt, self.handle_look_at),
                         "can_look_at": rospy.Service('/deictic_gestures/can_look_at', CanLookAt, self.handle_can_look_at)}

        self.services_proxy = {"enable_monitoring": rospy.ServiceProxy('multimodal_human_monitor/global_monitoring', SetBool)}

        self.tfListener = tf.TransformListener()
        self.parameters = {"fixed_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint"),
                           "look_at_max_speed": rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)}

        self.publishers = {
            "result_point": rospy.Publisher('/deictic_gestures/looking_point', PointStamped, queue_size=5)}

        self.log_pub = {"isLookingAt": rospy.Publisher("predicates_log/lookingat", String, queue_size=5),
                        "isMoving": rospy.Publisher("predicates_log/moving", String, queue_size=5)}

        self.current_situations_map = {}

        self.current_lookat_frame = None
        self.current_lookat_point = None

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
            rospy.logwarn("[look_at_srv] Exception occurred : " + str(e))

    def distance(self, point):
        x = point[0]-self.current_lookat_point[0]
        y = point[1]-self.current_lookat_point[1]
        z = point[2]-self.current_lookat_point[2]
        return math.sqrt(x*x+y*y+z*z)

    def handle_can_look_at(self, req):
        try:
            if self.tfListener.canTransform("/torso", req.point.header.frame_id, rospy.Time()):
                (translation, rotation) = self.tfListener.lookupTransform('/torso', req.point.header.frame_id, rospy.Time())
                t = transformation_matrix(translation, rotation)
                p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
                new_p = numpy.dot(t, p)
                angle = math.atan2(new_p[1, 0], new_p[0, 0])
                if math.degrees(math.fabs(angle)) > LOOK_AT_MAX_ANGLE:
                    if angle > 0:
                        angle += 0.3
                    else:
                        angle -= 0.3
                    return False, angle
                else:
                    return True, 0
        except Exception as e:
            rospy.logerr("[look_at_srv] Exception occurred :" + str(e))
            return False, 0

    def handle_look_at(self, req, ):
        # First version using naoqi
        self.parameters["look_at_max_speed"] = rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)
        try:
            if self.tfListener.canTransform("/torso", req.point.header.frame_id, rospy.Time()):
                (translation, rotation) = self.tfListener.lookupTransform('/torso', req.point.header.frame_id, rospy.Time())
                t = transformation_matrix(translation, rotation)
                p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
                new_p = numpy.dot(t, p)
                to_move = False
                dist = None
                speed_up = False
                if self.current_lookat_point:
                    dist = self.distance([new_p[1, 0], new_p[2, 0], new_p[3, 0]])
                    if dist > MIN_DIST_MOVE or self.current_lookat_frame != req.point.header.frame_id:
                        to_move = True

                else:
                    to_move = True
                if dist is not None:
                    speed_up = True if dist > 0.35 else False

                if to_move:
                    (translation, rotation) = self.tfListener.lookupTransform('/torso', req.point.header.frame_id,
                                                                              rospy.Time())
                    t = transformation_matrix(translation, rotation)
                    p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
                    new_p = numpy.dot(t, p)
                    if req.point.header.frame_id != self.current_lookat_frame:
                        if self.current_lookat_frame is not None:
                            self.end_predicate(self.world.timeline, "isLookingAt", "robot",
                                               object_name=self.current_lookat_frame)
                        self.start_predicate(self.world.timeline, "isLookingAt", "robot",
                                             object_name=req.point.header.frame_id)
                        self.current_lookat_frame = req.point.header.frame_id
                    self.start_predicate(self.world.timeline, "isMoving", "robot")
                    #self.tracker.stopTracker()
                    #self.publishers["result_point"].publish()
                    #rospy.logwarn("lookat")

                    if speed_up :
                        look_at_speed = LOOK_AT_MAX_SPEED
                    else:
                        look_at_speed = LOOK_AT_SPEED
                    self.current_lookat_point = [new_p[1, 0], new_p[2, 0], new_p[3, 0]]
                    try:
                        self.services_proxy["enable_monitoring"](False)
                        self.tracker.lookAt([new_p[0, 0], new_p[1, 0], new_p[2, 0]], look_at_speed, False)
                        time.sleep(0.1)
                        self.services_proxy["enable_monitoring"](True)
                    except Exception:
                        self.tracker = ALProxy("ALTracker", self.nao_ip, self.nao_port)
                        self.services_proxy["enable_monitoring"](False)
                        self.tracker.lookAt([new_p[0, 0], new_p[1, 0], new_p[2, 0]], look_at_speed, False)
                        time.sleep(0.1)
                        self.services_proxy["enable_monitoring"](True)
                    self.end_predicate(self.world.timeline, "isMoving", "robot")
                return True
            else:
                if self.current_lookat_frame is not None:
                    if req.point.header.frame_id == self.current_lookat_frame:
                        self.end_predicate(self.world.timeline, "isLookingAt", "robot", object_name=self.current_lookat_frame)
                        self.current_lookat_frame = None
                return False
        except Exception as e:
            rospy.logerr("[look_at_srv] Exception occurred :" + str(e))
            return False

if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description="Handle look at")
    parser.add_argument("world", help="The world where to write the situation associated to moving")
    parser.add_argument("--nao_ip", default="mummer-eth0.laas.fr", help="The robot IP")
    parser.add_argument("--nao_port", default="9559", help="The robot port")
    args = parser.parse_args()

    rospy.init_node('look_at_srv')
    with underworlds.Context("look_at_srv") as ctx:  # Here we connect to the server
        LookAtSrv(ctx, args.world, args.nao_ip, int(args.nao_port))
        rospy.spin()
        exit(0)
