#!/usr/bin/env python

import time
import rospy
import tf
import sys
import argparse
import numpy
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, PointStamped
from nao_interaction_msgs.srv import TrackerLookAt
from deictic_gestures.srv import LookAt
import underworlds
from underworlds.types import Situation
from underworlds.helpers.transformations import translation_matrix, quaternion_matrix

LOOK_AT_MAX_SPEED = 0.7

def transformation_matrix(t, q):
    translation_mat = translation_matrix(t)
    rotation_mat = quaternion_matrix(q)
    return numpy.dot(translation_mat, rotation_mat)


class LookAtSrv(object):
    def __init__(self, ctx, world):
        self.world = ctx.worlds[world]
        rospy.loginfo("waiting for service /naoqi_driver/tracker/look_at")
        rospy.wait_for_service("/naoqi_driver/tracker/look_at")
        rospy.loginfo("waiting for service /naoqi_driver/tracker/stop_tracker")
        rospy.wait_for_service("/naoqi_driver/tracker/stop_tracker")
        self.services_proxy = {
            "look_at": rospy.ServiceProxy("naoqi_driver/tracker/look_at", TrackerLookAt),
            "stop_tracker": rospy.ServiceProxy("/naoqi_driver/tracker/stop_tracker", Empty)}

        self.services = {"look_at": rospy.Service('/deictic_gestures/look_at', LookAt, self.handle_look_at)}

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

    def handle_look_at(self, req):
        # First version using naoqi
        self.parameters["look_at_max_speed"] = rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)
        try:
            #self.tfListener.waitForTransform("/base_footprint", req.point.header.frame_id, rospy.Time(0), rospy.Duration(0.3))
            if self.tfListener.canTransform("/torso",req.point.header.frame_id, rospy.Time()):
                (translation, rotation) = self.tfListener.lookupTransform('/torso', req.point.header.frame_id, rospy.Time(0))
                #self.publishers["result_point"].publish(req.point)

                t = transformation_matrix(translation, rotation)
                #rospy.logwarn(t)

                p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()

                #rospy.logwarn(p)
                new_p = numpy.dot(t, p)

                #rospy.logwarn(new_p)

                self.services_proxy["stop_tracker"]()
                target = Point(new_p[0, 0], new_p[1, 0], new_p[2, 0])
                self.start_predicate(self.world.timeline, "isMoving", "robot")
                if req.point.header.frame_id != self.current_lookat_frame:
                    if self.current_lookat_frame is not None:
                        self.end_predicate(self.world.timeline, "isLookingAt", "robot", object_name=self.current_lookat_frame)
                    self.start_predicate(self.world.timeline, "isLookingAt", "robot", object_name=req.point.header.frame_id)
                    self.current_lookat_frame = req.point.header.frame_id
                self.services_proxy["look_at"](target, 0, LOOK_AT_MAX_SPEED, False)
                #self.end_predicate(self.world.timeline, "isLookingAt", "robot", object_name=req.point.header.frame_id)
                self.end_predicate(self.world.timeline, "isMoving", "robot")
                return True
            else:
                if self.current_lookat_frame is not None:
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
    args = parser.parse_args()

    rospy.init_node('look_at_srv')
    with underworlds.Context("look_at_srv") as ctx:  # Here we connect to the server
        LookAtSrv(ctx, args.world)
        rospy.spin()
        exit(0)
