#!/usr/bin/env python

import time
import rospy
import tf
import sys
import argparse
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, PointStamped
from nao_interaction_msgs.srv import TrackerLookAt
from deictic_gestures.srv import LookAt
import underworlds
from underworlds.types import Situation

LOOK_AT_MAX_SPEED = 0.6

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

        self.log_pub = {"situation_log": rospy.Publisher("look_at_srv/log", String, queue_size=5)}

        self.current_situations_map = {}

    def start_n2_situation(self, timeline, predicate, subject_name, object_name, isevent=False):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub["situation_log"].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_n2_situation(self, timeline, predicate, subject_name, object_name):
        description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = self.current_situations_map[description]
        self.log_pub["situation_log"].publish("END " + description)
        try:
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[point_at_srv] Exception occurred : " + str(e))

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
            self.publishers["result_point"].publish(req.point)
            target = Point(req.point.point.x, req.point.point.y, req.point.point.z)
            self.start_n2_situation(self.world.timeline, "isMovingFrom", "robot", "map")
            self.services_proxy["look_at"](target, 0, LOOK_AT_MAX_SPEED, 0)
            self.end_n2_situation(self.world.timeline, "isMovingFrom", "robot", "map")
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
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
