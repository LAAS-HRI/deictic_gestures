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
from pepper_resources_synchronizer_msgs.srv import MetaStateMachineRegister, MetaStateMachineRegisterRequest
from pepper_resources_synchronizer_msgs.msg import SubStateMachine_pepper_arm_manager_msgs, SubStateMachine_pepper_head_manager_msgs
from pepper_arm_manager_msgs.msg import StateMachineStatePrioritizedPoint as ArmStatePoint
from pepper_head_manager_msgs.msg import StateMachineStatePrioritizedPoint as HeadStatePoint
from resource_management_msgs.msg import StateMachineTransition, MessagePriority
from naoqi import ALProxy
from tf.transformations import translation_matrix, quaternion_matrix, euler_from_quaternion, quaternion_from_euler

POINT_AT_MAX_SPEED = 0.7

def transformation_matrix(t, q):
    translation_mat = translation_matrix(t)
    rotation_mat = quaternion_matrix(q)
    return numpy.dot(translation_mat, rotation_mat)

class PointAtSrv(object):
    def __init__(self, ctx, world, nao_ip, nao_port):

        self.services = {"point_at": rospy.Service('/deictic_gestures/point_at', PointAt, self.handle_point_at),
                         "can_point_at": rospy.Service('/deictic_gestures/can_point_at', CanLookAt, self.handle_can_point_at)}

        self.tfListener = tf.TransformListener()
        self.parameters = {"fixed_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint"),
                           "point_at_max_speed": rospy.get_param("point_at_max_speed", POINT_AT_MAX_SPEED)}

        self.publishers = {
            "result_point": rospy.Publisher('/deictic_gestures/pointing_point_result', PointStamped, queue_size=5),
            "input_point": rospy.Publisher('/deictic_gestures/pointing_point_input', PointStamped, queue_size=5)}

        self.resource_synchronizer = rospy.ServiceProxy("/pepper_resources_synchronizer/state_machines_register", MetaStateMachineRegister)



    # def start_predicate(self, timeline, predicate, subject_name, object_name=None, isevent=False):
    #     if object_name is None:
    #         description = predicate + "(" + subject_name + ")"
    #     else:
    #         description = predicate + "(" + subject_name + "," + object_name + ")"
    #     sit = Situation(desc=description)
    #     sit.starttime = time.time()
    #     if isevent:
    #         sit.endtime = sit.starttime
    #     self.current_situations_map[description] = sit
    #     timeline.update(sit)
    #     self.log_pub[predicate].publish("START " + description)
    #     return sit.id
    #
    # def end_predicate(self, timeline, predicate, subject_name, object_name=None):
    #     if object_name is None:
    #         description = predicate + "(" + subject_name + ")"
    #     else:
    #         description = predicate + "(" + subject_name + "," + object_name + ")"
    #     try:
    #         sit = self.current_situations_map[description]
    #         timeline.end(sit)
    #         self.log_pub[predicate].publish("END " + description)
    #     except Exception as e:
    #         rospy.logwarn("[point_at_srv] Exception occurred : " + str(e))

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
        rospy.loginfo("Plop")
        self.parameters["point_at_max_speed"] = rospy.get_param("point_at_max_speed", POINT_AT_MAX_SPEED)
        try:
            self.publishers["input_point"].publish(req.point)
            if self.tfListener.canTransform("/base_link", req.point.header.frame_id, rospy.Time()):
                (translation, rotation) = self.tfListener.lookupTransform("/base_link", req.point.header.frame_id,
                                                                          rospy.Time())
                t = transformation_matrix(translation, rotation)
                p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
                new_p = numpy.dot(t, p)
                yaw = math.atan2(new_p[1], new_p[0])

                if abs(yaw) > math.pi / 2 and not req.with_base:
                    rot = yaw - math.pi / 2 if yaw > math.pi / 2 else yaw + math.pi / 2
                    rospy.loginfo("Out of bounds", rot)
                    return False

                effector = "LArm" if new_p[1, 0] > 0.0 else "RArm"
                r = MetaStateMachineRegisterRequest()
                r.header.timeout = rospy.Duration(-1)
                r.header.begin_dead_line = rospy.Time.now() + rospy.Duration(5)
                r.header.priority.value = MessagePriority.URGENT
                if not req.with_head:
                    arm_fsm = self.fill_arm_fsm(new_p)
                    if new_p[1, 0] > 0.0:
                        r.state_machine_pepper_arm_manager_left = arm_fsm
                    else:
                        r.state_machine_pepper_arm_manager_right = arm_fsm
                else:
                    arm_fsm, head_fsm = self.fill_arm_fsm_with_head(new_p)
                    if new_p[1, 0] > 0.0:
                        r.state_machine_pepper_arm_manager_left = arm_fsm
                    else:
                        r.state_machine_pepper_arm_manager_right = arm_fsm
                    r.state_machine_pepper_head_manager = head_fsm
                ret = self.resource_synchronizer.call(r)
                #rospy.loginfo("Plop", ret)
                return True
            return False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[point_at_srv] Exception occured :" + str(e))
            return False

    def fill_arm_fsm(self, point):
        pointing_arm = SubStateMachine_pepper_arm_manager_msgs()
        pointing_arm.header.begin_dead_line = rospy.Time.now() + rospy.Duration(5)
        pointing_arm.header.initial_state = "point"
        pointing_arm.header.timeout = rospy.Duration(-1)
        p1 = ArmStatePoint()
        p1.header.id = "point"
        p1.data.header.frame_id = "/base_link"
        p1.data.header.stamp = rospy.Time.now()
        p1.data.point.x, p1.data.point.y, p1.data.point.z = point[0, 0], point[1, 0], point[2, 0]
        t1 = StateMachineTransition()
        t1.next_state = "end"
        t1.end_condition.timeout = rospy.Duration(-1)
        t1.end_condition.duration = rospy.Duration(5)
        p1.header.transitions.append(t1)
        pointing_arm.state_machine.states_PrioritizedPoint.append(p1)
        return pointing_arm

    def fill_arm_fsm_with_head(self, point):
        arm = SubStateMachine_pepper_arm_manager_msgs()
        head = SubStateMachine_pepper_head_manager_msgs()
        arm.header.begin_dead_line = rospy.Time().now() + rospy.Duration(5)
        head.header.begin_dead_line = rospy.Time().now() + rospy.Duration(5)
        arm.header.initial_state = "_arm_initial_synchro"
        head.header.initial_state = "_head_initial_synchro"
        arm.header.timeout = rospy.Duration(-1)
        head.header.timeout = rospy.Duration(-1)

        t_arm_init = StateMachineTransition()
        t_arm_init.end_condition.regex_end_condition.append("__synchro__begin")
        t_head_init = StateMachineTransition()
        t_head_init.end_condition.regex_end_condition.append("__synchro__begin")
        t_head_init.end_condition.timeout = rospy.Duration(5)
        t_arm_init.end_condition.timeout = rospy.Duration(5)
        t_head_init.end_condition.duration = rospy.Duration(-1)
        t_arm_init.end_condition.duration = rospy.Duration(-1)
        t_head_init.next_state = "point_head"
        t_arm_init.next_state = "point_arm"

        arm_sync = ArmStatePoint()
        head_sync = HeadStatePoint()
        arm_sync.header.id = "_arm_initial_synchro"
        head_sync.header.id = "_head_initial_synchro"
        arm_sync.header.transitions.append(t_arm_init)
        head_sync.header.transitions.append(t_head_init)

        arm_point = ArmStatePoint()
        head_point = HeadStatePoint()
        arm_point.header.id = "point_arm"
        arm_point.data.header.frame_id = "/base_link"
        arm_point.data.header.stamp = rospy.Time.now()
        arm_point.data.point.x, arm_point.data.point.y, arm_point.data.point.z = point[0, 0], point[1, 0], point[2, 0]
        head_point.header.id = "point_head"
        head_point.data.header.stamp = rospy.Time.now()
        head_point.data.header.frame_id = "/base_link"
        head_point.data.point.x, head_point.data.point.y, head_point.data.point.z = point[0, 0], point[1, 0], point[2, 0]

        t_arm_point = StateMachineTransition()
        t_head_point = StateMachineTransition()
        t_arm_point.next_state = "end"
        t_head_point.next_state = "end"
        t_arm_point.end_condition.timeout = rospy.Duration(-1)
        t_head_point.end_condition.timeout = rospy.Duration(-1)
        t_arm_point.end_condition.duration = rospy.Duration(5)
        t_head_point.end_condition.duration = rospy.Duration(5)
        head_point.header.transitions.append(t_head_point)
        arm_point.header.transitions.append(t_arm_point)

        arm.state_machine.states_PrioritizedPoint.append(arm_sync)
        arm.state_machine.states_PrioritizedPoint.append(arm_point)
        head.state_machine.states_PrioritizedPoint.append(head_sync)
        head.state_machine.states_PrioritizedPoint.append(head_point)

        return arm, head



if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)


    rospy.init_node('point_at_srv')
    PointAtSrv(None, None, None, None)
    rospy.spin()
    exit(0)
