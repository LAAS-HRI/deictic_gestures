#!/usr/bin/env python

import rospy
import tf
import sys
import math
import numpy
from geometry_msgs.msg import PointStamped
from deictic_gestures.srv import LookAt, CanLookAt
from deictic_gestures.msg import LookAtStatus
from pepper_resources_synchronizer_msgs.srv import MetaStateMachineRegister, MetaStateMachineRegisterRequest
from pepper_resources_synchronizer_msgs.msg import SubStateMachine_pepper_arm_manager_msgs, SubStateMachine_pepper_head_manager_msgs, SubStateMachine_pepper_base_manager_msgs
from pepper_head_manager_msgs.msg import StateMachineStatePrioritizedPoint as HeadStatePoint
from pepper_base_manager_msgs.msg import StateMachineStatePrioritizedAngle as BaseStateAngle
from resource_management_msgs.msg import StateMachineTransition, MessagePriority
from resource_synchronizer_msgs.msg import MetaStateMachinesStatus


from tf.transformations import translation_matrix, quaternion_matrix, quaternion_from_euler

LOOK_AT_MAX_SPEED = 0.3
LOOK_AT_SPEED = 0.1
LOOK_AT_MAX_ANGLE = math.radians(80)
MIN_DIST_MOVE = 0.1

HEAD_MANAGER_NAMES = ["pepper_head_manager"]
BASE_MANAGER_NAMES = ["pepper_base_manager"]


def center_radians(a):
    while a >= math.pi:
        a -= 2*math.pi
    while a < -math.pi:
        a += 2*math.pi
    return a


def transformation_matrix(t, q):
    translation_mat = translation_matrix(t)
    rotation_mat = quaternion_matrix(q)
    return numpy.dot(translation_mat, rotation_mat)


class LookAtSrv(object):
    def __init__(self):

        self.services = {"look_at": rospy.Service('/deictic_gestures/look_at', LookAt, self.handle_look_at),
                         "can_look_at": rospy.Service('/deictic_gestures/can_look_at', CanLookAt, self.handle_can_look_at)}

        self.tfListener = tf.TransformListener()
        self.parameters = {"fixed_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint"),
                           "look_at_max_speed": rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)}

        self.publishers = {
            "result_point": rospy.Publisher('/deictic_gestures/looking_point', PointStamped, queue_size=5)}

        self.pointing_ids = []
        self.current_state = LookAtStatus.IDLE
        self.fsm_status = rospy.Subscriber("/pepper_resources_synchronizer/state_machine_status",
                                           MetaStateMachinesStatus, self.on_fsm_status)
        self.status_pub = rospy.Publisher("/deictic_gestures/look_at/status", LookAtStatus, queue_size=5, latch=True)

        self.resource_synchronizer = rospy.ServiceProxy("/pepper_resources_synchronizer/state_machines_register", MetaStateMachineRegister)


        self.current_lookat_frame = None
        self.current_lookat_point = None

    def on_fsm_status(self, msg):
        """

        :param msg:
        :type msg: MetaStateMachinesStatus
        :return:
        """
        for id in self.pointing_ids:
            if msg.id == id:
                rospy.loginfo("LOOK AT: state machine status")
                for i in range(len(msg.resource)):
                    if msg.resource[i] in HEAD_MANAGER_NAMES and msg.state_name[i] == "look" and self.current_state != LookAtStatus.LOOK:
                        self.current_state = LookAtStatus.LOOK
                        st = LookAtStatus()
                        st.status = LookAtStatus.LOOK
                        self.status_pub.publish(st)
                        rospy.loginfo("LOOK AT: status LOOK")
                    if msg.resource[i] in BASE_MANAGER_NAMES and msg.state_name[i] == "base_turn" and self.current_state != LookAtStatus.ROTATE:
                        self.current_state = LookAtStatus.ROTATE
                        st = LookAtStatus()
                        st.status = LookAtStatus.ROTATE
                        self.status_pub.publish(st)
                        rospy.loginfo("LOOK AT: status ROTATE")
                    if msg.resource[i] in HEAD_MANAGER_NAMES and msg.state_name[i] == "" and self.current_state != LookAtStatus.IDLE:
                        self.current_state = LookAtStatus.IDLE
                        st = LookAtStatus()
                        st.status = LookAtStatus.FINISHED
                        self.status_pub.publish(st)
                        rospy.loginfo("LOOK AT: status FINISHED")
                        st.status = LookAtStatus.IDLE
                        self.status_pub.publish(st)
                        self.pointing_ids.remove(id)
                        rospy.loginfo("LOOK AT: status IDLE")

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
                if math.fabs(angle) > LOOK_AT_MAX_ANGLE:
                    if angle > 0:
                        angle += 0.1
                    else:
                        angle -= 0.1
                    return False, angle
                else:
                    return True, 0
        except Exception as e:
            rospy.logerr("[look_at_srv] Exception occurred :" + str(e))
            return False, 0

    def handle_look_at(self, req, ):
        self.parameters["look_at_max_speed"] = rospy.get_param("look_at_max_speed", LOOK_AT_MAX_SPEED)
        rospy.loginfo("Look at request received")
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
                    dist = self.distance([new_p[0, 0], new_p[1, 0], new_p[2, 0]])
                    if dist > MIN_DIST_MOVE or self.current_lookat_frame != req.point.header.frame_id:
                        to_move = True

                else:
                    to_move = True
                if dist is not None:
                    speed_up = True if dist > 0.35 else False

                if to_move:
                    (translation, rotation) = self.tfListener.lookupTransform('torso', req.point.header.frame_id,
                                                                              rospy.Time())
                    t = transformation_matrix(translation, rotation)
                    p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
                    new_p = numpy.dot(t, p)
                    #self.tracker.stopTracker()
                    #self.publishers["result_point"].publish()
                    #rospy.logwarn("lookat")

                    if speed_up :
                        look_at_speed = LOOK_AT_MAX_SPEED
                    else:
                        look_at_speed = LOOK_AT_SPEED
                    self.current_lookat_point = [new_p[0, 0], new_p[1, 0], new_p[2, 0]]

                    angle = math.atan2(new_p[1, 0], new_p[0, 0])
                    angle_diff = 0.0
                    if math.fabs(angle) > LOOK_AT_MAX_ANGLE:
                        if not req.with_base:
                            return False
                        angle_diff = min(center_radians(angle - LOOK_AT_MAX_ANGLE), center_radians(angle + LOOK_AT_MAX_ANGLE), key=lambda x: abs(x))

                    if angle_diff != 0:
                        rospy.loginfo("Will turn: " + str(math.degrees(angle_diff)))

                    after_rot = transformation_matrix([0, 0, 0], quaternion_from_euler(0, 0, -angle_diff))
                    new_p = numpy.dot(after_rot, new_p)
                    rospy.loginfo("Point after rotation: " + str(new_p))

                    mfsm = MetaStateMachineRegisterRequest()
                    headfsm = SubStateMachine_pepper_head_manager_msgs()
                    basefsm = SubStateMachine_pepper_base_manager_msgs()
                    head_wait_turned = HeadStatePoint()
                    head_wait_turned.header.id = "_head_wait_turned"
                    base_turn = BaseStateAngle()
                    base_turn.data = angle_diff
                    base_turn.header.id = "base_turn"
                    head_moving = HeadStatePoint()
                    head_moving.header.id = "head_moving"
                    head_moving.data.header.frame_id = "torso"
                    head_moving.data.header.stamp = rospy.Time.now()
                    head_moving.data.point.x, head_moving.data.point.y, head_moving.data.point.z = new_p[0, 0], new_p[1, 0], new_p[2, 0]
                    head_look = HeadStatePoint()
                    head_look.header.id = "look"
                    head_look.data.header.frame_id = "torso"
                    head_look.data.header.stamp = rospy.Time.now()
                    head_look.data.point.x, head_look.data.point.y, head_look.data.point.z = new_p[0, 0], new_p[1, 0], new_p[2, 0]
                    base_idle = BaseStateAngle()
                    base_idle.header.id = "_base_idle"
                    head_end = HeadStatePoint()
                    head_end.header.id = "head_end"
                    head_end.data = head_look.data
                    t_head_wait_turn = StateMachineTransition()
                    t_head_wait_turn.next_state = "head_moving"
                    t_head_wait_turn.end_condition.duration = rospy.Duration(-1)
                    t_head_wait_turn.end_condition.timeout = rospy.Duration(-1)
                    t_head_wait_turn.end_condition.regex_end_condition.append("__synchro__turned")
                    head_wait_turned.header.transitions.append(t_head_wait_turn)
                    t_base_turn = StateMachineTransition()
                    t_base_turn.next_state = "_base_idle"
                    t_base_turn.end_condition.timeout = rospy.Duration(-1)
                    t_base_turn.end_condition.duration = rospy.Duration(-1)
                    t_base_turn.end_condition.regex_end_condition.append("__synchro__turned")
                    base_turn.header.transitions.append(t_base_turn)
                    t_head_moving = StateMachineTransition()
                    t_head_moving.end_condition.duration = rospy.Duration(-1)
                    t_head_moving.end_condition.timeout = rospy.Duration(-1)
                    t_head_moving.end_condition.regex_end_condition.append("__done__")
                    t_head_moving.next_state = "look"
                    head_moving.header.transitions.append(t_head_moving)
                    t_head_look = StateMachineTransition()
                    t_head_look.end_condition.duration = rospy.Duration(-1)
                    t_head_look.end_condition.timeout = rospy.Duration(-1)
                    t_head_look.end_condition.regex_end_condition.append("human_perceived")
                    t_head_look.next_state = "head_end"
                    t_head_look_fail = StateMachineTransition()
                    t_head_look_fail.end_condition.duration = rospy.Duration(-1)
                    t_head_look_fail.end_condition.timeout = rospy.Duration(-1)
                    t_head_look_fail.end_condition.regex_end_condition.append("stop_look_at")
                    t_head_look_fail.next_state = "head_end"
                    head_look.header.transitions.append(t_head_look)
                    head_look.header.transitions.append(t_head_look_fail)
                    t_head_end = StateMachineTransition()
                    t_head_end.next_state = "end"
                    t_head_end.end_condition.timeout = rospy.Duration(-1)
                    t_head_end.end_condition.duration = rospy.Duration(-1)
                    t_head_end.end_condition.regex_end_condition.append("__synchro__end")
                    head_end.header.transitions.append(t_head_end)
                    t_base_end = StateMachineTransition()
                    t_base_end.next_state = "end"
                    t_base_end.end_condition.duration = rospy.Duration(-1)
                    t_base_end.end_condition.timeout = rospy.Duration(-1)
                    t_base_end.end_condition.regex_end_condition.append("__synchro__end")
                    base_idle.header.transitions.append(t_base_end)

                    headfsm.state_machine.states_PrioritizedPoint.append(head_wait_turned)
                    headfsm.state_machine.states_PrioritizedPoint.append(head_moving)
                    headfsm.state_machine.states_PrioritizedPoint.append(head_look)
                    headfsm.state_machine.states_PrioritizedPoint.append(head_end)
                    basefsm.state_machine.states_PrioritizedAngle.append(base_turn)
                    basefsm.state_machine.states_PrioritizedAngle.append(base_idle)
                    if angle_diff != 0.0:
                        headfsm.header.initial_state = "_head_wait_turned"
                        basefsm.header.initial_state = "base_turn"
                    else:
                        headfsm.header.initial_state = "head_moving"
                        basefsm.header.initial_state = "_base_idle"
                    headfsm.header.timeout = rospy.Duration(-1)
                    headfsm.header.begin_dead_line = rospy.Time.now() + rospy.Duration(15)
                    basefsm.header.begin_dead_line = rospy.Time.now() + rospy.Duration(15)
                    basefsm.header.timeout = rospy.Duration(-1)

                    mfsm.state_machine_pepper_head_manager = headfsm
                    mfsm.state_machine_pepper_base_manager = basefsm
                    mfsm.header.timeout = rospy.Duration(-1)
                    mfsm.header.begin_dead_line = rospy.Time.now() + rospy.Duration(15)
                    mfsm.header.priority.value = MessagePriority.URGENT

                    ret = self.resource_synchronizer.call(mfsm)
                    self.pointing_ids.append(ret.id)

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

    rospy.init_node('look_at_srv')
    l = LookAtSrv()
    rospy.spin()
    exit(0)
