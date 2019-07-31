#!/usr/bin/env python

import rospy
import tf
import sys
import numpy
import math
from geometry_msgs.msg import PointStamped
from deictic_gestures.srv import PointAt, CanLookAt
from deictic_gestures.msg import PointAtStatus
from pepper_resources_synchronizer_msgs.srv import MetaStateMachineRegister, MetaStateMachineRegisterRequest
from pepper_resources_synchronizer_msgs.msg import SubStateMachine_pepper_arm_manager_msgs, SubStateMachine_pepper_head_manager_msgs, SubStateMachine_pepper_base_manager_msgs
from pepper_arm_manager_msgs.msg import StateMachineStatePrioritizedPoint as ArmStatePoint
from pepper_head_manager_msgs.msg import StateMachineStatePrioritizedPoint as HeadStatePoint
from pepper_base_manager_msgs.msg import StateMachineStatePrioritizedAngle as BaseStateAngle
from resource_management_msgs.msg import StateMachineTransition, MessagePriority
from resource_synchronizer_msgs.msg import MetaStateMachinesStatus
from tf.transformations import translation_matrix, quaternion_matrix, quaternion_from_euler

POINT_AT_MAX_SPEED = 0.7
POINT_AT_MAX_ANGLE = math.pi/3

ARM_MANAGER_NAMES = ["pepper_arm_manager_left", "pepper_arm_manager_right"]
BASE_MANAGER_NAME = ["pepper_base_manager"]


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

        self.pointing_ids = []
        self.current_state = PointAtStatus.IDLE
        self.fsm_status = rospy.Subscriber("/pepper_resources_synchronizer/state_machine_status", MetaStateMachinesStatus, self.on_fsm_status)
        self.status_pub = rospy.Publisher("/deictic_gestures/point_at/status", PointAtStatus, queue_size=5, latch=True)
        self.resource_synchronizer = rospy.ServiceProxy("/pepper_resources_synchronizer/state_machines_register", MetaStateMachineRegister)

    def on_fsm_status(self, msg):
        """

        :param msg:
        :type msg: MetaStateMachinesStatus
        :return:
        """
        for id in self.pointing_ids:
            if msg.id == id:
                for i in range(len(msg.resource)):
                    if msg.resource[i] in ARM_MANAGER_NAMES and msg.state_name[i] == "point_arm" and self.current_state != PointAtStatus.POINT:
                        self.current_state = PointAtStatus.POINT
                        st = PointAtStatus()
                        st.status = PointAtStatus.POINT
                        self.status_pub.publish(st)
                    if msg.resource[i] in BASE_MANAGER_NAME and msg.state_name[i] == "base_turn" and self.current_state != PointAtStatus.ROTATE:
                        self.current_state = PointAtStatus.ROTATE
                        st = PointAtStatus()
                        st.status = PointAtStatus.ROTATE
                        self.status_pub.publish(st)
                    if msg.resource[i] in ARM_MANAGER_NAMES and msg.state_name[i] == "" and self.current_state != PointAtStatus.IDLE:
                        self.current_state = PointAtStatus.IDLE
                        st = PointAtStatus()
                        st.status = PointAtStatus.FINISHED
                        self.status_pub.publish(st)
                        st.status = PointAtStatus.IDLE
                        self.status_pub.publish(st)
                        self.pointing_ids.remove(id)

    def handle_can_point_at(self, req):
        if self.tfListener.canTransform("/torso", req.point.header.frame_id, rospy.Time()):
            (translation, rotation) = self.tfListener.lookupTransform("base_link", req.point.header.frame_id,
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
        rospy.loginfo("Point at request received")
        self.parameters["point_at_max_speed"] = rospy.get_param("point_at_max_speed", POINT_AT_MAX_SPEED)
        try:
            self.publishers["input_point"].publish(req.point)
            rospy.loginfo("Transforming %s to base_link" % req.point.header.frame_id)
            if self.tfListener.canTransform("base_link", req.point.header.frame_id, rospy.Time()):
                (translation, rotation) = self.tfListener.lookupTransform("base_link", req.point.header.frame_id,
                                                                          rospy.Time())
                t = transformation_matrix(translation, rotation)
                p = numpy.atleast_2d([req.point.point.x, req.point.point.y, req.point.point.z, 1]).transpose()
                new_p = numpy.dot(t, p)
                yaw = math.atan2(new_p[1], new_p[0])

                rot = 0.0
                if abs(yaw) > POINT_AT_MAX_ANGLE:
                    rot = min(center_radians(yaw - POINT_AT_MAX_ANGLE), center_radians(yaw + POINT_AT_MAX_ANGLE), key=lambda x: abs(x))
                    if not req.with_base:
                        rospy.loginfo("Out of bounds : %s" % rot)
                        return False


                after_rot = transformation_matrix([0, 0, 0], quaternion_from_euler(0, 0, -rot))
                new_p = numpy.dot(after_rot, new_p)
                #rospy.loginfo("Point after rotation: " + str(new_p))
                r = MetaStateMachineRegisterRequest()
                r.header.timeout = rospy.Duration(-1)
                r.header.begin_dead_line = rospy.Time.now() + rospy.Duration(15)
                r.header.priority.value = MessagePriority.URGENT
                p_arm, idle_arm, head, base = self.fill_arm_fsm_with_head(new_p, rot)
                if new_p[1, 0] >= 0.0:
                    rospy.loginfo("Pointing with Left")
                    r.state_machine_pepper_arm_manager_left = p_arm
                    r.state_machine_pepper_arm_manager_right = idle_arm
                else:
                    r.state_machine_pepper_arm_manager_right = p_arm
                    r.state_machine_pepper_arm_manager_left = idle_arm
                if req.with_head:
                    r.state_machine_pepper_head_manager = head
                r.state_machine_pepper_base_manager = base
                ret = self.resource_synchronizer.call(r)
                self.pointing_ids.append(ret.id)
                return True
            return False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr("[point_at_srv] Exception occured :" + str(e))
            return False

    def fill_arm_fsm_with_head(self, point, base_angle=0.0):
        arm = SubStateMachine_pepper_arm_manager_msgs()
        head = SubStateMachine_pepper_head_manager_msgs()
        idle_arm = SubStateMachine_pepper_arm_manager_msgs()
        base = SubStateMachine_pepper_base_manager_msgs()
        arm.header.begin_dead_line = rospy.Time().now() + rospy.Duration(15)
        head.header.begin_dead_line = rospy.Time().now() + rospy.Duration(15)
        idle_arm.header.begin_dead_line = rospy.Time().now() + rospy.Duration(15)
        base.header.begin_dead_line = rospy.Time().now() + rospy.Duration(15)
        arm.header.initial_state = "_arm_initial_synchro"
        head.header.initial_state = "_head_initial_synchro"
        idle_arm.header.initial_state = "_idling"
        base.header.initial_state = "_base_initial_synchro"
        arm.header.timeout = rospy.Duration(-1)
        head.header.timeout = rospy.Duration(-1)
        idle_arm.header.timeout = rospy.Duration(-1)
        base.header.timeout = rospy.Duration(-1)

        t_arm_init = StateMachineTransition()
        t_arm_init.end_condition.regex_end_condition.append("__synchro__begin")
        t_head_init = StateMachineTransition()
        t_head_init.end_condition.regex_end_condition.append("__synchro__begin")
        t_base_init = StateMachineTransition()
        t_base_init.end_condition.regex_end_condition.append("__synchro__begin")
        t_head_init.end_condition.timeout = rospy.Duration(10)
        t_arm_init.end_condition.timeout = rospy.Duration(10)
        t_base_init.end_condition.timeout = rospy.Duration(10)
        t_head_init.end_condition.duration = rospy.Duration(-1)
        t_arm_init.end_condition.duration = rospy.Duration(-1)
        t_base_init.end_condition.duration = rospy.Duration(-1)
        if base_angle == 0.0:
            t_head_init.next_state = "moving_head"
            t_arm_init.next_state = "moving_arm"
            t_base_init.next_state = "_idle"
        else:
            t_head_init.next_state = "_head_wait_base"
            t_arm_init.next_state = "_arm_wait_base"
            t_base_init.next_state = "base_turn"

        arm_sync = ArmStatePoint()
        head_sync = HeadStatePoint()
        base_sync = BaseStateAngle()
        arm_sync.header.id = "_arm_initial_synchro"
        head_sync.header.id = "_head_initial_synchro"
        base_sync.header.id = "_base_initial_synchro"
        arm_sync.header.transitions.append(t_arm_init)
        head_sync.header.transitions.append(t_head_init)
        base_sync.header.transitions.append(t_base_init)

        # Only used if with_base != 0.0
        arm_wait_turn = ArmStatePoint()
        arm_wait_turn.header.id = "_arm_wait_base"
        head_wait_turn = HeadStatePoint()
        head_wait_turn.header.id = "_head_wait_base"
        base_turning = BaseStateAngle()
        base_turning.header.id = "base_turn"
        base_turning.data = base_angle
        base_turning_ended = BaseStateAngle()
        base_turning_ended.header.id = "_base_synchro"
        t_arm_wait_base = StateMachineTransition()
        t_arm_wait_base.end_condition.duration = rospy.Duration(-1)
        t_arm_wait_base.end_condition.timeout = rospy.Duration(-1)
        t_arm_wait_base.end_condition.regex_end_condition.append("__synchro__turnended")
        t_arm_wait_base.next_state = "moving_arm"
        t_head_wait_base = StateMachineTransition()
        t_head_wait_base.end_condition.duration = rospy.Duration(-1)
        t_head_wait_base.end_condition.timeout = rospy.Duration(-1)
        t_head_wait_base.end_condition.regex_end_condition.append("__synchro__turnended")
        t_head_wait_base.next_state = "moving_head"
        t_base_turn = StateMachineTransition()
        t_base_turn.end_condition.timeout = rospy.Duration(-1)
        t_base_turn.end_condition.duration = rospy.Duration(-1)
        t_base_turn.end_condition.regex_end_condition.append("__done__")
        t_base_turn.next_state = "_base_synchro"
        t_base_synchro = StateMachineTransition()
        t_base_synchro.end_condition.timeout = rospy.Duration(-1)
        t_base_synchro.end_condition.duration = rospy.Duration(-1)
        t_base_synchro.end_condition.regex_end_condition.append("__synchro__turnended")
        t_base_synchro.next_state = "_idle"
        arm_wait_turn.header.transitions.append(t_arm_wait_base)
        head_wait_turn.header.transitions.append(t_head_wait_base)
        base_turning.header.transitions.append(t_base_turn)
        base_turning_ended.header.transitions.append(t_base_synchro)

        # Pointing motion
        arm_motion = ArmStatePoint()
        head_motion = HeadStatePoint()
        arm_motion.header.id = "moving_arm"
        arm_motion.data.header.frame_id = "base_link"
        arm_motion.data.header.stamp = rospy.Time.now()
        arm_motion.data.point.x, arm_motion.data.point.y, arm_motion.data.point.z = point[0, 0], point[1, 0], point[2, 0]
        head_motion.header.id = "moving_head"
        head_motion.data.header.frame_id = "base_link"
        head_motion.data.header.stamp = rospy.Time.now()
        head_motion.data.point = arm_motion.data.point

        t_arm_moving = StateMachineTransition()
        t_head_moving = StateMachineTransition()
        t_arm_moving.next_state = "point_arm"
        t_head_moving.next_state = "point_head"
        t_arm_moving.end_condition.timeout = rospy.Duration(-1)
        t_head_moving.end_condition.timeout = rospy.Duration(-1)
        t_arm_moving.end_condition.duration = rospy.Duration(-1)
        t_head_moving.end_condition.duration = rospy.Duration(-1)
        t_arm_moving.end_condition.regex_end_condition.append("__done__")
        t_head_moving.end_condition.regex_end_condition.append("__done__")
        arm_motion.header.transitions.append(t_arm_moving)
        head_motion.header.transitions.append(t_head_moving)

        # Actual pointing
        arm_point = ArmStatePoint()
        head_point = HeadStatePoint()
        arm_point.header.id = "point_arm"
        arm_point.data.header.frame_id = "base_link"
        arm_point.data.header.stamp = rospy.Time.now()
        arm_point.data.point.x, arm_point.data.point.y, arm_point.data.point.z = point[0, 0], point[1, 0], point[2, 0]
        head_point.header.id = "point_head"
        head_point.data.header.stamp = rospy.Time.now()
        head_point.data.header.frame_id = "base_link"
        head_point.data.point.x, head_point.data.point.y, head_point.data.point.z = point[0, 0], point[1, 0], point[2, 0]

        t_arm_point = StateMachineTransition()
        t_head_point = StateMachineTransition()
        t_arm_point.next_state = "ending"
        t_head_point.next_state = "ending"
        t_arm_point.end_condition.timeout = rospy.Duration(-1)
        t_head_point.end_condition.timeout = rospy.Duration(-1)
        t_arm_point.end_condition.duration = rospy.Duration(5)
        t_head_point.end_condition.duration = rospy.Duration(5)
        head_point.header.transitions.append(t_head_point)
        arm_point.header.transitions.append(t_arm_point)

        # End synchro
        arm_end = ArmStatePoint()
        head_end = HeadStatePoint()
        idling_arm = ArmStatePoint()
        base_idle = BaseStateAngle()
        arm_end.header.id = "ending"
        arm_end.data = arm_point.data
        head_end.header.id = "ending"
        head_end.data = head_point.data
        idling_arm.header.id = "_idling"
        base_idle.header.id = "_idle"
        t_arm_end = StateMachineTransition()
        t_arm_end.end_condition.timeout = rospy.Duration(-1)
        t_arm_end.end_condition.duration = rospy.Duration(10)
        t_arm_end.end_condition.regex_end_condition.append("__synchro__end")
        t_arm_end.next_state = "end"
        t_head_end = StateMachineTransition()
        t_head_end.end_condition.timeout = rospy.Duration(-1)
        t_head_end.end_condition.duration = rospy.Duration(10)
        t_head_end.end_condition.regex_end_condition.append("__synchro__end")
        t_head_end.next_state = "end"
        t_idle_end = StateMachineTransition()
        t_idle_end.end_condition.timeout = rospy.Duration(-1)
        t_idle_end.end_condition.duration = rospy.Duration(10) # TODO
        t_idle_end.end_condition.regex_end_condition.append("__synchro__end")
        t_idle_end.next_state = "end"
        t_base_idle = StateMachineTransition()
        t_base_idle.end_condition.duration = rospy.Duration(10)
        t_base_idle.end_condition.timeout = rospy.Duration(-1)
        t_base_idle.end_condition.regex_end_condition.append("__synchro__end")
        t_base_idle.next_state = "end"
        arm_end.header.transitions.append(t_arm_end)
        head_end.header.transitions.append(t_head_end)
        idling_arm.header.transitions.append(t_idle_end)
        base_idle.header.transitions.append(t_base_idle)

        arm.state_machine.states_PrioritizedPoint.append(arm_sync)
        arm.state_machine.states_PrioritizedPoint.append(arm_wait_turn)
        arm.state_machine.states_PrioritizedPoint.append(arm_motion)
        arm.state_machine.states_PrioritizedPoint.append(arm_point)
        arm.state_machine.states_PrioritizedPoint.append(arm_end)
        head.state_machine.states_PrioritizedPoint.append(head_sync)
        head.state_machine.states_PrioritizedPoint.append(head_wait_turn)
        head.state_machine.states_PrioritizedPoint.append(head_motion)
        head.state_machine.states_PrioritizedPoint.append(head_point)
        head.state_machine.states_PrioritizedPoint.append(head_end)
        idle_arm.state_machine.states_PrioritizedPoint.append(idling_arm)
        base.state_machine.states_PrioritizedAngle.append(base_sync)
        base.state_machine.states_PrioritizedAngle.append(base_turning)
        base.state_machine.states_PrioritizedAngle.append(base_turning_ended)
        base.state_machine.states_PrioritizedAngle.append(base_idle)

        return arm, idle_arm, head, base


if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)


    rospy.init_node('point_at_srv')
    PointAtSrv(None, None, None, None)
    rospy.spin()
    exit(0)
