#!/usr/bin/env python

import rospy
import tf
import math
from std_srvs.srv import *
from geometry_msgs.msg import *
from nao_interaction_msgs.srv import *
from deictic_gestures.srv import *

LOOK_AT_MAX_SPEED = 0.6


class PointingPlannerSrv(object):
    def __init__(self):

        self.services = {"pointing_config": rospy.Service('/deictic_gestures/get_pointing_config', GetPointingConfig,
                                                          self.handle_get_pointing_config)}

        self.tfListener = tf.TransformListener()

        self.parameters = {"global_frame": rospy.get_param("global_frame_id", "/map"),
                           "robot_footprint": rospy.get_param("footprint_frame_id", "/base_footprint")}

        self.publishers = {
            "result_pose": rospy.Publisher('/deictic_gestures/pointing_planner_pose_result', geometry_msgs.msg.PoseStamped, queue_size=5)}

    def handle_get_pointing_config(self, req):

        self.tfListener.waitForTransform(self.parameters["global_frame"], req.human_footprint_frame_id,
                                         rospy.Time(0), rospy.Duration(2.0))
        (translation, rotation) = self.tfListener.lookupTransform(self.parameters["global_frame"],
                                                                  req.human_footprint_frame_id, rospy.Time(0))
        human_pose = geometry_msgs.msg.Pose(translation, rotation)
        rospy.loginfo(
            "Human current pose in " + str(self.parameters["global_frame"]) + " frame :\n\r" + str(human_pose))

        self.tfListener.waitForTransform(self.parameters["global_frame"], req.target.header.frame_id, rospy.Time(0),
                                         rospy.Duration(2.0))
        (translation, rotation) = self.tfListener.lookupTransform(self.parameters["global_frame"],
                                                                  req.target.header.frame_id, rospy.Time(0))
        target_pose = Pose(translation, rotation)
        print(str(target_pose))
        target_pose.position = [target_pose.position[0] + req.target.point.x,
                                target_pose.position[1] + req.target.point.y,
                                target_pose.position[2] + req.target.point.z]

        rospy.loginfo(
            "Target current pose in " + str(self.parameters["global_frame"]) + " frame:\n\r" + str(target_pose))

        self.tfListener.waitForTransform(self.parameters["global_frame"], self.parameters["robot_footprint"],
                                         rospy.Time(0), rospy.Duration(2.0))
        (translation, rotation) = self.tfListener.lookupTransform(self.parameters["global_frame"],
                                                                  self.parameters["robot_footprint"], rospy.Time(0))
        robot_pose = geometry_msgs.msg.Pose(translation, rotation)
        rospy.loginfo("Robot current pose in " + str(self.parameters["global_frame"]) + " frame :\n\r" + str(robot_pose))
        # we work in radians
        alpha_relative_to_direction = math.radians(req.alpha)
        # first we compute the z rot to have the base x axis aligned with the direction from human_base to target
        # to do so we compute theta_object that represent the angle of the direction beetween human & object in map
        delta_x_human_target = target_pose.position[0] - human_pose.position[0]
        delta_y_human_target = target_pose.position[1] - human_pose.position[1]
        rospy.loginfo("Delta x [m] :" + str(delta_x_human_target) + " Delta y [m] :" + str(delta_y_human_target))
        distance_target_human = math.hypot(delta_x_human_target, delta_y_human_target)
        rospy.loginfo("Distance target to human [m] : " + str(distance_target_human))
        theta_target = math.atan2(delta_y_human_target, delta_x_human_target)
        rospy.loginfo("Theta object [deg] : " + str(math.degrees(theta_target)))
        alpha_min_absolute = min(theta_target - alpha_relative_to_direction, theta_target + alpha_relative_to_direction)
        alpha_max_absolute = max(theta_target - alpha_relative_to_direction, theta_target + alpha_relative_to_direction)
        rospy.loginfo(
            "Alpha min absolute [deg]: " + str(math.degrees(alpha_min_absolute)) + " Alpha max absolute [deg]: " + str(
                math.degrees(alpha_max_absolute)))
        # Computation of the position of the pose corresponding to alpha min
        amin_position = [human_pose.position[0] + (req.distance_to_robot * math.cos(alpha_min_absolute)),
                         human_pose.position[1] + (req.distance_to_robot * math.sin(alpha_min_absolute)), 0]
        rospy.loginfo("Alpha min position :\n\r" + str(amin_position))
        # Computation of the position of the pose corresponding to alpha max
        amax_position = [human_pose.position[0] + (req.distance_to_robot * math.cos(alpha_max_absolute)),
                         human_pose.position[1] + (req.distance_to_robot * math.sin(alpha_max_absolute)), 0]
        rospy.loginfo("Alpha min position :\n\r" + str(amax_position))
        # then we choose between those two
        distance_current_amin = math.hypot(amin_position[0] - robot_pose.position[0],
                                           amin_position[1] - robot_pose.position[1])
        distance_current_amax = math.hypot(amax_position[0] - robot_pose.position[0],
                                           amax_position[1] - robot_pose.position[1])
        result_pose = geometry_msgs.msg.PoseStamped()
        if distance_current_amax < distance_current_amin:
            result_pose.pose.position.x = amax_position[0]
            result_pose.pose.position.y = amax_position[1]
            result_pose.pose.position.z = amax_position[2]
        else:
            result_pose.pose.position.x = amin_position[0]
            result_pose.pose.position.y = amin_position[1]
            result_pose.pose.position.z = amin_position[2]
        # Then we compute the orientation of the result pose (the robot should be facing the human)
        theta_robot = math.atan2(target_pose.position[1] - result_pose.pose.position.y,
                                 target_pose.position[0] - result_pose.pose.position.x)
        q = tf.transformations.quaternion_from_euler(0, 0, theta_robot)
        rospy.loginfo("Result position :\n\r" + str(result_pose.pose.position))

        result_pose.pose.orientation.x = q[0]
        result_pose.pose.orientation.y = q[1]
        result_pose.pose.orientation.z = q[2]
        result_pose.pose.orientation.w = q[3]
        result_pose.header.frame_id = self.parameters["global_frame"]
        result_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Result orientation :\n\r" + str(result_pose.pose.orientation))
        self.publishers["result_pose"].publish(result_pose)
        return True, result_pose


if __name__ == '__main__':
    rospy.init_node('pointing_planner')
    srv = PointingPlannerSrv()
    rospy.spin()
    exit(0)
