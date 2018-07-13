#! /usr/bin/env python

import argparse
import qi
import rospy
from guiding_as.srv import *


class RestArmSrv(object):
    def __init__(self, nao_ip, nao_port):
        self.nao_ip = nao_ip
        self.nao_port = nao_port
        try:
            self.session = qi.Session()
            self.session.connect("tcp://" + args.nao_ip + ":" + str(args.nao_port))
            self.motion_service = self.session.service("ALMotion")
            self.services = {"rest_arm": rospy.Service('/deictic_gestures/rest_arm', RestArm, self.handle_rest_arm)}
        except RuntimeError:
            rospy.logerr("Can't connect to Naoqi at ip \"" + args.nao_ip + "\" on port " + str(args.nao_port) + ".\n"
                         "Please check your script arguments. Run with -h option for help.")

    def print_angles(self, req):

        use_sensors = False
        angles_l = self.motion_service.getAngles(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
                                                  "LWristYaw"], use_sensors)
        angles_r = self.motion_service.getAngles(["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
                                                   "RWristYaw"], use_sensors)

        rospy.loginfo("LArm %s:", angles_l)
        rospy.loginfo("RArm %s:", angles_r)

    def handle_rest_arm(self, req):
        self.motion_service.setStiffnesses(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"], 1.0)

        if req.effector == "LArm":
            self.rest_l_arm()

        elif req.effector == "RArm":
            self.rest_r_arm()

        elif req.effector == "Arms":
            self.motion_service.setAngles(["LShoulderPitch"], [1.5596846342086792], 0.2)
            self.motion_service.setAngles(["RShoulderPitch"], [1.5596898794174194], 0.2)

            rospy.sleep(1)
            self.motion_service.setAngles(["LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"],
                                          [0.14271031320095062, -1.2282506227493286,
                                           -0.522533655166626, -0.0004972327733412385], 0.2)
            self.motion_service.setAngles(["RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"],
                                          [-0.14270132780075073, 1.2282465696334839,
                                           0.5225334763526917, 0.0004968706052750349], 0.2)
            self.motion_service.setStiffnesses(["LHand", "RHand"], 0.0)
        return True

    def rest_l_arm(self):
        self.motion_service.setAngles(["LShoulderPitch"], [1.5596846342086792], 0.2)

        rospy.sleep(1)
        self.motion_service.setAngles(["LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"],
                                      [0.14271031320095062, -1.2282506227493286,
                                       -0.522533655166626, -0.0004972327733412385], 0.2)
        self.motion_service.setStiffnesses(["LHand"], 0.0)

    def rest_r_arm(self):
        self.motion_service.setAngles(["RShoulderPitch"], [1.5596898794174194], 0.2)

        rospy.sleep(1)
        self.motion_service.setAngles(["RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"],
                                      [-0.14270132780075073, 1.2282465696334839,
                                       0.5225334763526917, 0.0004968706052750349], 0.2)
        self.motion_service.setStiffnesses(["RHand"], 0.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--nao_ip", default="mummer-eth0.laas.fr", help="The robot IP")
    parser.add_argument("--nao_port", default="9559", help="The robot port")

    args = parser.parse_args()

    rospy.init_node('rest_arm_srv')
    RestArmSrv(args.nao_ip, args.nao_port)
    rospy.spin()
