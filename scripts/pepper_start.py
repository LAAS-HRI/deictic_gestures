#! /usr/bin/env python

"""Example: Use setExternalCollisionProtectionEnabled Method"""

import rospy
import qi
import sys


def main(session):
    """
    This example uses the setExternalCollisionProtectionEnabled method.
    """
    # Get the service ALMotion.

    motion_service  = session.service("ALMotion")
    tracker_service  = session.service("ALTracker")

    # Example showing how to activate "Move", "LArm" and "RArm" external anti collision
    motion_service.setExternalCollisionProtectionEnabled("All", False)
    print("Safety Lock Disabled")
    motion_service.setBreathEnabled("Body",False)
    print("Artificial Breath Disabled")
    tracker_service.stopTracker()
    print("Auto Tracker Disabled")

if __name__ == "__main__":
    ip = "172.16.221.42"
    port = 9559
    session = qi.Session()
    try:
        session.connect("tcp://" + ip + ":" + str(port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)
