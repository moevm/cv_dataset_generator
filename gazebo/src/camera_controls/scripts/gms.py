#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import GetModelState


def gms_client(model_name, relative_entity_name):
    gms_service = '/gazebo/get_model_state'
    rospy.wait_for_service(gms_service)
    try:
        gms = rospy.ServiceProxy(gms_service, GetModelState)
        resp1 = gms(model_name, relative_entity_name)
        return resp1
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def usage():
    return f"{sys.argv[0]} [model_name]"


if __name__ == "__main__":
    if len(sys.argv) == 2:
        model_name = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)
    print(f"Requesting {model_name}")
    res = gms_client(model_name, '')
    print('Pose:', res.pose.position, sep='\n')
    print('Orientation:', res.pose.orientation, sep='\n')
