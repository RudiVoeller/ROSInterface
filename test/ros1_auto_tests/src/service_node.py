#!/usr/bin/env python3

## Insert import path to the ROSConverter package
import sys
sys.path.append('add/your/path/to/ROSInterface')
import ros_wrapper
from std_srvs.srv import Empty, EmptyResponse


def handle_empty_service(req):
    print("Empty service called")
    return EmptyResponse()

if __name__ == "__main__":
    ros_wrapper.init_node('empty_service_server')
    ros_wrapper.create_service('empty_service', Empty, handle_empty_service)
    print("Ready to handle empty service.")
    ros_wrapper.spin()