import time

from example_interfaces.srv import AddTwoInts

import sys
sys.path.append('/home/niklas/PycharmProjects/ROSConverter')
import ros_wrapper

def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    print(f"Incoming request: a={request.a}, b={request.b}, sum={response.sum}")
    return response

def main(args=None):
    ros_wrapper.init_node("test_service")
    ros_wrapper.create_service("add_two_ints", AddTwoInts, add_two_ints_callback)
    time.sleep(3)
    try:
        ros_wrapper.spin()
    finally:
        print("Shutting down")
        # ros_wrapper.shutdown()


if __name__ == '__main__':
    main()