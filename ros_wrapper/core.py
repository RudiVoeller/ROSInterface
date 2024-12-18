import os

global _ros_version

""" Detects if ROS1 or ROS2 is sourced/activated """
def _detect_ros_version():
    global _ros_version
    ros_distro = os.getenv('ROS_DISTRO')

    if ros_distro in ['noetic', 'melodic', 'kinetic']:
        _ros_version = 'ros1'
    elif ros_distro in ['foxy', 'galactic', 'humble']:
        _ros_version = 'ros2'
    else:
        raise RuntimeError(
            "Unsupported or unknown ROS distribution. Ensure you've sourced the appropriate setup.bash file.")


_detect_ros_version()
print(_ros_version)
""" Dynamic import of the correct functions """
if _ros_version == 'ros1':
    from .ros1_interface import init_node, create_publisher, create_subscriber, set_param, get_param, delete_param, spin, \
        create_service, call_service, get_all_nodes, get_all_services, subscription_count_per_topic, publisher_count_per_topic, get_all_topics, create_action_client, create_action_server
elif _ros_version == 'ros2':
    from .ros2_interface import init_node, create_publisher, create_subscriber, set_param, get_param, delete_param, spin, \
        create_service, call_service, get_all_nodes, get_all_services, subscription_count_per_topic, publisher_count_per_topic, get_all_topics, create_action_client, create_action_server

""" Getter for the ros version """
def ros_version():
    return _ros_version
