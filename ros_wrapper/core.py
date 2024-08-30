import os

# Globale Variable zur Speicherung des erkannten ROS-Systems
ros_version = None


def detect_ros_version():
    global ros_version
    ros_distro = os.getenv('ROS_DISTRO')

    if ros_distro in ['noetic', 'melodic', 'kinetic']:
        ros_version = 'ros1'
    elif ros_distro in ['foxy', 'galactic', 'humble']:
        ros_version = 'ros2'
    else:
        raise RuntimeError(
            "Unsupported or unknown ROS distribution. Ensure you've sourced the appropriate setup.bash file.")

detect_ros_version()

# Dynamischer Import der richtigen Schnittstelle
if ros_version == 'ros1':
    from .ros1_interface import init_node, create_publisher, create_subscriber, spin
elif ros_version == 'ros2':
    from .ros2_interface import init_node, create_publisher, create_subscriber, spin

def ros_version():
    return ros_version