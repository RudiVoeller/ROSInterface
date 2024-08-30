from setuptools import setup, find_packages

setup(
    name='ros_wrapper',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[],  # Abhängigkeiten wie `rospy` und `rclpy` sollten vom Nutzer manuell installiert werden
    description='A ROS1/ROS2 wrapper package',
    author='Dein Name',
    author_email='deine.email@example.com',
    url='https://github.com/dein-repo/ros_wrapper',
)