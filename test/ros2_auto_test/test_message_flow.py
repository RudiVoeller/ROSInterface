import rclpy
from std_msgs.msg import String
import pytest
import launch
import launch_ros.actions
import launch_testing
import unittest
import time

import sys
sys.path.append('/home/niklas/PycharmProjects/ROSConverter')

import ros_wrapper

# Konfigurieren des Loggings

@pytest.mark.rostest
def generate_test_description():
    # Startet die Publisher- und Subscriber-Nodes für den Test
    publisher_node = launch_ros.actions.Node(
        package='demo_package',
        executable='publisher_node',
        name='test_publisher'
    )

    subscriber_node = launch_ros.actions.Node(
        package='demo_package',
        executable='subscriber_node',
        name='test_subscriber'
    )

    service_node = launch_ros.actions.Node(
        package='demo_package',
        executable='service_node',
        name='test_service'
    )

    return (
        launch.LaunchDescription([
            publisher_node,
            subscriber_node,
            service_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'publisher_node': publisher_node,
            'subscriber_node': subscriber_node,
            'service_node': service_node,
        }
    )


class TestMessageFlow(unittest.TestCase):

    def setUpClass(self):
        ros_wrapper.init_node("test_message_flow")
    def test_message_transfer(self, launch_service, proc_output, publisher_node, subscriber_node):

        # Testknoten erstellen
        received_messages = []

        # Subscriber für den Test
        def callback(msg):
            received_messages.append(msg.data)

        ros_wrapper.create_subscriber("test_topic", String, callback)

        # Warten, bis eine Nachricht empfangen wird
        try:
            start_time = time.time()
            while len(received_messages) == 0 and (time.time() - start_time) < 5.0:
                ros_wrapper.spin_once()
                time.sleep(0.1)


            # Test: Überprüfung der empfangenen Nachricht


        except AssertionError as e:
            print(f"Test fehlgeschlagen: {e}")
        finally:
            # Testknoten ordnungsgemäß zerstören
            #test_node.destroy_node()
                rclpy.shutdown()

    def test_publisher_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Publisher
        assert ros_wrapper.publisher_count_per_topic("test_topic") == 1

    def test_subscriber_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Subscriber
        assert ros_wrapper.subscriber_count_per_topic("test_topic") == 1

    def test_node_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Knoten
        assert ros_wrapper.get_all_nodes() == 3
    def test_topic_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Topics
        assert ros_wrapper.get_all_topics() == 1
    def test_service_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Services
        assert ros_wrapper.get_all_services() == 1