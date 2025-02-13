#!/usr/bin/env python3


from std_msgs.msg import String
import unittest
import time
#Insert import path to the ROSInterface package
import sys
sys.path.append('/add/your/path/to/ROSInterface')

import ros_wrapper


class TestMessageFlow(unittest.TestCase):

    def test_message_transfer(self):

        # Testknoten erstellen
        received_messages = []

        # Subscriber für den Test
        def callback(msg):
            received_messages.append(msg.data)

        ros_wrapper.create_subscriber("test_topic", String, callback)

        # Warten, bis eine Nachricht empfangen wird
        try:
            start_time = time.time()
            while len(received_messages) == 0 and (time.time() - start_time) < 2:
                ros_wrapper.spin_once()
                time.sleep(0.1)

        except AssertionError as e:
            print(f"Test fehlgeschlagen: {e}")




    def test_publisher_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Publisher
        assert ros_wrapper.publisher_count_per_topic("test_topic") == 1

    def test_subscriber_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Subscriber
        assert ros_wrapper.subscriber_count_per_topic("/test_topic") == 1

    def test_node_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Knoten
        print(ros_wrapper.get_all_nodes())
        assert len(ros_wrapper.get_all_nodes()) == 3
    def test_topic_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Topics
        print(ros_wrapper.get_all_topics())
        if "test_topic" not in ros_wrapper.get_all_topics():
            assert False
    def test_service_count(self):
        # Testknoten erstellen
        # Test: Überprüfung der Anzahl der Services
        print(ros_wrapper.get_all_services())
        assert len(ros_wrapper.get_all_services()) == 3


if __name__ == '__main__':
    import rostest

    ros_wrapper.init_node("test_message_flow")
    rostest.rosrun('ros1_auto_tests', 'test_message_flow', TestMessageFlow)
    print(ros_wrapper.get_all_services)