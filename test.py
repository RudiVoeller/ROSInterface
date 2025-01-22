from time import sleep

import ros_wrapper as ros
import unittest
import multiprocessing

from test_utils import init_subscriber, init_publisher


class TestROSPublisherSubscriber(unittest.TestCase):

    def setUp(self):
        self.manager = multiprocessing.Manager()
        self.received_messages = self.manager.list()
        self.subscriber_proc = multiprocessing.Process(target=init_subscriber, args=(self.received_messages,))
        self.publisher_proc = multiprocessing.Process(target=init_publisher)


    def tearDown(self):
        if self.subscriber_proc and self.subscriber_proc.is_alive():
            self.subscriber_proc.terminate()

        if self.publisher_proc and self.publisher_proc.is_alive():
            self.publisher_proc.terminate()

    def test_receive_messages(self):
        self.subscriber_proc.start()
        sleep(3)
        self.publisher_proc.start()
        sleep(3)
        self.assertEqual(len(self.received_messages), 1, "Should have received 1 message")
        self.assertTrue(all(msg == "Hello World" for msg in self.received_messages),
                        "All messages should be 'Hello World'")

    def test_subscriber_count(self):
        self.subscriber_proc.start()
        sleep(3)
        self.assertEqual(ros.subscriber_count_per_topic("test_topic"), 1, "Should have 1 subscriber")
        self.publisher_proc.start()
        sleep(3)
        self.assertEqual(ros.subscriber_count_per_topic("test_topic"), 1, "Should have 1 subscriber")
        self.subscriber_proc.terminate()
        self.subscriber_proc.join()
        sleep(3)
        self.assertEqual(ros.subscriber_count_per_topic("test_topic"), 0, "Should have 0 subscribers")

if __name__ == '__main__':
    unittest.main()

