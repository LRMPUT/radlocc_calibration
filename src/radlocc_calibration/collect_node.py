from threading import Event
from time import sleep

import rospy

from sensor_msgs.msg import Image, LaserScan

class CollectNode:

    def __init__(self, image_topic, laserscan_topic, data_saver):

        self._images = []
        self._laserscans = []

        self._image_acquire = Event()
        self._image_acquire.set()
        self._laserscan_acquire = False

        self._image_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback, queue_size=10)
        self._laserscan_subscriber = rospy.Subscriber(
            laserscan_topic, LaserScan, callback=self.laser_callback, queue_size=10)

        self._data_saver = data_saver

    def run(self):
        self.capture_laser(True)
        self.capture_image()
        self.capture_laser(False)

    def finish(self):
        self._data_saver.save()

    def capture_image(self):
        sleep(0.5)
        self._image_acquire.clear()
        self._image_acquire.wait()

    def capture_laser(self, state):
        self._laserscan_acquire = state

    def laser_callback(self, laser_msg):
        if self._laserscan_acquire:
            self._data_saver.save_laser(laser_msg)

    def image_callback(self, image_msg):
        if not self._image_acquire.isSet():
            self._image_acquire.set()
            self._data_saver.save_image(image_msg)
