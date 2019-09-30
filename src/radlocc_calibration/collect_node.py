from threading import Event
from time import sleep

import rospy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation as R


class CollectNode:

    def __init__(self, image_topic, laserscan_topic, data_saver):

        self._images = []
        self._laserscans = []

        self._image_acquire = Event()
        self._image_acquire.set()
        self._laserscan_acquire = False

        self._image_subscriber = rospy.Subscriber(
            image_topic, Image, callback=self.image_callback, queue_size=10)
        # self._laserscan_subscriber = rospy.Subscriber(
        #     laserscan_topic, LaserScan, callback=self.laser_callback, queue_size=10)
        self._laserscan_subscriber = rospy.Subscriber(
                laserscan_topic, PointCloud2, callback=self.laser_callback, queue_size=10)

        self._laserscan_publisher = rospy.Publisher("laserscan", LaserScan, queue_size=10)

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

    def line_intersection(self, pt1, pt2, r):
        r3 = pt2 - pt1
        r3 = r3 / np.linalg.norm(r3)
        a = np.linalg.norm(np.cross(pt1, r3)) / np.linalg.norm(np.cross(r, r3))

        return a * r

    def interpolate_angular(self, pt1, ang1, pt2, ang2, ang_tgt):
        r1 = pt1 / np.linalg.norm(pt1)
        r2 = pt2 / np.linalg.norm(pt2)
        rot_vec = np.cross(r1, r2)
        # norm proportional to angle of rotation
        rot_vec = rot_vec / np.linalg.norm(rot_vec) * (ang_tgt - ang1)
        rot = R.from_rotvec(rot_vec)

        r_int = rot.apply(r1)
        pt_int = self.line_intersection(pt1, pt2, r_int)

        return pt_int

    def interpolate_z(self, pt1, pt2, z_tgt):
        a = (z_tgt - pt1[2]) / (pt2[2] - pt1[2])
        pt_int = pt1 + a * (pt2 - pt1)

        return pt_int

    def convert_pc2_to_laserscan(self, pc2_msg):
        scan = pc2.read_points(pc2_msg, field_names={'x', 'y', 'z', 'ring'})
        pts_up = []
        pts_down = []
        for cur_pt in scan:
            ang = np.arctan2(cur_pt[1], cur_pt[0])
            cur_pt_aug = {'pt': np.array([cur_pt[0], cur_pt[1], cur_pt[2]]),
                          'ang': ang}
            # d_hor = np.sqrt(cur_pt[0] * cur_pt[0] + cur_pt[1] * cur_pt[1])
            # ang_vert = np.arctan2(cur_pt[2], d_hor) * 180.0 / np.pi
            # 1 deg
            if cur_pt[3] == 8:
                pts_up.append(cur_pt_aug)
            # -1 deg
            elif cur_pt[3] == 7:
                pts_down.append(cur_pt_aug)
        pts_up.sort(key=lambda cur_pt: cur_pt['ang'])
        pts_down.sort(key=lambda cur_pt: cur_pt['ang'])
        angle_min = -45 * np.pi / 180.0
        angle_max = 45 * np.pi / 180.0
        angle_increment = 0.25 * np.pi / 180.0
        ranges = []
        up_idx = 0
        down_idx = 0
        for ang in np.arange(angle_min, angle_max + angle_increment, angle_increment):
            while up_idx < len(pts_up) and pts_up[up_idx]['ang'] < ang:
                up_idx += 1
            while down_idx < len(pts_down) and pts_down[down_idx]['ang'] < ang:
                down_idx += 1

            pt_inter_up = np.zeros([3], dtype=np.float)
            if up_idx < len(pts_up):
                pt_inter_up = self.interpolate_angular(pts_up[up_idx - 1]['pt'],
                                                       pts_up[up_idx - 1]['ang'],
                                                       pts_up[up_idx]['pt'],
                                                       pts_up[up_idx]['ang'],
                                                       ang)

                # ang_inter_up = np.arctan2(pt_inter_up[1], pt_inter_up[0])
                # area_up = np.linalg.norm(np.cross(pt_inter_up - pts_up[up_idx - 1]['pt'],
                #                          pts_up[up_idx]['pt'] - pts_up[up_idx - 1]['pt']))
                # if abs(ang_inter_up - ang) > 1e-4:
                #     print("Difference between angles: %f vs %f" % (ang, ang_inter_up))
                # if area_up > 1e-12:
                #     print("Area not zero: %f" % area_up)
            else:
                pt_inter_up = pts_up[-1]['pt']

            pt_inter_down = np.zeros([3], dtype=np.float)
            if down_idx < len(pts_down):
                pt_inter_down = self.interpolate_angular(pts_down[down_idx - 1]['pt'],
                                                         pts_down[down_idx - 1]['ang'],
                                                         pts_down[down_idx]['pt'],
                                                         pts_down[down_idx]['ang'],
                                                         ang)
                # ang_inter_down = np.arctan2(pt_inter_down[1], pt_inter_down[0])
                # area_down = np.linalg.norm(np.cross(pt_inter_down - pts_down[down_idx - 1]['pt'],
                #                            pts_down[down_idx]['pt'] - pts_down[down_idx - 1]['pt']))
                # if abs(ang_inter_down - ang) > 1e-4:
                #     print("Difference between angles: %f vs %f" % (ang, ang_inter_down))
                # if area_down > 1e-12:
                #     print("Area not zero: %f" % area_down)
            else:
                pt_inter_down = pts_down[-1]['pt']

            pt_inter = self.interpolate_z(pt_inter_down, pt_inter_up, 0.0)

            # ang_inter = np.arctan2(pt_inter[1], pt_inter[0])
            # area = np.linalg.norm(np.cross(pt_inter - pt_inter_down,
            #                                pt_inter_up - pt_inter_down))
            # if abs(ang_inter - ang) > 1e-4:
            #     print("Difference between angles: %f vs %f" % (ang, ang_inter_down))
            # if area > 1e-12:
            #     print("Area not zero: %f" % area)

            ranges.append(np.linalg.norm(pt_inter))

        laser_msg = LaserScan()
        laser_msg.header = pc2_msg.header

        laser_msg.angle_min = angle_min
        laser_msg.angle_max = angle_max
        laser_msg.angle_increment = angle_increment
        laser_msg.time_increment = 0.1
        laser_msg.scan_time = 0.1
        laser_msg.range_min = 0.3
        laser_msg.range_max = 120
        laser_msg.ranges = ranges

        return laser_msg

    def laser_callback(self, pc2_msg):
        if self._laserscan_acquire:
            # here, because it is slow
            laser_msg = self.convert_pc2_to_laserscan(pc2_msg)
            self._laserscan_publisher.publish(laser_msg)

            self._data_saver.save_laser(laser_msg)

    def image_callback(self, image_msg):
        if not self._image_acquire.isSet():
            self._image_acquire.set()
            self._data_saver.save_image(image_msg)
