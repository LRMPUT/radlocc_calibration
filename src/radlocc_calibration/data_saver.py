from __future__ import with_statement
import os
import cv2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation as R

class DataSaver(object):
    """
    The data saver for Radlocc.

    Stores the both the images and the laserscans into the text format
    required for the radlocc to read, which is the laserscans text file,
    which is of the following format:
    `<timestamp> StartAngleRads AngleIncrementRads EndAngleRads RangeUnitType NoAngles [Ranges]`
    and the images text file, which only has the timestamps, corresponding
    to an image stored in disk, one for each row in the text file.
    """

    def __init__(self, path):
        """
        Parameters
        ----------
        path: str
            The base path for storing the files. Either it exists and it's an empty directory or it
            is mkdir'ed during the initialization.
        """
        if os.path.exists(path):
            assert os.path.isdir(
                path), "path, if exists, should be a directory"
            assert os.listdir(path) == [], "path, if exists, should be empty"
        else:
            os.mkdir(path)

        self._base_path = path

        self._laserscans = []
        self._images = []

        self._image_id = 1

        self._cv_bridge = CvBridge()

    def save_image(self, image_msg):
        """
        Saves an image to the radlocc dataset.

        Saves the image to disk with the filename 'image_XXX.png' and
        the stamp to the 'image_stamps.txt' file.
        """

        image_id = self._image_id
        self._image_id += 1
        image_path = os.path.join(
            self._base_path, 'image_{:03d}.png'.format(image_id))

        image = self._cv_bridge.imgmsg_to_cv2(image_msg)
        cv2.imwrite(image_path, image)

        self._images.append({
            'secs': image_msg.header.stamp.secs,
            'nsecs': image_msg.header.stamp.nsecs,
        })

    def line_intersection(self, pt1, pt2, r):
        r3 = pt2 - pt1
        r3 = r3 / np.linalg.norm(r3)
        a = np.linalg.norm(np.cross(pt1, r3)) / np.linalg.norm(r, r3)

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

    def save_laser(self, laserscan):
        """
        Saves the laserscan to the file 'laser.txt'.
        """

        stamp = laserscan.header.stamp
        scan = pc2.read_points(laserscan, field_names={'x', 'y', 'z'})
        pts_up = []
        pts_down = []
        for cur_pt in scan:
            ang = np.arctan2(cur_pt['x'], cur_pt['y'])
            cur_pt_aug = {'pt': np.array([cur_pt['x'], cur_pt['y'], cur_pt['z']]),
                          'ang': ang}
            if cur_pt['z'] >= 0.0:
                pts_up.append(cur_pt_aug)
            else:
                pts_down.append(cur_pt_aug)
        pts_up.sort(key=lambda cur_pt: cur_pt['ang'])
        pts_down.sort(key=lambda cur_pt: cur_pt['ang'])
        angle_min = -45 * np.pi / 180.0
        angle_max = 45 * np.pi / 180.0
        angle_increment = 0.25
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
            else:
                pt_inter_up = pts_up[-1]['pt']

            pt_inter_down = np.zeros([3], dtype=np.float)
            if down_idx < len(pts_down):
                pt_inter_down = self.interpolate_angular(pts_down[down_idx - 1]['pt'],
                                                       pts_down[down_idx - 1]['ang'],
                                                       pts_down[down_idx]['pt'],
                                                       pts_down[down_idx]['ang'],
                                                       ang)
            else:
                pt_inter_down = pts_down[-1]['pt']

            pt_inter = self.interpolate_z(pt_inter_down, pt_inter_up, 0.0)

            ranges.append(np.linalg.norm(pt_inter))

        self._laserscans.append({
            'timestamp': '{secs}.{nsecs}'.format(secs=stamp.secs, nsecs=stamp.nsecs),
            'angle_min': angle_min,
            'angle_increment': angle_increment,
            'angle_max': angle_max,
            'range_unit_type': 3,
            'ranges': ranges,
        })

    def save(self):
        """
        Saves the files to the disk.
        """

        with open(os.path.join(self._base_path, 'laser.txt'), 'w') as laser_file:
            for scan in self._laserscans:
                ranges = scan['ranges']
                header = [
                    scan['timestamp'],
                    scan['angle_min'],
                    scan['angle_increment'],
                    scan['angle_max'],
                    scan['range_unit_type'],
                    len(ranges)
                ]
                row = ' '.join([str(h) for h in header]) + ' ' + \
                    ' '.join([str(r) for r in ranges]) + '\n'

                laser_file.write(row)

        with open(os.path.join(self._base_path, 'image_stamps.txt'), 'w') as image_stamps_file:
            for scan in self._images:
                row = '{secs}.{nsecs} {secs}.{nsecs}' \
                    .format(secs=str(scan['secs']), nsecs=str(scan['nsecs']))

                image_stamps_file.write(row + '\n')
