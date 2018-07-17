from __future__ import with_statement
import os
import cv2
from cv_bridge import CvBridge

class DataSaver:
    """
    The data saver for Radlocc.

    Stores the both the images and the laserscans into the text format required for the radlocc to read, 
    which is the laserscans text file, which is of the following format:
    `<timestamp> StartAngleRads AngleIncrementRads EndAngleRads RangeUnitType NoAngles [Ranges]`
    and the images text file, which only has the timestamps, corresponding to an image stored in disk,
    one for each row in the text file.
    """

    def __init__(self, path):
        """
        Parameters
        ----------
        path: str
            The base path for storing the files. Either it exists and it's an empty directory or it
            is mkdir'ed during the initialization.
        """
        if(os.path.exists(path)):
            assert os.path.isdir(path), "path, if exists, should be a directory"
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

        Saves the image to disk with the filename 'image_XXX.png' and the stamp to the 'image_stamps.txt' file.
        """

        id = self._image_id
        self._image_id += 1
        image_path = os.path.join(self._base_path, 'image_{:03d}.png'.format(id))

        image = self._cv_bridge.imgmsg_to_cv2(image_msg)
        cv2.imwrite(image_path, image)

        self._images.append({
            'secs': image_msg.header.stamp.secs,
            'nsecs': image_msg.header.stamp.nsecs,
        })

    def save_laser(self, laserscan):
        """
        Saves the laserscan to the file 'laser.txt'.
        """

        stamp = laserscan.header.stamp
        self._laserscans.append({
            'timestamp': '{secs}.{nsecs}'.format(secs=stamp.secs, nsecs=stamp.nsecs),
            'angle_min': laserscan.angle_min,
            'angle_increment': laserscan.angle_increment,
            'angle_max': laserscan.angle_max,
            'range_unit_type': 3,
            'ranges': laserscan.ranges,
        })

    def save(self):
        """
        Saves the files to the disk.
        """

        with open(os.path.join(self._base_path, 'laser.txt'), 'w') as f:
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
                row = ' '.join(list(map(str,header))) + ' ' + ' '.join(list(map(str,ranges))) + '\n'

                f.write(row)

        with open(os.path.join(self._base_path, 'image_stamps.txt'), 'w') as f:
            for scan in self._images:
                row = '{secs}.{nsecs} {secs}.{nsecs}' \
                    .format(secs=str(scan['secs']), nsecs=str(scan['nsecs']))

                f.write(row + '\n')
