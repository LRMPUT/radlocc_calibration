# radlocc_calibrate

A package for easy calibration of lasers and cameras. It just saves the data in a format that RADLOCC can understand.

## Dataset Format

The dataset created by this packages follows the convention:

- Laserscan data is saved on the file `laser.txt`. This is just an text file which saves the laserscan data in the format `<timestamp> StartAngleRads AngleIncrementRads EndAngleRads RangeUnitType NoAngles [Ranges]`.
- Image data is both stored as image files named `image_XXX.png` and a text file `image_stamps.txt` with the timestamps of the images.

### Example dataset

An example dataset produced by this package can be found [here](https://www.dropbox.com/s/9iy1u8p12q66z3m/radlocc_data_atlas_right_laser_001.tar.gz?dl=0)

## Use

To use this package use the node `radlocc_collect` to save the calibration data, as follows:

```
rosrun radlocc_calibration radlocc_collect laserscan:=/laserscan image:=/camera/image_raw _output_dir:=~/radlocc_data_001
```

This node is interactive, so a user input (enter) is required to save each image/laserscan. When finished, write `q` and enter to exit.

## Nodes

### radlocc_collect

#### Subscibed topic

- `image` (`sensor_msgs/Image`): the image topic of the camera to be calibrated
- `laserscan` (`sensor_msgs/Laserscan`): the laserscan topic of the laser to be calibrated

#### Parameters

- `output_dir`: the path to save the dataset.
