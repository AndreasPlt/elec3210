# Lidar Odometry Project

## Introduction

This Lidar odometry project is based on the Point-to-Point ICP (Iterative Closest Point) algorithm. We have implemented three pipelines to improve odometry estimation:

1. **ICP Based on Previous Frame**:
   - Set the mode in the config to `previous_frame`.
   - The reference cloud (`refCloud`) uses the last scan.

2. **ICP Based on Key Frames**:
   - Set the mode in the config to `key_frame`.
   - The reference cloud (`refCloud`) uses a key frame to prevent drift propagation.

3. **ICP Based on Map**:
   - Set the mode in the config to `map_mode`.
   - The reference cloud (`refCloud`) uses a map that is built with key frames.

## Files

### Config: odometry.yaml

#### ICP Settings:

- `max_distance`: Maximum allowable distance between corresponding points for ICP.
- `max_iterations`: Maximum number of iterations for ICP.
- `transformation_epsilon`: Convergence criterion for ICP.
- `weight_mode`: Mode for weighting pairs in ICP. Choices: `distance_inverse`, `distance_max_scaling`, `uniform`.
- `reject_mode`: Mode for rejecting pairs in ICP. Choices: `none`, `threshold`, `percentage`.
- `reject_percentage`: Percentage for ICP pair rejection.
- `reject_threshold`: Threshold for ICP pair rejection.
- `debug_mode`: Debug mode for ICP. Choices: `true`, `false`.

#### Pipeline Settings:

- `update_mode`: Mode for updating the pipeline. Choices: `previous_frame`, `key_frame`, `map_mode`.

#### Key Frame Settings:

- `key_frame_mode`: Mode for key frame selection. Choices: `time_mode`, `overlap_mode`.
- `time_threshold`: Time threshold for key frame mode and map mode (time between key frame updates).
- `overlap_threshold`: Minimum overlap between key frames (used only for key frame mode).

#### Map Settings:

- `map_size`: Number of points in the map.
- `map_range`: Maximum distance from robot to map points.
- `remove_mode`: Mode for removing points from the map. Choices: `euclidean_norm`, `inf_norm`.

#### Visualization:

- `map_sample_size`: Sample size of every scan that is added to the global map for visualization.

### odometry.cpp

This file implements the entire odometry pipeline process. The main component is the odometry loop, which estimates the trajectory in every iteration based on ICP. Another key component is the switch statement, which controls the pipelines based on config settings. In this loop, the entire map based on all laser scans is published, but for the reference, the internal `refCloud` is used.

### icp.cpp

This file serves as an interface for the ICP class.

### icp_2.cpp

This contains the actual implementation of the Point-to-Point ICP algorithm. It includes functions to reject pairs and weight pairs.

### parameters.cpp

This file reads the parameters from the config.

## Usage

To run the project, follow these steps:

1. Execute the following commands:
   ```shell
   catkin_make
   source ./devel/setup.sh
   roslaunch icp_odometry icp.launch
   ```

## Authors

- Andreas Pletschko
- Lukas Vierling
- Christian Pesch

---