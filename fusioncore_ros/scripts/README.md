# FusionCore Analysis Node

The analysis node is provided by the standalone `fusioncore_analysis` package. It records selected FusionCore and sensor topics, writes their data to CSV files, and generates diagnostic plots. It can also regenerate plots offline from an existing analysis directory.

## Requirements

Run the node in a sourced ROS 2 workspace with `fusioncore_analysis` built. It uses the ROS 2 Python packages for `rclpy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, and `fusioncore_ros`, along with NumPy and Matplotlib.

From the workspace root:

```bash
colcon build --packages-up-to fusioncore_analysis
source install/setup.bash
```

## Online recording

Start the node without `--offline-dir`:

```bash
ros2 run fusioncore_analysis fusioncore_analysis_node
```

The node subscribes to:

| Topic | Message type | Recorded data |
|---|---|---|
| `/imu/data` | `sensor_msgs/msg/Imu` | Angular velocity, linear acceleration, and their reported standard deviations |
| `/fix` | `sensor_msgs/msg/NavSatFix` | Valid GNSS fixes converted to local east/north coordinates and reported position uncertainty |
| `/fusion/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Position, yaw, and pose uncertainty |
| `/fusion/odom` | `nav_msgs/msg/Odometry` | Pose, twist, and their uncertainties |
| `/ona2/fake_imu` | `sensor_msgs/msg/Imu` | Heading yaw and orientation uncertainty |
| `/fusion/debug/filter_health` | `fusioncore_ros/msg/FilterHealth` | IMU bias estimates |

By default, each completed analysis is saved under `~/fusioncore_analysis/<timestamp>/`. Set a different output root with a ROS parameter:

```bash
ros2 run fusioncore_analysis fusioncore_analysis_node --ros-args -p output_dir:=/path/to/output
```

The node saves on shutdown, including Ctrl+C. By default, it also saves and shuts down after 5 seconds without receiving messages. Change or disable this behavior with `idle_timeout_sec`; a value of `0.0` disables the idle timeout. `save_period_sec` defaults to `0.0` (disabled). If set, it triggers one save at the first timer interval; the current implementation saves at most once per process, not repeatedly.

For example, to disable idle shutdown and take one snapshot after 30 seconds:

```bash
ros2 run fusioncore_analysis fusioncore_analysis_node --ros-args \
  -p idle_timeout_sec:=0.0 -p save_period_sec:=30.0
```

## Offline plot regeneration

Pass the path to an existing analysis directory using `--offline-dir`:

```bash
ros2 run fusioncore_analysis fusioncore_analysis_node \
  --offline-dir /Pathtocsvs
```

The script reads any available recognized CSVs and writes the regenerated images into that same directory. It does not subscribe to topics or rewrite the recorded CSVs. Existing images with the same names are overwritten. The directory must exist and contain at least one recognized CSV file.

The expected CSV files are:

- `imu.csv`
- `gnss.csv`
- `fusion_pose.csv`
- `fusion_odom.csv`
- `fake_imu.csv`
- `imu_bias.csv`

The files are read in their existing column order. They should be the CSVs produced by this script; arbitrary CSVs with different columns are not supported. A plot is skipped if its required data is missing. The yaw plot also requires both FusionCore pose/odometry data and `fake_imu.csv`.

## Generated plots

| File | Contents |
|---|---|
| `01_imu_measurements_and_covariance.png` | IMU angular velocity, acceleration, and reported standard deviations |
| `02_gnss_track_and_covariance.png` | GNSS local track, position uncertainty, and fix status |
| `03_fusion_pose_and_covariance.png` | Fusion position, yaw, and pose uncertainty |
| `04_fusion_odom_twist_and_covariance.png` | Fused twist and twist uncertainty |
| `05_gnss_vs_fusion.png` | GNSS and FusionCore position comparison, plus yaw and uncertainty |
| `06_fusion_yaw_vs_fake_imu.png` | FusionCore yaw compared with `/ona2/fake_imu` yaw |
| `07_imu_bias_estimates.png` | Estimated gyroscope and accelerometer biases |

During online recording, the script also writes `06_yaw_inputs.csv`, containing the paired yaw values and their difference. Offline regeneration does not recreate this derived CSV; it only regenerates plots.
