ouster_description
==================

ROS 2 / ament URDF descriptions for Ouster lidar sensors including Gazebo sensor simulation
configurations. Originally derived from the ROS 1 ouster_description package at
https://github.com/ouster-lidar/ouster_example.

All specs are sourced from the Ouster REV7 datasheets (FW 3.1 for OS0/OS1/OSDome,
FW 2.5 for OS2), located in the `datasheets/` directory.

## Sensor Variants

| URDF | Channels | Vertical FOV | Max Range | Range Noise (1σ) | IMU |
|---|---|---|---|---|---|
| `OS0-32` | 32 | ±45.4° (90.8° total) | 75 m | 8 mm | IAM-20680HT |
| `OS0-64` | 64 | ±45.4° (90.8° total) | 75 m | 8 mm | IAM-20680HT |
| `OS0-128` | 128 | ±45.4° (90.8° total) | 75 m | 8 mm | IAM-20680HT |
| `OS1-32` | 32 | ±21.2° (42.4° total) | 90 m | 5 mm | IAM-20680HT |
| `OS1-64` | 64 | ±21.2° (42.4° total) | 90 m | 5 mm | IAM-20680HT |
| `OS1-128` | 128 | ±21.2° (42.4° total) | 90 m | 5 mm | IAM-20680HT |
| `OS2-32` | 32 | ±11.25° (22.5° total) | 200 m | 20 mm | ICM-20948 |
| `OS2-64` | 64 | ±11.25° (22.5° total) | 200 m | 20 mm | ICM-20948 |
| `OS2-128` | 128 | ±11.25° (22.5° total) | 200 m | 20 mm | ICM-20948 |
| `OSDome-32` | 32 | ±90° (180° hemisphere) | 45 m | 10 mm | IAM-20680HT |
| `OSDome-64` | 64 | ±90° (180° hemisphere) | 45 m | 10 mm | IAM-20680HT |
| `OSDome-128` | 128 | ±90° (180° hemisphere) | 45 m | 10 mm | IAM-20680HT |

All sensors scan a full 360° horizontal FOV at 10 or 20 Hz (configurable). Max range values
are the 10% reflectivity figure for OS1 and the 80% reflectivity headline figure for all others.
The OS2 maximum range is 350 m at 80% reflectivity.

## IMU Noise Parameters

Each xacro exposes three parameters for the yaw-axis (Z) gyroscope noise model, which feeds
a Gauss-Markov dynamic bias (bias instability / low-frequency drift):

| Parameter | Description | IAM-20680HT default | ICM-20948 default |
|---|---|---|---|
| `imu_yaw_noise_stddev` | White noise std dev (rad/s) | 0.005 | 0.005 |
| `imu_yaw_bias_stddev` | Gauss-Markov steady-state bias std dev (rad/s) | 2.4e-5 (~5 °/hr) | 4.85e-5 (~10 °/hr) |
| `imu_yaw_bias_correlation_time` | Bias drift time constant (s) | 1000.0 | 1000.0 |

## Physical Notes

- **OSDome** faces upward and covers the full upper hemisphere. The `lidar_link` is placed at
  the top of the dome body (the beam aperture center).

## Usage

```xml
<xacro:include filename="$(find ouster_description)/urdf/OS1-32.urdf.xacro"/>
<xacro:OS1-32 parent="base_link" name="os1_sensor" hz="10" samples="1024">
  <origin xyz="0 0 0.25" rpy="0 0 0" />
</xacro:OS1-32>
```

All macro parameters have defaults and are overridable:

```xml
<xacro:OS2-64 parent="base_link" name="os2" hz="10" samples="1024"
    max_range="200.0" imu_yaw_bias_stddev="0.0000485">
  <origin xyz="0 0 0.3" rpy="0 0 0" />
</xacro:OS2-64>
```
