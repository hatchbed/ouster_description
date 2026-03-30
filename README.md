ouster_description
==================

ROS 2 / ament URDF descriptions for Ouster lidar sensors including Gazebo sensor simulation
configurations. Originally derived from the ROS 1 ouster_description package at
https://github.com/wilselby/ouster_example.


## Sensor Variants

| URDF | Channels | Vertical FOV | Max Range | Range Noise (1σ) | IMU |
|---|---|---|---|---|---|
| `OS0-32` | 32 | ±45.4° (90.8° total) | 75 m | 8 mm | IAM-20680HT |
| `OS0-64` | 64 | ±45.4° (90.8° total) | 75 m | 8 mm | IAM-20680HT |
| `OS0-128` | 128 | ±45.4° (90.8° total) | 75 m | 8 mm | IAM-20680HT |
| `OS1-32` | 32 | ±21.2° (42.4° total) | 90 m | 8 mm | IAM-20680HT |
| `OS1-64` | 64 | ±21.2° (42.4° total) | 90 m | 8 mm | IAM-20680HT |
| `OS1-128` | 128 | ±21.2° (42.4° total) | 90 m | 8 mm | IAM-20680HT |
| `OS2-32` | 32 | ±11.25° (22.5° total) | 200 m | 20 mm | ICM-20948 |
| `OS2-64` | 64 | ±11.25° (22.5° total) | 200 m | 20 mm | ICM-20948 |
| `OS2-128` | 128 | ±11.25° (22.5° total) | 200 m | 20 mm | ICM-20948 |
| `OSDome-32` | 32 | ±90° (180° hemisphere) | 45 m | 10 mm | IAM-20680HT |
| `OSDome-64` | 64 | ±90° (180° hemisphere) | 45 m | 10 mm | IAM-20680HT |
| `OSDome-128` | 128 | ±90° (180° hemisphere) | 45 m | 10 mm | IAM-20680HT |

All sensors scan a full 360° horizontal FOV at 10 or 20 Hz (configurable). Max range values
are the 80% reflectivity figure for OS0 and OSDome, and the 10% reflectivity figure for OS1
and OS2. OS2 maximum range is 350 m at 80% reflectivity.

## Macro Parameters

### OS0 variants

The OS0 macros expose `base` and `cap` parameters to control which physical components are
included. The sensor body, base, and caps are all part of a single link.

| Parameter | Default | Description |
|---|---|---|
| `base` | `true` | Include the rectangular mounting base |
| `cap` | `halo` | Top cap style: `halo`, `fin`, or `none` |

The channel count (32/64/128) is inferred automatically from the macro name and is not a
user-visible parameter.

### OS1 variants

The OS1 shares the same physical hardware as the OS0 — uses OS0 meshes and exposes the
same `base` and `cap` parameters.

| Parameter | Default | Description |
|---|---|---|
| `base` | `true` | Include the rectangular mounting base |
| `cap` | `halo` | Top cap style: `halo`, `fin`, or `none` |

### OS2 variants

| Parameter | Default | Description |
|---|---|---|
| `base` | `true` | Include the rectangular mounting base |

### OSDome variants

| Parameter | Default | Description |
|---|---|---|
| `base` | `true` | Include the rectangular mounting base (shared OS0 base) |

## IMU Noise Parameters

Each xacro exposes three parameters for the yaw-axis (Z) gyroscope noise model, which feeds
a Gauss-Markov dynamic bias (bias instability / low-frequency drift):

| Parameter | Description | IAM-20680HT default | ICM-20948 default |
|---|---|---|---|
| `imu_yaw_noise_stddev` | White noise std dev (rad/s) | 0.005 | 0.005 |
| `imu_yaw_bias_stddev` | Gauss-Markov steady-state bias std dev (rad/s) | 2.42e-5 (~5 °/hr) | 4.85e-5 (~10 °/hr) |
| `imu_yaw_bias_correlation_time` | Bias drift time constant (s) | 1000.0 | 1000.0 |

## Physical Notes

- **OSDome** faces upward and covers the full upper hemisphere. The `lidar_link` is placed at
  the top of the dome body (the beam aperture center).

## Usage

```xml
<xacro:include filename="$(find ouster_description)/urdf/OS0-128.urdf.xacro"/>
<xacro:OS0-128 parent="base_link" name="os0_sensor" hz="10">
  <origin xyz="0 0 0.25" rpy="0 0 0" />
</xacro:OS0-128>
```

```xml
<!-- OS0 without base, with fin cap -->
<xacro:OS0-64 parent="base_link" name="os0_sensor" base="false" cap="fin">
  <origin xyz="0 0 0.25" rpy="0 0 0" />
</xacro:OS0-64>
```

```xml
<xacro:include filename="$(find ouster_description)/urdf/OS2-64.urdf.xacro"/>
<xacro:OS2-64 parent="base_link" name="os2_sensor" hz="10" max_range="200.0">
  <origin xyz="0 0 0.3" rpy="0 0 0" />
</xacro:OS2-64>
```
