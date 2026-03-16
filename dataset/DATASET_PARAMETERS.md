# Dataset Parameters Reference

## Overview

The dataset is organized into two top-level directories:

```
dataset/
├── flight-dataset/         # UAV flight trials with UWB TDOA measurements
└── identification-dataset/ # LOS/NLOS classification experiments
```

---

## 1. Flight Dataset (`flight-dataset/`)

### Directory Structure

```
flight-dataset/
├── binary-data/           # Raw binary log files per constellation/trial
│   ├── const1/
│   ├── const2/
│   ├── const3/
│   └── const4/
├── csv-data/              # Parsed CSV files per constellation/trial
│   ├── const1/            # 12 files (6 trials × 2 TDOA modes)
│   ├── const2/            # 12 files
│   ├── const3/            # 16 files
│   └── const4/            # 39 files
├── rosbag-data/           # ROS bag files per constellation/trial (.bag)
│   ├── const1/
│   ├── const2/
│   ├── const3/
│   └── const4/
└── survey-results/        # Anchor position/orientation surveys per constellation
    ├── anchor_const1.npz
    ├── anchor_const1_survey.txt
    ├── anchor_const2.npz
    ├── anchor_const2_survey.txt
    ├── anchor_const3.npz
    ├── anchor_const3_survey.txt
    ├── anchor_const4.npz
    └── anchor_const4_survey.txt
```

File naming convention: `const{N}-trial{M}-tdoa{2|3}.{ext}`
- `const{N}` — anchor constellation index (1–4)
- `trial{M}` — trial index within that constellation
- `tdoa2` / `tdoa3` — TDOA measurement mode (TDoA2 or TDoA3 protocol)

---

### 1.1 CSV Data Files (`csv-data/`)

**File:** `const{N}-trial{M}-tdoa{2|3}.csv`

Each row is one measurement epoch. Columns:

| Column | Description |
|--------|-------------|
| `t_tdoa` | Timestamp of the TDOA measurement (s) |
| `idA` | ID of anchor A (transmitter / reference) |
| `idB` | ID of anchor B (responder) |
| `tdoa_meas` | Measured TDOA value (m, converted from time) |
| `t_acc` | Timestamp of accelerometer reading (s) |
| `acc_x` | Accelerometer X-axis (m/s²) |
| `acc_y` | Accelerometer Y-axis (m/s²) |
| `acc_z` | Accelerometer Z-axis (m/s²) |
| `t_gyro` | Timestamp of gyroscope reading (s) |
| `gyro_x` | Gyroscope X-axis (rad/s) |
| `gyro_y` | Gyroscope Y-axis (rad/s) |
| `gyro_z` | Gyroscope Z-axis (rad/s) |
| `t_tof` | Timestamp of time-of-flight reading (s) |
| `tof` | Time-of-flight distance (m) |
| `t_flow` | Timestamp of optical flow reading (s) |
| `deltaX` | Optical flow X displacement (pixels) |
| `deltaY` | Optical flow Y displacement (pixels) |
| `t_baro` | Timestamp of barometer reading (s) |
| `baro` | Barometric pressure/altitude reading (mbar or m) |
| `t_pose` | Timestamp of ground-truth pose (s) |
| `pose_x` | Ground-truth position X (m) |
| `pose_y` | Ground-truth position Y (m) |
| `pose_z` | Ground-truth position Z (m) |
| `pose_qx` | Ground-truth orientation quaternion — x component |
| `pose_qy` | Ground-truth orientation quaternion — y component |
| `pose_qz` | Ground-truth orientation quaternion — z component |
| `pose_qw` | Ground-truth orientation quaternion — w component |

---

### 1.2 Survey Result Files (`survey-results/`)

**File:** `anchor_const{N}_survey.txt`

Defines the 3D position and orientation of each anchor in the constellation.
One anchor per entry pair (8 anchors per constellation, indexed 0–7).

| Entry | Format | Description |
|-------|--------|-------------|
| `an{i}_p` | `x, y, z` | Anchor position in world frame (m) |
| `an{i}_quat` | `qx, qy, qz, qw` | Anchor orientation as unit quaternion |

Example (const1, anchor 0):
```
an0_p,-2.4174718660841163,-4.020796001114614,0.18179046793237785
an0_quat,0.5155113592874482,0.4787024508794819,0.5261555809407699,-0.4777575814285574
```

**File:** `anchor_const{N}.npz`

NumPy archive (`.npz`) containing the same anchor survey data in array format.

---

## 2. Identification Dataset (`identification-dataset/`)

Used for LOS/NLOS channel classification experiments.

### Directory Structure

```
identification-dataset/
├── los/                         # Line-of-sight measurements
│   ├── angleTest/               # Varying antenna angle (fixed distance)
│   │   ├── angleT1/ … angleT12/
│   │   │   ├── angleT{N}_data.csv
│   │   │   └── angleT{N}_pose.txt
│   └── distTest/                # Varying tag distance (fixed angle)
│       ├── distT1/ … distT12/
│       │   ├── distT{N}_data.csv
│       │   └── distT{N}_pose.txt
└── nlos/                        # Non-line-of-sight measurements
    ├── anAn/                    # Obstruction between anchor and anchor
    │   ├── cardboard/ … wooden-shelf/
    │   │   └── data{1..6}/
    │   │       ├── {material}-anAn-data{N}_data.csv
    │   │       └── {material}-anAn-data{N}_pose.txt
    ├── anTag/                   # Obstruction between anchor and tag
    │   ├── cardboard/ … wooden-shelf/
    │   │   └── data{1..6}/
    │   │       ├── {material}-anTag-data{N}_data.csv
    │   │       └── {material}-anTag-data{N}_pose.txt
    └── los-data/                # LOS reference data collected in the NLOS setup
        ├── los-data_data.csv
        └── los-data_pose.txt
```

NLOS obstruction materials: `cardboard`, `foam`, `metal`, `plastic`, `wooden-cabinet`, `wooden-shelf`

---

### 2.1 Data CSV Files (`*_data.csv`)

All data CSV files in the identification dataset share the same column schema:

| Column | Description |
|--------|-------------|
| `tdoa12` | TDOA measured at anchor 1 relative to anchor 2 (m) |
| `tdoa21` | TDOA measured at anchor 2 relative to anchor 1 (m) |
| `snr_an1` | Signal-to-noise ratio of signal received at anchor 1 (dB) |
| `power_dif_an1` | Receive power difference (first path vs. total) at anchor 1 (dB) |
| `snr_an2` | Signal-to-noise ratio of signal received at anchor 2 (dB) |
| `power_dif_an2` | Receive power difference at anchor 2 (dB) |
| `an1_rx_snr` | SNR of the signal received by the tag from anchor 1 (dB) |
| `an1_rx_powerdif` | Power difference of the signal received by the tag from anchor 1 (dB) |
| `an1_tof` | Time-of-flight from anchor 1 to tag (ns or m equivalent) |
| `an2_rx_snr` | SNR of the signal received by the tag from anchor 2 (dB) |
| `an2_rx_powerdif` | Power difference of the signal received by the tag from anchor 2 (dB) |
| `an2_tof` | Time-of-flight from anchor 2 to tag (ns or m equivalent) |

---

### 2.2 Pose Text Files (`*_pose.txt`)

Each experiment configuration is described by a pose file. Fields vary slightly between LOS and NLOS experiments.

#### LOS Pose Files (`los/angleTest/`, `los/distTest/`, `nlos/los-data/`)

| Entry | Format | Description |
|-------|--------|-------------|
| `an1_p` | `x, y, z` | Anchor 1 position in world frame (m) |
| `an2_p` | `x, y, z` | Anchor 2 position in world frame (m) |
| `tag_p` | `x, y, z` | Tag position in world frame (m) |
| `an1_quat` | `qx, qy, qz, qw` | Anchor 1 orientation as unit quaternion |
| `an2_quat` | `qx, qy, qz, qw` | Anchor 2 orientation as unit quaternion |
| `tag_quat` | `qx, qy, qz, qw` | Tag orientation as unit quaternion |

#### NLOS Pose Files (`nlos/anAn/`, `nlos/anTag/`)

Same fields as LOS plus obstacle corner markers:

| Entry | Format | Description |
|-------|--------|-------------|
| `an1_p` | `x, y, z` | Anchor 1 position (m) |
| `an2_p` | `x, y, z` | Anchor 2 position (m) |
| `tag_p` | `x, y, z` | Tag position (m) |
| `an1_quat` | `qx, qy, qz, qw` | Anchor 1 orientation |
| `an2_quat` | `qx, qy, qz, qw` | Anchor 2 orientation |
| `tag_quat` | `qx, qy, qz, qw` | Tag orientation |
| `obs_m1` | `x, y, z` | Obstacle corner marker 1 position (m) |
| `obs_m2` | `x, y, z` | Obstacle corner marker 2 position (m) |
| `obs_m3` | `x, y, z` | Obstacle corner marker 3 position (m) |
| `obs_m4` | `x, y, z` | Obstacle corner marker 4 position (m) |

The four obstacle markers (`obs_m1`–`obs_m4`) define the corners of the obstruction plane placed between the UWB devices.

---

## Parameter Units Summary

| Parameter | Unit |
|-----------|------|
| Position (`_p`, `pose_x/y/z`) | meters (m) |
| Orientation (`_quat`, `pose_q*`) | unit quaternion (dimensionless) |
| TDOA (`tdoa_meas`, `tdoa12`, `tdoa21`) | meters (m) |
| Time-of-flight (`an1_tof`, `an2_tof`, `tof`) | meters (m) or nanoseconds (ns) |
| SNR (`snr_*`, `*_rx_snr`) | dB |
| Power difference (`power_dif_*`, `*_rx_powerdif`) | dB |
| Timestamps (`t_*`) | seconds (s) |
| Accelerometer (`acc_*`) | m/s² |
| Gyroscope (`gyro_*`) | rad/s |
| Optical flow (`deltaX`, `deltaY`) | pixels |
| Barometer (`baro`) | mbar or m |
