# WHEELTEC_N100_IMU_ROS2

WHEELTEC N100 IMU ROS2 driver package.

## Branches
- `jazzy`: tuned for ROS 2 Jazzy / Ubuntu 24.04.

## Quick run
```bash
ros2 run wheeltec_n100_imu imu_node --ros-args -p serial_port:=/dev/ttyUSB0
```

> 기존 `work` 브랜치는 실험 코드용으로 남겨두고, `jazzy` 브랜치에서 안정화/최적화를 진행했습니다.
