# QRB ROS CAMERA

## Overview

`qrb_ros_camera` is a ROS package to publish the camera data from camera sensor.

`qrb_ros_camera` holds significant importance as it serves as a critical component for numerous other ROS nodes that 
rely on image data for analysis and processing. By providing images in the NV12 format, it enables seamless integration 
with a wide range of ROS nodes, empowering advanced image-based analysis and functionalities within the ROS ecosystem.

`qrb_ros_camera` features zero-copy transport for reduced latency, facilitating swift image transmission. It also 
supports concurrent output from multiple cameras, enhancing system performance and scalability in real-time ROS 
applications.

## Quick Start

> **Note：**
> This document 's build & run is the latest.
> If it conflict with the online document, please follow this.

We provide two ways to use this package.

<details>
<summary>Docker</summary>

#### Setup
1. Please follow this [steps](https://github.com/quic-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart) to setup docker env.
2. Download qrb_ros_camera and dependencies
    ```bash
    cd ${QRB_ROS_WS}/src

    git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/quic-qrb-ros/qrb_ros_camera.git
    git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
    ```

#### Build
```bash
colcon build
```

#### Run
```bash
cd ${QRB_ROS_WS}/src

source install/local_setup.sh
ros2 launch qrb_ros_camera qrb_ros_camera_launch.py
```

</details>
 

<details>
<summary>QIRP-SDK</summary>

#### Setup
1. Please follow this [steps](https://quic-qrb-ros.github.io/getting_started/index.html) to setup qirp-sdk env.
2. Download qrb_ros_imu and dependencies
    ```bash
    mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

    git clone https://github.com/quic-qrb-ros/qrb_ros_camera.git
    ```

#### Build
1. Build the project
    ```bash
    export AMENT_PREFIX_PATH="${OECORE_NATIVE_SYSROOT}/usr:${OECORE_TARGET_SYSROOT}/usr"
    export PYTHONPATH=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/:${OECORE_TARGET_SYSROOT}/usr/lib/python3.12/site-packages/

    colcon build --continue-on-error --cmake-args \
      -DCMAKE_TOOLCHAIN_FILE=${OE_CMAKE_TOOLCHAIN_FILE} \
      -DPYTHON_EXECUTABLE=${OECORE_NATIVE_SYSROOT}/usr/bin/python3 \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
      -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
      -DBUILD_TESTING=OFF
    ```
2. Install the package
    ```bash
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws/install/qrb_ros_camera
    tar -czvf qrb_ros_camera.tar.gz include lib share
    scp qrb_ros_camera.tar.gz root@[ip-addr]:/home/
    (ssh) mount -o remount rw /
    (ssh) tar --no-overwrite-dir --no-same-owner -zxf /home/qrb_ros_camera.tar.gz -C /usr/
    ```

#### Run
```bash
(ssh) export HOME=/home
(ssh) setenforce 0
(ssh) source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh
(ssh) ros2 launch qrb_ros_camera qrb_ros_camera_launch.py
```

</details>

<br>

You can get more details from [here](https://quic-qrb-ros.github.io/main/index.html).

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcomm RB3 gen2.

| Hardware                                                     | Software          |
| ------------------------------------------------------------ | ----------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | `LE.QCROBOTICS.1.0`, `Canonical Ubuntu Image for RB3 gen2` |

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

## Authors

* **Tian Ding** - *Initial work* - [dingtian777](https://github.com/dingtian777)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

