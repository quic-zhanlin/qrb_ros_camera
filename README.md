# qrb_ros_camera

qrb_ros_camera is a package to publish the camera data from camera sensor.

## Overview

Qualcomm Camera-Server provides camera data that obtained from the camera sensor. qrb_ros_camera use this camera-server to get the latest camera data. With camera ros2 node, data can achieve zero copy performance when coming out of the camera-server. This will greatly reduce the latency between ROS nodes.

This package is accelerated by [QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport), it leverages type adaption and intra process communication to optimize message formats and
dramatically accelerate communication between participating nodes.

> **Note:** To change the frame size and cameraid of the camera data, we need to change parameters in camera ros node launch file.
>
## Compile

Currently, we only support compiling using QCLINUX

1. Setup environments follow this document 's `6.1 Set up the runtime environment for samples` part in: <document link>

2. Clone this repository under `${QIRP_SDK_PATH}/ws`
     ```bash
     git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
     git clone https://github.qualcomm.com/QUIC-QRB-ROS/qrb_ros_camera.git
     ```
3. Build this project
     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

    colcon build --merge-install --cmake-args \
      -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
      -DSYSROOT_LIBDIR=${OECORE_TARGET_SYSROOT}/usr/lib \
      -DSYSROOT_INCDIR=${OECORE_TARGET_SYSROOT}/usr/include \
      -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
      -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
      -DBUILD_TESTING=OFF
     ```
4. Push to the device & Install
     ```bash
     cd ${QIRP_SDK_PATH}/ws/install
     tar czvf qrb_ros_camera.tar.gz include lib share
     scp qrb_ros_camera.tar.gz root@[ip-addr]:/opt/
     ssh ssh root@[ip-addr]
     (ssh) tar -zxf /opt/qrb_ros_camera.tar.gz -C /opt/qcom/qirp-sdk/usr/
     ```

## Execute

This package supports direct execution and dynamic addition to the component container

a.Direct Execution

1. Source this file to set up the environment on your device:
    ```bash
    ssh root@[ip-addr]
    (ssh) export HOME=/opt
    (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
    (ssh) export ROS_DOMAIN_ID=xx
    (ssh) source /usr/bin/ros_setup.bash
    ```

2. Use this command to run this package
    ```bash
    (ssh) ros2 launch qrb_ros_camera qrb_ros_camera_launch.py
    ```

b. Add it to the component container
```python
ComposableNode(
    package='qrb_ros_camera',
    plugin='qrb_ros::camera::CameraNode',
    name='camera_node'
)
```

## Acceleration

This package is powered by [QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport) to optimize message formats and accelerate communication between participating nodes.

## Packages

Will update in the future.

## Resources

- [ROS2 Type Adaption](https://ros.org/reps/rep-2007.html)

## Contributions

Thanks for your interest in contributing to qrb_ros_camera! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_camera is licensed under the BSD-3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.
