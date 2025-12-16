English| [简体中文](./README_cn.md)

Getting Started with faceid
=======

# Feature Introduction

The faceid package is a usage example based on face instance detecion model designed by D-robotics. The image data comes from local image feedback and subscribed image msg. The faceid relies on the input of the person face's detection box, and extract the feature compared with the dataset, finally get the face id. 

# Development Environment

- Programming Language: C/C++
- Development Platform: X5
- System Version: Ubuntu 22.04
- Compilation Toolchain: Linaro GCC 11.4.0

# Compilation

- X5 Version: Supports compilation on the X5 Ubuntu system and cross-compilation using Docker on a PC.

It also supports controlling the dependencies and functionality of the compiled pkg through compilation options.

## Dependency Libraries

- OpenCV: 3.4.5

ROS Packages:

- dnn node
- cv_bridge
- sensor_msgs
- hbm_img_msgs
- ai_msgs

hbm_img_msgs is a custom image message format used for image transmission in shared memory scenarios. The hbm_img_msgs pkg is defined in hobot_msgs; therefore, if shared memory is used for image transmission, this pkg is required.

## Compilation Options

1. SHARED_MEM

- Shared memory transmission switch, enabled by default (ON), can be turned off during compilation using the -DSHARED_MEM=OFF command.
- When enabled, compilation and execution depend on the hbm_img_msgs pkg and require the use of tros for compilation.
- When disabled, compilation and execution do not depend on the hbm_img_msgs pkg, supporting compilation using native ROS and tros.
- For shared memory communication, only subscription to nv12 format images is currently supported.## Compile on X3/Rdkultra Ubuntu System

1. Compilation Environment Verification

- The X5 Ubuntu system is installed on the board.
- The current compilation terminal has set up the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- The ROS2 compilation tool colcon is installed. If the installed ROS does not include the compilation tool colcon, it needs to be installed manually. Installation command for colcon: `pip install -U colcon-common-extensions`.
- The dnn node package has been compiled.

2. Compilation

- Compilation command: `colcon build --packages-select faceid`

## Docker Cross-Compilation for X5 Version

1. Compilation Environment Verification

- Compilation within docker, and TogetherROS has been installed in the docker environment. For instructions on docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.
- The dnn node package has been compiled.
- The hbm_img_msgs package has been compiled (see Dependency section for compilation methods).

2. Compilation

- Compilation command:

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s faceid
  ```

- Shared memory communication method is enabled by default in the compilation options.

## Notes


# Instructions

## Dependencies

- mipi_cam package: Publishes image messages
- usb_cam package: Publishes image messages
- websocket package: Renders images and AI perception messages

## Parameters

| Parameter Name      | Explanation                            | Mandatory            | Type | Default Value       |                                                                  |
| ------------------- | -------------------------------------- | -------------------- | ------------------- |------------------------------------ |----------------------------------- |
| feed_type           | Image source, 0: local; 1: subscribe   | No                   | int |0                   |
| is_sync_mode   | 0: Synchronous Inference, 1: Asynchronous Inference | No  | int |0                   |
| model_file_name               | model file name                       | No        | string           | config/faceid.hbm     |
| is_shared_mem_sub   | Subscribe to images using shared memory communication method | No  | int |0                   |
| dump_render_img     | Whether to render, 0: no; 1: yes       | No                   | int |0                   |
| ai_msg_sub_topic_name | Topic name for subscribing ai msg to change detect box | No | string | /hobot_mono2d_body_detection |
| ai_msg_pub_topic_name | Topic name for publishing intelligent results for web display | No | string | /perception/detection/faceid |
| ros_img_sub_topic_name | Topic name for subscribing image msg | No | string | /image_raw |

## Running

## Running on X5 Ubuntu System

Running method 1, use the executable file to start:
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
# The config includes models used by the example and local images for filling
# Copy based on the actual installation path (the installation path in the docker is install/lib/faceid/config/, the copy command is cp -r install/lib/faceid/config/ .).
cp -r install/lib/faceid/config/ .

# Run mode 1:Use local JPG format images for backflow prediction:

ros2 run faceid faceid --ros-args -p feed_type:=0 -p dump_render_img:=1

# Run mode 2: Shared memory communication method (topic name: /hbmem_img), set the controlled topic name (topic name: /hobot_mono2d_body_detection) to and set the log level to warn. At the same time, send a ai topic (topic name: /hobot_mono2d_body_detection) in another window get the person body box:

ros2 run faceid faceid --ros-args -p feed_type:=1 --ros-args --log-level warn -p ai_msg_sub_topic_name:="/hobot_mono2d_body_detection"

ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
```

Running method 2, use a launch file:

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/setup.bash
# Copy the configuration based on the actual installation path
cp -r install/lib/faceid/config/ .

# Configure MIPI camera
export CAM_TYPE=mipi

# Start the launch file, run faceid node only.
ros2 launch faceid faceid.launch.py
```

## Run on X5 Yocto system:

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Copy the configuration used by the example and the local image used for inference
cp -r install/lib/faceid/config/ .

# Run mode 1:Use local JPG format images for backflow prediction:

./install/lib/faceid/faceid --ros-args -p feed_type:=0 -p dump_render_img:=1

# Run mode 3: Shared memory communication method (topic name: /hbmem_img), set the controlled topic name (topic name: /hobot_mono2d_body_detection) to and set the log level to warn. At the same time, send a ai topic (topic name: /hobot_mono2d_body_detection) get the person body ai msg:

./install/lib/faceid/faceid --ros-args -p feed_type:=1 --ros-args --log-level warn -p ai_msg_sub_topic_name:="/hobot_mono2d_body_detection"
```

# Results Analysis

## X5 Results Display

log:

Command executed: `ros2 run faceid faceid --ros-args -p feed_type:=0 -p dump_render_img:=1`

```shell
[WARN] [1765890688.911255695] [faceid_node]: Parameter:
 feed_type(0:local, 1:sub): 0
 db_file: faceid.db
 model_file_name: config/faceID.hbm
 dump_render_img: 1
 is_sync_mode: 0
 is_shared_mem_sub: 1
 threshold: 0.7
 ai_msg_pub_topic_name: /perception/detection/faceid
 ai_msg_sub_topic_name: /hobot_mono2d_body_detection
 ros_img_topic_name: /image_raw
 sharedmem_img_topic_name: /hbmem_img
[INFO] [1765890688.911947864] [dnn]: Node init.
[INFO] [1765890688.912003531] [faceid_node]: Set node para.
[INFO] [1765890688.912094031] [dnn]: Model init.
[BPU_PLAT]BPU Platform Version(1.3.6)! soc info(x5)
[HBRT] set log level as 0. version = 3.15.55.0
[DNN] Runtime version = 1.24.5_(3.15.55 HBRT)
[INFO] [1765890689.226533305] [dnn]: The model input 0 width is 112 and height is 112
[INFO] [1765890689.226748180] [dnn]:
Model Info:
name: faceID.
[input]
 - (0) Layout: NCHW, Shape: [1, 3, 112, 112], Type: HB_DNN_IMG_TYPE_NV12_SEPARATE.
[output]
 - (0) Layout: NCHW, Shape: [1, 128, 1, 1], Type: HB_DNN_TENSOR_TYPE_S32.

[INFO] [1765890689.226812639] [dnn]: Task init.
[INFO] [1765890689.229001271] [dnn]: Set task_num [4]
[WARN] [1765890689.229056730] [hobot_dosod]: Get model name: faceID from load model.
[INFO] [1765890689.229096146] [faceid_node]: The model input width is 112 and height is 112
[WARN] [1765890689.257784407] [faceid_output]:
[WARN] [1765890689.268785192] [faceid_output]: Query failed, storage: 1.
```