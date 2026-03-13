English| [简体中文](./README_cn.md)

Getting Started with faceid
=======

# Feature Introduction

The faceid package is a multi-model face recognition solution based on D-Robotics' face detection models. It supports two models:
- **FaceID Model**: Original int32 output, 128-dimensional features
- **InsightFace Model**: Float output, 512-dimensional features

The image data comes from local image feedback and subscribed image messages. The faceid relies on the input of detected face boxes, extracts features, compares with the dataset, and finally gets the face ID.

## Architecture Highlights

- **Polymorphic Model Adapter**: Unified interface supporting different model types (int32, float, and future models)
- **Multi-Feature Matching**: Each ID stores up to 6 features (1 primary + 5 auxiliary) for robust matching
- **Adaptive Threshold**: Automatically adjusts based on database size
- **Gap Verification**: Prevents ambiguous matches with second-best comparison

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

- dnn_node
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
- mono2d_body_detection: Provides face detection boxes

## Parameters

| Parameter Name | Explanation | Mandatory | Type | Default Value |
| ---------------- | -------------------------------------- | -------- | ---------------- |------------------|
| feed_type | Image source, 0: local; 1: subscribe | No | int | 0 |
| is_sync_mode | 0: Synchronous Inference, 1: Asynchronous Inference | No | int | 0 |
| model_type | Model type: "faceid" or "insightface" | No | string | faceid |
| model_file_name | Model file name | No | string | config/faceID.hbm |
| feature_dim | Feature dimension (auto-set by model_type) | No | int | 128 |
| threshold | Similarity threshold for matching | No | float | 0.70 |
| is_shared_mem_sub | Subscribe to images using shared memory | No | int | 0 |
| dump_render_img | Whether to render, 0: no; 1: yes | No | int | 0 |
| ai_msg_sub_topic_name | Topic for subscribing detection boxes | No | string | /hobot_mono2d_body_detection |
| ai_msg_pub_topic_name | Topic for publishing intelligent results | No | string | /perception/detection/faceid |
| ros_img_sub_topic_name | Topic for subscribing image msg | No | string | /image_raw |

### Model Type Details

#### FaceID Model (model_type:=faceid)
- Output: int32
- Feature Dimension: 128
- Default Model: config/faceID.hbm
- Default Threshold: 0.70
- Feature Stride: 4 (special handling)

#### InsightFace Model (model_type:=insightface)
- Output: float
- Feature Dimension: 512
- Default Model: config/insightface.bin
- Default Threshold: 0.93
- Preprocessing: uint8 [0,255], NCHW layout

## Running

### Important: Setup Config Directory

Before running, you **MUST** copy the config directory to your working directory:

```shell
# From your workspace root (e.g., /mnt/wang.liu/tros)
cp -r install/lib/faceid/config/ .
```

The config directory contains:
- Model files (faceID.hbm, insightface.bin)
- Test images (960x544.nv12)
- Other required configuration files

## Running on X5 Ubuntu System

### Using FaceID Model (Default)

**Step 1: Copy config files**
```shell
cd tros  
cp -r install/lib/faceid/config/ .
```

**Step 2: Run the node**
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

# Using launch file
ros2 launch faceid faceid.launch.py
```

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
cp -r install/lib/faceid/config/ .

# Using launch file
ros2 launch faceid faceid.launch.py

# Or run directly
ros2 run faceid faceid --ros-args -p model_type:=faceid -p threshold:=0.70
```

### Using InsightFace Model

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
cp -r install/lib/faceid/config/ .

# Using launch file (recommended)
ros2 launch faceid insightface.launch.py

# Or run directly with custom threshold
ros2 run faceid faceid --ros-args \
  -p model_type:=insightface \
  -p model_file_name:=config/insightface.bin \
  -p feature_dim:=512 \
  -p threshold:=0.93
```

### Using Local Images

```shell
# FaceID model with local image
ros2 run faceid faceid --ros-args -p feed_type:=0 -p model_type:=faceid -p dump_render_img:=1

# InsightFace model with local image
ros2 run faceid faceid --ros-args -p feed_type:=0 -p model_type:=insightface -p dump_render_img:=1
```

### Using Shared Memory with Detection

```shell
# Terminal 1: Start face detection
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py

# Terminal 2: Start faceid with shared memory
ros2 run faceid faceid --ros-args \
  -p feed_type:=1 \
  -p model_type:=insightface \
  -p is_shared_mem_sub:=1 \
  -p ai_msg_sub_topic_name:="/hobot_mono2d_body_detection"
```

## Run on X5 Yocto system:

### FaceID Model
```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/
cp -r install/lib/faceid/config/ .

# Default model
./install/lib/faceid/faceid --ros-args -p feed_type:=0 -p dump_render_img:=1
```

### InsightFace Model
```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/
cp -r install/lib/faceid/config/ .

# InsightFace model
./install/lib/faceid/faceid --ros-args \
  -p model_type:=insightface \
  -p model_file_name:=config/insightface.bin \
  -p feature_dim:=512 \
  -p threshold:=0.93
```

# Face Matching Strategy

## Multi-Feature Fusion

The system uses a three-layer matching strategy:

1. **Full Database Scan**: Compare with all features of each ID
2. **Multi-Feature Comparison**: Each ID can have up to 6 features (1 primary + 5 auxiliary)
3. **Adaptive Threshold**: 
   - Database ≤ 3: threshold = max(configured, 0.94)
   - Database > 3: threshold = configured value

## Gap Verification

To avoid ambiguous matches:
- **Gap ≥ 0.03**: Confirm match if similarity ≥ threshold
- **Gap < 0.03 but similarity ≥ threshold + 0.02**: Still match (same person, different angle)
- **Otherwise**: Create new ID

## Matching Logic

```
if (max_similarity >= threshold) {
    if (gap >= 0.03) {
        → MATCH confirmed
    } else if (max_similarity >= threshold + 0.02) {
        → MATCH (small gap but high confidence)
    } else {
        → NEW ID (ambiguous)
    }
} else {
    → NEW ID
}
```

# Results Analysis

## X5 Results Display

### FaceID Model Results

Command: `ros2 run faceid faceid --ros-args -p feed_type:=0 -p model_type:=faceid -p dump_render_img:=1`

```shell
[WARN] [1765890688.911255695] [faceid_node]: Parameter:
 feed_type(0:local, 1:sub): 0
 db_file: faceid.db
  model_file_name: config/faceID.hbm
 model_type: faceid
 feature_dim: 128
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
...
[INFO] [1765890689.226812639] [faceid_node]: The model input width is 112 and height is 112
[WARN] [1765890689.257784407] [faceid_output]: Database has 1 entries, threshold=0.700
[WARN] [1765890689.268785192] [faceid_output]: Query success, result: 1.
```

### InsightFace Model Results

Command: `ros2 run faceid faceid --ros-args -p model_type:=insightface`

```shell
[INFO] [faceid]: ModelAdapter initialized: type=insightface, feature_dim=512
[INFO] [faceid_output]: === Comparing with ALL database entries ===
[INFO] [faceid_output]:   ID=1: similarity=0.9234 [ABOVE THRESHOLD]
[INFO] [faceid_output]: === Top 5 similarities ===
[INFO] [faceid_output]:   #1: ID=1, similarity=0.9234 <-- BEST
[INFO] [faceid_output]: ✓ MATCHED: roi_idx=0 -> id=1 (sim=0.9234, threshold=0.930, gap=0.045)
```

# Architecture

## Polymorphic Model Adapter

```cpp
class FeatureExtractor {
  virtual float CosineSimilarity(const void* data1, const void* data2) = 0;
  virtual std::vector<float> ExtractFeatures(...) = 0;
};

class FaceIDExtractor : public FeatureExtractor { /* int32, 128-dim */ };
class InsightFaceExtractor : public FeatureExtractor { /* float, 512-dim */ };
```

## Adding New Models

To add a new model type:

1. Create new extractor class:
```cpp
class NewModelExtractor : public FeatureExtractor {
  int GetFeatureDim() const override { return NEW_DIM; }
  float CosineSimilarity(const void* data1, const void* data2) override { ... }
  // ... other methods
};
```

2. Add to ModelAdapter constructor:
```cpp
switch (model_type_) {
  case ModelType::FACEID: extractor_ = std::make_unique<FaceIDExtractor>(); break;
  case ModelType::INSIGHTFACE: extractor_ = std::make_unique<InsightFaceExtractor>(); break;
  case ModelType::NEWMODEL: extractor_ = std::make_unique<NewModelExtractor>(); break;
}
```

3. Update StringToModelType:
```cpp
if (str == "newmodel") return ModelType::NEWMODEL;
```