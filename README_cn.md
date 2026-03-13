[English](./README.md) | 简体中文

# 功能介绍

人脸Faceid跟随算法示例, 是地瓜自研的人脸识别方案, 通过对人脸特征进行提取, 匹配, 完成对人脸示例ID识别。

# 开发环境

- 编程语言: C/C++
- 开发平台: X5
- 系统版本：Ubuntu 22.04
- 编译工具链: Linux GCC 11.4.0

# 编译

- X5版本：支持在 X5 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

同时支持通过编译选项控制编译pkg的依赖和pkg的功能。

## 依赖库

- opencv:3.4.5

ros package：

- dnn node
- cv_bridge
- sensor_msgs
- hbm_img_msgs
- ai_msgs

hbm_img_msgs为自定义的图片消息格式, 用于shared mem场景下的图片传输, hbm_img_msgs pkg定义在hobot_msgs中, 因此如果使用shared mem进行图片传输, 需要依赖此pkg。


## 编译选项

1、SHARED_MEM

- shared mem（共享内存传输）使能开关, 默认打开（ON）, 编译时使用-DSHARED_MEM=OFF命令关闭。
- 如果打开, 编译和运行会依赖hbm_img_msgs pkg, 并且需要使用tros进行编译。
- 如果关闭, 编译和运行不依赖hbm_img_msgs pkg, 支持使用原生ros和tros进行编译。
- 对于shared mem通信方式, 当前只支持订阅nv12格式图片。

## X5 Ubuntu系统上编译

1、编译环境确认

- 板端已安装X5 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon, 需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select faceid`

## docker交叉编译 X5版本

1、编译环境确认

- 在docker中编译, 并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译dnn node package
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令：

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s faceid
  ```

- 编译选项中默认打开了shared mem通信方式。

## 注意事项


# 使用介绍

## 依赖

- mipi_cam package：发布图片msg
- usb_cam package：发布图片msg
- websocket package：渲染图片和ai感知msg

## 参数

| 参数名             | 解释                                  | 是否必须             | 数值类型 | 默认值                 |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type           | int         | 本地/订阅推理模式。0：加载本地图片；1：订阅图片话题                                                                         | 否       | 0/1                  | 0                            |
| is_sync_mode           | int         | 同步/异步推理模式。0：异步模式；1：同步模式                                                                         | 否       | 0/1                  | 0                            |
| model_file_name        | std::string | 推理使用的模型文件                                                                                                  | 否       | 根据实际模型路径配置 | config/faceID.hbm          |
| is_shared_mem_sub      | int         | 是否使用shared mem通信方式订阅图片消息。打开和关闭shared mem通信方式订阅图片的topic名分别为/hbmem_img和/image_raw。 | 0/1      | 0/1                  | 0                            |
| ai_msg_pub_topic_name  | std::string | 发布包含人体跟随ID结果的消息topic名                                                                         | 否       | 根据实际部署环境配置 | /perception/detection/faceid    |
| ai_msg_sub_topic_name | std::string | 订阅包含人体框检测结果的消息topic名                                                                             | 否       | 根据实际部署环境配置 | /hobot_mono2d_body_detection |
| ros_img_topic_name | std::string | 订阅Ros图片话题消息topic名                                                                             | 否       | 根据实际部署环境配置 | /image_raw |

### 模型类型详情

#### FaceID模型 (model_type:=faceid)
- 输出类型：int32
- 特征维度：128
- 默认模型：config/faceID.hbm
- 默认阈值：0.70
- 特征步长：4（特殊处理）

#### InsightFace模型 (model_type:=insightface)
- 输出类型：float
- 特征维度：512
- 默认模型：config/insightface.bin
- 默认阈值：0.93
- 预处理：uint8 [0,255], NCHW布局


## 运行

- faceid 使用到的模型在安装包'config'路径下。

- 编译成功后, 将生成的install路径拷贝到地平线RDK上（如果是在RDK上编译, 忽略拷贝步骤）, 并执行如下命令运行。

## X5 Ubuntu系统上运行

### 准备工作

在运行前，**必须**将配置目录复制到工作目录：

```shell
# 从工作空间根目录（例如 /mnt/wang.liu/tros）
cp -r install/lib/faceid/config/ .
```

配置目录包含：
- 模型文件（faceID.hbm, insightface.bin）
- 测试图片（960x544.nv12）
- 其他必需的配置文件

### 使用FaceID模型（默认）

**步骤1：复制配置文件**
```shell
cd /mnt/wang.liu/tros  # 你的工作空间目录
cp -r install/lib/faceid/config/ .
```

**步骤2：运行节点**
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

# 使用launch文件
ros2 launch faceid faceid.launch.py
```

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
cp -r install/lib/faceid/config/ .

# 使用launch文件
ros2 launch faceid faceid.launch.py

# 或直接运行
ros2 run faceid faceid --ros-args -p model_type:=faceid -p threshold:=0.70
```

### 使用InsightFace模型

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
cp -r install/lib/faceid/config/ .

# 使用launch文件（推荐）
ros2 launch faceid insightface.launch.py

# 或直接运行（自定义阈值）
ros2 run faceid faceid --ros-args \
  -p model_type:=insightface \
  -p model_file_name:=config/insightface.bin \
  -p feature_dim:=512 \
  -p threshold:=0.93
```

### 使用本地图片

```shell
# FaceID模型使用本地图片
ros2 run faceid faceid --ros-args -p feed_type:=0 -p model_type:=faceid -p dump_render_img:=1

# InsightFace模型使用本地图片
ros2 run faceid faceid --ros-args -p feed_type:=0 -p model_type:=insightface -p dump_render_img:=1
```

### 使用共享内存配合检测

```shell
# 终端1：启动人脸检测
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py

# 终端2：启动faceid使用共享内存
ros2 run faceid faceid --ros-args \
  -p feed_type:=1 \
  -p model_type:=insightface \
  -p is_shared_mem_sub:=1 \
  -p ai_msg_sub_topic_name:="/hobot_mono2d_body_detection"
```

## X5 Yocto系统上运行

### FaceID模型
```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/
cp -r install/lib/faceid/config/ .

# 默认模型
./install/lib/faceid/faceid --ros-args -p feed_type:=0 -p dump_render_img:=1
```

### InsightFace模型
```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/
cp -r install/lib/faceid/config/ .

# InsightFace模型
./install/lib/faceid/faceid --ros-args \
  -p model_type:=insightface \
  -p model_file_name:=config/insightface.bin \
  -p feature_dim:=512 \
  -p threshold:=0.93
```

# 人脸匹配策略

## 多特征融合

系统使用三层匹配策略：

1. **全数据库扫描**：与每个ID的所有特征进行比较
2. **多特征比较**：每个ID最多可存储6个特征（1个主特征 + 5个辅助特征）
3. **自适应阈值**：
   - 数据库 ≤ 3：阈值 = max(配置值, 0.94)
   - 数据库 > 3：阈值 = 配置值

## 间隙验证

为避免模糊匹配：
- **间隙 ≥ 0.03**：如果相似度 ≥ 阈值，确认匹配
- **间隙 < 0.03 但相似度 ≥ 阈值 + 0.02**：仍然匹配（同一人，不同角度）
- **否则**：创建新ID

## 匹配逻辑

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

# 结果分析

## X5结果展示

log：

运行命令：`ros2 run faceid faceid --ros-args -p feed_type:=0 -p dump_render_img:=1`

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