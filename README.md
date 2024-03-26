English| [简体中文](./README_cn.md)

# hobot_image_publisher Topic Publishing Node

# Function Introduction
hobot_image_publisher achieves the publishing of image data and video data through configuring parameters.

1. Supports publishing images in local image formats jpeg/jpg/nv12/png. When publishing images, the message type is nv12.
2. Supports publishing videos in local video formats mp4/h264/h265. When publishing videos in h264/h265 format, the message type is the corresponding video format h264/h265.
   When publishing videos in mp4 format, hobot_image_publisher will extract the h264 video stream from the mp4 file before publishing, and the message type of the publication is h264.

# Compilation

## Dependencies

ROS packages:

- hbm_img_msgs
- img_msgs

hbm_img_msgs is a custom message format used to publish shared memory type image data, defined in hobot_msgs.
img_msgs is a custom message format used to publish ROS type video stream data, defined in hobot_msgs.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Compilation

### Compilation of X3 Version on X3 Ubuntu System

1. Compilation Environment Verification
   - X3 Ubuntu system has been installed on the board.
   - The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
   - ROS 2 compilation tool colcon is installed. If the installed ROS does not include the compilation tool colcon, it needs to be manually installed. The installation command for colcon: `pip install -U colcon-common-extensions`

2. Compilation
   - Compilation command: `colcon build --packages-select hobot_image_publisher`

### Docker Cross Compilation of X3 Version

1. Compilation Environment Verification

- Compiled in the docker and TogetherROS has been installed in the docker. For detailed instructions on docker installation, cross compilation, TogetherROS compilation, and deployment, refer to the README.md in the robot development platform robot_dev_config repo.
- hbm_img_msgs package has been compiled
- img_msgs package has been compiled

2. Compilation- Compilation command:

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

  colcon build --packages-select hobot_image_publisher \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  ```

### Compilation of X86 Version on X86 Ubuntu System

1. Confirmation of Compilation Environment

- X86 Ubuntu Version: Ubuntu 20.04
- Opencv: 4.2.0

2. Compilation

- Compilation command:

  ```
  colcon build --packages-select hobot_image_publisher \
     --merge-install \
     --cmake-args \
     -DPLATFORM_X86=ON \ 
     -DTHIRD_PARTY=`pwd`/../sysroot_docker \
  ```

# User Guide

## Parameter Description
| Parameter Name      | Description                            | Type        | Supported Configuration                                    | Required | Default Value                |
| ------------------ | -------------------------------------- | ----------- | ------------------------------------------------------------| -------- | ---------------------------- |
| image_source       | Source of Images/Videos (Folder/Path/List File) | std::string | Configure according to actual file path                       | No       | config/image/test1.jpg        |
| image_format       | Image/Video Format                     | std::string | jpeg/jpg/png/nv12/h264/h265/mp4 (Format must be consistent with the file extension in image_source) | Yes      | None                          |
| msg_pub_topic_name | Topic Name for Publishing               | std::string | Set according to the desired topic to publish                 | No       | If using share_mem: "/hbmem_img"; If not using share_mem: "/image_raw" |
| source_image_w     | Width of Source Image                   | int         | Configure according to the original image size                 | No (Required if image format is nv12) | Original image size |
| source_image_h     | Height of Source Image                  | int         | Configure according to the original image size                  | No (Required if image format is nv12) | Original image size |
| output_image_w     | Width of Output Image                   | int         | Set the resolution of the image to be published as needed        | No       | 0 |
| output_image_h     | Height of Output Image                  | int         | Set the resolution of the image to be published as needed        | No       | 0 |
| fps                | Publishing Frame Rate                   | int         | [1, 30], no frame rate control outside this range               | No       | 10 |
| is_loop            | Whether to Publish in a Loop            | bool        | True/False                                                     | No       | True |
| is_shared_mem      | Whether to Use Share Memory for Communication | bool | True/False                                        | No       | True |
| is_compressed_img_pub | Whether to Directly Publish Compressed jpeg/jpg/png Images | bool | True: Publish compressed images directly; False: Decode images to NV12 format before publishing | No       | False |## Notice
- If you need to specify image or video files using a list, please write img.list or video.list under config folder. Pay attention to the format of the list file: one file path per line.
- When using a list file, the file format in the list needs to be consistent with the parameter image_format. All images/videos in the list should have the same resolution, otherwise decoding will fail.
- When publishing nv12 images using a list file, the resolution of images in the list needs to be consistent with the input resolution.
- Can read specific format images/videos under a folder.
- When the file format is nv12, please input the original image resolution, or an error will occur.
- Currently supports a maximum frame rate of 15, exceeding this rate is not supported.
- When changing the image/video path, ensure that the parameter image_format matches the image/video format.
- When output_image_w and output_image_h parameters are set to 0 or not set, the image resolution will not be changed.
- When publishing videos, hobot_image_publisher will automatically obtain the resolution of the video, and does not support resolution changes, making the resolution configuration ineffective.
- When publishing images, the is_compressed_img_pub parameter is only effective when the image_format is jpeg/jpg/png.

## Running
- Execute with ros2 run (please replace image_source with your own file path)
  ```
  export COLCON_CURRENT_PREFIX=./install
  source ./install/local_setup.bash
  # config contains example image and video files, copy based on the actual installation path
  # For onboard compilation (without the --merge-install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/, where PKG_NAME is the specific package name
  cp -r install/lib/hobot_image_publisher/config/ .
  ```
  Read all images in the folder with the jpg format and publish nv12 image data at a frequency of 5 frames per second
  ```
  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config -p fps:=5 -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544
  ```
  Read the ./config/test1.jpg image and publish jpg format image data
  ```
  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p is_compressed_img_pub:=True
  ```
  Read the img.list file, get all jpg images paths listed in img.list, publish nv12 image data, frame rate at 5
  ```
  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/img.list -p fps:=5 -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544
  ```
  Read the ./config/test1.jpg image and publish, with a frame rate of 5
  ```
  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p fps:=5 -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544
  ```
  Read the video list file video.list, get all mp4 video file paths listed in video.list, and publish video topic with topic type as h264
  ```
  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/video.list -p fps:=30 -p image_format:=mp4
  ```
  Read the video file test1.h264, video format is h264, and publish h264 video stream topic
  ```
  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.h264 -p fps:=30 -p image_format:=h264
  ```

- Execute with ros2 launch
```
# The images and videos files used in config are provided as examples and should be copied according to the actual installation path
# If compiling for a board end (without the --merge-install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/., where PKG_NAME is the specific package name
cp -r install/lib/hobot_image_publisher/config/.

```

1. Image publishing demonstration, which will start hobot_codec and websocket. The images will be displayed in the browser. Please input "IP address:8000" in the browser to view the effects. Refer to the following demonstration for the effect (for specific usage of websocket, please refer to hobot_websocket)
```
ros2 launch hobot_image_publisher hobot_image_publisher_demo.launch.py
```

2. Using the hobot_image_publisher node independently, this example reads nv12 format images and publishes the topic as /test_msg. Parameter settings can refer to this launch file
```
ros2 launch hobot_image_publisher hobot_image_publisher.launch.py
```

3. Video publishing demonstration, which will start hobot_codec and websocket. The videos will be displayed in the browser. Please input "IP address:8000" in the browser to view the effects (since the current X86 version of hobot_codec does not support decoding of h264/h265, this command only supports X3 batch version). Refer to the following demonstration for the effect (for specific usage of websocket, please refer to hobot_websocket). This example reads the video.list file and cyclically publishes videos with the topic as /hbmem_img. Parameter settings can refer to this launch file.
```
ros2 launch hobot_image_publisher hobot_image_publisher_videolist_demo.launch.py
```
4. Using the hobot_image_publisher node independently, this example reads h264 format images and publishes the topic as /test_h264. Parameter settings can refer to this launch file
```
ros2 launch hobot_image_publisher hobot_image_publisher_video_demo.launch.py
```

## Demonstration
test1.jpg
![image](config/show.png)

Screenshot of the published video
![image](config/mp4show.jpg)
