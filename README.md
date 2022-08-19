# hobot_image_publisher话题发布节点

# 功能介绍
hobot_image_publisher实现了通过参数配置实现不同的消息发布的方式的功能,发布消息格式类型为nv12

# 编译

## 依赖库

ros package：

- hbm_img_msgs

hbm_img_msgs为自定义消息格式，用于发布shared memory类型图像数据，定义在hobot_msgs中。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

### X3 Ubuntu系统上编译

1、编译环境确认
  - 板端已安装X3 Ubuntu系统。
  - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
  - 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`

2. 编译
  - 编译命令：`colcon build --packages-select hobot_image_publisher`

### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译hbm_img_msgs package

2、编译

- 编译命令：

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


# 使用介绍

## 参数说明
| 参数名          | 解释                | 类型        | 支持的配置                                                   | 是否必须 | 默认值                       |
| --------------- | ------------------- | ----------- | ------------------------------------------------------------ | -------- | ---------------------------- |
| image_source      | 图片文件来源（文件夹/图片路径/list文件）      | std::string | 根据实际文件路径配置 | 否 | config/image/test1.jpg        |
| image_format   |        文件格式      | std::string     |   jpeg/nv12                     |           是       | 无   |
| msg_pub_topic_name     | 发布的话题名称 | std::string |      根据需要发布的话题名称设置                          | 否       | share_mem默认为"/hbmem_img";不使用share_mem默认为"/image_raw" |
| source_image_w     | 源图片的宽度   | int| 根据原始图片尺寸配置                                    | 否(若图片格式为nv12则必填)      | 原始图片尺寸 |
| source_image_h     | 源图片的高度   | int | 根据原始图片尺寸配置                                   | 否(若图片格式为nv12则必填)     | 原始图片尺寸 |
| output_image_w     | 输出的图片宽度 | int | 根据需要发布的图片分辨率设置                                   | 否        | 原始图片尺寸 |
| output_image_h     | 输出的图片高度 | int | 根据需要发布的图片分辨率设置                                   | 否        | 原始图片尺寸 |
| fps     | 图片发布帧率 | int | [1, 30]，在此范围外的配置表示不做帧率控制                                     | 否       | 10 |
| is_loop     | 是否进行循环发布 | bool | true/false                                 | 否       | true |
| is_shared_mem     | 是否使用is_share_mem的方式通信 | bool | true/false                                      | 否       | true |




## 运行
- ros2 run运行(请将image_source更换成自己的文件路径)
  ```
  export COLCON_CURRENT_PREFIX=./install
  source ./install/local_setup.sh
  # config中为示例使用的图片文件，根据实际安装路径进行拷贝
  # 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名
  cp -r install/lib/hobot_audio/config/ .
  ```
  依次为读取文件夹/读取list文件/读取图片文件
  ```
  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=config -p fps:=20 -p output_image_w:=960 -p output_image_h:=544 -p is_shared_mem:=true  -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544 -p is_loop:=true

  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=config/img.list -p fps:=20 -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544

  ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=config/config/test1.jpg -p fps:=20 -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544

  ```

- ros2 launch运行
  ```
  # config中为示例使用的图片文件，根据实际安装路径进行拷贝
  # 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名
  cp -r install/lib/hobot_audio/config/ .
  ```

  1.效果展示，会启动hobot_codec以及websocket，显示图片于浏览器，在浏览器输入IP地址查看，效果见下方效果展示(websocket具体用法参考hobot_websocket)
  ```
  ros2 launch hobot_image_publisher hobot_image_publisher_demo.launch.py
  ```

  2.单独使用hobot_image_publisher节点，该示例读取nv12格式图片，路径为相对路径，请按实际路径修改，发布话题为/test_msg，参数设置可参考该launch文件
  ```
  ros2 launch hobot_image_publisher hobot_image_publisher.launch.py
  ```

## 注意事项
- 需使用list指定图片文件，请编写config下的img.list，注意list文件一个图片路径为一行
- 可实现读取文件夹下特定格式的图片
- 文件格式为nv12时，请输入原图片的分辨率，否则会报错
- 目前支持帧率最高为15，超过此帧率无法支持

## 效果展示
test1.jpg
![image](config/show.png)