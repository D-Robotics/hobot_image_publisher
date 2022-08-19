// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

#ifndef INCLUDE_IMAGE_PUB_NODE_H_
#define INCLUDE_IMAGE_PUB_NODE_H_

struct ImageCache {
  std::string image_;
  cv::Mat nv12_mat;
  uint8_t* img_data = nullptr;
  int32_t width = 0;
  int32_t height = 0;
  int32_t data_len = 0;
  int32_t count_ = 0;
};


class PubNode : public rclcpp::Node {
 public:
  PubNode(const std::string &node_name,
                 const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~PubNode() override;

 private:
  std::vector<std::string> data_list_;

  std::string image_format_;
  std::string msg_pub_topic_name_;
  std::string hbmem_pub_topic_ = "/hbmem_img";
  std::string ros_pub_topic_ = "/image_raw";
  std::string image_source_ = "./config/test1.jpg";

  int32_t source_image_w_ = 0;
  int32_t source_image_h_ = 0;
  int32_t output_image_w_ = 0;
  int32_t output_image_h_ = 0;
  int32_t fps_ = 10;

  bool is_shared_mem_ = true;
  bool is_loop_ = true;

  ImageCache image_cache_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
                  publisher_hbmem_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
                  ros_publisher_ = nullptr;

  void timer_callback();
};

#endif  // INCLUDE_IMAGE_PUB_NODE_H_
