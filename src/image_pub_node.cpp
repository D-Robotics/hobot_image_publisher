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

#include "include/image_pub_node.h"

#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#define MAX_SIZE 1920*1080*3

int32_t BGRToNv12(const cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;
  if (height % 2 || width % 2) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
          "Image height and width must aligned by 2\n"
          "height: %d \nwidth: %d", height, width);
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
          "yuv_mat.data is null pointer");
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
          "yuv_mat.data is null pointer");
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}

int32_t checkPathType(const std::string &source) {
  struct stat image_source_stat;
  int32_t ret = 0;
  if (stat(source.c_str(), &image_source_stat) == 0) {
    if (image_source_stat.st_mode & S_IFDIR) {
      RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
                            "Your path is a folder");
    } else if (image_source_stat.st_mode & S_IFREG) {
      RCLCPP_INFO(rclcpp::get_logger("image_pub_node"),
                            "Your path is a file");
      ret = 1;
    }
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("image_pub_node"),
              "Source: %s not exist!", source.c_str());
      rclcpp::shutdown();
      return -1;
  }
  return ret;
}

void resizeImage(cv::Mat &in_mat, cv::Mat &out_mat,
                  const int32_t &out_height, const int32_t &out_width,
                  const int32_t &in_width, const int32_t &in_height) {
  float ratio_w = 0.f;
  float ratio_h = 0.f;
  float dst_ratio = 0.f;
  uint32_t resized_width = 0;
  uint32_t resized_height = 0;
  if (in_width > out_width || in_height > out_height) {
    ratio_w =  static_cast<float>(in_width) / static_cast<float>(out_width);
    ratio_h =  static_cast<float>(in_height) / static_cast<float>(out_height);
    dst_ratio = std::max(ratio_w, ratio_h);
    resized_width = static_cast<float>(in_width) / dst_ratio;
    resized_height = static_cast<float>(in_height) / dst_ratio;
    cv::resize(in_mat, in_mat, cv::Size(resized_width, resized_height));
  } else {
    ratio_w =  static_cast<float>(out_width) / static_cast<float>(in_width);
    ratio_h =  static_cast<float>(out_height) / static_cast<float>(in_height);
    dst_ratio = std::min(ratio_w, ratio_h);
    resized_width = static_cast<float>(in_width) * dst_ratio;
    resized_height = static_cast<float>(in_height) * dst_ratio;
    cv::resize(in_mat, in_mat, cv::Size(resized_width, resized_height));
  }
  in_mat.copyTo(out_mat(cv::Rect((out_width - in_mat.cols) / 2,
              (out_height - in_mat.rows) / 2, in_mat.cols, in_mat.rows)));
}

void processImage(ImageCache &image_cache, const std::string &image_source,
                  const int32_t &output_image_w, const int32_t &output_image_h,
                  const int32_t &source_image_w, const int32_t &source_image_h,
                  const std::string &image_format) {
  image_cache.image_ = image_source;
  if (access(image_cache.image_.c_str(), R_OK) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
          "Image: %s not exist!", image_source.c_str());
    rclcpp::shutdown();
    return;
  }
  cv::Mat bgr_mat;
  cv::Mat nv12_tmp;

  // 获取图片
  if (image_format == "nv12") {
    std::ifstream ifs(image_cache.image_, std::ios::in | std::ios::binary);
    if (!ifs) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
              "Image: %s not exist!", image_source.c_str());
      rclcpp::shutdown();
      return;
    }
    ifs.seekg(0, std::ios::end);
    int32_t len = ifs.tellg();
    if(len != source_image_h * source_image_w * 3 / 2) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
              "Parameters: source_image_w and source_image_h are set incorrectly!\n"
              "The length of the nv12 file should be equal to source_image_h * source_image_w * 3 / 2 \n"
              "Length of %s: %d \nsource_image_h: %d \nsource_image_w: %d",
              image_source.c_str(),len,source_image_h,source_image_w);
      rclcpp::shutdown();
      return;
    }
    ifs.seekg(0, std::ios::beg);
    nv12_tmp = cv::Mat(source_image_h * 3 / 2, source_image_w, CV_8UC1);
    auto *nv12_data_ptr = reinterpret_cast<char *>(nv12_tmp.ptr<uint8_t>());
    ifs.read(nv12_data_ptr, len);
    ifs.close();
  } else {
    bgr_mat = cv::imread(image_cache.image_, cv::IMREAD_COLOR);
  }

  // 根据配置参数获取分辨率
  int32_t ori_width = (image_format == "nv12") ? source_image_w : bgr_mat.cols;
  int32_t ori_height = (image_format == "nv12") ? source_image_h : bgr_mat.rows;
  int32_t pad_width = (output_image_w == std::numeric_limits<int>::max()) ? ori_width : output_image_w;
  int32_t pad_height = (output_image_h == std::numeric_limits<int>::max()) ? ori_height : output_image_h;
  if ((pad_width <= 0) || (pad_height <= 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
    "Parameters: output_image_w and output_image_h are set incorrectly! "
    "output_image_w and output_image_h should be greater than 0!\n"
    "output_image_w :%d \noutput_image_h :%d", output_image_w ,output_image_h);
    rclcpp::shutdown();
    return;
  }
  // 根据配置参数改变图片分辨率以及格式转换
  cv::Mat pad_frame(pad_height, pad_width, CV_8UC3, cv::Scalar::all(0));
  cv::Mat& nv12_mat = image_cache.nv12_mat;

  if (ori_width != pad_width || ori_height != pad_height) {
    if (image_format == "nv12") {
      cv::cvtColor(nv12_tmp, bgr_mat, CV_YUV2BGR_NV12);
    }
    resizeImage(bgr_mat, pad_frame, pad_height,
                pad_width, ori_width, ori_height);
  } else {
    pad_frame = bgr_mat;
  }
  if (image_format == "nv12" &&
      (ori_width == pad_width && ori_height == pad_height)) {
    nv12_mat = nv12_tmp;
  } else {
    auto ret = BGRToNv12(pad_frame, nv12_mat);
    if (ret) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
          "Image: %s get nv12 image failed", image_source.c_str());
      rclcpp::shutdown();
      return;
    }
  }
  image_cache.img_data = nv12_mat.data;
  image_cache.data_len = pad_width * pad_height * 3 / 2;
  image_cache.width = pad_width;
  image_cache.height = pad_height;
}


PubNode::PubNode(const std::string &node_name,
                               const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  this->declare_parameter<int32_t>("source_image_w", source_image_w_);
  this->declare_parameter<int32_t>("source_image_h", source_image_h_);
  this->declare_parameter<int32_t>("output_image_w", output_image_w_);
  this->declare_parameter<int32_t>("output_image_h", output_image_h_);
  this->declare_parameter<int32_t>("fps", fps_);
  this->declare_parameter<bool>("is_shared_mem", is_shared_mem_);
  this->declare_parameter<bool>("is_loop", is_loop_);
  this->declare_parameter<std::string>("image_source", image_source_);
  this->declare_parameter<std::string>("image_format", image_format_);
  this->declare_parameter<std::string>("msg_pub_topic_name",
                                       msg_pub_topic_name_);

  this->get_parameter<std::string>("image_source", image_source_);
  this->get_parameter<int32_t>("source_image_w", source_image_w_);
  this->get_parameter<int32_t>("source_image_h", source_image_h_);
  this->get_parameter<int32_t>("output_image_w", output_image_w_);
  this->get_parameter<int32_t>("output_image_h", output_image_h_);
  this->get_parameter<int32_t>("fps", fps_);
  this->get_parameter<bool>("is_shared_mem", is_shared_mem_);
  this->get_parameter<bool>("is_loop", is_loop_);
  this->get_parameter<std::string>("image_format", image_format_);
  this->get_parameter<std::string>("msg_pub_topic_name",
                                       msg_pub_topic_name_);

  if (image_format_.size() == 0){
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
          "Please add parameter: image_format to your command!\n"
          "Example: -p image_format:=jpeg/jpg/png/nv12");
    rclcpp::shutdown();
    return;
  } else {
    if ((source_image_w_ == 0 || source_image_h_ == 0) &&
            image_format_ == "nv12") {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
            "Your image format is nv12. Please add parameters:source_image_w and source_image_h to your command!\n"
            "Example: -p source_image_w:=<source_image_w>  -p source_image_h:=<source_image_h>");
      rclcpp::shutdown();
      return;
    }
  }
  if (fps_ <= 0 || fps_>30) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
    "Parameter: fps setting error! fps should be greater than 0 and less than 30!\n"
    "Fps: %d\n",fps_);
    rclcpp::shutdown();
    return;
  }

  int32_t type = checkPathType(image_source_);
  if (type == 1) {
    std::string file_extension = image_source_.substr(image_source_.find_last_of('.') + 1);
    if (image_format_ == file_extension) {
      // 路径为图片文件
      processImage(image_cache_, image_source_, output_image_w_,
      output_image_h_, source_image_w_, source_image_h_, image_format_);
    } else if (file_extension == "list") {
      // 路径为list文件
      std::string list_tmp;
      std::fstream list_ifs(image_source_);
      assert(list_ifs.is_open());
      while (getline(list_ifs, list_tmp)) {
        std::string list_file_extension = list_tmp.substr(list_tmp.find_last_of('.') + 1);
        if (list_file_extension == image_format_) {
          data_list_.push_back(list_tmp);
        }
      }
      list_ifs.close();
      if (data_list_.size() == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
                "The image with format %s cannot be found in %s",
                image_format_.c_str(),image_source_.c_str());
      rclcpp::shutdown();
      return;
      }
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("image_pub_node"),
                "There is no matching file in your path\n"
                "image_source: %s\n"
                "image_format: %s\n"
                "You can try changing the image_format to: %s",
                image_source_.c_str(),image_format_.c_str(),file_extension.c_str()
                );
      rclcpp::shutdown();
      return;
    }
  } else if (type == 0) {
    // 路径为文件夹
    DIR *pDir;
    struct dirent* ptr;
    if (!(pDir = opendir(image_source_.c_str()))) {
      RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
              "Can not open the dir: %s", image_source_.c_str());
      rclcpp::shutdown();
      return;
    }
    while ((ptr = readdir(pDir)) != 0) {
      std::string file_name(ptr->d_name);
      std::string file_extension = file_name.substr(file_name.find_last_of('.') + 1);
      if ((strcmp(ptr->d_name, ".") != 0) && (strcmp(ptr->d_name, "..") != 0)
            && (image_format_ == file_extension)) {
        data_list_.push_back(image_source_ + "/" + ptr->d_name);
      }
    }
    closedir(pDir);
    if (data_list_.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("image_pub_node"),
              "The image with format %s cannot be found in %s",
              image_format_.c_str(),image_source_.c_str());
    rclcpp::shutdown();
    return;
    }
  }


  hbmem_pub_topic_ = ((is_shared_mem_ == true) &&
    (msg_pub_topic_name_.size() != 0)) ? msg_pub_topic_name_ : hbmem_pub_topic_;
  ros_pub_topic_ = ((is_shared_mem_ == false) &&
    (msg_pub_topic_name_.size() != 0)) ? msg_pub_topic_name_ : ros_pub_topic_;

  if (is_shared_mem_ == true) {
    publisher_hbmem_ = this->create_publisher_hbmem
        <hbm_img_msgs::msg::HbmMsg1080P>(hbmem_pub_topic_, 10);
  } else {
    ros_publisher_ = this->create_publisher
        <sensor_msgs::msg::Image>(ros_pub_topic_, 10);
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/fps_),
                                    std::bind(&PubNode::timer_callback, this));
}

PubNode::~PubNode() {
  RCLCPP_WARN(rclcpp::get_logger("image_pub_node"), "destructor");
}

void PubNode::timer_callback() {
  static uint64_t pub_index = 0;
  if (data_list_.size() != 0) {
    processImage(image_cache_, data_list_[pub_index], output_image_w_,
      output_image_h_, source_image_w_, source_image_h_, image_format_);
  }
  if (is_shared_mem_) {
    auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      msg.height = image_cache_.height;
      msg.width = image_cache_.width;
      memcpy(msg.encoding.data(),
              "nv12",strlen("nv12"));
      memcpy(&msg.data[0], image_cache_.img_data, image_cache_.data_len);
      struct timespec time_start = {0, 0};
      clock_gettime(CLOCK_REALTIME, &time_start);
      msg.time_stamp.sec = time_start.tv_sec;
      msg.time_stamp.nanosec = time_start.tv_nsec;
      msg.index = ++image_cache_.count_;
      msg.data_size = image_cache_.data_len;
      RCLCPP_INFO(
        rclcpp::get_logger("sender"),
          "Publish hbm image msg,file: %s, encoding: %s, img h: %d, w: %d, size: %d, topic: %s",
        image_cache_.image_.c_str(),
        msg.encoding.data(),
        msg.height,
        msg.width,
        msg.data_size,
        hbmem_pub_topic_.data());
      publisher_hbmem_->publish(std::move(loanedMsg));
    }
  } else {
    auto msg = sensor_msgs::msg::Image();
    msg.height = image_cache_.height;
    msg.width = image_cache_.width;
    msg.encoding = "nv12";
    msg.data.resize(image_cache_.data_len);
    memcpy(&msg.data[0], image_cache_.img_data, image_cache_.data_len);
    struct timespec time_start = {0, 0};
    clock_gettime(CLOCK_REALTIME, &time_start);
    msg.header.stamp.sec = time_start.tv_sec;
    msg.header.stamp.nanosec = time_start.tv_nsec;
    msg.header.frame_id = std::to_string(++image_cache_.count_);
    RCLCPP_INFO(
      rclcpp::get_logger("sender"),
        "Publish ros image msg, file: %s, encoding: %s, img h: %d, w: %d, topic: %s",
      image_cache_.image_.c_str(),
      msg.encoding.data(),
      msg.height,
      msg.width,
      ros_pub_topic_.data());
    ros_publisher_->publish(msg);
  }
  // 处理循环
  pub_index++;
  if (is_loop_ == true) {
    if (data_list_.size() != 0) {
      pub_index = pub_index % data_list_.size();
    }
  } else {
    if (data_list_.size() != 0) {
      if (pub_index == data_list_.size()) {
        rclcpp::shutdown();
        return;
      }
    } else {
      rclcpp::shutdown();
      return;
    }
  }
}
