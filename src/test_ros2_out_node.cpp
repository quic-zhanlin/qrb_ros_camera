// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear
#include "qrb_ros_camera/test_ros2_out_node.hpp"

#include "rclcpp/subscription_options.hpp"

namespace qrb_ros::camera
{
// Constructor must be with parameter NodeOptions (component 's requirement)
TestOutNode::TestOutNode(const rclcpp::NodeOptions & options)
  : TestOutNode("test_out_node", options)
{
}

TestOutNode::TestOutNode(const std::string & name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(name, options)
{
  m_p_frame_ = std::make_unique<CameraRos2Frame>();
  dump_ = this->declare_parameter("dump", false);
  dump_camera_info_ = this->declare_parameter("dump_camera_info", false);
  rclcpp::SubscriptionOptions sub_options;
  auto qos = rclcpp::QoS(rclcpp::KeepAll(), rmw_qos_profile_default);
  // create image type adaption 's subscriber
  handler_sub_ = this->create_subscription<IMAGE_TOPIC_TYPE>(IMAGE_TOPIC_NAME, qos,
      std::bind(&TestOutNode::handler_callback, this, std::placeholders::_1), sub_options);
  camera_info_sub_ = this->create_subscription<CAMERA_INFO_TOPIC_TYPE>(CAMERA_INFO_TOPIC_NAME, 10,
      std::bind(&TestOutNode::camera_info_callback, this, std::placeholders::_1), sub_options);
}

int64_t TestOutNode::get_cur_time()
{
  int64_t cur_time = (this->get_clock()->now()).nanoseconds();
  return cur_time;
}

int64_t TestOutNode::cal_latency(int64_t sec, int64_t nsec)
{
  int64_t curtime = TestOutNode::get_cur_time();
  int64_t ts = 1000000000 * sec + nsec;
  int64_t diff = curtime - ts;
  return diff;
}

void TestOutNode::show_receive_fps(CameraRos2Frame * frame)
{
  uint32_t receive_pipeline_id = frame->info.pipeline_id;
  struct timespec time;
  time.tv_sec = time.tv_nsec = 0;
  clock_gettime(CLOCK_MONOTONIC, &time);
  int64_t now = S2NS(time.tv_sec) + time.tv_nsec;
  if (frame->info.frame_id == 0 || g_receive_frame_count == 0) {
    /// reset
    g_receive_last_fpstime = now;
    g_receive_total_latency = 0;
    g_receive_frame_count = 0;
  }
  g_receive_frame_count++;
  volatile int64_t receive_diff = now - g_receive_last_fpstime;
  g_receive_total_latency += frame->info.latency;
  if (receive_diff > S2NS(5)) {
    double fps = (((double)(g_receive_frame_count)) * (double)(S2NS(1))) / (double)receive_diff;
    int64_t latency = g_receive_total_latency / g_receive_frame_count / 1000;
    RCLCPP_INFO(this->get_logger(),
        "test Pipeline %u, Camera %d, fmt %s, w x h[%u x %u],  "
        "frame_id %u, frameCnt %u, Fps: %.4lf, ts %lld ns, latency %ld us \n",
        frame->info.pipeline_id, frame->info.camera_id, frame->info.format.c_str(),
        frame->info.width, frame->info.height, frame->info.frame_id, g_receive_frame_count, fps,
        frame->info.timestamp, latency);
    g_receive_last_fpstime = now;
    g_receive_total_latency = 0;
    g_receive_frame_count = 0;
  }
}

void TestOutNode::handler_callback(sensor_msgs::msg::Image::ConstSharedPtr image)
{
  m_p_frame_->info.width = image->width;
  m_p_frame_->info.height = image->height;
  m_p_frame_->info.format = image->encoding;
  int size = m_p_frame_->info.width * m_p_frame_->info.height * 1.5;
  sscanf(image->header.frame_id.c_str(), "cam[%d_%d] id[%d]", &m_p_frame_->info.pipeline_id,
      &m_p_frame_->info.camera_id, &m_p_frame_->info.frame_id);
  int64_t latency = this->cal_latency(image->header.stamp.sec, image->header.stamp.nanosec);
  m_p_frame_->info.latency = latency;
  this->show_receive_fps(m_p_frame_.get());
  RCLCPP_DEBUG(this->get_logger(), "camera receive latency is %ld(us)", (latency / 1000));
  this->get_parameter("dump", dump_);
  RCLCPP_DEBUG(this->get_logger(), "Received image size:%d", size);
  if (dump_) {
    char fname[256];
    const char * name = "test";
    snprintf(fname, sizeof(fname), "%s/%s_wh[%dx%d]_id[%d-%d-%d].%s", CAMERAROS2_WORKDIR, name,
        m_p_frame_->info.width, m_p_frame_->info.height, m_p_frame_->info.pipeline_id,
        m_p_frame_->info.camera_id, m_p_frame_->info.frame_id,
        get_file_type(get_format_type(m_p_frame_->info.format)));

    FILE * fd = fopen(fname, "wb");
    if (fd) {
      fwrite(image->data.data(), size, 1, fd);
      fclose(fd);
      fd = NULL;
      RCLCPP_INFO(this->get_logger(), "Dump successful: %s\n", fname);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s\n", fname);
    }
  }
}

void TestOutNode::camera_info_callback(const sensor_msgs::msg::CameraInfo & camera_info)
{
  RCLCPP_DEBUG(this->get_logger(), "Received CameraInfo:");
  this->get_parameter("dump_camera_info", dump_camera_info_);
  if (dump_camera_info_) {
    RCLCPP_INFO(this->get_logger(), "distortion_coefficients:");
    for (int i = 0; i < camera_info.d.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "%f", camera_info.d[i]);
    }

    RCLCPP_INFO(this->get_logger(), "camera_matrix:");
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        RCLCPP_INFO(this->get_logger(), "%f", camera_info.k[i * 3 + j]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Projection Matrix:");
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        RCLCPP_INFO(this->get_logger(), "%f", camera_info.p[i * 4 + j]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "rectification_matrix:");
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        RCLCPP_INFO(this->get_logger(), "%f", camera_info.r[i * 3 + j]);
      }
    }
  }
}
}  // namespace qrb_ros::camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::camera::TestOutNode)