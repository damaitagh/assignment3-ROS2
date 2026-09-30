#pragma once

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "MvCameraControl.h"

#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace hikrobot_camera
{

class HikrobotCameraNode : public rclcpp::Node
{
public:
  HikrobotCameraNode();
  ~HikrobotCameraNode() override;

private:
  bool initCamera();
  bool findCamera();
  bool openCamera();
  bool configureCamera();
  void closeCamera();

  void grabLoop();
  bool grabImage();

  bool reconnect();

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & params);

private:
  void * handle_;
  MV_CC_DEVICE_INFO_LIST device_list_{};

  std::atomic<bool> running_;
  std::thread grab_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  std::string image_topic_;

  double exposure_time_;
  double gain_;
  double frame_rate_;
  int grab_timeout_;

  int fail_count_;
};

}  // namespace hikrobot_camera
