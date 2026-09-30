#include "hikrobot_camera/camera_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>

#include <chrono>
#include <cstring>
#include <vector>

using namespace std::chrono_literals;

namespace hikrobot_camera
{

HikrobotCameraNode::HikrobotCameraNode()
: Node("hikrobot_camera"),
  handle_(nullptr),
  running_(false),
  exposure_time_(5000.0),
  gain_(10.0),
  frame_rate_(60.0),
  grab_timeout_(1000),
  fail_count_(0)
{
  declare_parameter("image_topic", "/image_raw");
  declare_parameter("exposure_time", 5000.0);
  declare_parameter("gain", 10.0);
  declare_parameter("frame_rate", 60.0);
  declare_parameter("grab_timeout", 1000);

  get_parameter("image_topic", image_topic_);
  get_parameter("exposure_time", exposure_time_);
  get_parameter("gain", gain_);
  get_parameter("frame_rate", frame_rate_);
  get_parameter("grab_timeout", grab_timeout_);

  image_pub_ =
    create_publisher<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::QoS(1).best_effort());

  parameter_callback_handle_ =
    add_on_set_parameters_callback(
      std::bind(
        &HikrobotCameraNode::parameterCallback,
        this,
        std::placeholders::_1));

  if (initCamera()) {
    running_ = true;

    grab_thread_ =
      std::thread(
        &HikrobotCameraNode::grabLoop,
        this);

    RCLCPP_INFO(
      get_logger(),
      "Hikrobot camera started");
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Camera initialization failed");
  }
}


HikrobotCameraNode::~HikrobotCameraNode()
{
  running_ = false;

  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }

  closeCamera();
}


bool HikrobotCameraNode::initCamera()
{
  const int init_ret = MV_CC_Initialize();

  if (init_ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_Initialize failed: 0x%x",
      init_ret);
    return false;
  }

  std::memset(
    &device_list_,
    0,
    sizeof(device_list_));

  return findCamera() && openCamera();
}


bool HikrobotCameraNode::findCamera()
{
  const int ret =
    MV_CC_EnumDevices(
      MV_USB_DEVICE | MV_GIGE_DEVICE,
      &device_list_);

  if (
    ret != MV_OK ||
    device_list_.nDeviceNum == 0)
  {
    RCLCPP_ERROR(
      get_logger(),
      "No Hikrobot camera found");
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Found %u Hikrobot camera(s)",
    device_list_.nDeviceNum);

  return true;
}


bool HikrobotCameraNode::openCamera()
{
  int ret =
    MV_CC_CreateHandle(
      &handle_,
      device_list_.pDeviceInfo[0]);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_CreateHandle failed: 0x%x",
      ret);
    handle_ = nullptr;
    return false;
  }

  ret = MV_CC_OpenDevice(handle_);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_OpenDevice failed: 0x%x",
      ret);

    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  ret =
    MV_CC_SetEnumValue(
      handle_,
      "TriggerMode",
      0);

  if (ret != MV_OK) {
    RCLCPP_WARN(
      get_logger(),
      "Set TriggerMode failed: 0x%x",
      ret);
  }

  configureCamera();

  ret = MV_CC_StartGrabbing(handle_);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_StartGrabbing failed: 0x%x",
      ret);

    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Camera opened successfully");

  return true;
}


bool HikrobotCameraNode::configureCamera()
{
  if (handle_ == nullptr) {
    return false;
  }

  MV_CC_SetEnumValue(
    handle_,
    "ExposureAuto",
    0);

  MV_CC_SetFloatValue(
    handle_,
    "ExposureTime",
    static_cast<float>(exposure_time_));

  MV_CC_SetEnumValue(
    handle_,
    "GainAuto",
    0);

  MV_CC_SetFloatValue(
    handle_,
    "Gain",
    static_cast<float>(gain_));

  MV_CC_SetBoolValue(
    handle_,
    "AcquisitionFrameRateEnable",
    true);

  MV_CC_SetFloatValue(
    handle_,
    "AcquisitionFrameRate",
    static_cast<float>(frame_rate_));

  return true;
}


void HikrobotCameraNode::grabLoop()
{
  while (running_) {
    if (!grabImage()) {
      std::this_thread::sleep_for(10ms);
    }
  }
}


bool HikrobotCameraNode::grabImage()
{
  if (handle_ == nullptr) {
    return false;
  }

  MV_FRAME_OUT frame{};

  const int ret =
    MV_CC_GetImageBuffer(
      handle_,
      &frame,
      static_cast<unsigned int>(grab_timeout_));

  if (ret != MV_OK) {
    ++fail_count_;

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "GetImageBuffer failed: 0x%x",
      ret);

    if (fail_count_ > 20) {
      reconnect();
      fail_count_ = 0;
    }

    return false;
  }

  fail_count_ = 0;

  const int w =
    static_cast<int>(
      frame.stFrameInfo.nWidth);

  const int h =
    static_cast<int>(
      frame.stFrameInfo.nHeight);

  std::vector<unsigned char> dst(
    static_cast<std::size_t>(w) *
    static_cast<std::size_t>(h) *
    3U);

  MV_CC_PIXEL_CONVERT_PARAM param{};

  param.nWidth =
    frame.stFrameInfo.nWidth;

  param.nHeight =
    frame.stFrameInfo.nHeight;

  param.pSrcData =
    frame.pBufAddr;

  param.nSrcDataLen =
    frame.stFrameInfo.nFrameLen;

  param.enSrcPixelType =
    frame.stFrameInfo.enPixelType;

  param.enDstPixelType =
    PixelType_Gvsp_BGR8_Packed;

  param.pDstBuffer =
    dst.data();

  param.nDstBufferSize =
    static_cast<unsigned int>(
      dst.size());

  if (
    MV_CC_ConvertPixelType(
      handle_,
      &param) == MV_OK)
  {
    cv::Mat img(
      h,
      w,
      CV_8UC3,
      dst.data());

    auto msg =
      cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "bgr8",
        img)
      .toImageMsg();

    msg->header.stamp =
      now();

    msg->header.frame_id =
      "camera";

    image_pub_->publish(
      *msg);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "MV_CC_ConvertPixelType failed");
  }

  MV_CC_FreeImageBuffer(
    handle_,
    &frame);

  return true;
}


bool HikrobotCameraNode::reconnect()
{
  RCLCPP_WARN(
    get_logger(),
    "Reconnecting camera...");

  closeCamera();

  std::this_thread::sleep_for(1s);

  return initCamera();
}


void HikrobotCameraNode::closeCamera()
{
  if (handle_ != nullptr) {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
  }

  MV_CC_Finalize();
}


rcl_interfaces::msg::SetParametersResult
HikrobotCameraNode::parameterCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & p : params) {
    if (p.get_name() == "exposure_time") {
      const double value = p.as_double();

      if (value <= 0.0) {
        result.successful = false;
        result.reason = "exposure_time must be > 0";
        return result;
      }

      if (handle_ != nullptr) {
        const int ret =
          MV_CC_SetFloatValue(
            handle_,
            "ExposureTime",
            static_cast<float>(value));

        if (ret != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set ExposureTime";
          return result;
        }
      }

      exposure_time_ = value;
    }

    if (p.get_name() == "gain") {
      const double value = p.as_double();

      if (value < 0.0) {
        result.successful = false;
        result.reason = "gain must be >= 0";
        return result;
      }

      if (handle_ != nullptr) {
        const int ret =
          MV_CC_SetFloatValue(
            handle_,
            "Gain",
            static_cast<float>(value));

        if (ret != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set Gain";
          return result;
        }
      }

      gain_ = value;
    }

    if (p.get_name() == "frame_rate") {
      const double value = p.as_double();

      if (value <= 0.0) {
        result.successful = false;
        result.reason = "frame_rate must be > 0";
        return result;
      }

      if (handle_ != nullptr) {
        const int ret =
          MV_CC_SetFloatValue(
            handle_,
            "AcquisitionFrameRate",
            static_cast<float>(value));

        if (ret != MV_OK) {
          result.successful = false;
          result.reason = "Failed to set AcquisitionFrameRate";
          return result;
        }
      }

      frame_rate_ = value;
    }
  }

  return result;
}

}  // namespace hikrobot_camera
