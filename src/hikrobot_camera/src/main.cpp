#include "hikrobot_camera/camera_node.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<
      hikrobot_camera::HikrobotCameraNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
