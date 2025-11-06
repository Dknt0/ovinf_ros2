#include <ament_index_cpp/get_package_share_directory.hpp>

#include "user_func.hpp"

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);

  std::string config_path =
      ament_index_cpp::get_package_share_directory("ovinf_ros2");
  MakeBitbotEverywhere everyone(config_path + "/config/fc2/bhr8_cifx.xml",
                                config_path + "/config/fc2/bhr8_deploy.yaml");
  everyone.WillMake();
  everyone.BeMaking();
  everyone.HaveMade();

  rclcpp::shutdown();
  return 0;
}
