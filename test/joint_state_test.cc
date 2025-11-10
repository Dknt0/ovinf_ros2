#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

constexpr std::array<std::string_view, 14> joint_name = {
    "l_hip_p",   "l_hip_r",   "l_hip_y",      "l_knee",       "l_ankle_p",
    "l_ankle_r", "r_hip_p",   "r_hip_r",      "r_hip_y",      "r_knee",
    "r_ankle_p", "r_ankle_r", "l_shoulder_p", "r_shoulder_p",
};

class JointStatePublishTest : public rclcpp::Node {
 public:
  JointStatePublishTest() : rclcpp::Node("joint_state_publish_test") {
    joint_state_publish_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JointStatePublishTest::TimerCallback, this));
  }

 private:
  void TimerCallback() {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->now();
    joint_state.name.resize(joint_name.size());
    joint_state.position.resize(joint_name.size());
    joint_state.velocity.resize(joint_name.size());
    joint_state.effort.resize(joint_name.size());
    for (size_t i = 0; i < joint_name.size(); ++i) {
      joint_state.name[i] = joint_name[i];
      joint_state.position[i] = std::sin(2 * M_PI * t);
    }
    joint_state_publish_->publish(joint_state);
    t += 0.01;
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publish_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStatePublishTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
