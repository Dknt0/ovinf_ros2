#ifndef ROBOT_HHFC_ROS2_HPP
#define ROBOT_HHFC_ROS2_HPP
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>
#include <vector>

#include "filter/filter_factory.hpp"
#include "hhfc_ros2_common.h"
#include "robot/base/robot_base.hpp"
#include "utils/csv_logger.hpp"

namespace ovinf {

class RobotHhfcRos2 : public RobotBase<float> {
  using VectorT = Eigen::Matrix<float, Eigen::Dynamic, 1>;

 public:
  using Ptr = std::shared_ptr<RobotHhfcRos2>;

  /**
   * @brief Ros interface to get elevation map and publish joint states
   */
  class RosInterface : public rclcpp::Node {
   public:
    using Ptr = std::shared_ptr<RosInterface>;

    /**
     * @brief Constructor
     *
     * @param[in] config Observer config
     */
    RosInterface(YAML::Node const& config)
        : rclcpp::Node("bitbot_scan_interface") {
      scan_x_num_ = config["scan_x_num"].as<size_t>();
      scan_y_num_ = config["scan_y_num"].as<size_t>();
      scan_x_res_ = config["scan_x_res"].as<float>();
      scan_y_res_ = config["scan_y_res"].as<float>();
      scan_x_bias_ = config["scan_x_bias"].as<float>();
      scan_y_bias_ = config["scan_y_bias"].as<float>();
      scan_height_bias_ = config["scan_height_bias"].as<float>();

      base_frame_name_ = config["base_frame_name"].as<std::string>();
      world_frame_name_ = config["world_frame_name"].as<std::string>();

      height_points_ =
          Eigen::MatrixX<Eigen::Vector3f>(scan_x_num_, scan_y_num_);
      for (size_t i = 0; i < scan_x_num_; ++i) {
        for (size_t j = 0; j < scan_y_num_; ++j) {
          height_points_(i, j) =
              Eigen::Vector3f(i * scan_x_res_ + scan_x_bias_,
                              j * scan_y_res_ + scan_y_bias_, 0.0f);
        }
      }
      height_measurement_ =
          Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>(
              height_points_.rows(), height_points_.cols());

      // ROS utilities.
      pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "scan_points", 10);

      joint_state_publish_ =
          this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
                                                               10);

      auto joint_names = config["joint_names"].as<std::vector<std::string>>();
      joint_state_msg_.name.resize(joint_names.size());
      joint_state_msg_.position.resize(joint_names.size());
      joint_state_msg_.velocity.resize(joint_names.size());
      joint_state_msg_.effort.resize(joint_names.size());
      for (size_t i = 0; i < joint_names.size(); ++i) {
        joint_state_msg_.name[i] = joint_names[i];
      }

      grid_map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
          "elevation_map", 10,
          std::bind(&RosInterface::GridMapCallback, this,
                    std::placeholders::_1));

      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&RosInterface::TimerCallback, this));
    }

    ~RosInterface() {
      if (ros_loop_ && ros_loop_->joinable()) {
        ros_loop_->join();
      }
    }

    /**
     * @brief Create a thread to handle ROS message
     *
     * @param[in] ptr Pointer to RosInterface
     */
    static void RunRosSpin(Ptr ptr) {
      RCLCPP_INFO(rclcpp::get_logger("bitbot_ros_interface"),
                  "Starting ROS spin loop...");
      ptr->ros_loop_ =
          std::make_shared<std::thread>([ptr]() { rclcpp::spin(ptr); });
    }

    /**
     * @brief Get height scan observation
     *
     */
    auto const& GetHeightMeasurement() {
      std::lock_guard<std::mutex> lock(height_mutex_);
      return height_measurement_;
    }

    sensor_msgs::msg::JointState& JointStateMsg() { return joint_state_msg_; }

    void PubJointState() { joint_state_publish_->publish(joint_state_msg_); }

   private:
    /**
     * @brief Callback function to save grid map and global map position
     *
     * @param[in] msg Message
     */
    void GridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
      grid_map::GridMapRosConverter::fromMessage(*msg, grid_map_);
      Twm_ = Eigen::Isometry3f(
          Eigen::Translation3f(msg->info.pose.position.x,
                               msg->info.pose.position.y,
                               msg->info.pose.position.z) *
          Eigen::Quaternionf(
              msg->info.pose.orientation.w, msg->info.pose.orientation.x,
              msg->info.pose.orientation.y, msg->info.pose.orientation.z));
    }

    /**
     * @brief Periodically retrieve robot pose from tf and update scan
     * observation
     *
     */
    void TimerCallback() {
      if (grid_map_.getLayers().empty()) {
        return;
      }

      // Get robot body pose
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_->lookupTransform(
            world_frame_name_, base_frame_name_, tf2::TimePointZero);
      } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s",
                    ex.what());
        return;
      }

      Eigen::Isometry3f Twb = Eigen::Isometry3f(
          Eigen::Translation3f(transform.transform.translation.x,
                               transform.transform.translation.y,
                               transform.transform.translation.z) *
          Eigen::Quaternionf(
              transform.transform.rotation.w, transform.transform.rotation.x,
              transform.transform.rotation.y, transform.transform.rotation.z));
      Eigen::Isometry3f Twb_yaw(
          Eigen::Translation3f(transform.transform.translation.x,
                               transform.transform.translation.y,
                               transform.transform.translation.z) *
          Eigen::Quaternionf(transform.transform.rotation.w, 0.0f, 0.0f,
                             transform.transform.rotation.z));
      Eigen::Isometry3f Tmb_yaw = Twm_.inverse() * Twb_yaw;

      float resolution = grid_map_.getResolution();
      float length_x = grid_map_.getLength().x();
      float length_y = grid_map_.getLength().y();

      // Create an evevation points height matrix
      Eigen::MatrixX<float> elevation_matrix(grid_map_.getSize()(0),
                                             grid_map_.getSize()(1));

      for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        grid_map_.getPosition(*it, position);
        auto index = it.getUnwrappedIndex();
        float elevation = grid_map_.at("elevation", *it);
        if (std::isnan(elevation)) {
          elevation = -1.0;  // Use -1.0 for NaN values
        }
        elevation_matrix(index(0), index(1)) = elevation;
      }

      auto height_measurement_local = height_measurement_;
      {
        std::lock_guard<std::mutex> lock(height_mutex_);
        for (size_t i = 0; i < height_points_.rows(); ++i) {
          for (size_t j = 0; j < height_points_.cols(); ++j) {
            Eigen::Vector3f point_b = height_points_(i, j);
            // Point pos w.r.t. map frame
            auto point_m = Tmb_yaw * point_b;
            int x_index =
                std::round((-point_m.x() + length_x / 2.0f) / resolution);
            int y_index =
                std::round((-point_m.y() + length_y / 2.0f) / resolution);
            x_index = std::max(
                0, std::min(x_index,
                            static_cast<int>(elevation_matrix.rows()) - 2));
            y_index = std::max(
                0, std::min(y_index,
                            static_cast<int>(elevation_matrix.cols()) - 2));

            // Asign lowest value to current height sample position
            float elevation_1 = elevation_matrix(x_index, y_index);
            float elevation_2 = elevation_matrix(x_index + 1, y_index);
            float elevation_3 = elevation_matrix(x_index, y_index + 1);
            float elevation_4 = elevation_matrix(x_index + 1, y_index + 1);
            float elevation = std::min(
                elevation_4,
                std::min(std::min(elevation_1, elevation_2), elevation_3));
            // TODO: Check this transform in your training code!
            height_measurement_(i, j) =
                Twb.translation().z() + scan_height_bias_ - elevation;
          }
        }
        height_measurement_local = height_measurement_;
      }

      // Publish point cloud for checking
      pointcloud_msg_.header.stamp = transform.header.stamp;
      pointcloud_msg_.header.frame_id = world_frame_name_;
      pointcloud_msg_.height = 1;
      pointcloud_msg_.width =
          height_measurement_local.cols() * height_measurement_local.rows();
      pointcloud_msg_.is_dense = false;
      pointcloud_msg_.is_bigendian = false;
      sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg_);
      modifier.setPointCloud2FieldsByString(1, "xyz");

      sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg_, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg_, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg_, "z");

      for (size_t i = 0; i < height_measurement_local.size(); ++i) {
        float x =
            scan_x_res_ * static_cast<int>(i / scan_y_num_) + scan_x_bias_;
        float y = scan_y_res_ * (i % scan_y_num_) + scan_y_bias_;
        float elevation =
            scan_height_bias_ - height_measurement_local.data()[i];
        Eigen::Vector3f point(x, y, elevation);  // Pos in body
        point = Twm_ * Tmb_yaw * point;
        // point = Twm_ * point;

        *iter_x = point.x();
        *iter_y = point.y();
        *iter_z = point.z();

        ++iter_x;
        ++iter_y;
        ++iter_z;
      }

      pointcloud_pub_->publish(pointcloud_msg_);
    }

   private:
    size_t scan_x_num_;
    size_t scan_y_num_;
    float scan_x_res_;
    float scan_y_res_;
    float scan_x_bias_;
    float scan_y_bias_;
    float scan_height_bias_;

    std::string base_frame_name_;
    std::string world_frame_name_;

    std::shared_ptr<std::thread> ros_loop_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_publish_;

    sensor_msgs::msg::JointState joint_state_msg_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::PointCloud2 pointcloud_msg_;

    std::mutex height_mutex_;
    Eigen::Isometry3f Twm_;

    // Height points position w.r.t. robot
    Eigen::MatrixX<Eigen::Vector3f> height_points_;

    // This matrix serves as scan observation input
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        height_measurement_;

    grid_map::GridMap grid_map_;
    // TODO: Add joint state publisher here. Publish robot state in each
    // iteration.
  };

 private:
  class ObserverHhfcRos2 : public ObserverBase {
   public:
    ObserverHhfcRos2() = delete;
    ObserverHhfcRos2(RobotBase<float>* robot, const YAML::Node& config)
        : ObserverBase(robot, config) {
      // Create Filter
      motor_pos_filter_ =
          FilterFactory::CreateFilter(config["motor_pos_filter"]);
      motor_vel_filter_ =
          FilterFactory::CreateFilter(config["motor_vel_filter"]);
      ang_vel_filter_ = FilterFactory::CreateFilter(config["ang_vel_filter"]);
      acc_filter_ = FilterFactory::CreateFilter(config["acc_filter"]);
      eluer_rpy_filter_ = FilterFactory::CreateFilter(config["euler_filter"]);

      scan_size_ = config["scan_size"].as<size_t>();
      scan_.resize(scan_size_);
      ros_interface_ = std::make_shared<RosInterface>(config);
      RosInterface::RunRosSpin(ros_interface_);

      // Create Logger
      log_flag_ = config["log_data"].as<bool>();
      if (log_flag_) {
        CreateLog(config);
      }
    }

    virtual bool Update() final {
      auto robot_cifx = dynamic_cast<RobotHhfcRos2*>(robot_);

      // Get motor posision and velocity
      for (size_t i = 0; i < motor_size_; ++i) {
        motor_actual_position_[i] =
            robot_cifx->motors_[i]->GetActualPosition() *
            robot_->motor_direction_(i, 0);
        motor_actual_velocity_[i] =
            robot_cifx->motors_[i]->GetActualVelocity() *
            robot_->motor_direction_(i, 0);
      }

      // Filter the data
      motor_actual_position_ =
          motor_pos_filter_->Filter(motor_actual_position_);
      motor_actual_velocity_ =
          motor_vel_filter_->Filter(motor_actual_velocity_);

      joint_actual_position_ = motor_actual_position_;
      joint_actual_velocity_ = motor_actual_velocity_;

      // Parallel ankle handle
      if constexpr (true) {
        auto left_joint_pos = robot_cifx->ankles_[LEFT]->ForwardKinematics(
            motor_actual_position_[LAnkleLongMotor],
            motor_actual_position_[LAnkleShortMotor]);
        auto right_joint_pos = robot_cifx->ankles_[RIGHT]->ForwardKinematics(
            motor_actual_position_[RAnkleShortMotor],
            motor_actual_position_[RAnkleLongMotor]);
        auto left_joint_vel = robot_cifx->ankles_[LEFT]->VelocityMapping(
            motor_actual_velocity_[LAnkleLongMotor],
            motor_actual_velocity_[LAnkleShortMotor]);
        auto right_joint_vel = robot_cifx->ankles_[RIGHT]->VelocityMapping(
            motor_actual_velocity_[RAnkleShortMotor],
            motor_actual_velocity_[RAnkleLongMotor]);

        joint_actual_position_[LAnklePitchJoint] = left_joint_pos(0, 0);
        joint_actual_position_[LAnkleRollJoint] = left_joint_pos(1, 0);
        joint_actual_position_[RAnklePitchJoint] = right_joint_pos(0, 0);
        joint_actual_position_[RAnkleRollJoint] = right_joint_pos(1, 0);

        joint_actual_velocity_[LAnklePitchJoint] = left_joint_vel(0, 0);
        joint_actual_velocity_[LAnkleRollJoint] = left_joint_vel(1, 0);
        joint_actual_velocity_[RAnklePitchJoint] = right_joint_vel(0, 0);
        joint_actual_velocity_[RAnkleRollJoint] = right_joint_vel(1, 0);

        // Set frontend extra data
        robot_cifx->extra_data_->Set<"l_p_pos">(
            joint_actual_position_[LAnklePitchJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"l_r_pos">(
            joint_actual_position_[LAnkleRollJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"r_p_pos">(
            joint_actual_position_[RAnklePitchJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"r_r_pos">(
            joint_actual_position_[RAnkleRollJoint] / M_PI * 180.0);

        robot_cifx->extra_data_->Set<"l_p_vel">(
            joint_actual_velocity_[LAnklePitchJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"l_r_vel">(
            joint_actual_velocity_[LAnkleRollJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"r_p_vel">(
            joint_actual_velocity_[RAnklePitchJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"r_r_vel">(
            joint_actual_velocity_[RAnkleRollJoint] / M_PI * 180.0);

        robot_cifx->extra_data_->Set<"l_p_tor">(
            robot_cifx->Executor()->JointTargetTorque()[LAnklePitchJoint]);
        robot_cifx->extra_data_->Set<"l_r_tor">(
            robot_cifx->Executor()->JointTargetTorque()[LAnkleRollJoint]);
        robot_cifx->extra_data_->Set<"r_p_tor">(
            robot_cifx->Executor()->JointTargetTorque()[RAnklePitchJoint]);
        robot_cifx->extra_data_->Set<"r_r_tor">(
            robot_cifx->Executor()->JointTargetTorque()[RAnkleRollJoint]);
      }

      // Anti-parallellogram linkage handle
      if constexpr (true) {
        joint_actual_position_[LKneeJoint] =
            -robot_cifx->ap_linkages_[LEFT]->ForwardKinematics(
                motor_actual_position_[LKneeMotor]);
        joint_actual_position_[RKneeJoint] =
            -robot_cifx->ap_linkages_[RIGHT]->ForwardKinematics(
                motor_actual_position_[RKneeMotor]);

        joint_actual_velocity_[LKneeJoint] =
            robot_cifx->ap_linkages_[LEFT]->VelocityMapping(
                motor_actual_velocity_[LKneeMotor]);
        joint_actual_velocity_[RKneeJoint] =
            robot_cifx->ap_linkages_[RIGHT]->VelocityMapping(
                motor_actual_velocity_[RKneeMotor]);

        robot_cifx->extra_data_->Set<"l_knee_pos">(
            joint_actual_position_[LKneeJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"r_knee_pos">(
            joint_actual_position_[RKneeJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"l_knee_vel">(
            joint_actual_velocity_[LKneeJoint] / M_PI * 180.0);
        robot_cifx->extra_data_->Set<"r_knee_vel">(
            joint_actual_velocity_[RKneeJoint] / M_PI * 180.0);
      }

      // cifx imu returns angles in degree
      euler_rpy_ = eluer_rpy_filter_->Filter(
          (VectorT(3) << robot_cifx->imu_->GetRoll() / 180 * M_PI,
           robot_cifx->imu_->GetPitch() / 180 * M_PI,
           robot_cifx->imu_->GetYaw() / 180 * M_PI)
              .finished());

      acceleration_ = acc_filter_->Filter(
          (VectorT(3) << robot_cifx->imu_->GetAccX(),
           robot_cifx->imu_->GetAccY(), robot_cifx->imu_->GetAccZ())
              .finished());

      angular_velocity_ = ang_vel_filter_->Filter(
          (VectorT(3) << robot_cifx->imu_->GetGyroX(),
           robot_cifx->imu_->GetGyroY(), robot_cifx->imu_->GetGyroZ())
              .finished());

      Eigen::Matrix3f Rwb(
          Eigen::AngleAxisf(euler_rpy_[2], Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(euler_rpy_[1], Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(euler_rpy_[0], Eigen::Vector3f::UnitX()));
      proj_gravity_ =
          VectorT(Rwb.transpose() * Eigen::Vector3f{0.0, 0.0, -1.0});

      auto copied_scan = ros_interface_->GetHeightMeasurement();
      std::copy(copied_scan.data(), copied_scan.data() + scan_size_,
                scan_.data());

      // Publish joint position
      for (size_t i = 0; i < ros_interface_->JointStateMsg().velocity.size();
           i++) {
        ros_interface_->JointStateMsg().position[i] = joint_actual_position_[i];
      }
      ros_interface_->JointStateMsg().header.stamp = ros_interface_->now();
      ros_interface_->PubJointState();

      if (log_flag_) {
        WriteLog();
      }
      return true;
    }

   private:
    inline void CreateLog(YAML::Node const& config);
    inline void WriteLog();

   private:
    FilterBase<VectorT>::Ptr motor_pos_filter_;
    FilterBase<VectorT>::Ptr motor_vel_filter_;
    FilterBase<VectorT>::Ptr ang_vel_filter_;
    FilterBase<VectorT>::Ptr acc_filter_;
    FilterBase<VectorT>::Ptr eluer_rpy_filter_;

    size_t scan_size_ = 0;
    RosInterface::Ptr ros_interface_;

    bool log_flag_ = false;
    CsvLogger::Ptr csv_logger_;
  };

  class ExecutorHhfcRos2 : public ExecutorBase {
   public:
    ExecutorHhfcRos2() = delete;
    ExecutorHhfcRos2(RobotBase<float>* robot, const YAML::Node& config)
        : ExecutorBase(robot, config) {}

    virtual bool ExecuteJointTorque() final {
      auto robot_cifx = dynamic_cast<RobotHhfcRos2*>(robot_);
      motor_target_position_ = joint_target_position_;
      motor_target_torque_ = joint_target_torque_;

      // Parallel ankle handle
      if constexpr (true) {
        auto left_mot_target_pos = robot_cifx->ankles_[LEFT]->InverseKinematics(
            joint_target_position_[LAnklePitchJoint],
            joint_target_position_[LAnkleRollJoint]);
        auto right_mot_target_pos =
            robot_cifx->ankles_[RIGHT]->InverseKinematics(
                joint_target_position_[RAnklePitchJoint],
                joint_target_position_[RAnkleRollJoint]);
        motor_target_position_[LAnkleLongMotor] = left_mot_target_pos[0];
        motor_target_position_[LAnkleShortMotor] = left_mot_target_pos[1];
        motor_target_position_[RAnkleLongMotor] = right_mot_target_pos[1];
        motor_target_position_[RAnkleShortMotor] = right_mot_target_pos[0];

        auto left_mot_target_tor = robot_cifx->ankles_[LEFT]->TorqueRemapping(
            joint_target_torque_[LAnklePitchJoint],
            joint_target_torque_[LAnkleRollJoint]);
        auto right_mot_target_tor = robot_cifx->ankles_[RIGHT]->TorqueRemapping(
            joint_target_torque_[RAnklePitchJoint],
            joint_target_torque_[RAnkleRollJoint]);
        motor_target_torque_[LAnkleLongMotor] = left_mot_target_tor[0];
        motor_target_torque_[LAnkleShortMotor] = left_mot_target_tor[1];
        motor_target_torque_[RAnkleLongMotor] = right_mot_target_tor[1];
        motor_target_torque_[RAnkleShortMotor] = right_mot_target_tor[0];
      }

      // Anti-parallellogram linkage handle
      if constexpr (true) {
        motor_target_position_[LKneeJoint] =
            robot_cifx->ap_linkages_[LEFT]->InverseKinematics(
                -joint_target_position_[LKneeJoint]);
        motor_target_position_[RKneeJoint] =
            robot_cifx->ap_linkages_[RIGHT]->InverseKinematics(
                -joint_target_position_[RKneeJoint]);

        motor_target_torque_[LKneeJoint] =
            robot_cifx->ap_linkages_[LEFT]->TorqueRemapping(
                joint_target_torque_[LKneeJoint]);
        motor_target_torque_[RKneeJoint] =
            robot_cifx->ap_linkages_[RIGHT]->TorqueRemapping(
                joint_target_torque_[RKneeJoint]);
      }

      ExecuteMotorTorque();
      return true;
    }

    virtual bool ExecuteMotorTorque() final {
      auto robot_cifx = dynamic_cast<RobotHhfcRos2*>(robot_);
      for (size_t i = 0; i < motor_size_; ++i) {
        // Torque limit
        if (motor_target_torque_[i] > torque_limit_[i]) {
          motor_target_torque_[i] = torque_limit_[i];
        } else if (motor_target_torque_[i] < -torque_limit_[i]) {
          motor_target_torque_[i] = -torque_limit_[i];
        }

        // Position limit
        if (robot_->Observer()->MotorActualPosition()[i] >
            motor_upper_limit_[i]) {
          motor_target_torque_[i] = 0.0;
        } else if (robot_->Observer()->MotorActualPosition()[i] <
                   motor_lower_limit_[i]) {
          motor_target_torque_[i] = 0.0;
        }
      }

      // Set target
      for (size_t i = 0; i < motor_size_; ++i) {
        robot_cifx->motors_[i]->SetTargetTorque(motor_target_torque_[i] *
                                                robot_->motor_direction_(i, 0));
        robot_cifx->motors_[i]->SetTargetPosition(
            motor_target_position_[i] * robot_->motor_direction_(i, 0));
      }
      return true;
    }

    virtual bool ExecuteMotorCurrent() final {
      throw std::runtime_error(
          "ExecuteMotorCurrent is not supported in mujoco");
      return false;
    }

   private:
  };

 public:
  RobotHhfcRos2() = delete;
  RobotHhfcRos2(const YAML::Node& config) : RobotBase(config) {
    motors_.resize(motor_size_);
    this->observer_ = std::make_shared<ObserverHhfcRos2>(
        (RobotBase<float>*)this, config["observer"]);
    this->executor_ = std::make_shared<ExecutorHhfcRos2>(
        (RobotBase<float>*)this, config["executor"]);

    // Create ankle resolver from yaml config file
    this->ankles_.resize(2);
    this->ankles_[LEFT] = AnkleFromYaml(config["ankle_left"]);
    this->ankles_[RIGHT] = AnkleFromYaml(config["ankle_right"]);

    this->ap_linkages_.resize(2);
    this->ap_linkages_[LEFT] = ApLinkageFromYaml(config["ap_linkage_left"]);
    this->ap_linkages_[RIGHT] = ApLinkageFromYaml(config["ap_linkage_right"]);
  }

  AnklePtr AnkleFromYaml(YAML::Node const& config) {
    return std::make_shared<AnkleT>(AnkleT::AnkleParameters{
        .l_bar1 = config["l_bar1"].as<float>(),
        .l_rod1 = config["l_rod1"].as<float>(),
        .r_a1 = Yaml2Eigen(config["r_a1"]),
        .r_b1_0 = Yaml2Eigen(config["r_b1_0"]),
        .r_c1_0 = Yaml2Eigen(config["r_c1_0"]),
        .l_bar2 = config["l_bar2"].as<float>(),
        .l_rod2 = config["l_rod2"].as<float>(),
        .r_a2 = Yaml2Eigen(config["r_a2"]),
        .r_b2_0 = Yaml2Eigen(config["r_b2_0"]),
        .r_c2_0 = Yaml2Eigen(config["r_c2_0"]),
    });
  }

  APLPtr ApLinkageFromYaml(YAML::Node const& config) {
    return std::make_shared<APLT>(
        APLT::APLParameters{.r = config["r"].as<float>(),
                            .l = config["l"].as<float>(),
                            .theta_bias = config["theta_bias"].as<float>(),
                            .phi_bias = config["phi_bias"].as<float>()});
  }

  inline VectorT Yaml2Eigen(YAML::Node const& config) {
    return Eigen::Map<VectorT>(config.as<std::vector<float>>().data(),
                               config.size());
  }

  inline void GetDevice(const KernelBus& bus);

  void SetExtraData(Kernel::ExtraData& extra_data) {
    extra_data_ = &extra_data;
  }

  virtual void PrintInfo() final {
    for (auto const& pair : motor_names_) {
      std::cout << "Motor id: " << pair.second << ", name: " << pair.first
                << std::endl;
      std::cout << "  - direction: " << motor_direction_(pair.second, 0)
                << std::endl;
      std::cout << "  - upper limit: "
                << Executor()->MotorUpperLimit()(pair.second, 0) << std::endl;
      std::cout << "  - lower limit: "
                << Executor()->MotorLowerLimit()(pair.second, 0) << std::endl;
      std::cout << "  - torque limit: "
                << Executor()->TorqueLimit()(pair.second, 0) << std::endl;
    }
    for (auto const& pair : joint_names_) {
      std::cout << "Joint id: " << pair.second << " name: " << pair.first
                << std::endl;
    }
  }

 private:
  std::vector<MotorPtr> motors_ = {};
  ImuPtr imu_;
  std::vector<AnklePtr> ankles_;
  std::vector<APLPtr> ap_linkages_;
  Kernel::ExtraData* extra_data_;
};

void RobotHhfcRos2::GetDevice(const KernelBus& bus) {
  motors_[LHipPitchMotor] = bus.GetDevice<MotorDevice>(10).value();
  motors_[LHipRollMotor] = bus.GetDevice<MotorDevice>(11).value();
  motors_[LHipYawMotor] = bus.GetDevice<MotorDevice>(12).value();
  motors_[LKneeMotor] = bus.GetDevice<MotorDevice>(13).value();
  motors_[LAnkleLongMotor] = bus.GetDevice<MotorDevice>(14).value();
  motors_[LAnkleShortMotor] = bus.GetDevice<MotorDevice>(15).value();

  motors_[RHipPitchMotor] = bus.GetDevice<MotorDevice>(3).value();
  motors_[RHipRollMotor] = bus.GetDevice<MotorDevice>(4).value();
  motors_[RHipYawMotor] = bus.GetDevice<MotorDevice>(5).value();
  motors_[RKneeMotor] = bus.GetDevice<MotorDevice>(6).value();
  motors_[RAnkleLongMotor] = bus.GetDevice<MotorDevice>(7).value();
  motors_[RAnkleShortMotor] = bus.GetDevice<MotorDevice>(8).value();

  motors_[LShoulderPitchMotor] = bus.GetDevice<MotorDevice>(1).value();
  motors_[RShoulderPitchMotor] = bus.GetDevice<MotorDevice>(0).value();

  imu_ = bus.GetDevice<ImuDevice>(16).value();
}

void RobotHhfcRos2::ObserverHhfcRos2::CreateLog(YAML::Node const& config) {
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm* now_tm = std::localtime(&now_time);
  std::stringstream ss;
  ss << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
  std::string current_time = ss.str();

  std::string log_dir = config["log_dir"].as<std::string>();
  std::filesystem::path config_file_path(log_dir);
  if (config_file_path.is_relative()) {
    config_file_path = canonical(config_file_path);
  }

  std::string logger_file =
      config_file_path.string() + "/" + current_time + "_extra.csv";

  if (!exists(config_file_path)) {
    create_directories(config_file_path);
  }

  // Get headers
  std::vector<std::string> headers;

  // Motor actual pos
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_actual_pos_" + std::to_string(i));
  }

  // Motor actual vel
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_actual_vel_" + std::to_string(i));
  }

  // Joint actual pos
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_actual_pos_" + std::to_string(i));
  }

  // Joint actual vel
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_actual_vel_" + std::to_string(i));
  }

  // Motor target pos
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_target_pos_" + std::to_string(i));
  }

  // Motor target torque
  for (size_t i = 0; i < motor_size_; ++i) {
    headers.push_back("motor_target_torque_" + std::to_string(i));
  }

  // Joint target pos
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_target_pos_" + std::to_string(i));
  }

  // Joint target torque
  for (size_t i = 0; i < joint_size_; ++i) {
    headers.push_back("joint_target_torque_" + std::to_string(i));
  }

  // Acc
  headers.push_back("acc_x");
  headers.push_back("acc_y");
  headers.push_back("acc_z");

  // Ang vel
  headers.push_back("ang_vel_x");
  headers.push_back("ang_vel_y");
  headers.push_back("ang_vel_z");

  // Euler RPY
  headers.push_back("euler_roll");
  headers.push_back("euler_pitch");
  headers.push_back("euler_yaw");

  // Proj gravity
  headers.push_back("proj_gravity_x");
  headers.push_back("proj_gravity_y");
  headers.push_back("proj_gravity_z");

  csv_logger_ = std::make_shared<CsvLogger>(logger_file, headers);
}

void RobotHhfcRos2::ObserverHhfcRos2::WriteLog() {
  std::vector<CsvLogger::Number> datas;

  // Motor actual pos
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(motor_actual_position_[i]);
  }

  // Motor actual vel
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(motor_actual_velocity_[i]);
  }

  // Joint actual pos
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(joint_actual_position_[i]);
  }

  // Joint actual vel
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(joint_actual_velocity_[i]);
  }

  // Motor target pos
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(robot_->Executor()->MotorTargetPosition()[i]);
  }

  // Motor target torque
  for (size_t i = 0; i < motor_size_; ++i) {
    datas.push_back(robot_->Executor()->MotorTargetTorque()[i]);
  }

  // Joint target pos
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(robot_->Executor()->JointTargetPosition()[i]);
  }

  // Joint target torque
  for (size_t i = 0; i < joint_size_; ++i) {
    datas.push_back(robot_->Executor()->JointTargetTorque()[i]);
  }

  // Acc
  datas.push_back(acceleration_[0]);
  datas.push_back(acceleration_[1]);
  datas.push_back(acceleration_[2]);

  // Ang vel
  datas.push_back(angular_velocity_[0]);
  datas.push_back(angular_velocity_[1]);
  datas.push_back(angular_velocity_[2]);

  // Euler RPY
  datas.push_back(euler_rpy_[0]);
  datas.push_back(euler_rpy_[1]);
  datas.push_back(euler_rpy_[2]);

  // Proj gravity
  datas.push_back(proj_gravity_[0]);
  datas.push_back(proj_gravity_[1]);
  datas.push_back(proj_gravity_[2]);

  csv_logger_->Write(datas);
}

}  // namespace ovinf

#endif  // !ROBOT_HHFC_ROS2_HPP
