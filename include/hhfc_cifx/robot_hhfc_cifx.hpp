#ifndef ROBOT_HHFC_CIFX_HPP
#define ROBOT_HHFC_CIFX_HPP

#include <filesystem>
#include <vector>

#include "filter/filter_factory.hpp"
#include "robot/base/robot_base.hpp"
#include "robot/hhfc_cifx/hhfc_cifx_common.h"
#include "utils/csv_logger.hpp"

namespace ovinf {

class RobotHhfcCifx : public RobotBase<float> {
  using VectorT = Eigen::Matrix<float, Eigen::Dynamic, 1>;

 public:
  using Ptr = std::shared_ptr<RobotHhfcCifx>;

 private:
  class ObserverHhfcCifx : public ObserverBase {
   public:
    ObserverHhfcCifx() = delete;
    ObserverHhfcCifx(RobotBase<float>* robot, const YAML::Node& config)
        : ObserverBase(robot, config) {
      // Create Filter
      motor_pos_filter_ =
          FilterFactory::CreateFilter(config["motor_pos_filter"]);
      motor_vel_filter_ =
          FilterFactory::CreateFilter(config["motor_vel_filter"]);
      ang_vel_filter_ = FilterFactory::CreateFilter(config["ang_vel_filter"]);
      acc_filter_ = FilterFactory::CreateFilter(config["acc_filter"]);
      eluer_rpy_filter_ = FilterFactory::CreateFilter(config["euler_filter"]);

      // Create Logger
      log_flag_ = config["log_data"].as<bool>();
      if (log_flag_) {
        CreateLog(config);
      }
    }

    virtual bool Update() final {
      auto robot_cifx = dynamic_cast<RobotHhfcCifx*>(robot_);

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

    bool log_flag_ = false;
    CsvLogger::Ptr csv_logger_;
  };

  class ExecutorHhfcCifx : public ExecutorBase {
   public:
    ExecutorHhfcCifx() = delete;
    ExecutorHhfcCifx(RobotBase<float>* robot, const YAML::Node& config)
        : ExecutorBase(robot, config) {}

    virtual bool ExecuteJointTorque() final {
      auto robot_cifx = dynamic_cast<RobotHhfcCifx*>(robot_);
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
      auto robot_cifx = dynamic_cast<RobotHhfcCifx*>(robot_);
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
  RobotHhfcCifx() = delete;
  RobotHhfcCifx(const YAML::Node& config) : RobotBase(config) {
    motors_.resize(motor_size_);
    this->observer_ = std::make_shared<ObserverHhfcCifx>(
        (RobotBase<float>*)this, config["observer"]);
    this->executor_ = std::make_shared<ExecutorHhfcCifx>(
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

void RobotHhfcCifx::GetDevice(const KernelBus& bus) {
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

void RobotHhfcCifx::ObserverHhfcCifx::CreateLog(YAML::Node const& config) {
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

void RobotHhfcCifx::ObserverHhfcCifx::WriteLog() {
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

#endif  // !ROBOT_HHFC_CIFX_HPP
