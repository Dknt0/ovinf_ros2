#ifndef HHFC_CIFX_COMMON_HPP
#define HHFC_CIFX_COMMON_HPP

#include "bitbot_cifx/device/imu_mti300.h"
#include "bitbot_cifx/device/joint_elmo.h"
#include "bitbot_cifx/kernel/cifx_kernel.hpp"
#include "utils/antiparallelogram.hpp"
#include "utils/parallel_ankle.hpp"

namespace ovinf {

enum LeftRight {
  LEFT = 0,
  RIGHT = 1,
};

enum MotorIdx {
  LHipPitchMotor = 0,
  LHipRollMotor = 1,
  LHipYawMotor = 2,
  LKneeMotor = 3,
  LAnkleLongMotor = 4,
  LAnkleShortMotor = 5,
  // LAnklePitchMotor = 4,
  // LAnkleRollMotor = 5,

  RHipPitchMotor = 6,
  RHipRollMotor = 7,
  RHipYawMotor = 8,
  RKneeMotor = 9,
  RAnkleLongMotor = 10,
  RAnkleShortMotor = 11,
  // RAnklePitchMotor = 10,
  // RAnkleRollMotor = 11,

  LShoulderPitchMotor = 12,
  RShoulderPitchMotor = 13,
};

enum JointIdx {
  LHipPitchJoint = 0,
  LHipRollJoint = 1,
  LHipYawJoint = 2,
  LKneeJoint = 3,
  LAnklePitchJoint = 4,
  LAnkleRollJoint = 5,

  RHipPitchJoint = 6,
  RHipRollJoint = 7,
  RHipYawJoint = 8,
  RKneeJoint = 9,
  RAnklePitchJoint = 10,
  RAnkleRollJoint = 11,

  LShoulderPitchJoint = 12,
  RShoulderPitchJoint = 13,
};

}  // namespace ovinf

using KernelBus = bitbot::CifxBus;
using ImuDevice = bitbot::ImuMti300;
using ImuPtr = ImuDevice*;
using MotorDevice = bitbot::JointElmo;
using MotorPtr = MotorDevice*;
using AnkleT = ovinf::ParallelAnkle<float>;
using AnklePtr = std::shared_ptr<AnkleT>;
using APLT = ovinf::AntiparallelogramLinkage<float>;
using APLPtr = std::shared_ptr<APLT>;

struct UserData {};

using Kernel =
    bitbot::CifxKernel<UserData, "l_p_pos", "l_p_vel", "l_r_pos", "l_r_vel",
                       "r_p_pos", "r_p_vel", "r_r_pos", "r_r_vel", "l_p_tor",
                       "l_r_tor", "r_p_tor", "r_r_tor", "l_knee_pos",
                       "r_knee_pos", "l_knee_vel", "r_knee_vel">;

#endif  // !HHFC_CIFX_COMMON_HPP
