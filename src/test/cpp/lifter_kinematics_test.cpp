/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <argos_lib/testing/assertions.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/length.h>

#include <numbers>

#include "utils/lifter_kinematics.h"

using argos_lib::testing::UnitEqual;

TEST(LifterKinematicsTest, LineArm_OriginFulcrum_IK) {
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    frc::Translation2d pose{1_m, 0_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    frc::Translation2d pose{0_m, 1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    frc::Translation2d pose{1_m, 1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2)));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    frc::Translation2d pose{1_m, -1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(-std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2)));
  }
}

TEST(LifterKinematicsTest, OffsetLineArm_OriginFulcrum_IK) {
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    frc::Translation2d pose{1_m, -0.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    frc::Translation2d pose{1_m, 0.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    frc::Translation2d pose{0.1_m, 1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    frc::Translation2d pose{-0.1_m, 1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    frc::Translation2d pose{1_m + (0.1_m / std::numbers::sqrt2), 1_m - (0.1_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2)));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    frc::Translation2d pose{1_m - (0.1_m / std::numbers::sqrt2), 1_m + (0.1_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2)));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    frc::Translation2d pose{1_m - (0.1_m / std::numbers::sqrt2), -1_m - (0.1_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(-std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2)));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    frc::Translation2d pose{1_m + (0.1_m / std::numbers::sqrt2), -1_m + (0.1_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(-std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2)));
  }
}

TEST(LifterKinematicsTest, OffsetLineArm_EffectorOffset_OriginFulcrum_IK) {
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1.1_m, 0.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1.1_m, 0.3_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, -0.2_m});
    frc::Translation2d pose{1.1_m, -0.3_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{-0.1_m, 1.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1.1_m, -0.3_m};
    auto joints = kinematics.GetJoints(pose, true);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1.1_m, -0.1_m};
    auto joints = kinematics.GetJoints(pose, true);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, -0.2_m});
    frc::Translation2d pose{1.1_m, 0.1_m};
    auto joints = kinematics.GetJoints(pose, true);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{0.3_m, 1.1_m};
    auto joints = kinematics.GetJoints(pose, true);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{-0.3_m, 1.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, -0.2_m});
    frc::Translation2d pose{0.3_m, 1.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, -0.2_m});
    frc::Translation2d pose{1_m + (0.4_m / std::numbers::sqrt2), 1_m + (-0.2_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1_m + (-0.2_m / std::numbers::sqrt2), 1_m + (0.4_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1_m + (0.2_m / std::numbers::sqrt2), -1_m + (0.0_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(-std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1_m + (0.4_m / std::numbers::sqrt2), -1_m + (0.2_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(-std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
}

TEST(LifterKinematicsTest, OffsetLineArm_EffectorOffset_TranslatedFulcrum_IK) {
  {
    LifterKinematics kinematics({1_m, 2_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{2.1_m, 2.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({-1_m, 3_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{0.1_m, 3.3_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({-1_m, -1_m}, -0.1_m, {0.1_m, -0.2_m});
    frc::Translation2d pose{0.1_m, -1.3_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({6_m, 4_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{5.9_m, 5.1_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({3.5_m, 1.5_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{3.2_m, 2.6_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 2));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({5_m, 10_m}, -0.1_m, {0.1_m, -0.2_m});
    frc::Translation2d pose{6.1_m, 9.7_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.005_m));
  }
  {
    LifterKinematics kinematics({30_m, -10_m}, -0.1_m, {0.1_m, -0.2_m});
    frc::Translation2d pose{31_m + (0.4_m / std::numbers::sqrt2), -9_m + (-0.2_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
  {
    LifterKinematics kinematics({1_m, 1_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{2_m + (-0.2_m / std::numbers::sqrt2), 2_m + (0.4_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
  {
    LifterKinematics kinematics({3_m, 9_m}, -0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{4_m + (0.2_m / std::numbers::sqrt2), 8_m + (0.0_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(-std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
  {
    LifterKinematics kinematics({0.1_m, 0.2_m}, 0.1_m, {0.1_m, 0.2_m});
    frc::Translation2d pose{1.1_m + (0.4_m / std::numbers::sqrt2), -0.8_m + (0.2_m / std::numbers::sqrt2)};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(-std::numbers::pi) / 4));
    EXPECT_TRUE(UnitEqual(joints.armLen, units::meter_t(std::numbers::sqrt2), 0.005_m));
  }
}

TEST(LifterKinematicsTest, LineArm_OriginFulcrum_FK) {
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 0_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    ArmState state{1_m, 45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    ArmState state{1_m, 90_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 0_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    ArmState state{1_m, -45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), -1_m / std::numbers::sqrt2));
  }
}

TEST(LifterKinematicsTest, OffsetLineArm_OriginFulcrum_FK) {
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 0.1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), -0.1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    ArmState state{1_m, 45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 0.9_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1.1_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    ArmState state{1_m, 45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.1_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 0.9_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    ArmState state{1_m, 90_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), -0.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    ArmState state{1_m, 90_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 0.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0_m, 0_m});
    ArmState state{1_m, -45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.1_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), -0.9_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0_m, 0_m});
    ArmState state{1_m, -45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 0.9_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), -1.1_m / std::numbers::sqrt2));
  }
}

TEST(LifterKinematicsTest, OffsetLineArm_EffectorOffset_OriginFulcrum_FK) {
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 0.3_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 0.1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, -0.2_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), -0.1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 0.8_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1.4_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.0_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1.2_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 90_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), -0.3_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1.1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 90_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), -0.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 1.1_m));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, -45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.4_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), -0.8_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({0_m, 0_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, -45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 1.2_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), -1.0_m / std::numbers::sqrt2));
  }
}

TEST(LifterKinematicsTest, OffsetLineArm_EffectorOffset_TranslatedFulcrum_FK) {
  {
    LifterKinematics kinematics({1_m, 2_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 2.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 2.3_m));
  }
  {
    LifterKinematics kinematics({3_m, 5_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 4.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 5.1_m));
  }
  {
    LifterKinematics kinematics({7_m, 3_m}, 0.1_m, {0.1_m, -0.2_m});
    ArmState state{1_m, 0_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 8.1_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 2.9_m));
  }
  {
    LifterKinematics kinematics({0.5_m, 0.6_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 0.5_m + 0.8_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 0.6_m + 1.4_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({100_m, 200_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 100_m + 1.0_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 200_m + 1.2_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({15_m, 10_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 90_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 14.7_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 11.1_m));
  }
  {
    LifterKinematics kinematics({25_m, 20_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, 90_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 24.9_m));
    EXPECT_TRUE(UnitEqual(pose.Y(), 21.1_m));
  }
  {
    LifterKinematics kinematics({0.1_m, 0.2_m}, 0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, -45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 0.1_m + 1.4_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 0.2_m - 0.8_m / std::numbers::sqrt2));
  }
  {
    LifterKinematics kinematics({9_m, 8_m}, -0.1_m, {0.1_m, 0.2_m});
    ArmState state{1_m, -45_deg};
    auto pose = kinematics.GetPose(state, false);
    EXPECT_TRUE(UnitEqual(pose.X(), 9_m + 1.2_m / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(pose.Y(), 8_m - 1.0_m / std::numbers::sqrt2));
  }
}
