/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include <numbers>

#include "utils/lifter_kinematics.h"

template <class UnitTypeLhs, class UnitTypeRhs, class UnitTypeTol>
::testing::AssertionResult UnitEqual(UnitTypeLhs actual, UnitTypeRhs expected, UnitTypeTol tolerance) {
  if (units::math::abs(actual - expected) < tolerance) {
    return ::testing::AssertionSuccess();
  }
  return ::testing::AssertionFailure() << fmt::format("{} not within {} of expected {}", actual, tolerance, expected);
}

template <class UnitTypeLhs, class UnitTypeRhs>
::testing::AssertionResult UnitEqual(UnitTypeLhs actual, UnitTypeRhs expected) {
  return UnitEqual(actual, expected, decltype(actual)(0.001));
}

TEST(LifterKinematicsTest, LineArm_IK) {
  {
    LifterKinematics kinematics({0_m, 0_m}, 0_m, {0_m, 0_m});
    frc::Translation2d pose{1_m, 0_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, 0_rad));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m, 0.1_m));
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
    frc::Translation2d pose{-1_m, 0_m};
    auto joints = kinematics.GetJoints(pose, false);
    EXPECT_TRUE(UnitEqual(joints.shoulderAngle, units::radian_t(std::numbers::pi)));
    EXPECT_TRUE(UnitEqual(joints.armLen, 1_m));
  }
}
