/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <argos_lib/testing/assertions.h>
#include <gtest/gtest.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#include "utils/path_planning/convert_path.h"

using namespace path_planning;
using argos_lib::testing::UnitEqual;

TEST(CalcCuspAngleTest, Invalid) {
  {
    ArmPath path = {};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
    angle = CalculateCuspAngle(path, 100);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
  }
  {
    ArmPath path = {{1_in, 5_in}};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
    angle = CalculateCuspAngle(path, 1);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
  }
  {
    ArmPath path = {{1_in, 5_in}, {1_in, 10_in}};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
    angle = CalculateCuspAngle(path, 1);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
    angle = CalculateCuspAngle(path, 2);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
  }
  {
    ArmPath path = {{1_in, 5_in}, {1_in, 10_in}, {5_in, 10_in}};
    auto angle = CalculateCuspAngle(path, 1);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
    angle = CalculateCuspAngle(path, 2);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
  }
}

TEST(CalcCuspAngleTest, Valid) {
  {
    ArmPath path = {{1_in, 5_in}, {1_in, 10_in}, {5_in, 10_in}};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, 90_deg));
  }
  {
    ArmPath path = {{-5_in, -5_in}, {0_in, 0_in}, {5_in, 5_in}};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, 180_deg));
  }
  {
    ArmPath path = {{-5_in, -5_in}, {0_in, 0_in}, {-5_in, -5_in}};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, 0_deg));
  }
  {
    ArmPath path = {{5_in, 5_in}, {0_in, 0_in}, {0_in, -5_in}};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, -135_deg));
  }
  {
    ArmPath path = {{5_in, 5_in}, {0_in, 0_in}, {0_in, -5_in}, {-5_in, 0_in}};
    auto angle = CalculateCuspAngle(path, 0);
    EXPECT_TRUE(UnitEqual(angle, -135_deg));
    angle = CalculateCuspAngle(path, 1);
    EXPECT_TRUE(UnitEqual(angle, 45_deg));
  }
}

TEST(DecomposeVelocityTest, OriginFulcrum) {
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(3_ft, 0_ft), .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        ArmPathPoint(3_ft, 0_in));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(2_ft, 2_ft), .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        ArmPathPoint(2_ft, 2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(3_ft, 0_ft), .velocity = {.v = 10_ips, .v_x = 10_ips, .v_z = 0_ips}},
        ArmPathPoint(3_ft, 0_in));
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(3_ft, 0_ft), .velocity = {.v = 10_ips, .v_x = -10_ips, .v_z = 0_ips}},
        ArmPathPoint(3_ft, 0_in));
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(2_ft, 2_ft),
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft, 2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(2_ft, 2_ft),
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft, 2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(2_ft, -2_ft),
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft, -2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(2_ft, 0_ft), .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = 10_ips}},
        ArmPathPoint(2_ft, 0_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(2_ft, 0_ft), .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = -10_ips}},
        ArmPathPoint(2_ft, 0_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(2_ft / std::numbers::sqrt2, 2_ft / std::numbers::sqrt2),
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft / std::numbers::sqrt2, 2_ft / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(2_ft / std::numbers::sqrt2, 2_ft / std::numbers::sqrt2),
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft / std::numbers::sqrt2, 2_ft / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
}

TEST(DecomposeVelocityTest, OffsetFulcrum) {
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(4_ft, 7_ft), .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        ArmPathPoint(3_ft, 0_in));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(1_ft, -7_ft), .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        ArmPathPoint(2_ft, 2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity({.time = 0_ms,
                                         .position = ArmPathPoint(0.3_ft, 0.4_ft),
                                         .velocity = {.v = 10_ips, .v_x = 10_ips, .v_z = 0_ips}},
                                        ArmPathPoint(3_ft, 0_in));
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(35_ft, 10_ft), .velocity = {.v = 10_ips, .v_x = -10_ips, .v_z = 0_ips}},
        ArmPathPoint(3_ft, 0_in));
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(20_ft, 5_ft),
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft, 2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(-102_ft, 23_ft),
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft, 2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(5_ft, 5_ft),
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft, -2_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(1_m, 3_in), .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = 10_ips}},
        ArmPathPoint(2_ft, 0_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = ArmPathPoint(4_cm, 100_in), .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = -10_ips}},
        ArmPathPoint(2_ft, 0_ft));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(3.14_ft, 1.57_ft),
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft / std::numbers::sqrt2, 2_ft / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = ArmPathPoint(0_ft, 0_ft),
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        ArmPathPoint(2_ft / std::numbers::sqrt2, 2_ft / std::numbers::sqrt2));
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
}

TEST(SegmentProfileTest, LongPathsContinuity) {
  {
    auto path =
        path_planning::ArmPath{{0_in, 0_in}, {0_in, 100_in}, {100_in, 100_in}, {200_in, 100_in}, {100_in, 100_in}};
    auto segmentLengths = path_planning::GenerateSegmentLengths(path);
    auto constraints = path_planning::PathDynamicsConstraints{60_ips, 60_ips2};

    auto segmentProfiles = path_planning::GenerateContinuousSegmentProfiles(segmentLengths, path, constraints);

    for (size_t index = 0; index < segmentProfiles.size() - 1; ++index) {
      auto endPointState = segmentProfiles.at(index).Calculate(segmentProfiles.at(index).TotalTime());
      auto startPointState = segmentProfiles.at(index + 1).Calculate(0_s);
      EXPECT_TRUE(UnitEqual(endPointState.velocity, startPointState.velocity));
    }
  }
}

TEST(SegmentProfileTest, ShortPathsContinuity) {
  {
    auto path = path_planning::ArmPath{{29.5_in, 7.5_in}, {19.5_in, 18_in}, {16.5_in, 18_in}, {14_in, 18_in}};
    auto segmentLengths = path_planning::GenerateSegmentLengths(path);
    auto constraints = path_planning::PathDynamicsConstraints{60_ips, 60_ips2};

    auto segmentProfiles = path_planning::GenerateContinuousSegmentProfiles(segmentLengths, path, constraints);

    for (size_t index = 0; index < segmentProfiles.size() - 1; ++index) {
      auto endPointState = segmentProfiles.at(index).Calculate(segmentProfiles.at(index).TotalTime());
      auto startPointState = segmentProfiles.at(index + 1).Calculate(0_s);
      EXPECT_TRUE(UnitEqual(endPointState.velocity, startPointState.velocity, 1_ips));
    }
  }
}
