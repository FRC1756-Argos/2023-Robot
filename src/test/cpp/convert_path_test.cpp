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

TEST(DecomposeVelocityTest, OriginFulcrum) {
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 3_ft, .z = 0_ft}, .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        {.x = 3_ft, .z = 0_in});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 2_ft, .z = 2_ft}, .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        {.x = 2_ft, .z = 2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 3_ft, .z = 0_ft}, .velocity = {.v = 10_ips, .v_x = 10_ips, .v_z = 0_ips}},
        {.x = 3_ft, .z = 0_in});
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 3_ft, .z = 0_ft}, .velocity = {.v = 10_ips, .v_x = -10_ips, .v_z = 0_ips}},
        {.x = 3_ft, .z = 0_in});
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 2_ft, .z = 2_ft},
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        {.x = 2_ft, .z = 2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 2_ft, .z = 2_ft},
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        {.x = 2_ft, .z = 2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 2_ft, .z = -2_ft},
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        {.x = 2_ft, .z = -2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 2_ft, .z = 0_ft}, .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = 10_ips}},
        {.x = 2_ft, .z = 0_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 2_ft, .z = 0_ft}, .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = -10_ips}},
        {.x = 2_ft, .z = 0_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 2_ft / std::numbers::sqrt2, .z = 2_ft / std::numbers::sqrt2},
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        {.x = 2_ft / std::numbers::sqrt2, .z = 2_ft / std::numbers::sqrt2});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 2_ft / std::numbers::sqrt2, .z = 2_ft / std::numbers::sqrt2},
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        {.x = 2_ft / std::numbers::sqrt2, .z = 2_ft / std::numbers::sqrt2});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
}

TEST(DecomposeVelocityTest, OffsetFulcrum) {
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 4_ft, .z = 7_ft}, .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        {.x = 3_ft, .z = 0_in});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 1_ft, .z = -7_ft}, .velocity = {.v = 0_ips, .v_x = 0_ips, .v_z = 0_ips}},
        {.x = 2_ft, .z = 2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 0.3_ft, .z = 0.4_ft}, .velocity = {.v = 10_ips, .v_x = 10_ips, .v_z = 0_ips}},
        {.x = 3_ft, .z = 0_in});
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 35_ft, .z = 10_ft}, .velocity = {.v = 10_ips, .v_x = -10_ips, .v_z = 0_ips}},
        {.x = 3_ft, .z = 0_in});
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 20_ft, .z = 5_ft},
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        {.x = 2_ft, .z = 2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = -102_ft, .z = 23_ft},
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        {.x = 2_ft, .z = 2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 5_ft, .z = 5_ft},
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        {.x = 2_ft, .z = -2_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, -10_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 0_deg_per_s));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 1_m, .z = 3_in}, .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = 10_ips}},
        {.x = 2_ft, .z = 0_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms, .position = {.x = 4_cm, .z = 100_in}, .velocity = {.v = 10_ips, .v_x = 0_ips, .v_z = -10_ips}},
        {.x = 2_ft, .z = 0_ft});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 3.14_ft, .z = 1.57_ft},
         .velocity = {.v = 10_ips, .v_x = -10_ips / std::numbers::sqrt2, .v_z = 10_ips / std::numbers::sqrt2}},
        {.x = 2_ft / std::numbers::sqrt2, .z = 2_ft / std::numbers::sqrt2});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, 1_rad * 10_ips / 2_ft));
  }
  {
    auto components = DecomposeVelocity(
        {.time = 0_ms,
         .position = {.x = 0_ft, .z = 0_ft},
         .velocity = {.v = 10_ips, .v_x = 10_ips / std::numbers::sqrt2, .v_z = -10_ips / std::numbers::sqrt2}},
        {.x = 2_ft / std::numbers::sqrt2, .z = 2_ft / std::numbers::sqrt2});
    EXPECT_TRUE(UnitEqual(components.v_radial, 0_ips));
    EXPECT_TRUE(UnitEqual(components.v_tangential, -1_rad * 10_ips / 2_ft));
  }
}
