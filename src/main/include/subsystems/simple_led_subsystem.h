/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include "argos_lib/config/config_types.h"

class SimpleLedSubsystem : public frc2::SubsystemBase {
 public:
  explicit SimpleLedSubsystem(argos_lib::RobotInstance instance);
  void SetBackLeftSolidColor(frc::Color color);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void FireEverywhere();
  void Blind();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  CANdle m_CANdle;
  constexpr static int startIndex_frontLeft = 8;     ///< Address of first LED in strip
  constexpr static int length_frontLeft = 30;        ///< Number of LEDs in strip
  constexpr static bool inverted_frontLeft = false;  ///< true indicates first index is at top of tower
  constexpr static int startIndex_frontRight = 38;
  constexpr static int length_frontRight = 30;
  constexpr static bool inverted_frontRight = true;
  constexpr static int startIndex_sideFront = 68;
  constexpr static int length_sideFront = 61;
  constexpr static bool inverted_sideFront = false;
  constexpr static int startIndex_sideBack = 129;
  constexpr static int length_sideBack = 61;
  constexpr static bool inverted_sideBack = true;
  constexpr static int startIndex_backRight = 190;
  constexpr static int length_backRight = 58;
  constexpr static bool inverted_backRight = false;
  constexpr static int startIndex_backLeft = 248;
  constexpr static int length_backLeft = 57;
  constexpr static bool inverted_backLeft = true;
};
