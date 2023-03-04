/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/general/log.h>

// Include GamePiece enum
#include <constants/field_points.h>
#include <ctre/Phoenix.h>
#include <frc/util/Color.h>
#include <frc2/command/SubsystemBase.h>

#include "argos_lib/config/config_types.h"
#include "argos_lib/general/color.h"

#include <chrono>
#include <functional>

enum class LedGroup { SIDES, BACK, FRONT };
enum class LedStrip { FrontLeft, FrontRight, SideFront, SideBack, BackRight, BackLeft };
enum class AlignLedStatus { NoTarget, FlashLeft, FlashRight, Aligned };

class SimpleLedSubsystem : public frc2::SubsystemBase {
 public:
  explicit SimpleLedSubsystem(argos_lib::RobotInstance instance);

  /// @brief Sets group of leds to given color
  /// @param group The group of leds to set
  /// @param color an ArgosColor to set the LEDs too
  void SetLedGroupColor(LedGroup group, argos_lib::ArgosColor color, bool restorable = true);
  void SetLedStripColor(LedStrip strip, argos_lib::ArgosColor color, bool restorable = true);

  /// @brief Sets all led groups to a given color
  /// @param color an ArgosColor to set the LEDs too
  void SetAllGroupsColor(argos_lib::ArgosColor color, bool restorable = true);

  void SetAllGroupsFade(argos_lib::ArgosColor color, bool restorable = true);

  void SetAllGroupsFlash(argos_lib::ArgosColor color, bool restorable = true);

  void FlashStrip(LedStrip strip, argos_lib::ArgosColor color, bool restorable = true);

  void SetAllGroupsLarson(argos_lib::ArgosColor color, bool restorable = true);

  argos_lib::ArgosColor GetAllianceColor();

  /// @brief Set all groups of LEDs to the alliance color
  void SetAllGroupsAllianceColor(bool fade, bool restorable = true);

  void StopAllAnimations(bool restorable = true);

  /// @brief Set all groups to color of given game piece
  void SetAllGroupsGamePieceColor(GamePiece gp, bool restorable = true);

  /// @brief Turn off all LEDs
  void SetAllGroupsOff(bool restorable = true);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void FireEverywhere(bool restorable = true);
  void Blind(bool restorable = true);

  void ColorSweep(argos_lib::ArgosColor color, bool correctGamma = true, bool restorable = true);

  void TemporaryAnimate(std::function<void()> animationFunction, units::millisecond_t duration);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  CANdle m_CANdle;
  argos_lib::ArgosLogger m_log;

  const std::function<void()> m_ledsOffFunction;
  std::function<void()> m_ledUpdateFunction;
  std::optional<std::function<void(void)>> m_restoreAnimationFunction;
  std::chrono::time_point<std::chrono::steady_clock> m_startTime;
  units::millisecond_t m_temporaryDuration;

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
