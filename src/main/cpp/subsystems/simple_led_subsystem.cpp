/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/simple_led_subsystem.h"

#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <frc/DriverStation.h>

#include "argos_lib/config/config_types.h"
#include "constants/addresses.h"

SimpleLedSubsystem::SimpleLedSubsystem(argos_lib::RobotInstance instance)
    : m_CANdle{GetCANAddr(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance),
               std::string(GetCANBus(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance))}
    , m_log{"SIMPLE_LED_SUBSYSTEM"} {
  SetAllGroupsOff();
}
// This method will be called once per scheduler run
void SimpleLedSubsystem::Periodic() {}

void SimpleLedSubsystem::SetLedGroupColor(LedGroup group, argos_lib::ArgosColor color) {
  int startIndx = -1;
  int len = -1;
  switch (group) {
    case LedGroup::SIDES:
      m_CANdle.ClearAnimation(2);
      m_CANdle.ClearAnimation(3);
      startIndx = startIndex_sideFront;
      len = length_sideBack + length_sideFront;
      break;
    case LedGroup::BACK:
      m_CANdle.ClearAnimation(4);
      m_CANdle.ClearAnimation(5);
      startIndx = startIndex_backRight;
      len = length_backRight + length_backLeft;
      break;
    case LedGroup::FRONT:
      m_CANdle.ClearAnimation(0);
      m_CANdle.ClearAnimation(1);
      startIndx = startIndex_frontLeft;
      len = length_frontLeft + length_frontRight;
      break;

    default:
      break;
  }

  if (startIndx < 0 || len < 0) {
    m_log.Log(argos_lib::LogLevel::ERR, "INVALID LED LENGTH OR START INDEX\n");
  }

  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.SetLEDs(color.r, color.g, color.b, 0, startIndx, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsColor(argos_lib::ArgosColor color) {
  StopAllAnimations();
  int len =
      length_backLeft + length_backRight + length_sideBack + length_sideFront + length_frontLeft + length_frontRight;
  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.SetLEDs(color.r, color.g, color.b, 0, startIndex_frontLeft, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsFade(argos_lib::ArgosColor color) {
  std::array<int, 6> lengths = {
      length_frontLeft, length_frontRight, length_sideFront, length_sideBack, length_backRight, length_backLeft};
  std::array<int, 6> offsets = {startIndex_frontLeft,
                                startIndex_frontRight,
                                startIndex_sideFront,
                                startIndex_sideBack,
                                startIndex_backRight,
                                startIndex_backLeft};
  for (size_t i = 0; i < lengths.size(); ++i) {
    auto fadeAnimation =
        ctre::phoenix::led::SingleFadeAnimation(color.r, color.g, color.b, 0, 0.9, lengths.at(i), offsets.at(i));
    m_CANdle.Animate(fadeAnimation, i);
  }
}

void SimpleLedSubsystem::SetAllGroupsFlash(argos_lib::ArgosColor color) {
  std::array<int, 6> lengths = {
      length_frontLeft, length_frontRight, length_sideFront, length_sideBack, length_backRight, length_backLeft};
  std::array<int, 6> offsets = {startIndex_frontLeft,
                                startIndex_frontRight,
                                startIndex_sideFront,
                                startIndex_sideBack,
                                startIndex_backRight,
                                startIndex_backLeft};
  for (size_t i = 0; i < lengths.size(); ++i) {
    auto flashAnimation =
        ctre::phoenix::led::StrobeAnimation(color.r, color.g, color.b, 0, 0.1, lengths.at(i), offsets.at(i));
    m_CANdle.Animate(flashAnimation, i);
  }
}

void SimpleLedSubsystem::SetAllGroupsLarson(argos_lib::ArgosColor color) {
  std::array<int, 6> lengths = {
      length_frontLeft, length_frontRight, length_sideFront, length_sideBack, length_backRight, length_backLeft};
  std::array<int, 6> offsets = {startIndex_frontLeft,
                                startIndex_frontRight,
                                startIndex_sideFront,
                                startIndex_sideBack,
                                startIndex_backRight,
                                startIndex_backLeft};
  for (size_t i = 0; i < lengths.size(); ++i) {
    auto larsonAnimation = ctre::phoenix::led::LarsonAnimation(color.r,
                                                               color.g,
                                                               color.b,
                                                               0,
                                                               0.1,
                                                               lengths.at(i),
                                                               ctre::phoenix::led::LarsonAnimation::Front,
                                                               10,
                                                               offsets.at(i));
    m_CANdle.Animate(larsonAnimation, i);
  }
}

argos_lib::ArgosColor SimpleLedSubsystem::GetAllianceColor() {
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    return argos_lib::colors::kReallyBlue;
  } else {
    return argos_lib::colors::kReallyRed;
  }
}

void SimpleLedSubsystem::SetAllGroupsAllianceColor(bool fade) {
  frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();
  // If invalid, set all groups just off
  auto color = argos_lib::colors::kOff;
  if (allianceColor == frc::DriverStation::Alliance::kBlue) {
    color = argos_lib::colors::kReallyBlue;
  } else if (allianceColor == frc::DriverStation::Alliance::kRed) {
    color = argos_lib::colors::kReallyRed;
  }
  if (fade) {
    SetAllGroupsFade(color);
  } else {
    SetAllGroupsColor(color);
  }
}

void SimpleLedSubsystem::StopAllAnimations() {
  m_CANdle.ClearAnimation(0);
  m_CANdle.ClearAnimation(1);
  m_CANdle.ClearAnimation(2);
  m_CANdle.ClearAnimation(3);
  m_CANdle.ClearAnimation(4);
  m_CANdle.ClearAnimation(5);
}

void SimpleLedSubsystem::SetAllGroupsOff() {
  SetAllGroupsColor(argos_lib::colors::kOff);
}

void SimpleLedSubsystem::FireEverywhere() {
  std::array<int, 6> lengths = {
      length_frontLeft, length_frontRight, length_sideFront, length_sideBack, length_backRight, length_backLeft};
  std::array<int, 6> offsets = {startIndex_frontLeft,
                                startIndex_frontRight,
                                startIndex_sideFront,
                                startIndex_sideBack,
                                startIndex_backRight,
                                startIndex_backLeft};
  std::array<bool, 6> inverts = {inverted_frontLeft,
                                 inverted_frontRight,
                                 inverted_sideFront,
                                 inverted_sideBack,
                                 inverted_backRight,
                                 inverted_backLeft};
  for (size_t i = 0; i < lengths.size(); ++i) {
    auto fireAnimation =
        ctre::phoenix::led::FireAnimation(0.5, 0.7, lengths.at(i), 1, 0.2, inverts.at(i), offsets.at(i));
    m_CANdle.Animate(fireAnimation, i);
  }
}
void SimpleLedSubsystem::Blind() {
  auto strobeAnimationBL =
      ctre::phoenix::led::StrobeAnimation(0, 100, 0, 0, 0.14, length_backLeft, startIndex_backLeft);
  m_CANdle.Animate(strobeAnimationBL, 0);
  auto strobeAnimationFR =
      ctre::phoenix::led::StrobeAnimation(211, 138, 31, 0, 0.15, length_frontRight, startIndex_frontRight);
  m_CANdle.Animate(strobeAnimationFR, 1);
  auto strobeAnimationBR =
      ctre::phoenix::led::StrobeAnimation(0, 0, 90, 0, 0.16, length_backRight, startIndex_backRight);
  m_CANdle.Animate(strobeAnimationBR, 2);
  auto strobeAnimationSB =
      ctre::phoenix::led::StrobeAnimation(100, 100, 0, 0, 0.17, length_sideBack, startIndex_sideBack);
  m_CANdle.Animate(strobeAnimationSB, 3);
  auto strobeAnimationSF =
      ctre::phoenix::led::StrobeAnimation(0, 100, 100, 0, 0.18, length_sideFront, startIndex_sideFront);
  m_CANdle.Animate(strobeAnimationSF, 4);
  auto strobeAnimationFL =
      ctre::phoenix::led::StrobeAnimation(211, 138, 31, 0, 0.19, length_frontLeft, startIndex_frontLeft);
  m_CANdle.Animate(strobeAnimationFL, 5);
}
