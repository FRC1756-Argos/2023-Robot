/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/simple_led_subsystem.h"

#include <ctre/phoenix/led/FireAnimation.h>
#include <frc/DriverStation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>

#include "argos_lib/config/config_types.h"
#include "constants/addresses.h"

SimpleLedSubsystem::SimpleLedSubsystem(argos_lib::RobotInstance instance)
    : m_CANdle{GetCANAddr(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance),
               std::string(GetCANBus(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance))}
    , m_log{"SIMPLE_LED_SUBSYSTEM"} {
  SetAllGropusOff();
}
// This method will be called once per scheduler run
void SimpleLedSubsystem::Periodic() {}

void SimpleLedSubsystem::SetLedGroupColor(LedGroup group, argos_lib::ArgosColor color) {
  m_CANdle.ClearAnimation(0);
  int startIndx = -1;
  int len = -1;
  switch (group) {
    case LedGroup::SIDES:
      startIndx = startIndex_sideFront;
      len = length_sideBack + length_sideFront;
      break;
    case LedGroup::BACK:
      startIndx = startIndex_backRight;
      len = length_backRight + length_backLeft;
      break;
    case LedGroup::FRONT:
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
  int len =
      length_backLeft + length_backRight + length_sideBack + length_sideFront + length_frontLeft + length_frontRight;
  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.SetLEDs(color.r, color.g, color.b, 0, startIndex_frontLeft, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsAllianceColor() {
  frc::DriverStation::Alliance allianceColor = frc::DriverStation::GetAlliance();
  // If invalid, set all groups just off
  if (allianceColor == frc::DriverStation::Alliance::kInvalid) {
    SetAllGropusOff();
  } else if (allianceColor == frc::DriverStation::Alliance::kBlue) {
    SetAllGroupsColor(argos_lib::colors::kReallyBlue);
  } else if (allianceColor == frc::DriverStation::Alliance::kRed) {
    SetAllGroupsColor(argos_lib::colors::kReallyRed);
  }
}

void SimpleLedSubsystem::SetAllGropusOff() {
  m_CANdle.ClearAnimation(0);
  m_CANdle.ClearAnimation(1);
  m_CANdle.ClearAnimation(2);
  m_CANdle.ClearAnimation(3);
  m_CANdle.ClearAnimation(4);
  m_CANdle.ClearAnimation(5);
  int len =
      length_backLeft + length_backRight + length_sideBack + length_sideFront + length_frontLeft + length_frontRight;
  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.SetLEDs(0, 0, 0, 0, startIndex_frontLeft, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::FireEverywhere() {
  auto fireAnimationBL =
      ctre::phoenix::led::FireAnimation(0.5, 0.7, length_backLeft, 1, 0.2, inverted_backLeft, startIndex_backLeft);
  m_CANdle.Animate(fireAnimationBL, 0);
  auto fireAnimationBR =
      ctre::phoenix::led::FireAnimation(0.5, 0.7, length_backRight, 1, 0.2, inverted_backRight, startIndex_backRight);
  m_CANdle.Animate(fireAnimationBR, 1);
  auto fireAnimationSB =
      ctre::phoenix::led::FireAnimation(0.5, 0.7, length_sideBack, 1, 0.2, inverted_sideBack, startIndex_sideBack);
  m_CANdle.Animate(fireAnimationSB, 2);
  auto fireAnimationSF =
      ctre::phoenix::led::FireAnimation(0.5, 0.7, length_sideFront, 1, 0.2, inverted_sideFront, startIndex_sideFront);
  m_CANdle.Animate(fireAnimationSF, 3);
  auto fireAnimationFL =
      ctre::phoenix::led::FireAnimation(0.5, 0.7, length_frontLeft, 1, 0.2, inverted_frontLeft, startIndex_frontLeft);
  m_CANdle.Animate(fireAnimationFL, 4);
  auto fireAnimationFR = ctre::phoenix::led::FireAnimation(
      0.5, 0.7, length_frontRight, 1, 0.2, inverted_frontRight, startIndex_frontRight);
  m_CANdle.Animate(fireAnimationFR, 5);
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
