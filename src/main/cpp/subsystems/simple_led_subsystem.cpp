/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/simple_led_subsystem.h"

#include <ctre/phoenix/led/FireAnimation.h>

#include "argos_lib/config/config_types.h"
#include "constants/addresses.h"

SimpleLedSubsystem::SimpleLedSubsystem(argos_lib::RobotInstance instance)
    : m_CANdle{GetCANAddr(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance),
               std::string(GetCANBus(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance))} {}
// This method will be called once per scheduler run
void SimpleLedSubsystem::Periodic() {}

void SimpleLedSubsystem::SetBackLeftSolidColor(frc::Color color) {
  m_CANdle.ClearAnimation(0);  // Stop any animations so we can do solid color
  m_CANdle.SetLEDs(color.red, color.green, color.blue, 0, startIndex_backLeft, length_backLeft);
}

void SimpleLedSubsystem::FireEverywhere() {
  auto fireAnimationBL =
      ctre::phoenix::led::FireAnimation(1, 1, length_backLeft, 1, 0.2, inverted_backLeft, startIndex_backLeft);
  m_CANdle.Animate(fireAnimationBL, 0);
  auto fireAnimationBR =
      ctre::phoenix::led::FireAnimation(1, 1, length_backRight, 1, 0.2, inverted_backRight, startIndex_backRight);
  m_CANdle.Animate(fireAnimationBR, 1);
  auto fireAnimationSB =
      ctre::phoenix::led::FireAnimation(1, 1, length_sideBack, 1, 0.2, inverted_sideBack, startIndex_sideBack);
  m_CANdle.Animate(fireAnimationSB, 2);
  auto fireAnimationSF =
      ctre::phoenix::led::FireAnimation(1, 1, length_sideFront, 1, 0.2, inverted_sideFront, startIndex_sideFront);
  m_CANdle.Animate(fireAnimationSF, 3);
  auto fireAnimationFL =
      ctre::phoenix::led::FireAnimation(1, 1, length_frontLeft, 1, 0.2, inverted_frontLeft, startIndex_frontLeft);
  m_CANdle.Animate(fireAnimationFL, 4);
  auto fireAnimationFR =
      ctre::phoenix::led::FireAnimation(1, 1, length_frontRight, 1, 0.2, inverted_frontRight, startIndex_frontRight);
  m_CANdle.Animate(fireAnimationFR, 5);
}
