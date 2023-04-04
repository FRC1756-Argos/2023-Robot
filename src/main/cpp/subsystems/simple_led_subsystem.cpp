/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "subsystems/simple_led_subsystem.h"

#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <frc/DriverStation.h>

#include <chrono>
#include <numbers>

#include "argos_lib/config/config_types.h"
#include "constants/addresses.h"

using namespace std::chrono_literals;

SimpleLedSubsystem::SimpleLedSubsystem(argos_lib::RobotInstance instance)
    : m_CANdle{instance == argos_lib::RobotInstance::Competition ?
                   std::make_optional<CANdle>(
                       GetCANAddr(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance),
                       std::string(
                           GetCANBus(address::comp_bot::led::CANdle, address::practice_bot::led::CANdle, instance))) :
                   std::nullopt}
    , m_log{"SIMPLE_LED_SUBSYSTEM"}
    , m_enabled{false}
    , m_hasBeenConnected{false}
    , m_disableUpdateFunction{[]() {}}
    , m_ledUpdateFunction{[]() {}}
    , m_restoreAnimationFunction{std::nullopt}
    , m_startTime{std::chrono::steady_clock::now()}
    , m_temporaryDuration{0_ms} {
  SetAllGroupsOff();
}

void SimpleLedSubsystem::Enable() {
  m_enabled = true;
}
void SimpleLedSubsystem::Disable() {
  m_enabled = false;
  if (m_restoreAnimationFunction) {
    m_ledUpdateFunction = m_restoreAnimationFunction.value();
    m_restoreAnimationFunction = std::nullopt;
  }
}

void SimpleLedSubsystem::SetLedsConnectedBrightness(bool connected) {
  if (!m_CANdle) {
    return;  // Do nothing, because there is no CANdle on here anyway
  }
  m_CANdle.value().ConfigBrightnessScalar(connected ? 0.8 : 0.1);
  if (connected) {
    m_hasBeenConnected = true;
  }
}

void SimpleLedSubsystem::SetDisableAnimation(std::function<void()> animationFunction) {
  m_disableUpdateFunction = animationFunction;
}

// This method will be called once per scheduler run
void SimpleLedSubsystem::Periodic() {
  if (m_enabled) {
    m_ledUpdateFunction();
    if (m_restoreAnimationFunction &&
        units::millisecond_t(std::chrono::steady_clock::now() - m_startTime) > m_temporaryDuration) {
      m_ledUpdateFunction = m_restoreAnimationFunction.value();
      m_restoreAnimationFunction = std::nullopt;
    }
  } else {
    m_disableUpdateFunction();
  }
}

void SimpleLedSubsystem::SetLedGroupColor(LedGroup group, argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, group, color]() { this->SetLedGroupColor(group, color, false); };
  }

  int startIndx = -1;
  int len = -1;
  switch (group) {
    case LedGroup::SIDES:
      m_CANdle.value().ClearAnimation(2);
      m_CANdle.value().ClearAnimation(3);
      startIndx = startIndex_sideFront;
      len = length_sideBack + length_sideFront;
      break;
    case LedGroup::BACK:
      m_CANdle.value().ClearAnimation(4);
      m_CANdle.value().ClearAnimation(5);
      startIndx = startIndex_backRight;
      len = length_backRight + length_backLeft;
      break;
    case LedGroup::FRONT:
      m_CANdle.value().ClearAnimation(0);
      m_CANdle.value().ClearAnimation(1);
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
  rslt = m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, startIndx, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetLedStripColor(LedStrip strip, argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, strip, color]() { this->SetLedStripColor(strip, color, false); };
  }

  int startIndex = -1;
  int len = -1;
  switch (strip) {
    case LedStrip::FrontLeft:
      m_CANdle.value().ClearAnimation(0);
      startIndex = startIndex_frontLeft;
      len = length_frontLeft;
      break;
    case LedStrip::FrontRight:
      m_CANdle.value().ClearAnimation(1);
      startIndex = startIndex_frontRight;
      len = length_frontRight;
      break;
    case LedStrip::SideFront:
      m_CANdle.value().ClearAnimation(2);
      startIndex = startIndex_sideFront;
      len = length_sideFront;
      break;
    case LedStrip::SideBack:
      m_CANdle.value().ClearAnimation(3);
      startIndex = startIndex_sideBack;
      len = length_sideBack;
      break;
    case LedStrip::BackLeft:
      m_CANdle.value().ClearAnimation(5);
      startIndex = startIndex_backLeft;
      len = length_backLeft;
      break;
    case LedStrip::BackRight:
      m_CANdle.value().ClearAnimation(4);
      startIndex = startIndex_backRight;
      len = length_backRight;
      break;
  }

  if (startIndex < 0 || len < 0) {
    m_log.Log(argos_lib::LogLevel::ERR, "INVALID LED LENGTH OR START INDEX\n");
  }

  ctre::phoenix::ErrorCode rslt;
  rslt = m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, startIndex, len);
  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsColor(argos_lib::ArgosColor color,
                                           bool restorable,
                                           std::optional<std::function<GamePiece()>> tipColor) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color, tipColor]() { this->SetAllGroupsColor(color, false, tipColor); };
  }

  StopAllAnimations(false);
  if (!tipColor) {
    int len =
        length_backLeft + length_backRight + length_sideBack + length_sideFront + length_frontLeft + length_frontRight;
    ctre::phoenix::ErrorCode rslt;
    rslt = m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, startIndex_frontLeft, len);
    if (rslt != ctre::phoenix::ErrorCode::OKAY) {
      m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
    }
  } else {
    const int tipSize = 10;

    std::array<int, 6> lengths = {length_frontLeft - tipSize,
                                  length_frontRight - tipSize,
                                  length_sideFront - tipSize,
                                  length_sideBack - tipSize,
                                  length_backRight - tipSize,
                                  length_backLeft - tipSize};
    std::array<int, 6> offsets = {(inverted_frontLeft ? tipSize : 0) + startIndex_frontLeft,
                                  (inverted_frontRight ? tipSize : 0) + startIndex_frontRight,
                                  (inverted_sideFront ? tipSize : 0) + startIndex_sideFront,
                                  (inverted_sideBack ? tipSize : 0) + startIndex_sideBack,
                                  (inverted_backRight ? tipSize : 0) + startIndex_backRight,
                                  (inverted_backLeft ? tipSize : 0) + startIndex_backLeft};
    for (size_t i = 0; i < lengths.size(); ++i) {
      m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, offsets.at(i), lengths.at(i));
    }

    std::array<int, 6> tipOffsets = {(inverted_frontLeft ? 0 : length_frontLeft - tipSize) + startIndex_frontLeft,
                                     (inverted_frontRight ? 0 : length_frontRight - tipSize) + startIndex_frontRight,
                                     (inverted_sideFront ? 0 : length_sideFront - tipSize) + startIndex_sideFront,
                                     (inverted_sideBack ? 0 : length_sideBack - tipSize) + startIndex_sideBack,
                                     (inverted_backRight ? 0 : length_backRight - tipSize) + startIndex_backRight,
                                     (inverted_backLeft ? 0 : length_backLeft - tipSize) + startIndex_backLeft};
    auto color = GetGamePieceColor(tipColor.value()());
    for (size_t i = 0; i < tipOffsets.size(); ++i) {
      m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, tipOffsets.at(i), tipSize);
    }
  }
}

void SimpleLedSubsystem::SetAllGroupsFade(argos_lib::ArgosColor color,
                                          bool restorable,
                                          std::optional<std::function<GamePiece()>> tipColor) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color, tipColor]() { this->SetAllGroupsFade(color, false, tipColor); };
  }

  const int tipSize = tipColor ? 10 : 0;

  std::array<int, 6> lengths = {length_frontLeft - tipSize,
                                length_frontRight - tipSize,
                                length_sideFront - tipSize,
                                length_sideBack - tipSize,
                                length_backRight - tipSize,
                                length_backLeft - tipSize};
  std::array<int, 6> offsets = {(inverted_frontLeft ? tipSize : 0) + startIndex_frontLeft,
                                (inverted_frontRight ? tipSize : 0) + startIndex_frontRight,
                                (inverted_sideFront ? tipSize : 0) + startIndex_sideFront,
                                (inverted_sideBack ? tipSize : 0) + startIndex_sideBack,
                                (inverted_backRight ? tipSize : 0) + startIndex_backRight,
                                (inverted_backLeft ? tipSize : 0) + startIndex_backLeft};
  for (size_t i = 0; i < lengths.size(); ++i) {
    auto fadeAnimation =
        ctre::phoenix::led::SingleFadeAnimation(color.r, color.g, color.b, 0, 0.7, lengths.at(i), offsets.at(i));
    m_CANdle.value().Animate(fadeAnimation, i);
  }

  if (tipColor) {
    std::array<int, 6> tipOffsets = {(inverted_frontLeft ? 0 : length_frontLeft - tipSize) + startIndex_frontLeft,
                                     (inverted_frontRight ? 0 : length_frontRight - tipSize) + startIndex_frontRight,
                                     (inverted_sideFront ? 0 : length_sideFront - tipSize) + startIndex_sideFront,
                                     (inverted_sideBack ? 0 : length_sideBack - tipSize) + startIndex_sideBack,
                                     (inverted_backRight ? 0 : length_backRight - tipSize) + startIndex_backRight,
                                     (inverted_backLeft ? 0 : length_backLeft - tipSize) + startIndex_backLeft};
    auto color = GetGamePieceColor(tipColor.value()());
    for (size_t i = 0; i < tipOffsets.size(); ++i) {
      m_CANdle.value().SetLEDs(color.r, color.g, color.b, 0, tipOffsets.at(i), tipSize);
    }
  }
}

void SimpleLedSubsystem::SetAllGroupsFlash(argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;
  }
  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsFlash(color, false); };
  }

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
    m_CANdle.value().Animate(flashAnimation, i);
  }
}

void SimpleLedSubsystem::FlashStrip(LedStrip strip, argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing;
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsFlash(color, false); };
  }

  int startIndex = -1;
  int len = -1;

  switch (strip) {
    case LedStrip::FrontLeft:
      startIndex = startIndex_frontLeft;
      len = length_frontLeft;
      break;
    case LedStrip::FrontRight:
      startIndex = startIndex_frontRight;
      len = length_frontRight;
      break;
    case LedStrip::SideFront:
      startIndex = startIndex_sideFront;
      len = length_sideFront;
      break;
    case LedStrip::SideBack:
      startIndex = startIndex_sideBack;
      len = length_sideBack;
      break;
    case LedStrip::BackLeft:
      startIndex = startIndex_backLeft;
      len = length_backLeft;
      break;
    case LedStrip::BackRight:
      startIndex = startIndex_backRight;
      len = length_backRight;
      break;
  }

  if (startIndex < 0 || len < 0) {
    m_log.Log(argos_lib::LogLevel::ERR, "INVALID LED LENGTH OR START INDEX\n");
  }

  auto flashAnimation = ctre::phoenix::led::StrobeAnimation(color.r, color.g, color.b, 0, 0.1, len, startIndex);
  auto rslt = m_CANdle.value().Animate(flashAnimation, static_cast<int>(strip));

  if (rslt != ctre::phoenix::ErrorCode::OKAY) {
    m_log.Log(argos_lib::LogLevel::ERR, "CANDle::SetLEDs() returned error[%d]", rslt);
  }
}

void SimpleLedSubsystem::SetAllGroupsLarson(argos_lib::ArgosColor color, bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this, color]() { this->SetAllGroupsLarson(color, false); };
  }

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
    m_CANdle.value().Animate(larsonAnimation, i);
  }
}

argos_lib::ArgosColor SimpleLedSubsystem::GetAllianceColor() {
  if (!m_hasBeenConnected) {
    return argos_lib::gamma_corrected_colors::kCatYellow;
  } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    return argos_lib::colors::kReallyBlue;
  } else {
    return argos_lib::colors::kReallyRed;
  }
}

void SimpleLedSubsystem::SetAllGroupsAllianceColor(bool fade,
                                                   bool restorable,
                                                   std::optional<std::function<GamePiece()>> tipColor) {
  if (restorable) {
    m_ledUpdateFunction = [this, fade, tipColor]() { this->SetAllGroupsAllianceColor(fade, false, tipColor); };
  }

  auto color = GetAllianceColor();
  if (fade) {
    SetAllGroupsFade(color, false, tipColor);
  } else {
    SetAllGroupsColor(color, false, tipColor);
  }
}

void SimpleLedSubsystem::SetAllGroupsGamePieceColor(GamePiece gp, bool restorable) {
  if (restorable) {
    m_ledUpdateFunction = [this, gp]() { this->SetAllGroupsGamePieceColor(gp, false); };
  }

  SetAllGroupsColor(GetGamePieceColor(gp), false);
}

void SimpleLedSubsystem::StopAllAnimations(bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this]() { this->StopAllAnimations(false); };
  }

  m_CANdle.value().ClearAnimation(0);
  m_CANdle.value().ClearAnimation(1);
  m_CANdle.value().ClearAnimation(2);
  m_CANdle.value().ClearAnimation(3);
  m_CANdle.value().ClearAnimation(4);
  m_CANdle.value().ClearAnimation(5);
}

void SimpleLedSubsystem::SetAllGroupsOff(bool restorable) {
  if (restorable) {
    m_ledUpdateFunction = [this]() { this->SetAllGroupsOff(false); };
  }

  SetAllGroupsColor(argos_lib::colors::kOff, false);
}

void SimpleLedSubsystem::FireEverywhere(bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }
  if (restorable) {
    m_ledUpdateFunction = [this]() { this->FireEverywhere(false); };
  }

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
    m_CANdle.value().Animate(fireAnimation, i);
  }
}
void SimpleLedSubsystem::Blind(bool restorable) {
  if (!m_CANdle) {
    return;  // No CANdle, so do nothing
  }

  if (restorable) {
    m_ledUpdateFunction = [this]() { this->Blind(false); };
  }

  auto strobeAnimationFL =
      ctre::phoenix::led::StrobeAnimation(211, 138, 31, 0, 0.19, length_frontLeft, startIndex_frontLeft);
  m_CANdle.value().Animate(strobeAnimationFL, 0);
  auto strobeAnimationFR =
      ctre::phoenix::led::StrobeAnimation(211, 138, 31, 0, 0.15, length_frontRight, startIndex_frontRight);
  m_CANdle.value().Animate(strobeAnimationFR, 1);
  auto strobeAnimationSF =
      ctre::phoenix::led::StrobeAnimation(0, 100, 100, 0, 0.18, length_sideFront, startIndex_sideFront);
  m_CANdle.value().Animate(strobeAnimationSF, 2);
  auto strobeAnimationSB =
      ctre::phoenix::led::StrobeAnimation(100, 100, 0, 0, 0.17, length_sideBack, startIndex_sideBack);
  m_CANdle.value().Animate(strobeAnimationSB, 3);
  auto strobeAnimationBR =
      ctre::phoenix::led::StrobeAnimation(0, 0, 90, 0, 0.16, length_backRight, startIndex_backRight);
  m_CANdle.value().Animate(strobeAnimationBR, 4);
  auto strobeAnimationBL =
      ctre::phoenix::led::StrobeAnimation(0, 100, 0, 0, 0.14, length_backLeft, startIndex_backLeft);
  m_CANdle.value().Animate(strobeAnimationBL, 5);
}

void SimpleLedSubsystem::ColorSweep(argos_lib::ArgosColor color, bool correctGamma, bool restorable) {
  if (restorable) {
    m_restoreAnimationFunction = [this, color, correctGamma]() { this->ColorSweep(color, correctGamma, false); };
  }
  // No continuous update required
  m_ledUpdateFunction = [this, color, correctGamma]() { this->ColorSweep(color, correctGamma, false); };

  const auto period_ms = 2000.0;

  const auto now = std::chrono::steady_clock::now();
  const auto timeWithinPeriod =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() -
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) / (1ms * period_ms);

  const auto frontTime = std::fmod(static_cast<double>(timeWithinPeriod), period_ms);
  const auto sideFrontTime = std::fmod(frontTime + 250, period_ms);
  const auto sideBackTime = std::fmod(sideFrontTime + 250, period_ms);
  const auto backTime = std::fmod(sideBackTime + 250, period_ms);

  const auto frontAngle = 2 * std::numbers::pi * frontTime / period_ms;
  const auto sideFrontAngle = 2 * std::numbers::pi * sideFrontTime / period_ms;
  const auto sideBackAngle = 2 * std::numbers::pi * sideBackTime / period_ms;
  const auto backAngle = 2 * std::numbers::pi * backTime / period_ms;

  auto frontColor = color * (0.5 + std::sin(frontAngle) / 2.0);
  auto sideFrontColor = color * (0.5 + std::sin(sideFrontAngle) / 2.0);
  auto sideBackColor = color * (0.5 + std::sin(sideBackAngle) / 2.0);
  auto backColor = color * (0.5 + std::sin(backAngle) / 2.0);

  if (correctGamma) {
    frontColor = argos_lib::GammaCorrect(frontColor);
    sideFrontColor = argos_lib::GammaCorrect(sideFrontColor);
    sideBackColor = argos_lib::GammaCorrect(sideBackColor);
    backColor = argos_lib::GammaCorrect(backColor);
  }

  SetLedStripColor(LedStrip::FrontLeft, frontColor, false);
  SetLedStripColor(LedStrip::FrontRight, frontColor, false);
  SetLedStripColor(LedStrip::SideFront, sideFrontColor, false);
  SetLedStripColor(LedStrip::SideBack, sideBackColor, false);
  SetLedStripColor(LedStrip::BackLeft, backColor, false);
  SetLedStripColor(LedStrip::BackRight, backColor, false);
}

void SimpleLedSubsystem::TemporaryAnimate(std::function<void()> animationFunction, units::millisecond_t duration) {
  m_startTime = std::chrono::steady_clock::now();
  m_temporaryDuration = duration;
  if (!m_restoreAnimationFunction) {
    m_restoreAnimationFunction = m_ledUpdateFunction;
  }
  m_ledUpdateFunction = animationFunction;
}

constexpr argos_lib::ArgosColor SimpleLedSubsystem::GetGamePieceColor(GamePiece gp, bool gammaCorrect) {
  if (gp == GamePiece::CONE) {
    return gammaCorrect ? argos_lib::gamma_corrected_colors::kConeYellow : argos_lib::colors::kConeYellow;
  } else {
    return gammaCorrect ? argos_lib::gamma_corrected_colors::kCubePurple : argos_lib::colors::kCubePurple;
  }
}
