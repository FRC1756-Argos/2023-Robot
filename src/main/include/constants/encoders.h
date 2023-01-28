/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once
#include <ctre/phoenix/sensors/CANCoder.h>

namespace encoder_conf {
  namespace comp_bot {
    struct shoulderEncoderConf {
      constexpr static auto direction = true;
      constexpr static auto range = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    };
  }  // namespace comp_bot

  namespace practice_bot {
    using shoulderEncoderConf = encoder_conf::comp_bot::shoulderEncoderConf;
  }  // namespace practice_bot

}  // namespace encoder_conf
