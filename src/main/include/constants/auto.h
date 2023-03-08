/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "constants/field_points.h"
#include "constants/measure_up.h"

namespace thresholds {
  /// @brief Target pitch to reach when climbing up the charge station
  constexpr units::degree_t robotClimbPitch = 12_deg;
  constexpr units::degree_t robotHitChargingStationPitch = 3_deg;
  constexpr units::degree_t robotLeftChargingStationPitch = -1_deg;
  constexpr units::degrees_per_second_t robotTippingPitchRate = -15_deg_per_s;
}  // namespace thresholds

namespace timeouts {
  /// @brief The amount of time to wait before robot should abort trying to achieve target pitch on charge station
  constexpr units::second_t robotClimbStation = 5_s;
  /// @todo add max time the robot should take to level the charge station
}  // namespace timeouts

namespace place_positions {
  namespace blue_alliance {
    constexpr auto loadingStationCube3d =
        field_points::blue_alliance::inner_grid::middleRowMiddle.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension, 0_m, 0_m};
    constexpr auto loadingStationCube = frc::Pose2d{{place_positions::blue_alliance::loadingStationCube3d.X(),
                                                     place_positions::blue_alliance::loadingStationCube3d.Y()},
                                                    180_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto loadingStationCube = utils::ReflectFieldPoint(place_positions::blue_alliance::loadingStationCube);
  }  // namespace red_alliance
}  // namespace place_positions

namespace starting_positions {
  namespace blue_alliance {
    constexpr auto loadingStationCone3d =
        field_points::blue_alliance::inner_grid::middleRowLeft.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension, 0_m, 0_m};
    constexpr auto loadingStationCone = frc::Pose2d{{starting_positions::blue_alliance::loadingStationCone3d.X(),
                                                     starting_positions::blue_alliance::loadingStationCone3d.Y()},
                                                    180_deg};

    constexpr auto cableProtectorCone3d =
        field_points::blue_alliance::outer_grid::topRowRight.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension, 0_m, 0_m};
    constexpr auto cableProtectorCone = frc::Pose2d{{starting_positions::blue_alliance::cableProtectorCone3d.X(),
                                                     starting_positions::blue_alliance::cableProtectorCone3d.Y()},
                                                    180_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto loadingStationCone =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::loadingStationCone);
    static const auto cableProtectorCone =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::cableProtectorCone);
  }  // namespace red_alliance
}  // namespace starting_positions

namespace interim_waypoints {
  namespace blue_alliance {
    constexpr auto backAwayFromLoadingStationCone =
        frc::Pose2d{{starting_positions::blue_alliance::loadingStationCone.X() + 90_in,
                     starting_positions::blue_alliance::loadingStationCone.Y()},
                    starting_positions::blue_alliance::loadingStationCone.Rotation()};
    constexpr auto backAwayFromCableProtectorCone =
        frc::Pose2d{{starting_positions::blue_alliance::cableProtectorCone.X() + 110_in,
                     starting_positions::blue_alliance::cableProtectorCone.Y() + 12_in},
                    starting_positions::blue_alliance::cableProtectorCone.Rotation()};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto backAwayFromLoadingStationCone =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromLoadingStationCone);
    static const auto backAwayFromCableProtectorCone =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromCableProtectorCone);
  }  // namespace red_alliance
}  // namespace interim_waypoints

namespace game_piece_pickup {
  namespace blue_alliance {
    constexpr auto gamePiece0_3d =
        field_points::blue_alliance::game_pieces::gp_0 -
        frc::Translation3d{measure_up::chassis::length / 2 + measure_up::bumperExtension - 12_in, 0_m, 0_m};
    constexpr auto gamePiece3_3d =
        field_points::blue_alliance::game_pieces::gp_3 -
        frc::Translation3d{measure_up::chassis::length / 2 + measure_up::bumperExtension - 12_in, 0_m, 0_m};
    constexpr auto gamePiece0 = frc::Pose2d{
        {game_piece_pickup::blue_alliance::gamePiece0_3d.X(), game_piece_pickup::blue_alliance::gamePiece0_3d.Y()},
        0_deg};
    constexpr auto gamePiece3 = frc::Pose2d{
        {game_piece_pickup::blue_alliance::gamePiece3_3d.X(), game_piece_pickup::blue_alliance::gamePiece3_3d.Y()},
        0_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto gamePiece0 = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece0);
    static const auto gamePiece3 = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece3);
  }  // namespace red_alliance
}  // namespace game_piece_pickup

namespace path_constraints {
  namespace translation {
    static const auto loadingStationBackOut = frc::TrapezoidProfile<units::inches>::Constraints{4_fps, 8_fps_sq};
    static const auto loadingStationGridToGp0 = frc::TrapezoidProfile<units::inches>::Constraints{3_fps, 8_fps_sq};
    static const auto gp0ToScore = frc::TrapezoidProfile<units::inches>::Constraints{6_fps, 10_fps_sq};
    static const auto loadingStationPullIn = frc::TrapezoidProfile<units::inches>::Constraints{6.5_fps, 10_fps_sq};

    static const auto cableProtectorBackOut = frc::TrapezoidProfile<units::inches>::Constraints{3.5_fps, 6.5_fps_sq};
    static const auto cableProtectorGridToGp3 = frc::TrapezoidProfile<units::inches>::Constraints{4_fps, 6_fps_sq};
    static const auto gp3ToScore = frc::TrapezoidProfile<units::inches>::Constraints{3_fps, 6_fps_sq};
    static const auto cableProtectorPullIn = frc::TrapezoidProfile<units::inches>::Constraints{3_fps, 6_fps_sq};
  }  // namespace translation
  namespace rotation {
    static const auto loadingStationBackOut =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationGridToGp0 =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto gp0ToScore = frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationPullIn =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
  }  // namespace rotation
}  // namespace path_constraints
