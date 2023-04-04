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
                                                     place_positions::blue_alliance::loadingStationCube3d.Y() - 6_in},
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
    constexpr auto loadingStationConeReverse3d =
        field_points::blue_alliance::inner_grid::middleRowLeft.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension, 0_m, 0_m};
    constexpr auto loadingStationConeReverse =
        frc::Pose2d{{starting_positions::blue_alliance::loadingStationConeReverse3d.X(),
                     starting_positions::blue_alliance::loadingStationConeReverse3d.Y()},
                    0_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto loadingStationCone =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::loadingStationCone);
    static const auto loadingStationConeReverse =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::loadingStationConeReverse);
  }  // namespace red_alliance
}  // namespace starting_positions

namespace interim_waypoints {
  namespace blue_alliance {

    namespace fudges {
      constexpr auto stationStageFudge = 10_in;
    }  // namespace fudges

    constexpr auto backAwayFromLoadingStationCone =
        frc::Pose2d{{starting_positions::blue_alliance::loadingStationCone.X() + 90_in,
                     starting_positions::blue_alliance::loadingStationCone.Y() - 6_in},
                    starting_positions::blue_alliance::loadingStationCone.Rotation()};

    constexpr auto backAwayFromLoadingStationConeReverse =
        frc::Pose2d{{place_positions::blue_alliance::loadingStationCube.X() + 90_in,
                     interim_waypoints::blue_alliance::backAwayFromLoadingStationCone.Y()},
                    starting_positions::blue_alliance::loadingStationConeReverse.Rotation()};

    constexpr auto backAwayFromLoadingStationCube =
        frc::Pose2d{{place_positions::blue_alliance::loadingStationCube.X() + 120_in,
                     interim_waypoints::blue_alliance::backAwayFromLoadingStationCone.Y()},
                    place_positions::blue_alliance::loadingStationCube.Rotation()};

    // Center of robot when staging for dock, outside of the community
    constexpr auto chargingStationStage =
        frc::Pose2d{{field_points::charge_station::outerEdgeX + measure_up::chassis::length / 2 +
                         measure_up::bumperExtension + fudges::stationStageFudge,
                     field_points::blue_alliance::charge_station::chargeStationCenter.Y()},
                    180_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto backAwayFromLoadingStationCone =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromLoadingStationCone);
    static const auto backAwayFromLoadingStationCube =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromLoadingStationCube);
    static const auto backAwayFromLoadingStationConeReverse =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromLoadingStationConeReverse);
    static const auto chargingStationStage =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::chargingStationStage);
  }  // namespace red_alliance
}  // namespace interim_waypoints

namespace game_piece_pickup {
  namespace blue_alliance {
    constexpr auto gamePiece0_3d =
        field_points::blue_alliance::game_pieces::gp_0 -
        frc::Translation3d{measure_up::chassis::length / 2 + measure_up::bumperExtension - 12_in, 0_m, 0_m};
    constexpr auto gamePiece0 = frc::Pose2d{
        {game_piece_pickup::blue_alliance::gamePiece0_3d.X(), game_piece_pickup::blue_alliance::gamePiece0_3d.Y()},
        0_deg};
    constexpr auto gamePiece1_3d =
        field_points::blue_alliance::game_pieces::gp_1 -
        frc::Translation3d{measure_up::chassis::length / 2 + measure_up::bumperExtension - 12_in, 0_m, 0_m};
    constexpr auto gamePiece1 = frc::Pose2d{{game_piece_pickup::blue_alliance::gamePiece1_3d.X(),
                                             game_piece_pickup::blue_alliance::gamePiece1_3d.Y() + 6_in},
                                            -20_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto gamePiece0 = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece0);
    static const auto gamePiece1 = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece1);
  }  // namespace red_alliance
}  // namespace game_piece_pickup

namespace path_constraints {
  namespace translation {
    static const auto loadingStationBackOut = frc::TrapezoidProfile<units::inches>::Constraints{4_fps, 8_fps_sq};
    static const auto stageChargeStationPullIn = frc::TrapezoidProfile<units::inches>::Constraints{4_fps, 8_fps_sq};
    static const auto loadingStationGridToGp0 = frc::TrapezoidProfile<units::inches>::Constraints{3_fps, 8_fps_sq};
    static const auto gp0ToScore = frc::TrapezoidProfile<units::inches>::Constraints{6_fps, 10_fps_sq};
    static const auto loadingStationPullIn = frc::TrapezoidProfile<units::inches>::Constraints{6.5_fps, 10_fps_sq};

    static const auto loadingStationBackOut_3gp = frc::TrapezoidProfile<units::inches>::Constraints{6_fps, 8_fps_sq};
    static const auto stageChargeStationPullIn_3gp = frc::TrapezoidProfile<units::inches>::Constraints{6_fps, 8_fps_sq};
    static const auto loadingStationGridToGp0_3gp = frc::TrapezoidProfile<units::inches>::Constraints{5_fps, 8_fps_sq};
    static const auto gp0ToScore_3gp = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 8_fps_sq};
    static const auto loadingStationPullIn_3gp = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 8_fps_sq};
  }  // namespace translation
  namespace rotation {
    static const auto loadingStationBackOut =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto stageChargeStationPullIn =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationGridToGp0 =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto gp0ToScore = frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationPullIn =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};

    static const auto loadingStationBackOut_3gp =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto stageChargeStationPullIn_3gp =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationGridToGp0_3gp =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto gp0ToScore_3gp =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationPullIn_3gp =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
  }  // namespace rotation
}  // namespace path_constraints
