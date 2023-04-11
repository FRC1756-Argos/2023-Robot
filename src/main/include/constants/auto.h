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
  /// @brief The max ammount of time the robot is allowed to talk while slamming a cone using the oui oui placer
  constexpr units::millisecond_t robotSlamCone = units::millisecond_t{1500};
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
    constexpr auto cableProtectorCube3d =
        field_points::blue_alliance::outer_grid::topRowMiddle.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension, 0_m, 0_m};
    constexpr auto cableProtectorCube = frc::Pose2d{{place_positions::blue_alliance::cableProtectorCube3d.X(),
                                                     place_positions::blue_alliance::cableProtectorCube3d.Y() + 6_in},
                                                    180_deg};

    constexpr auto cableProtectorShoot3d =
        field_points::blue_alliance::outer_grid::topRowMiddle.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension, 0_m, 0_m};
    constexpr auto cableProtectorShoot =
        frc::Pose2d{{place_positions::blue_alliance::cableProtectorShoot3d.X() + 100_in,
                     place_positions::blue_alliance::cableProtectorShoot3d.Y() + 10_in},
                    180_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto loadingStationCube = utils::ReflectFieldPoint(place_positions::blue_alliance::loadingStationCube);
    static const auto cableProtectorCube = utils::ReflectFieldPoint(place_positions::blue_alliance::cableProtectorCube);
    static const auto cableProtectorShoot =
        utils::ReflectFieldPoint(place_positions::blue_alliance::cableProtectorShoot);
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
                                                    179_deg};
    constexpr auto loadingStationConeReverse3d =
        field_points::blue_alliance::inner_grid::middleRowLeft.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension,
            -measure_up::oui_oui_place::lateralOffset,
            0_m};
    constexpr auto loadingStationConeReverse =
        frc::Pose2d{{starting_positions::blue_alliance::loadingStationConeReverse3d.X(),
                     starting_positions::blue_alliance::loadingStationConeReverse3d.Y()},
                    0_deg};

    constexpr auto cableProtectorConeReverse3d =
        field_points::blue_alliance::outer_grid::middleRowRight.m_position +
        frc::Translation3d{
            field_points::grids::gridDepth + measure_up::chassis::length / 2 + measure_up::bumperExtension,
            measure_up::oui_oui_place::lateralOffset,
            0_m};
    constexpr auto cableProtectorConeReverse =
        frc::Pose2d{{starting_positions::blue_alliance::cableProtectorConeReverse3d.X(),
                     starting_positions::blue_alliance::cableProtectorConeReverse3d.Y()},
                    1_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto loadingStationCone =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::loadingStationCone);
    static const auto cableProtectorCone =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::cableProtectorCone);
    static const auto loadingStationConeReverse =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::loadingStationConeReverse);
    static const auto cableProtectorConeReverse =
        utils::ReflectFieldPoint(starting_positions::blue_alliance::cableProtectorConeReverse);
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
    constexpr auto backAwayFromCableProtectorCone =
        frc::Pose2d{{starting_positions::blue_alliance::cableProtectorCone.X() + 110_in,
                     starting_positions::blue_alliance::cableProtectorCone.Y() + 10_in},
                    starting_positions::blue_alliance::cableProtectorCone.Rotation()};

    constexpr auto backAwayFromLoadingStationConeReverse =
        frc::Pose2d{{place_positions::blue_alliance::loadingStationCube.X() + 90_in,
                     interim_waypoints::blue_alliance::backAwayFromLoadingStationCone.Y()},
                    starting_positions::blue_alliance::loadingStationConeReverse.Rotation()};

    constexpr auto backAwayFromLoadingStationCube =
        frc::Pose2d{{place_positions::blue_alliance::loadingStationCube.X() + 72_in,
                     interim_waypoints::blue_alliance::backAwayFromLoadingStationCone.Y()},
                    place_positions::blue_alliance::loadingStationCube.Rotation()};

    constexpr auto backAwayFromCableProtectorConeReverse =
        frc::Pose2d{{place_positions::blue_alliance::cableProtectorCube.X() + 100_in,
                     place_positions::blue_alliance::cableProtectorCube.Y() + 10_in},
                    starting_positions::blue_alliance::cableProtectorConeReverse.Rotation()};
    constexpr auto approachSecondCube =
        frc::Pose2d{{interim_waypoints::blue_alliance::backAwayFromLoadingStationCube.X(),
                     interim_waypoints::blue_alliance::backAwayFromLoadingStationCube.Y() - 8_in},
                    interim_waypoints::blue_alliance::backAwayFromLoadingStationCube.Rotation()};

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
    static const auto backAwayFromCableProtectorCone =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromCableProtectorCone);
    static const auto backAwayFromLoadingStationCube =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromLoadingStationCube);
    static const auto backAwayFromLoadingStationConeReverse =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromLoadingStationConeReverse);
    static const auto approachSecondCube =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::approachSecondCube);
    static const auto chargingStationStage =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::chargingStationStage);
    static const auto backAwayFromCableProtectorConeReverse =
        utils::ReflectFieldPoint(interim_waypoints::blue_alliance::backAwayFromCableProtectorConeReverse);
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
    constexpr auto gamePiece1_3gp_3d =
        field_points::blue_alliance::game_pieces::gp_1 -
        frc::Translation3d{measure_up::chassis::length / 2 + measure_up::bumperExtension - 12_in, 0_m, 0_m};
    constexpr auto gamePiece1_3gp = frc::Pose2d{{game_piece_pickup::blue_alliance::gamePiece1_3gp_3d.X(),
                                                 game_piece_pickup::blue_alliance::gamePiece1_3gp_3d.Y() + 6_in},
                                                -90_deg};
    constexpr auto gamePiece3 = frc::Pose2d{
        {game_piece_pickup::blue_alliance::gamePiece3_3d.X(), game_piece_pickup::blue_alliance::gamePiece3_3d.Y()},
        1_deg};
    constexpr auto gamePiece2_3d =
        field_points::blue_alliance::game_pieces::gp_2 -
        frc::Translation3d{measure_up::chassis::length / 2 + measure_up::bumperExtension - 12_in, 0_m, 0_m};
    constexpr auto gamePiece2 = frc::Pose2d{{game_piece_pickup::blue_alliance::gamePiece2_3d.X(),
                                             game_piece_pickup::blue_alliance::gamePiece2_3d.Y() - 6_in},
                                            89_deg};
  }  // namespace blue_alliance
  namespace red_alliance {
    static const auto gamePiece0 = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece0);
    static const auto gamePiece1_3gp = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece1_3gp);
    static const auto gamePiece2 = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece2);
    static const auto gamePiece3 = utils::ReflectFieldPoint(game_piece_pickup::blue_alliance::gamePiece3);
  }  // namespace red_alliance
}  // namespace game_piece_pickup

namespace path_constraints {
  namespace translation {
    static const auto loadingStationBackOut = frc::TrapezoidProfile<units::inches>::Constraints{4_fps, 8_fps_sq};
    static const auto loadingStationReverseBackOut = frc::TrapezoidProfile<units::inches>::Constraints{8_fps, 8_fps_sq};
    static const auto stageChargeStationPullIn = frc::TrapezoidProfile<units::inches>::Constraints{4_fps, 8_fps_sq};
    static const auto loadingStationGridToGp0 = frc::TrapezoidProfile<units::inches>::Constraints{3_fps, 8_fps_sq};
    static const auto gp0ToScore = frc::TrapezoidProfile<units::inches>::Constraints{6_fps, 10_fps_sq};
    static const auto loadingStationPullIn = frc::TrapezoidProfile<units::inches>::Constraints{6.5_fps, 10_fps_sq};

    static const auto cableProtectorBackOut = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 7_fps_sq};
    static const auto cableProtectorBackOut2Gp = frc::TrapezoidProfile<units::inches>::Constraints{5_fps, 5_fps_sq};
    static const auto cableProtectorGridToGp3 = frc::TrapezoidProfile<units::inches>::Constraints{5_fps, 6_fps_sq};
    static const auto gp3ToScore = frc::TrapezoidProfile<units::inches>::Constraints{4.5_fps, 5_fps_sq};
    static const auto gp2ToScore = frc::TrapezoidProfile<units::inches>::Constraints{6_fps, 5_fps_sq};
    static const auto cableProtectorPullIn = frc::TrapezoidProfile<units::inches>::Constraints{5_fps, 5_fps_sq};

    static const auto loadingStationBackOut_3gp = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 8_fps_sq};
    static const auto stageChargeStationPullIn_3gp = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 8_fps_sq};
    static const auto loadingStationGridToGp0_3gp = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 8_fps_sq};
    static const auto gp0ToScore_3gp = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 8_fps_sq};
    static const auto loadingStationPullIn_3gp = frc::TrapezoidProfile<units::inches>::Constraints{7_fps, 8_fps_sq};
  }  // namespace translation
  namespace rotation {
    static const auto loadingStationBackOut =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationReverseBackOut =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto stageChargeStationPullIn =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationGridToGp0 =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto gp0ToScore = frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};
    static const auto loadingStationPullIn =
        frc::TrapezoidProfile<units::degrees>::Constraints{360_deg_per_s, 360_deg_per_s_sq};

    static const auto cableProtectorBackOut =
        frc::TrapezoidProfile<units::degrees>::Constraints{300_deg_per_s, 300_deg_per_s_sq};
    static const auto cableProtectorBackOut2Gp =
        frc::TrapezoidProfile<units::degrees>::Constraints{300_deg_per_s, 300_deg_per_s_sq};
    static const auto cableProtectorGridToGp3 =
        frc::TrapezoidProfile<units::degrees>::Constraints{300_deg_per_s, 300_deg_per_s_sq};
    static const auto cableProtectorPullIn =
        frc::TrapezoidProfile<units::degrees>::Constraints{300_deg_per_s, 300_deg_per_s_sq};
    static const auto gp3ToScore = frc::TrapezoidProfile<units::degrees>::Constraints{300_deg_per_s, 300_deg_per_s_sq};
    static const auto gp2ToScore = frc::TrapezoidProfile<units::degrees>::Constraints{300_deg_per_s, 300_deg_per_s_sq};

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
