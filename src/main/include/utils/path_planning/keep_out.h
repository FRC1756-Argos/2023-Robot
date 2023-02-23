/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/length.h>

#include <optional>
#include <vector>

#include "types.h"

namespace path_planning {

  enum class Orientation { Colinear, Clockwise, Counterclockwise };

  Orientation GetOrientation(const ArmPathPoint& point1, const ArmPathPoint& point2, const ArmPathPoint& point3);

  bool OnSegment(const ArmPathPoint& p, const ArmPathPoint& q, const ArmPathPoint& r);

  bool SegmentsIntersect(const LineSegment& segment1, const LineSegment& segment2);

  std::optional<ArmPathPoint> Intersection(const LineSegment& segment1, const LineSegment& segment2);

  std::vector<LineSegment> PolygonSegments(const Polygon& polygon);

  units::inch_t SegmentLength(const LineSegment& segment);

  ArmPath KeepOut(LineSegment candidatePath, const Polygon& avoidancePolygon);
}  // namespace path_planning
