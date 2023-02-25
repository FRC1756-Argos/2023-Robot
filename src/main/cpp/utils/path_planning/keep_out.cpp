/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/path_planning/keep_out.h"

#include <units/length.h>
#include <units/math.h>

#include <algorithm>
#include <utility>
#include <vector>

using namespace path_planning;

// Segment intersection math from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

Orientation path_planning::GetOrientation(const ArmPathPoint& point1,
                                          const ArmPathPoint& point2,
                                          const ArmPathPoint& point3) {
  auto val = (point2.z - point1.z) * (point3.x - point2.x) - (point2.x - point1.x) * (point3.z - point2.z);

  if (val.to<double>() == 0)
    return Orientation::Colinear;

  return (val.to<double>() > 0) ? Orientation::Clockwise : Orientation::Counterclockwise;
}

bool path_planning::OnSegment(const ArmPathPoint& p, const ArmPathPoint& q, const ArmPathPoint& r) {
  return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) && q.z <= std::max(p.z, r.z) &&
          q.z >= std::min(p.z, r.z));
}

bool path_planning::SegmentsIntersect(const LineSegment& segment1, const LineSegment& segment2) {
  // Find the four orientations needed for general and
  // special cases
  auto o1 = GetOrientation(segment1.start, segment1.end, segment2.start);
  auto o2 = GetOrientation(segment1.start, segment1.end, segment2.end);
  auto o3 = GetOrientation(segment2.start, segment2.end, segment1.start);
  auto o4 = GetOrientation(segment2.start, segment2.end, segment1.end);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases
  // segment1.start, segment1.end and segment2.start are collinear and segment2.start lies on segment (segment1.start, segment1.end)
  if (o1 == Orientation::Colinear && OnSegment(segment1.start, segment2.start, segment1.end))
    return true;

  // segment1.start, segment1.end and segment2.end are collinear and segment2.end lies on segment (segment1.start, segment1.end)
  if (o2 == Orientation::Colinear && OnSegment(segment1.start, segment2.end, segment1.end))
    return true;

  // segment2.start, segment2.end and segment1.start are collinear and segment1.start lies on segment (segment2.start, segment2.end)
  if (o3 == Orientation::Colinear && OnSegment(segment2.start, segment1.start, segment2.end))
    return true;

  // segment2.start, segment2.end and segment1.end are collinear and segment1.end lies on segment (segment2.start, segment2.end)
  if (o4 == Orientation::Colinear && OnSegment(segment2.start, segment1.end, segment2.end))
    return true;

  return false;  // Doesn't fall in any of the above cases
}

// Source: https://stackoverflow.com/a/58605767
std::optional<ArmPathPoint> path_planning::Intersection(const LineSegment& segment1, const LineSegment& segment2) {
  if (SegmentsIntersect(segment1, segment2)) {
    return ArmPathPoint(
        ((segment1.start.x * segment1.end.z - segment1.start.z * segment1.end.x) * (segment2.start.x - segment2.end.x) -
         (segment2.start.x * segment2.end.z - segment2.start.z * segment2.end.x) *
             (segment1.start.x - segment1.end.x)) /
            ((segment1.start.x - segment1.end.x) * (segment2.start.z - segment2.end.z) -
             (segment1.start.z - segment1.end.z) * (segment2.start.x - segment2.end.x)),
        ((segment1.start.x * segment1.end.z - segment1.start.z * segment1.end.x) * (segment2.start.z - segment2.end.z) -
         (segment2.start.x * segment2.end.z - segment2.start.z * segment2.end.x) *
             (segment1.start.z - segment1.end.z)) /
            ((segment1.start.x - segment1.end.x) * (segment2.start.z - segment2.end.z) -
             (segment1.start.z - segment1.end.z) * (segment2.start.x - segment2.end.x)));
  }
  return std::nullopt;
}

std::vector<LineSegment> path_planning::PolygonSegments(const Polygon& polygon) {
  if (polygon.size() < 2) {
    return {};
  }
  if (polygon.size() == 2) {
    LineSegment newSegment{.start = polygon.at(0), .end = polygon.at(1)};
    return {newSegment};
  }
  std::vector<LineSegment> retVal;
  retVal.reserve(polygon.size());
  for (size_t i = 0; i < polygon.size(); ++i) {
    LineSegment newSegment;
    newSegment.start = polygon.at(i);
    newSegment.end = i == (polygon.size() - 1) ? polygon.at(0) : polygon.at(i + 1);
    retVal.emplace_back(newSegment);
  }
  return retVal;
}

units::inch_t path_planning::SegmentLength(const LineSegment& segment) {
  return units::math::hypot(segment.end.x - segment.start.x, segment.end.z - segment.start.z);
}

ArmPath path_planning::KeepOut(const ArmPath& initialPath, const Polygon& avoidancePolygon) {
  if (initialPath.size() < 2) {
    return initialPath;
  }

  auto polygonSegments = PolygonSegments(avoidancePolygon);

  std::vector<PathIntersection> segmentsWithIntersections;

  auto cumulativePathLength = 0_in;

  for (size_t pathIndex = 0; pathIndex < initialPath.size() - 1; ++pathIndex) {
    for (size_t i = 0; i < polygonSegments.size(); ++i) {
      auto intersectionPoint =
          Intersection({initialPath.at(pathIndex), initialPath.at(pathIndex + 1)}, polygonSegments.at(i));
      if (intersectionPoint) {
        segmentsWithIntersections.emplace_back(
            pathIndex, i, cumulativePathLength + SegmentLength({initialPath.at(pathIndex), intersectionPoint.value()}));
      }
    }
    cumulativePathLength += SegmentLength({initialPath.at(pathIndex), initialPath.at(pathIndex + 1)});
  }

  // If only one segment intersects, ignore since it's probably just hitting the endpoint
  if (segmentsWithIntersections.size() > 1) {
    // Order intersections by position along candidate path.  This should prevent back tracking output path
    std::sort(segmentsWithIntersections.begin(),
              segmentsWithIntersections.end(),
              [](const PathIntersection& a, const PathIntersection& b) { return a.path1Distance < b.path1Distance; });

    ArmPath safePath;
    safePath.reserve(initialPath.size() + segmentsWithIntersections.size());
    safePath.push_back(initialPath.front());

    size_t initialPathIndexCheckpoint = 0;

    // Analyze pairs of intersections because these are where we need to follow the avoidance polygon
    for (auto intersectionIt = segmentsWithIntersections.begin();
         intersectionIt != std::prev(segmentsWithIntersections.end()) &&
         intersectionIt != segmentsWithIntersections.end();
         std::advance(intersectionIt, 2)) {
      if (intersectionIt->path1SegmentIndex > initialPathIndexCheckpoint) {
        // Insert initial path points that aren't involved in intersection regions
        safePath.insert(safePath.end(),
                        std::next(initialPath.begin(), initialPathIndexCheckpoint + 1),
                        std::next(initialPath.begin(), intersectionIt->path1SegmentIndex + 1));
      }
      // This handles removing points from initial path if needed
      initialPathIndexCheckpoint = std::next(intersectionIt)->path1SegmentIndex;

      if (intersectionIt->path2SegmentIndex < std::next(intersectionIt)->path2SegmentIndex &&
          std::next(intersectionIt)->path2SegmentIndex - intersectionIt->path2SegmentIndex <=
              (avoidancePolygon.size() - std::next(intersectionIt)->path2SegmentIndex +
               intersectionIt->path2SegmentIndex)) {
        // Shortest path is forward order from first index to second index
        for (size_t i = intersectionIt->path2SegmentIndex; i < std::next(intersectionIt)->path2SegmentIndex; ++i) {
          safePath.push_back(polygonSegments.at(i).end);
        }
      } else if (intersectionIt->path2SegmentIndex - std::next(intersectionIt)->path2SegmentIndex <
                 std::next(intersectionIt)->path2SegmentIndex + avoidancePolygon.size() -
                     intersectionIt->path2SegmentIndex) {
        // Shortest path is reverse order from first index to second index
        for (size_t i = intersectionIt->path2SegmentIndex; i > std::next(intersectionIt)->path2SegmentIndex; --i) {
          safePath.push_back(polygonSegments.at(i).start);
        }
      } else if (intersectionIt->path2SegmentIndex < std::next(intersectionIt)->path2SegmentIndex &&
                 std::next(intersectionIt)->path2SegmentIndex - intersectionIt->path2SegmentIndex >
                     (avoidancePolygon.size() - std::next(intersectionIt)->path2SegmentIndex +
                      intersectionIt->path2SegmentIndex)) {
        // Shortest path is reverse order from first index to second index with wrap around 0
        size_t i = intersectionIt->path2SegmentIndex;
        while (i != std::next(intersectionIt)->path2SegmentIndex) {
          safePath.push_back(polygonSegments.at(i).start);

          i == 0 ? i = polygonSegments.size() - 1 : --i;
        }
      } else {
        // Shortest path is forward order from first index to second index with wrap around 0
        size_t i = intersectionIt->path2SegmentIndex;
        while (i != std::next(intersectionIt)->path2SegmentIndex) {
          safePath.push_back(polygonSegments.at(i).end);

          ++i;
          if (i >= polygonSegments.size()) {
            // wrap around
            i = 0;
          }
        }
      }
    }
    safePath.insert(safePath.end(), std::next(initialPath.begin(), initialPathIndexCheckpoint + 1), initialPath.end());
    return safePath;
  }
  return initialPath;
}
