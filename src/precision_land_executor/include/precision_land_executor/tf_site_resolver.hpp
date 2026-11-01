#pragma once

#include "precision_land_executor/shared_landing_site.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>

#include <optional>
#include <string>

namespace precision_land_executor {

// Uses TF2's latest available frame mapping without blocking the ROS callback.
// A missing or disconnected TF chain produces nullopt and therefore SAFE_HOLD.
// Current demo uses static map->odom and odom->odom_ned transforms; a future
// flight-control integration must additionally enforce dynamic-TF freshness.
inline std::optional<ResolvedSite> resolveSiteWithTf(
    tf2_ros::Buffer& buffer, std::uint64_t mission_id, const Site& candidate,
    const std::string& source_frame, const std::string& target_frame) {
  if (mission_id == 0 || candidate.id.empty() || !finite(candidate.center) ||
      source_frame.empty() || target_frame.empty()) {
    return std::nullopt;
  }

  try {
    const auto transform = buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
    if (transform.header.frame_id != target_frame ||
        transform.child_frame_id != source_frame) {
      return std::nullopt;
    }

    geometry_msgs::msg::PointStamped source{};
    source.header.frame_id = source_frame;
    source.point.x = candidate.center.x;
    source.point.y = candidate.center.y;
    source.point.z = candidate.center.z;
    geometry_msgs::msg::PointStamped target{};
    tf2::doTransform(source, target, transform);
    const Point3 local{.x = target.point.x,
                       .y = target.point.y,
                       .z = target.point.z};
    if (target.header.frame_id != target_frame || !finite(local)) {
      return std::nullopt;
    }
    return ResolvedSite{.mission_id = mission_id,
                        .site_id = candidate.id,
                        .source_frame = source_frame,
                        .target_frame = target_frame,
                        .local_position = local};
  } catch (const tf2::TransformException&) {
    return std::nullopt;
  }
}

}  // namespace precision_land_executor
