#include "precision_land_executor/tf_site_resolver.hpp"

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.hpp>

#include <cmath>
#include <limits>
#include <memory>

using namespace precision_land_executor;

namespace {

geometry_msgs::msg::TransformStamped staticTransform(
    const char* parent, const char* child, double x, double y, double z,
    double qx, double qy, double qz, double qw) {
  geometry_msgs::msg::TransformStamped result{};
  result.header.frame_id = parent;
  result.child_frame_id = child;
  result.transform.translation.x = x;
  result.transform.translation.y = y;
  result.transform.translation.z = z;
  result.transform.rotation.x = qx;
  result.transform.rotation.y = qy;
  result.transform.rotation.z = qz;
  result.transform.rotation.w = qw;
  return result;
}

Site siteB() {
  return Site{.id = "secondary_position",
              .center = {.x = 5.0, .y = 1.0, .z = 0.0},
              .clearance_radius_m = 0.75,
              .verified = true,
              .occupied = false};
}

}  // namespace

TEST(TfLandingSite, UsesTheExistingMapToDroneBNedChain) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  const double half = std::sqrt(0.5);
  ASSERT_TRUE(buffer.setTransform(staticTransform(
      "map", "px4_1/odom", 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), "test", true));
  ASSERT_TRUE(buffer.setTransform(staticTransform(
      "px4_1/odom", "px4_1/odom_ned", 0.0, 0.0, 0.0,
      half, half, 0.0, 0.0), "test", true));

  const auto result = resolveSiteWithTf(buffer, 1, siteB(), "map", "px4_1/odom_ned");
  ASSERT_TRUE(result);
  EXPECT_EQ(result->mission_id, 1u);
  EXPECT_EQ(result->source_frame, "map");
  EXPECT_EQ(result->target_frame, "px4_1/odom_ned");
  // map=(5 east, 1 north, 0 up); B's map origin=(3,0,0).
  // Relative ENU=(2,1,0) becomes local NED=(1,2,0).
  EXPECT_NEAR(result->local_position.x, 1.0, 1e-9);
  EXPECT_NEAR(result->local_position.y, 2.0, 1e-9);
  EXPECT_NEAR(result->local_position.z, 0.0, 1e-9);
}

TEST(TfLandingSite, DisconnectedOrUnknownFrameIsRejected) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  EXPECT_FALSE(resolveSiteWithTf(buffer, 1, siteB(), "map", "px4_1/odom_ned"));
  EXPECT_FALSE(resolveSiteWithTf(buffer, 1, siteB(), "gazebo_world_enu", "px4_1/odom_ned"));
}

TEST(TfLandingSite, RejectsInvalidInputBeforeTfLookup) {
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  auto candidate = siteB();
  candidate.center.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(resolveSiteWithTf(buffer, 1, candidate, "map", "px4_1/odom_ned"));
  EXPECT_FALSE(resolveSiteWithTf(buffer, 0, siteB(), "map", "px4_1/odom_ned"));
}
