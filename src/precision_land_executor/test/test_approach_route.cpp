#include "precision_land_executor/approach_route.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <vector>

TEST(ApproachRoute, RequiresFiniteCompleteTriples)
{
  using precision_land_executor::validApproachRoute;
  EXPECT_FALSE(validApproachRoute({}));
  EXPECT_FALSE(validApproachRoute({1.0, 2.0}));
  EXPECT_FALSE(validApproachRoute({1.0, 2.0, 3.0, 4.0}));
  EXPECT_FALSE(validApproachRoute({0.0, 0.0, -5.0}));
  EXPECT_FALSE(validApproachRoute({0.0, 0.0, 1.0}));
  EXPECT_FALSE(validApproachRoute({0.0, 0.0, std::numeric_limits<double>::infinity()}));
  EXPECT_TRUE(validApproachRoute({0.0, 0.0, 5.0, 8.0, -1.0, 5.0}));
  EXPECT_TRUE(validApproachRoute({3.0, 0.0, 8.0, 8.0, 2.0, 8.0}));
}
