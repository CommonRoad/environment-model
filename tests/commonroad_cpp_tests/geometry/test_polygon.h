#pragma once

#include <gtest/gtest.h>

#include "commonroad_cpp/geometry/polygon.h"

class PolygonTest : public testing::Test {
  protected:
    polygon_type polygonBoost =
        polygon_type{{point_type{0, 0}, point_type{1, 0}, point_type{1, 1}, point_type{0, 1}, point_type{0, 0}}};
    std::vector<vertex> polygonVertices = {vertex{0, 0}, vertex{1, 0}, vertex{1, 1}, vertex{0, 1}};
    Polygon polygonOne{polygonBoost};
    Polygon polygonTwo{polygonVertices};

  private:
    void SetUp() override;
};
