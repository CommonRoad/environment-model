#pragma once

#include <gtest/gtest.h>

#include "commonroad_cpp/geometry/circle.h"
#include "commonroad_cpp/geometry/polygon.h"
#include "commonroad_cpp/geometry/rectangle.h"
#include "commonroad_cpp/geometry/shape_group.h"

class ShapeGroupTest : public testing::Test {
  protected:
    Polygon polygonOne{
        polygon_type{{point_type{0, 0}, point_type{1, 0}, point_type{1, 1}, point_type{0, 1}, point_type{0, 0}}}};
    Polygon polygonTwo{{vertex{0, 0}, vertex{1, 0}, vertex{1, 1}, vertex{0, 1}}};
    Circle circleOne;
    Rectangle rectangleOne;
    ShapeGroup shapeGroupOne;
    ShapeGroup shapeGroupTwo;

  private:
    void SetUp() override;
};
