#include "test_shape_group.h"

void ShapeGroupTest::SetUp() {
    polygonOne = Polygon({{point_type{0, 0}, point_type{1, 0}, point_type{1, 1}, point_type{0, 1}, point_type{0, 0}}});
    polygonTwo = Polygon({{point_type{0, 0}, point_type{2, 0}, point_type{2, 1}, point_type{0, 2}}});
    circleOne = Circle();
    rectangleOne = Rectangle();
    shapeGroupOne = ShapeGroup({std::make_shared<Polygon>(polygonOne), std::make_shared<Circle>(circleOne)});
    shapeGroupTwo = ShapeGroup({std::make_shared<Polygon>(polygonTwo), std::make_shared<Rectangle>(rectangleOne),
                                std::make_shared<Circle>(circleOne)});
}

TEST_F(ShapeGroupTest, Initialization) {
    EXPECT_EQ(shapeGroupOne.getType(), ShapeType::shapeGroup);
    EXPECT_EQ(shapeGroupOne.getShapes().size(), 2);
    EXPECT_EQ(shapeGroupTwo.getShapes().size(), 3);
    EXPECT_EQ(shapeGroupOne.getShapes().at(1)->getType(), ShapeType::circle);
}

TEST_F(ShapeGroupTest, PrintParameters) { EXPECT_NO_THROW(polygonOne.printParameters()); }
