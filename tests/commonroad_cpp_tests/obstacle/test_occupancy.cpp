#include "test_occupancy.h"

void OccupancyTest::SetUp() {
    polygonOne = Polygon({{point_type{0, 0}, point_type{1, 0}, point_type{1, 1}, point_type{0, 1}, point_type{0, 0}}});
    circleOne = Circle();
    shapeGroupOne = ShapeGroup({std::make_shared<Polygon>(polygonOne), std::make_shared<Circle>(circleOne)});
    occ1 = Occupancy(0, std::make_shared<Rectangle>(2.0, 1.0));
    occ2 = Occupancy(1, std::make_shared<ShapeGroup>(shapeGroupOne));
}

TEST_F(OccupancyTest, InitializationComplete) {
    EXPECT_EQ(occ1.getTimeStep(), 0);
    EXPECT_EQ(occ2.getTimeStep(), 1);

    EXPECT_EQ(occ1.getShape()->getType(), ShapeType::rectangle);
    EXPECT_EQ(occ2.getShape()->getType(), ShapeType::shapeGroup);

    EXPECT_EQ(occ1.getShape()->getLength(), 2);
    auto sg{std::dynamic_pointer_cast<ShapeGroup>(occ2.getShape())};
    EXPECT_EQ(sg->getShapes().size(), 2);
}
