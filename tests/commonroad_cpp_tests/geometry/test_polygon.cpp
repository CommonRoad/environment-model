#include "test_polygon.h"

void PolygonTest::SetUp() {
    polygonOne = Polygon(polygonBoost);
    polygonTwo = Polygon(polygonVertices);
}

TEST_F(PolygonTest, Initialization) {
    EXPECT_EQ(polygonOne.getType(), ShapeType::polygon);
    for (size_t idx{0}; idx < polygonOne.getPolygon().outer().size(); ++idx) {
        EXPECT_EQ(polygonOne.getPolygon().outer()[idx].x(), polygonBoost.outer()[idx].x());
        EXPECT_EQ(polygonOne.getPolygon().outer()[idx].y(), polygonBoost.outer()[idx].y());
    }

    for (const auto &vertex : polygonOne.getPolygon().outer())
        std::cout << vertex.x() << " " << vertex.y() << std::endl;
}

TEST_F(PolygonTest, PrintParameters) { EXPECT_NO_THROW(polygonOne.printParameters()); }
