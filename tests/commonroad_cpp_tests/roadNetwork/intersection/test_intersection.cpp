#include "test_intersection.h"

void IntersectionTest::SetUp() { setUpIncoming(); }

TEST_F(IntersectionTest, InitializationComplete) {
    EXPECT_EQ(intersection1->getId(), 1000);
    EXPECT_EQ(intersection2->getId(), 1001);

    EXPECT_EQ(intersection1->getIncomingGroups().size(), 3);
    EXPECT_EQ(intersection1->getIncomingGroups().at(0)->getId(), 13);
    EXPECT_EQ(intersection2->getIncomingGroups().size(), 1);
    EXPECT_EQ(intersection2->getIncomingGroups().at(0)->getId(), 16);
}

TEST_F(IntersectionTest, ComputeMemberLanelets) {
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).size(), 15);
    EXPECT_EQ(intersection2->getMemberLanelets(roadNetwork).size(), 5);

    auto laneletTypeIncoming{std::set<LaneletType>{LaneletType::urban, LaneletType::incoming}};
    auto laneletTypeStraight{
        std::set<LaneletType>{LaneletType::urban, LaneletType::intersection, LaneletType::straight}};
    auto laneletTypeLeft{std::set<LaneletType>{LaneletType::urban, LaneletType::intersection, LaneletType::left}};
    auto laneletTypeRight{std::set<LaneletType>{LaneletType::urban, LaneletType::intersection, LaneletType::right}};

    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(1)->getLaneletTypes(), laneletTypeStraight);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(5)->getLaneletTypes(), laneletTypeIncoming);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(6)->getLaneletTypes(), laneletTypeLeft);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(13)->getLaneletTypes(), laneletTypeRight);

    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(1)->getId(), 80);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(5)->getId(), 3);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(6)->getId(), 100);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(13)->getId(), 90);
}
