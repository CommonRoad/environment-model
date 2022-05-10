//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_intersection.h"

void IntersectionTestInitialization::setUpIntersection() {
    size_t lanelet20Id{20};
    std::vector<vertex> lanelet20LeftVertices{{1020, -1002}, {1020, 1012}};
    std::vector<vertex> lanelet20RightVertices{{1025, -1002}, {1025, 1012}};
    std::set<LaneletType> lanelet20LaneletTypes{LaneletType::crosswalk};
    auto lanelet20{
        std::make_shared<Lanelet>(lanelet20Id, lanelet20LeftVertices, lanelet20RightVertices, lanelet20LaneletTypes)};

    size_t intersection1Id{1000};
    std::vector<std::shared_ptr<Incoming>> incomingsIntersection1{incomingOne, incomingTwo, incomingThree};
    std::vector<std::shared_ptr<Lanelet>> crossingsIntersection1{lanelet20};
    intersection1 = std::make_shared<Intersection>(intersection1Id, incomingsIntersection1, crossingsIntersection1);

    size_t intersection2Id{1001};
    std::vector<std::shared_ptr<Incoming>> incomingsIntersection2{incomingTwo};
    std::vector<std::shared_ptr<Lanelet>> crossingsIntersection2{lanelet20};
    intersection2 = std::make_shared<Intersection>();
    intersection2->setId(intersection2Id);
    intersection2->setCrossings(crossingsIntersection2);
    intersection2->setIncomings(incomingsIntersection2);
    roadNetwork = std::make_shared<RoadNetwork>(
        RoadNetwork(lanelets, SupportedTrafficSignCountry::GERMANY, {}, {}, {intersection1, intersection2}));
}

void IntersectionTest::SetUp() {
    setUpIncoming();
    setUpIntersection();
}

TEST_F(IntersectionTest, InitializationComplete) {
    EXPECT_EQ(intersection1->getId(), 1000);
    EXPECT_EQ(intersection2->getId(), 1001);

    EXPECT_EQ(intersection1->getCrossings().size(), 1);
    EXPECT_EQ(intersection1->getCrossings().at(0)->getId(), 20);
    EXPECT_EQ(intersection2->getCrossings().size(), 1);
    EXPECT_EQ(intersection2->getCrossings().at(0)->getId(), 20);

    EXPECT_EQ(intersection1->getIncomings().size(), 3);
    EXPECT_EQ(intersection1->getIncomings().at(0)->getId(), 13);
    EXPECT_EQ(intersection2->getIncomings().size(), 1);
    EXPECT_EQ(intersection2->getIncomings().at(0)->getId(), 14);
}

TEST_F(IntersectionTest, ComputeMemberLanelets) {
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).size(), 15);
    EXPECT_EQ(intersection2->getMemberLanelets(roadNetwork).size(), 5);

    auto laneletTypeIncoming{std::set<LaneletType>{LaneletType::incoming}};
    auto laneletTypeStraight{std::set<LaneletType>{LaneletType::intersection, LaneletType::intersectionStraight}};
    auto laneletTypeLeft{std::set<LaneletType>{LaneletType::intersection, LaneletType::intersectionLeftTurn}};
    auto laneletTypeRight{std::set<LaneletType>{LaneletType::intersection, LaneletType::intersectionRightTurn}};

    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(1)->getLaneletTypes(), laneletTypeStraight);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(5)->getLaneletTypes(), laneletTypeIncoming);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(6)->getLaneletTypes(), laneletTypeLeft);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(13)->getLaneletTypes(), laneletTypeRight);

    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(1)->getId(), 80);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(5)->getId(), 3);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(6)->getId(), 100);
    EXPECT_EQ(intersection1->getMemberLanelets(roadNetwork).at(13)->getId(), 90);
}