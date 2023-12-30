//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_incoming.h"

void IncomingTestInitialization::setUpIncoming() {
    // left to right: 2->80-81->4
    // right to left: 3->70-71->1
    // bottom to right: 6->90->91->4
    // right to bottom: 3->100->101->5
    // bottom to left: 6->120->121->1
    // left to bottom: 2->110->111->5
    size_t lanelet1Id{1};
    std::vector<vertex> lanelet1LeftVertices{{1000, 1005}, {900, 1005}};
    std::vector<vertex> lanelet1RightVertices{{1000, 1010}, {900, 1010}};
    std::set<LaneletType> lanelet1LaneletTypes{LaneletType::unknown};
    auto lanelet1{
        std::make_shared<Lanelet>(lanelet1Id, lanelet1LeftVertices, lanelet1RightVertices, lanelet1LaneletTypes)};
    size_t lanelet2Id{2};
    std::vector<vertex> lanelet2LeftVertices{{900, 1005}, {1000, 1005}};
    std::vector<vertex> lanelet2RightVertices{{900, 1000}, {1000, 1000}};
    std::set<LaneletType> lanelet2LaneletTypes{LaneletType::incoming};
    auto lanelet2{
        std::make_shared<Lanelet>(lanelet2Id, lanelet2LeftVertices, lanelet2RightVertices, lanelet2LaneletTypes)};
    size_t lanelet3Id{3};
    std::vector<vertex> lanelet3LeftVertices{{1200, 1005}, {1015, 1005}};
    std::vector<vertex> lanelet3RightVertices{{1200, 1010}, {1015, 1010}};
    std::set<LaneletType> lanelet3LaneletTypes{LaneletType::incoming};
    auto lanelet3{
        std::make_shared<Lanelet>(lanelet3Id, lanelet3LeftVertices, lanelet3RightVertices, lanelet3LaneletTypes)};
    size_t lanelet4Id{4};
    std::vector<vertex> lanelet4LeftVertices{{1015, 1005}, {1200, 1005}};
    std::vector<vertex> lanelet4RightVertices{{1015, 1000}, {1200, 1000}};
    std::set<LaneletType> lanelet4LaneletTypes{LaneletType::unknown};
    auto lanelet4{
        std::make_shared<Lanelet>(lanelet4Id, lanelet4LeftVertices, lanelet4RightVertices, lanelet4LaneletTypes)};
    size_t lanelet5Id{5};
    std::vector<vertex> lanelet5LeftVertices{{1007, 998}, {1007, 900}};
    std::vector<vertex> lanelet5RightVertices{{1002, 998}, {1002, 900}};
    std::set<LaneletType> lanelet5LaneletTypes{LaneletType::unknown};
    auto lanelet5{
        std::make_shared<Lanelet>(lanelet5Id, lanelet5LeftVertices, lanelet5RightVertices, lanelet5LaneletTypes)};
    size_t lanelet6Id{6};
    std::vector<vertex> lanelet6LeftVertices{{1007, 900}, {1007, 998}};
    std::vector<vertex> lanelet6RightVertices{{1012, 900}, {1012, 998}};
    std::set<LaneletType> lanelet6LaneletTypes{LaneletType::incoming};
    auto lanelet6{
        std::make_shared<Lanelet>(lanelet6Id, lanelet6LeftVertices, lanelet6RightVertices, lanelet6LaneletTypes)};
    size_t lanelet70Id{70};
    std::vector<vertex> lanelet70LeftVertices{{1015, 1005}, {1007.5, 1005}};
    std::vector<vertex> lanelet70RightVertices{{1015, 1010}, {1007.5, 1010}};
    std::set<LaneletType> lanelet70LaneletTypes{LaneletType::intersection};
    auto lanelet70{
        std::make_shared<Lanelet>(lanelet70Id, lanelet70LeftVertices, lanelet70RightVertices, lanelet70LaneletTypes)};
    size_t lanelet7Id{71};
    std::vector<vertex> lanelet71LeftVertices{{1007.5, 1005}, {1000, 1005}};
    std::vector<vertex> lanelet71RightVertices{{1007.5, 1010}, {1000, 1010}};
    std::set<LaneletType> lanelet71LaneletTypes{LaneletType::intersection};
    auto lanelet71{
        std::make_shared<Lanelet>(lanelet7Id, lanelet71LeftVertices, lanelet71RightVertices, lanelet71LaneletTypes)};
    size_t lanelet80Id{80};
    std::vector<vertex> lanelet80LeftVertices{{1000, 1005}, {1007.5, 1005}};
    std::vector<vertex> lanelet80RightVertices{{1000, 1010}, {1007.5, 1010}};
    std::set<LaneletType> lanelet80LaneletTypes{LaneletType::intersection};
    auto lanelet80{
        std::make_shared<Lanelet>(lanelet80Id, lanelet80LeftVertices, lanelet80RightVertices, lanelet80LaneletTypes)};
    size_t lanelet81Id{81};
    std::vector<vertex> lanelet81LeftVertices{{1007.5, 1005}, {1015, 1005}};
    std::vector<vertex> lanelet81RightVertices{{1007.5, 1010}, {1015, 1010}};
    std::set<LaneletType> lanelet81LaneletTypes{LaneletType::intersection};
    auto lanelet81{
        std::make_shared<Lanelet>(lanelet81Id, lanelet81LeftVertices, lanelet81RightVertices, lanelet81LaneletTypes)};
    size_t lanelet90Id{90};
    std::vector<vertex> lanelet90LeftVertices{{1007, 998}, {1007, 1002}};
    std::vector<vertex> lanelet90RightVertices{{1012, 998}, {1012, 999}};
    std::set<LaneletType> lanelet90LaneletTypes{LaneletType::intersection};
    auto lanelet90{
        std::make_shared<Lanelet>(lanelet90Id, lanelet90LeftVertices, lanelet90RightVertices, lanelet90LaneletTypes)};
    size_t lanelet91Id{91};
    std::vector<vertex> lanelet91LeftVertices{{1007, 1002}, {1015, 1005}};
    std::vector<vertex> lanelet91RightVertices{{1012, 999}, {1015, 1000}};
    std::set<LaneletType> lanelet91LaneletTypes{LaneletType::intersection};
    auto lanelet91{
        std::make_shared<Lanelet>(lanelet91Id, lanelet91LeftVertices, lanelet91RightVertices, lanelet91LaneletTypes)};
    size_t lanelet100Id{100};
    std::vector<vertex> lanelet100LeftVertices{{1015, 1005}, {1007, 1002}};
    std::vector<vertex> lanelet100RightVertices{{1015, 1010}, {1002, 1008}};
    std::set<LaneletType> lanelet100LaneletTypes{LaneletType::intersection};
    auto lanelet100{std::make_shared<Lanelet>(lanelet100Id, lanelet100LeftVertices, lanelet100RightVertices,
                                              lanelet100LaneletTypes)};
    size_t lanelet101Id{101};
    std::vector<vertex> lanelet101LeftVertices{{1007, 1002}, {1007, 998}};
    std::vector<vertex> lanelet101RightVertices{{1002, 1008}, {1002, 998}};
    std::set<LaneletType> lanelet101LaneletTypes{LaneletType::intersection};
    auto lanelet101{std::make_shared<Lanelet>(lanelet101Id, lanelet101LeftVertices, lanelet101RightVertices,
                                              lanelet101LaneletTypes)};
    size_t lanelet110Id{110};
    std::vector<vertex> lanelet110LeftVertices{{1000, 1005}, {1007, 1002}};
    std::vector<vertex> lanelet110RightVertices{{1000, 1000}, {1002, 999}};
    std::set<LaneletType> lanelet110LaneletTypes{LaneletType::intersection};
    auto lanelet110{std::make_shared<Lanelet>(lanelet110Id, lanelet110LeftVertices, lanelet110RightVertices,
                                              lanelet110LaneletTypes)};
    size_t lanelet111Id{111};
    std::vector<vertex> lanelet111LeftVertices{{1007, 1002}, {1007, 998}};
    std::vector<vertex> lanelet111RightVertices{{1002, 999}, {1002, 998}};
    std::set<LaneletType> lanelet111LaneletTypes{LaneletType::intersection};
    auto lanelet111{std::make_shared<Lanelet>(lanelet111Id, lanelet111LeftVertices, lanelet111RightVertices,
                                              lanelet111LaneletTypes)};
    size_t lanelet120Id{120};
    std::vector<vertex> lanelet120LeftVertices{{1007, 998}, {1007, 1002}};
    std::vector<vertex> lanelet120RightVertices{{1012, 990}, {1012, 1008}};
    std::set<LaneletType> lanelet120LaneletTypes{LaneletType::intersection};
    auto lanelet120{std::make_shared<Lanelet>(lanelet120Id, lanelet120LeftVertices, lanelet120RightVertices,
                                              lanelet120LaneletTypes)};
    size_t lanelet121Id{121};
    std::vector<vertex> lanelet121LeftVertices{{1007, 1002}, {1000, 1005}};
    std::vector<vertex> lanelet121RightVertices{{1012, 1008}, {1000, 1010}};
    std::set<LaneletType> lanelet121LaneletTypes{LaneletType::intersection};
    auto lanelet121{std::make_shared<Lanelet>(lanelet121Id, lanelet121LeftVertices, lanelet121RightVertices,
                                              lanelet121LaneletTypes)};

    lanelet1->addPredecessor(lanelet71);
    lanelet1->addPredecessor(lanelet121);
    lanelet2->addSuccessor(lanelet80);
    lanelet2->addSuccessor(lanelet110);
    lanelet3->addSuccessor(lanelet70);
    lanelet3->addSuccessor(lanelet100);
    lanelet4->addPredecessor(lanelet81);
    lanelet4->addPredecessor(lanelet91);
    lanelet5->addPredecessor(lanelet111);
    lanelet5->addPredecessor(lanelet101);
    lanelet6->addSuccessor(lanelet90);
    lanelet6->addSuccessor(lanelet120);
    lanelet70->addPredecessor(lanelet3);
    lanelet71->addPredecessor(lanelet70);
    lanelet70->addSuccessor(lanelet71);
    lanelet71->addSuccessor(lanelet1);
    lanelet80->addPredecessor(lanelet2);
    lanelet81->addPredecessor(lanelet80);
    lanelet80->addSuccessor(lanelet81);
    lanelet81->addSuccessor(lanelet4);
    lanelet90->addPredecessor(lanelet6);
    lanelet90->addSuccessor(lanelet91);
    lanelet91->addPredecessor(lanelet90);
    lanelet91->addSuccessor(lanelet4);
    lanelet100->addPredecessor(lanelet3);
    lanelet100->addSuccessor(lanelet101);
    lanelet101->addPredecessor(lanelet100);
    lanelet101->addSuccessor(lanelet5);
    lanelet110->addPredecessor(lanelet2);
    lanelet110->addSuccessor(lanelet111);
    lanelet111->addPredecessor(lanelet110);
    lanelet111->addSuccessor(lanelet5);
    lanelet120->addPredecessor(lanelet6);
    lanelet120->addSuccessor(lanelet121);
    lanelet121->addPredecessor(lanelet120);
    lanelet121->addSuccessor(lanelet1);
    lanelet70->setRightAdjacent(lanelet81, DrivingDirection::opposite);
    lanelet71->setRightAdjacent(lanelet80, DrivingDirection::opposite);
    lanelet80->setLeftAdjacent(lanelet71, DrivingDirection::opposite);
    lanelet81->setLeftAdjacent(lanelet70, DrivingDirection::opposite);

    size_t incomingId1{13};
    std::vector<std::shared_ptr<Lanelet>> incoming1IncomingLanelets{lanelet2};
    std::shared_ptr<Incoming> isLeftOfIncoming1{nullptr};
    std::vector<std::shared_ptr<Lanelet>> straightOutgoingsIncoming1{lanelet81};
    std::vector<std::shared_ptr<Lanelet>> leftOutgoingsIncoming1{};
    std::vector<std::shared_ptr<Lanelet>> rightOutgoingsIncoming1{lanelet111};
    std::vector<std::shared_ptr<Lanelet>> oncomingsIncoming1{lanelet71};
    incomingOne = std::make_shared<Incoming>(incomingId1, incoming1IncomingLanelets, isLeftOfIncoming1,
                                             straightOutgoingsIncoming1, leftOutgoingsIncoming1,
                                             rightOutgoingsIncoming1, oncomingsIncoming1);

    size_t incomingId2{14};
    std::vector<std::shared_ptr<Lanelet>> incoming2IncomingLanelets{lanelet3};
    std::shared_ptr<Incoming> isLeftOfIncoming2{nullptr};
    std::vector<std::shared_ptr<Lanelet>> straightOutgoingsIncoming2{lanelet71};
    std::vector<std::shared_ptr<Lanelet>> leftOutgoingsIncoming2{lanelet101};
    std::vector<std::shared_ptr<Lanelet>> rightOutgoingsIncoming2{};
    std::vector<std::shared_ptr<Lanelet>> oncomingsIncoming2{lanelet81};
    incomingTwo = std::make_shared<Incoming>();
    incomingTwo->setId(incomingId2);
    incomingTwo->setIsLeftOf(isLeftOfIncoming2);
    incomingTwo->setIncomingLanelets(incoming2IncomingLanelets);
    incomingTwo->setLeftOutgoings(leftOutgoingsIncoming2);
    incomingTwo->setRightOutgoings(rightOutgoingsIncoming2);
    incomingTwo->setStraightOutgoings(straightOutgoingsIncoming2);
    incomingTwo->setOncomings(oncomingsIncoming2);

    size_t incomingId3{15};
    std::vector<std::shared_ptr<Lanelet>> incoming3IncomingLanelets{lanelet6};
    std::shared_ptr<Incoming> isLeftOfIncoming3{incomingOne};
    std::vector<std::shared_ptr<Lanelet>> straightOutgoingsIncoming3{};
    std::vector<std::shared_ptr<Lanelet>> leftOutgoingsIncoming3{lanelet121};
    std::vector<std::shared_ptr<Lanelet>> rightOutgoingsIncoming3{lanelet91};
    std::vector<std::shared_ptr<Lanelet>> oncomingsIncoming3{};
    incomingThree = std::make_shared<Incoming>(incomingId3, incoming3IncomingLanelets, isLeftOfIncoming3,
                                               straightOutgoingsIncoming3, leftOutgoingsIncoming3,
                                               rightOutgoingsIncoming3, oncomingsIncoming3);
    incomingTwo->setIsLeftOf(incomingThree);
    lanelets = {lanelet1,   lanelet2,   lanelet3,   lanelet4,   lanelet5,   lanelet6,
                lanelet70,  lanelet71,  lanelet80,  lanelet81,  lanelet90,  lanelet91,
                lanelet100, lanelet101, lanelet110, lanelet111, lanelet120, lanelet121};
}

void IncomingTest::SetUp() { setUpIncoming(); }

TEST_F(IncomingTest, InitializationComplete) {
    EXPECT_EQ(incomingOne->getId(), 13);
    EXPECT_EQ(incomingTwo->getId(), 14);
    EXPECT_EQ(incomingThree->getId(), 15);

    EXPECT_EQ(incomingOne->getIncomingLanelets().size(), 1);
    EXPECT_EQ(incomingTwo->getIncomingLanelets().size(), 1);
    EXPECT_EQ(incomingThree->getIncomingLanelets().size(), 1);
    EXPECT_EQ(incomingOne->getIncomingLanelets().at(0)->getId(), 2);
    EXPECT_EQ(incomingTwo->getIncomingLanelets().at(0)->getId(), 3);
    EXPECT_EQ(incomingThree->getIncomingLanelets().at(0)->getId(), 6);

    EXPECT_EQ(incomingOne->getLeftOutgoings().size(), 0);
    EXPECT_EQ(incomingTwo->getLeftOutgoings().size(), 1);
    EXPECT_EQ(incomingThree->getLeftOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getLeftOutgoings().at(0)->getId(), 101);
    EXPECT_EQ(incomingThree->getLeftOutgoings().at(0)->getId(), 121);

    EXPECT_EQ(incomingOne->getRightOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getRightOutgoings().size(), 0);
    EXPECT_EQ(incomingThree->getRightOutgoings().size(), 1);
    EXPECT_EQ(incomingOne->getRightOutgoings().at(0)->getId(), 111);
    EXPECT_EQ(incomingThree->getRightOutgoings().at(0)->getId(), 91);

    EXPECT_EQ(incomingOne->getStraightOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getStraightOutgoings().size(), 1);
    EXPECT_EQ(incomingThree->getStraightOutgoings().size(), 0);
    EXPECT_EQ(incomingOne->getStraightOutgoings().at(0)->getId(), 81);
    EXPECT_EQ(incomingTwo->getStraightOutgoings().at(0)->getId(), 71);

    EXPECT_EQ(incomingOne->getOncomings().size(), 1);
    EXPECT_EQ(incomingTwo->getOncomings().size(), 1);
    EXPECT_EQ(incomingThree->getOncomings().size(), 0);
    EXPECT_EQ(incomingOne->getOncomings().at(0)->getId(), 71);
    EXPECT_EQ(incomingTwo->getOncomings().at(0)->getId(), 81);

    EXPECT_EQ(incomingOne->getIsLeftOf(), nullptr);
    EXPECT_EQ(incomingTwo->getIsLeftOf()->getId(), 15);
    EXPECT_EQ(incomingThree->getIsLeftOf()->getId(), 13);
}