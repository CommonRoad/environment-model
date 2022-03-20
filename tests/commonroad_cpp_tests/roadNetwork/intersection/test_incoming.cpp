//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_incoming.h"

void IncomingTestInitialization::setUpIncoming() {
    size_t lanelet1Id{1};
    std::vector<vertex> lanelet1LeftVertices{{0, 5}, {-20, 5}};
    std::vector<vertex> lanelet1RightVertices{{0, 10}, {-20, 10}};
    std::set<LaneletType> lanelet1LaneletTypes{LaneletType::unknown};
    auto lanelet1{
        std::make_shared<Lanelet>(lanelet1Id, lanelet1LeftVertices, lanelet1RightVertices, lanelet1LaneletTypes)};
    size_t lanelet2Id{2};
    std::vector<vertex> lanelet2LeftVertices{{-20, 5}, {0, 5}};
    std::vector<vertex> lanelet2RightVertices{{-20, 0}, {0, 0}};
    std::set<LaneletType> lanelet2LaneletTypes{LaneletType::incoming};
    auto lanelet2{
        std::make_shared<Lanelet>(lanelet2Id, lanelet2LeftVertices, lanelet2RightVertices, lanelet2LaneletTypes)};
    size_t lanelet3Id{3};
    std::vector<vertex> lanelet3LeftVertices{{35, 5}, {15, 5}};
    std::vector<vertex> lanelet3RightVertices{{35, 10}, {15, 10}};
    std::set<LaneletType> lanelet3LaneletTypes{LaneletType::incoming};
    auto lanelet3{
        std::make_shared<Lanelet>(lanelet3Id, lanelet3LeftVertices, lanelet3RightVertices, lanelet3LaneletTypes)};
    size_t lanelet4Id{4};
    std::vector<vertex> lanelet4LeftVertices{{15, 5}, {35, 5}};
    std::vector<vertex> lanelet4RightVertices{{15, 0}, {35, 0}};
    std::set<LaneletType> lanelet4LaneletTypes{LaneletType::unknown};
    auto lanelet4{
        std::make_shared<Lanelet>(lanelet4Id, lanelet4LeftVertices, lanelet4RightVertices, lanelet4LaneletTypes)};
    size_t lanelet5Id{5};
    std::vector<vertex> lanelet5LeftVertices{{7, -2}, {7, -22}};
    std::vector<vertex> lanelet5RightVertices{{2, -2}, {2, -22}};
    std::set<LaneletType> lanelet5LaneletTypes{LaneletType::unknown};
    auto lanelet5{
        std::make_shared<Lanelet>(lanelet5Id, lanelet5LeftVertices, lanelet5RightVertices, lanelet5LaneletTypes)};
    size_t lanelet6Id{6};
    std::vector<vertex> lanelet6LeftVertices{{7, -22}, {7, -2}};
    std::vector<vertex> lanelet6RightVertices{{12, -22}, {12, -2}};
    std::set<LaneletType> lanelet6LaneletTypes{LaneletType::incoming};
    auto lanelet6{
        std::make_shared<Lanelet>(lanelet6Id, lanelet6LeftVertices, lanelet6RightVertices, lanelet6LaneletTypes)};
    size_t lanelet7Id{7};
    std::vector<vertex> lanelet7LeftVertices{{15, 5}, {0, 5}};
    std::vector<vertex> lanelet7RightVertices{{15, 10}, {0, 10}};
    std::set<LaneletType> lanelet7LaneletTypes{LaneletType::intersection};
    auto lanelet7{
        std::make_shared<Lanelet>(lanelet7Id, lanelet7LeftVertices, lanelet7RightVertices, lanelet7LaneletTypes)};
    size_t lanelet8Id{8};
    std::vector<vertex> lanelet8LeftVertices{{0, 5}, {15, 5}};
    std::vector<vertex> lanelet8RightVertices{{0, 10}, {15, 10}};
    std::set<LaneletType> lanelet8LaneletTypes{LaneletType::intersection};
    auto lanelet8{
        std::make_shared<Lanelet>(lanelet8Id, lanelet8LeftVertices, lanelet8RightVertices, lanelet8LaneletTypes)};
    size_t lanelet9Id{9};
    std::vector<vertex> lanelet9LeftVertices{{7, -2}, {7, 2}, {15, 5}};
    std::vector<vertex> lanelet9RightVertices{{12, -2}, {12, -1}, {15, 0}};
    std::set<LaneletType> lanelet9LaneletTypes{LaneletType::intersection};
    auto lanelet9{
        std::make_shared<Lanelet>(lanelet9Id, lanelet9LeftVertices, lanelet9RightVertices, lanelet9LaneletTypes)};
    size_t lanelet10Id{10};
    std::vector<vertex> lanelet10LeftVertices{{15, 5}, {7, 2}, {7, -2}};
    std::vector<vertex> lanelet10RightVertices{{15, 10}, {2, 8}, {2, -2}};
    std::set<LaneletType> lanelet10LaneletTypes{LaneletType::intersection};
    auto lanelet10{
        std::make_shared<Lanelet>(lanelet10Id, lanelet10LeftVertices, lanelet10RightVertices, lanelet10LaneletTypes)};
    size_t lanelet11Id{11};
    std::vector<vertex> lanelet11LeftVertices{{0, 5}, {7, 2}, {7, -2}};
    std::vector<vertex> lanelet11RightVertices{{0, 0}, {2, -1}, {2, -2}};
    std::set<LaneletType> lanelet11LaneletTypes{LaneletType::intersection};
    auto lanelet11{
        std::make_shared<Lanelet>(lanelet11Id, lanelet11LeftVertices, lanelet11RightVertices, lanelet11LaneletTypes)};
    size_t lanelet12Id{12};
    std::vector<vertex> lanelet12LeftVertices{{7, -2}, {7, 2}, {0, 5}};
    std::vector<vertex> lanelet12RightVertices{{12, -2}, {12, 8}, {0, 10}};
    std::set<LaneletType> lanelet12LaneletTypes{LaneletType::intersection};
    auto lanelet12{
        std::make_shared<Lanelet>(lanelet12Id, lanelet12LeftVertices, lanelet12RightVertices, lanelet12LaneletTypes)};

    lanelet1->addPredecessor(lanelet7);
    lanelet1->addPredecessor(lanelet12);
    lanelet2->addSuccessor(lanelet8);
    lanelet2->addSuccessor(lanelet11);
    lanelet3->addSuccessor(lanelet7);
    lanelet3->addSuccessor(lanelet10);
    lanelet4->addPredecessor(lanelet8);
    lanelet4->addPredecessor(lanelet9);
    lanelet5->addPredecessor(lanelet11);
    lanelet6->addSuccessor(lanelet12);
    lanelet7->addPredecessor(lanelet3);
    lanelet7->addSuccessor(lanelet1);
    lanelet8->addPredecessor(lanelet2);
    lanelet8->addSuccessor(lanelet4);
    lanelet9->addPredecessor(lanelet6);
    lanelet9->addSuccessor(lanelet4);
    lanelet10->addPredecessor(lanelet3);
    lanelet10->addSuccessor(lanelet5);
    lanelet11->addPredecessor(lanelet2);
    lanelet11->addSuccessor(lanelet5);
    lanelet12->addPredecessor(lanelet6);
    lanelet12->addSuccessor(lanelet1);
    lanelet7->setRightAdjacent(lanelet8, DrivingDirection::opposite);
    lanelet8->setLeftAdjacent(lanelet7, DrivingDirection::opposite);

    size_t incomingId1{13};
    std::vector<std::shared_ptr<Lanelet>> incoming1IncomingLanelets{lanelet2};
    std::shared_ptr<Incoming> isLeftOfIncoming1{nullptr};
    std::vector<std::shared_ptr<Lanelet>> straightOutgoingsIncoming1{lanelet8};
    std::vector<std::shared_ptr<Lanelet>> leftOutgoingsIncoming1{};
    std::vector<std::shared_ptr<Lanelet>> rightOutgoingsIncoming1{lanelet11};
    std::vector<std::shared_ptr<Lanelet>> oncomingsIncoming1{lanelet7};
    incomingOne = std::make_shared<Incoming>(incomingId1, incoming1IncomingLanelets, isLeftOfIncoming1,
                                             straightOutgoingsIncoming1, leftOutgoingsIncoming1,
                                             rightOutgoingsIncoming1, oncomingsIncoming1);

    size_t incomingId2{14};
    std::vector<std::shared_ptr<Lanelet>> incoming2IncomingLanelets{lanelet3};
    std::shared_ptr<Incoming> isLeftOfIncoming2{nullptr};
    std::vector<std::shared_ptr<Lanelet>> straightOutgoingsIncoming2{lanelet7};
    std::vector<std::shared_ptr<Lanelet>> leftOutgoingsIncoming2{lanelet12};
    std::vector<std::shared_ptr<Lanelet>> rightOutgoingsIncoming2{};
    std::vector<std::shared_ptr<Lanelet>> oncomingsIncoming2{lanelet8};
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
    std::vector<std::shared_ptr<Lanelet>> leftOutgoingsIncoming3{lanelet10};
    std::vector<std::shared_ptr<Lanelet>> rightOutgoingsIncoming3{lanelet9};
    std::vector<std::shared_ptr<Lanelet>> oncomingsIncoming3{};
    incomingThree = std::make_shared<Incoming>(incomingId3, incoming3IncomingLanelets, isLeftOfIncoming3,
                                               straightOutgoingsIncoming3, leftOutgoingsIncoming3,
                                               rightOutgoingsIncoming3, oncomingsIncoming3);
    incomingTwo->setIsLeftOf(incomingThree);
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
    EXPECT_EQ(incomingTwo->getLeftOutgoings().at(0)->getId(), 12);
    EXPECT_EQ(incomingThree->getLeftOutgoings().at(0)->getId(), 10);

    EXPECT_EQ(incomingOne->getRightOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getRightOutgoings().size(), 0);
    EXPECT_EQ(incomingThree->getRightOutgoings().size(), 1);
    EXPECT_EQ(incomingOne->getRightOutgoings().at(0)->getId(), 11);
    EXPECT_EQ(incomingThree->getRightOutgoings().at(0)->getId(), 9);

    EXPECT_EQ(incomingOne->getStraightOutgoings().size(), 1);
    EXPECT_EQ(incomingTwo->getStraightOutgoings().size(), 1);
    EXPECT_EQ(incomingThree->getStraightOutgoings().size(), 0);
    EXPECT_EQ(incomingOne->getStraightOutgoings().at(0)->getId(), 8);
    EXPECT_EQ(incomingTwo->getStraightOutgoings().at(0)->getId(), 7);

    EXPECT_EQ(incomingOne->getOncomings().size(), 1);
    EXPECT_EQ(incomingTwo->getOncomings().size(), 1);
    EXPECT_EQ(incomingThree->getOncomings().size(), 0);
    EXPECT_EQ(incomingOne->getOncomings().at(0)->getId(), 7);
    EXPECT_EQ(incomingTwo->getOncomings().at(0)->getId(), 8);

    EXPECT_EQ(incomingOne->getIsLeftOf(), nullptr);
    EXPECT_EQ(incomingTwo->getIsLeftOf()->getId(), 15);
    EXPECT_EQ(incomingThree->getIsLeftOf()->getId(), 13);
}