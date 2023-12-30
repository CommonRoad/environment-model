//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_traffic_sign.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"

void TrafficSignTest::SetUp() {
    const auto *trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(SupportedTrafficSignCountry::GERMANY);

    std::vector<std::string> trafficSignElementOneValues{"20"};
    auto signElement1 = std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::MAX_SPEED), trafficSignElementOneValues)};

    std::vector<std::string> trafficSignElementTwoValues{"10"};
    auto signElement2 = std::vector<std::shared_ptr<TrafficSignElement>>{std::make_shared<TrafficSignElement>(
        trafficSignIDLookupTable->at(TrafficSignTypes::MIN_SPEED), trafficSignElementTwoValues)};

    size_t signId1{1};
    vertex pos1{1, 2};
    sign1 = std::make_shared<TrafficSign>(signId1, signElement1, pos1, false);

    size_t signId2{2};
    vertex pos2{3, 4};
    sign2 = std::make_shared<TrafficSign>();
    sign2->setId(signId2);
    sign2->setPosition(pos2);
    sign2->setTrafficSignElement(signElement2);
    sign2->addTrafficSignElement(signElement1.at(0));
    sign2->setVirtualElement(true);
}

TEST_F(TrafficSignTest, InitializationComplete) {
    EXPECT_EQ(sign1->getId(), 1);
    EXPECT_EQ(sign1->getPosition().x, 1);
    EXPECT_EQ(sign1->getPosition().y, 2);
    EXPECT_EQ(sign1->isVirtualElement(), false);
    EXPECT_EQ(sign1->getTrafficSignElements().size(), 1);
    EXPECT_EQ(sign1->getTrafficSignElements().at(0)->getId(), "274");

    EXPECT_EQ(sign2->getId(), 2);
    EXPECT_EQ(sign2->getPosition().x, 3);
    EXPECT_EQ(sign2->getPosition().y, 4);
    EXPECT_EQ(sign2->isVirtualElement(), true);
    EXPECT_EQ(sign2->getTrafficSignElements().size(), 2);
    EXPECT_EQ(sign2->getTrafficSignElements().at(0)->getId(), "275");
    EXPECT_EQ(sign2->getTrafficSignElements().at(1)->getId(), "274");
}

TEST_F(TrafficSignTest, GetTrafficSignElementsOfType) {
    EXPECT_EQ(sign1->getTrafficSignElementsOfType("274").size(), 1);
    EXPECT_EQ(sign1->getTrafficSignElementsOfType("123").size(), 0);

    EXPECT_EQ(sign2->getTrafficSignElementsOfType("274").size(), 1);
    EXPECT_EQ(sign2->getTrafficSignElementsOfType("275").size(), 1);
}
