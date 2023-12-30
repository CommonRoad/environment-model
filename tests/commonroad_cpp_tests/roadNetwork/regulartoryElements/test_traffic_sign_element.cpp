//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_traffic_sign_element.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"

void TrafficSignElementTest::SetUp() {
    const auto *trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(SupportedTrafficSignCountry::GERMANY);

    std::vector<std::string> trafficSignElementOneValues{"20"};
    element1 = std::shared_ptr<TrafficSignElement>{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::MAX_SPEED, trafficSignElementOneValues)};

    std::vector<std::string> trafficSignElementTwoValues{"10"};
    element2 = std::shared_ptr<TrafficSignElement>{std::make_shared<TrafficSignElement>(TrafficSignTypes::MIN_SPEED)};
    element2->setAdditionalValues(trafficSignElementTwoValues);
    element2->addAdditionalValue("25");
}

TEST_F(TrafficSignElementTest, InitializationComplete) {
    EXPECT_EQ(element1->getTrafficSignType(), TrafficSignTypes::MAX_SPEED);
    EXPECT_EQ(element1->getAdditionalValues().size(), 1);
    EXPECT_EQ(element1->getAdditionalValues().at(0), "20");

    EXPECT_EQ(element2->getTrafficSignType(), TrafficSignTypes::MIN_SPEED);
    EXPECT_EQ(element2->getAdditionalValues().size(), 2);
    EXPECT_EQ(element2->getAdditionalValues().at(0), "10");
    EXPECT_EQ(element2->getAdditionalValues().at(1), "25");
}

TEST_F(TrafficSignElementTest, convertGermanTrafficSignIdToString) {
    EXPECT_EQ(element1->convertGermanTrafficSignIdToString(TrafficSignTypes::MAX_SPEED), "274");
}