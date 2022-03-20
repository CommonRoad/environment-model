//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_traffic_light.h"

void TrafficLightTest::SetUp() {
    size_t lightId1{1};
    vertex pos1{1, 2};
    size_t offset1{2};
    std::vector<TrafficLightCycleElement> cycle1{{TrafficLightState::green, 20}, {TrafficLightState::red, 100}};
    light1 = std::make_shared<TrafficLight>(lightId1, cycle1, offset1, TurningDirections::left, true, pos1);

    size_t lightId2{2};
    vertex pos2{3, 4};
    std::vector<TrafficLightCycleElement> cycle2{{TrafficLightState::inactive, 1000}, {TrafficLightState::yellow, 1}};
    light2 = std::make_shared<TrafficLight>();
    light2->setId(lightId2);
    light2->setPosition(pos2);
    light2->setActive(false);
    light2->setCycle(cycle2);
    light2->setDirection(TurningDirections::straightRight);
    light2->setOffset(0);
    light2->addCycleElement({TrafficLightState::green, 50});
}

TEST_F(TrafficLightTest, InitializationComplete) {
    EXPECT_EQ(light1->getId(), 1);
    EXPECT_EQ(light1->getPosition().x, 1);
    EXPECT_EQ(light1->getPosition().y, 2);
    EXPECT_EQ(light1->isActive(), true);
    EXPECT_EQ(light1->getOffset(), 2);
    EXPECT_EQ(light1->getDirection(), TurningDirections::left);
    EXPECT_EQ(light1->getElementAtTime(0).color, TrafficLightState::green);
    EXPECT_EQ(light1->getElementAtTime(5).color, TrafficLightState::green);
    EXPECT_EQ(light1->getElementAtTime(22).color, TrafficLightState::red);
    EXPECT_EQ(light1->getElementAtTime(50).color, TrafficLightState::red);
    EXPECT_EQ(light1->getElementAtTime(105).color, TrafficLightState::red);
    EXPECT_EQ(light1->getElementAtTime(150).color, TrafficLightState::red);
    EXPECT_EQ(light1->getCycle().size(), 2);
    EXPECT_EQ(light1->getCycle().at(0).color, TrafficLightState::green);
    EXPECT_EQ(light1->getCycle().at(0).duration, 20);
    EXPECT_EQ(light1->getCycle().at(1).color, TrafficLightState::red);
    EXPECT_EQ(light1->getCycle().at(1).duration, 100);

    EXPECT_EQ(light2->getId(), 2);
    EXPECT_EQ(light2->getPosition().x, 3);
    EXPECT_EQ(light2->getPosition().y, 4);
    EXPECT_EQ(light2->isActive(), false);
    EXPECT_EQ(light2->getOffset(), 0);
    EXPECT_EQ(light2->getDirection(), TurningDirections::straightRight);
    EXPECT_EQ(light2->getElementAtTime(10).color, TrafficLightState::inactive);
    EXPECT_EQ(light2->getElementAtTime(1000).color, TrafficLightState::yellow);
    EXPECT_EQ(light2->getElementAtTime(1001).color, TrafficLightState::green);
    EXPECT_EQ(light2->getElementAtTime(1050).color, TrafficLightState::green);
    EXPECT_EQ(light2->getElementAtTime(1051).color, TrafficLightState::inactive);
    EXPECT_EQ(light2->getCycle().size(), 3);
    EXPECT_EQ(light2->getCycle().at(0).color, TrafficLightState::inactive);
    EXPECT_EQ(light2->getCycle().at(0).duration, 1000);
    EXPECT_EQ(light2->getCycle().at(1).color, TrafficLightState::yellow);
    EXPECT_EQ(light2->getCycle().at(1).duration, 1);
    EXPECT_EQ(light2->getCycle().at(2).color, TrafficLightState::green);
    EXPECT_EQ(light2->getCycle().at(2).duration, 50);
}
