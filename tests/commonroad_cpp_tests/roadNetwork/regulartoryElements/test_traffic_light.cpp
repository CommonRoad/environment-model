//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2022 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_traffic_light.h"

void TrafficLightTest::SetUp() {
    size_t lightId1{1};
    vertex pos1{1001, 1002};
    size_t offset1{2};
    std::vector<TrafficLightCycleElement> cycle1{{TrafficLightState::green, 20}, {TrafficLightState::red, 100}};
    light1 = std::make_shared<TrafficLight>(lightId1, cycle1, offset1, TurningDirection::left, true, pos1);

    size_t lightId2{2};
    vertex pos2{1003, 1004};
    std::vector<TrafficLightCycleElement> cycle2{{TrafficLightState::inactive, 1000}, {TrafficLightState::yellow, 1}};
    light2 = std::make_shared<TrafficLight>();
    light2->setId(lightId2);
    light2->setPosition(pos2);
    light2->setActive(false);
    light2->setCycle(cycle2);
    light2->setDirection(TurningDirection::straightRight);
    light2->setOffset(0);
    light2->addCycleElement({TrafficLightState::green, 50});

    size_t lightId3{3};
    vertex pos3{1005, 1006};
    size_t offset3{0};
    std::vector<TrafficLightCycleElement> cycle3{{TrafficLightState::red_yellow, 20}, {TrafficLightState::yellow, 100}};
    light3 = std::make_shared<TrafficLight>(lightId3, cycle3, offset3, TurningDirection::leftStraight, true, pos1);

    size_t lightId4{4};
    vertex pos4{1007, 1008};
    size_t offset4{0};
    std::vector<TrafficLightCycleElement> cycle4{{TrafficLightState::green, 20}, {TrafficLightState::red, 100}};
    light4 = std::make_shared<TrafficLight>(lightId4, cycle4, offset4, TurningDirection::leftRight, true, pos1);
}

TEST_F(TrafficLightTest, InitializationComplete) {
    EXPECT_EQ(light1->getId(), 1);
    EXPECT_EQ(light1->getPosition().x, 1001);
    EXPECT_EQ(light1->getPosition().y, 1002);
    EXPECT_EQ(light1->isActive(), true);
    EXPECT_EQ(light1->getOffset(), 2);
    EXPECT_EQ(light1->getDirection(), TurningDirection::left);
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
    EXPECT_EQ(light2->getPosition().x, 1003);
    EXPECT_EQ(light2->getPosition().y, 1004);
    EXPECT_EQ(light2->isActive(), false);
    EXPECT_EQ(light2->getOffset(), 0);
    EXPECT_EQ(light2->getDirection(), TurningDirection::straightRight);
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

    EXPECT_EQ(light3->getId(), 3);
    EXPECT_EQ(light3->getDirection(), TurningDirection::leftStraight);
    EXPECT_EQ(light3->getElementAtTime(0).color, TrafficLightState::red_yellow);

    EXPECT_EQ(light4->getId(), 4);
    EXPECT_EQ(light4->getDirection(), TurningDirection::leftRight);
}

TEST_F(TrafficLightTest, MatchTurningDirections) {
    EXPECT_EQ(TrafficLight::matchTurningDirections("right"), TurningDirection::right);
    EXPECT_EQ(TrafficLight::matchTurningDirections("straight"), TurningDirection::straight);
    EXPECT_EQ(TrafficLight::matchTurningDirections("left"), TurningDirection::left);
    EXPECT_EQ(TrafficLight::matchTurningDirections("leftStraight"), TurningDirection::leftStraight);
    EXPECT_EQ(TrafficLight::matchTurningDirections("straightRight"), TurningDirection::straightRight);
    EXPECT_EQ(TrafficLight::matchTurningDirections("leftRight"), TurningDirection::leftRight);
    EXPECT_THROW(TrafficLight::matchTurningDirections("test"), std::logic_error);
}

TEST_F(TrafficLightTest, MatchTrafficLightState) {
    EXPECT_EQ(TrafficLight::matchTrafficLightState("green"), TrafficLightState::green);
    EXPECT_EQ(TrafficLight::matchTrafficLightState("yellow"), TrafficLightState::yellow);
    EXPECT_EQ(TrafficLight::matchTrafficLightState("redYellow"), TrafficLightState::red_yellow);
    EXPECT_EQ(TrafficLight::matchTrafficLightState("red"), TrafficLightState::red);
    EXPECT_THROW(TrafficLight::matchTrafficLightState("test"), std::logic_error);
    EXPECT_EQ(TrafficLight::matchTrafficLightState("inactive"), TrafficLightState::inactive);
}
