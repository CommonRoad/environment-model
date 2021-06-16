//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "traffic_light.h"
#include <algorithm>

TrafficLight::TrafficLight() : id(0), offset(0), direction(TurningDirections::all) {}

void TrafficLight::setId(const size_t num) { id = num; }

void TrafficLight::setCycle(const std::vector<TrafficLightCycleElement> &light_cycle) { cycle = light_cycle; }

void TrafficLight::setOffset(const size_t ofst) { offset = ofst; }

size_t TrafficLight::getId() const { return id; }

size_t TrafficLight::getOffset() const { return offset; }

std::vector<TrafficLightCycleElement> TrafficLight::getCycle() const { return cycle; }

TrafficLightCycleElement TrafficLight::getElementAtTime(size_t time) {
    std::vector<size_t> cycleInitTimeSteps{offset};
    for (size_t i{0}; i < cycle.size(); ++i) {
        if (i == 0)
            cycleInitTimeSteps.push_back(cycle.at(i).duration + offset);
        else
            cycleInitTimeSteps.push_back(cycle.at(i).duration + offset + cycleInitTimeSteps.back());
    }

    size_t timeStepMod{((time - offset) % (cycleInitTimeSteps.back() - offset)) + offset};

    auto cycleIndex{std::distance(cycleInitTimeSteps.begin(),
                                  std::find_if(cycleInitTimeSteps.begin(), cycleInitTimeSteps.end(),
                                               [timeStepMod](size_t cyc) { return timeStepMod < cyc; })) -
                    1};
    return cycle.at(static_cast<unsigned long>(cycleIndex));
}

TurningDirections TrafficLight::getDirection() const { return direction; }

void TrafficLight::setDirection(TurningDirections dir) { TrafficLight::direction = dir; }

bool TrafficLight::isActive() const { return active; }

void TrafficLight::setActive(bool ac) { TrafficLight::active = ac; }

void TrafficLight::addCycleElement(TrafficLightCycleElement ce) { cycle.push_back(ce); }

void TrafficLight::setPosition(vertex pos) { position = pos; }

vertex TrafficLight::getPosition() const { return position; }

TurningDirections TrafficLight::matchTurningDirections(const std::string &dir) {
    if (dir == "right")
        return TurningDirections::right;
    else if (dir == "straight")
        return TurningDirections::straight;
    else if (dir == "left")
        return TurningDirections::left;
    else if (dir == "leftStraight")
        return TurningDirections::leftStraight;
    else if (dir == "straightRight")
        return TurningDirections::straightRight;
    else if (dir == "leftRight")
        return TurningDirections::leftRight;
    else
        return TurningDirections::all;
}

TrafficLightState TrafficLight::matchTrafficLightState(const std::string &trafficLightState) {
    if (trafficLightState == "green")
        return TrafficLightState::green;
    else if (trafficLightState == "yellow")
        return TrafficLightState::yellow;
    else if (trafficLightState == "red_yellow")
        return TrafficLightState::red_yellow;
    else
        return TrafficLightState::red; // default case -> consider also trafficLightState == "red"
}
