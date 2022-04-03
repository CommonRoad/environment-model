//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "traffic_light.h"
#include <algorithm>
#include <utility>

void TrafficLight::setId(const size_t num) { id = num; }

void TrafficLight::setCycle(const std::vector<TrafficLightCycleElement> &light_cycle) { cycle = light_cycle; }

void TrafficLight::setOffset(const size_t timeStepsOffset) { offset = timeStepsOffset; }

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

void TrafficLight::setActive(bool trafficLightActive) { TrafficLight::active = trafficLightActive; }

void TrafficLight::addCycleElement(TrafficLightCycleElement cycleElement) { cycle.push_back(cycleElement); }

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
    else if (trafficLightState == "red")
        return TrafficLightState::red;
    else if (trafficLightState == "inactive")
        return TrafficLightState::inactive;
    else
        return TrafficLightState::red; // default case -> consider also trafficLightState == "red"
}
TrafficLight::TrafficLight(size_t trafficLightId, std::vector<TrafficLightCycleElement> cycle, size_t offset,
                           TurningDirections direction, bool active, const vertex &position)
    : id(trafficLightId), cycle(std::move(cycle)), offset(offset), direction(direction), active(active),
      position(position) {}
