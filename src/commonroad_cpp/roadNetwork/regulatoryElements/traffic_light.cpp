//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>
#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <stdexcept>
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

TurningDirection TrafficLight::getDirection() const { return direction; }

void TrafficLight::setDirection(TurningDirection dir) { TrafficLight::direction = dir; }

bool TrafficLight::isActive() const { return active; }

void TrafficLight::setActive(bool trafficLightActive) { TrafficLight::active = trafficLightActive; }

void TrafficLight::addCycleElement(TrafficLightCycleElement cycleElement) { cycle.push_back(cycleElement); }

void TrafficLight::setPosition(vertex pos) { position = pos; }

vertex TrafficLight::getPosition() const { return position; }

TurningDirection TrafficLight::matchTurningDirections(const std::string &dir) {
    std::string str{dir};
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    if (TurningDirectionNames.count(str) == 1)
        return TurningDirectionNames.at(str);
    else
        throw std::logic_error("TrafficLight::matchTurningDirections: Invalid turning direction state!");
}

TrafficLightState TrafficLight::matchTrafficLightState(const std::string &trafficLightState) {
    std::string str{trafficLightState};
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    if (TrafficLightStateNames.count(str) == 1)
        return TrafficLightStateNames.at(str);
    else
        throw std::logic_error("TrafficLight::matchTrafficLightState: Invalid traffic light state!");
}

TrafficLight::TrafficLight(size_t trafficLightId, std::vector<TrafficLightCycleElement> cycle, size_t offset,
                           TurningDirection direction, bool active, const vertex &position)
    : id(trafficLightId), cycle(std::move(cycle)), offset(offset), direction(direction), active(active),
      position(position) {}
