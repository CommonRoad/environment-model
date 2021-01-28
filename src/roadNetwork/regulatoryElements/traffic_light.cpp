//
// Created by Sebastian Maierhofer on 31.10.20.
//

#include "traffic_light.h"

TrafficLight::TrafficLight() : id(0), offset(0), direction(TrafficLightDirection::all), active(false){ }

void TrafficLight::setId(const int num) { id = num; }

void TrafficLight::setCycle(const std::vector<TrafficLightCycleElement> &light_cycle) { cycle = light_cycle; }

void TrafficLight::setOffset(const int ofst) { offset = ofst; }

int TrafficLight::getId() const { return id; }

int TrafficLight::getOffset() const { return offset; }

std::vector<TrafficLightCycleElement> TrafficLight::getCycle() const { return cycle; }

TrafficLightCycleElement TrafficLight::getElementAtTime(int time) {
    time += offset;
    TrafficLightCycleElement current = cycle.front();
    int i = 0;
    while (time > 0) {
        if (i >= cycle.size()) {
            i = 0;
        }
        current = cycle[i];
        time -= current.duration;
        i++;
    }
    return current;
}

TrafficLightDirection TrafficLight::getDirection() const { return direction; }

void TrafficLight::setDirection(TrafficLightDirection dir) { TrafficLight::direction = dir; }

bool TrafficLight::isActive() const { return active; }

void TrafficLight::setActive(bool ac) { TrafficLight::active = ac; }

void TrafficLight::addCycleElement(TrafficLightCycleElement ce) { cycle.push_back(ce); }

