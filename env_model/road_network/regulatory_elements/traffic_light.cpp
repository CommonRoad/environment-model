//
// Created by Sebastian Maierhofer on 31.10.20.
//

#include "traffic_light.h"

TrafficLight::TrafficLight() {
    id = 0;
    offset = 0;
    direction = all;
    active = true;
}


// Setter
void TrafficLight::setId(const size_t num) { id = num; }

void TrafficLight::setCycle(const std::vector<CycleElement>& light_cycle) { cycle = light_cycle; }

void TrafficLight::setOffset(const int ofst) { offset = ofst; }

// Getter
size_t TrafficLight::getId() const { return id; }
float TrafficLight::getOffset() const { return offset; }

std::vector<CycleElement> TrafficLight::getCycle() const { return cycle; }

// functions
CycleElement TrafficLight::getElementAtTime(float time) {
    time += offset;
    CycleElement current = cycle.front();
    size_t i = 0;
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

TrafficLightDirection TrafficLight::getDirection() const {
    return direction;
}

void TrafficLight::setDirection(TrafficLightDirection dir) {
    TrafficLight::direction = dir;
}

bool TrafficLight::isActive() const {
    return active;
}

void TrafficLight::setActive(bool ac) {
    TrafficLight::active = ac;
}

void TrafficLight::addCycleElement(CycleElement ce){
    cycle.push_back(ce);
}
