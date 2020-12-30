//
// Created by Sebastian Maierhofer on 31.10.20.
//

#include "traffic_sign.h"

TrafficSign::TrafficSign() {
    id = 0;
    virtualElement = false;
}

// Setter
void TrafficSign::setId(const int num) { id = num; }

void TrafficSign::addTrafficSignElement(const TrafficSignElement &elem) { trafficSignElement.push_back(elem); }


// Getter
int TrafficSign::getId() const { return id; }


void TrafficSign::setTrafficSignElement(const std::vector<TrafficSignElement> &newTrafficSignElement) {
    trafficSignElement = newTrafficSignElement;
}

bool TrafficSign::isVirtualElement() const {
    return virtualElement;
}

void TrafficSign::setVirtualElement(bool elem) {
    virtualElement = elem;
}

std::vector<TrafficSignElement> TrafficSign::getTrafficSignElement() const {
    return trafficSignElement;
}

