//
// Created by sebastian on 31.10.20.
//

#include "traffic_sign.h"

TrafficSign::TrafficSign() {
    id = 0;
    virtualElement = false;
    position.x = 0.0;
    position.y = 0.0;
}



// Setter
void TrafficSign::setId(const size_t num) { id = num; }
void TrafficSign::addTrafficSignElement(TrafficSignElement elem) { trafficSignElement.push_back(elem); }


// Getter
size_t TrafficSign::getId() const { return id; }



void TrafficSign::setTrafficSignElement(const std::vector<TrafficSignElement> newTrafficSignElement) {
    trafficSignElement = newTrafficSignElement;
}

bool TrafficSign::isVirtualElement() const {
    return virtualElement;
}

void TrafficSign::setVirtualElement(bool elem) {
    virtualElement = elem;
}

const vertice &TrafficSign::getPosition() const {
    return position;
}

void TrafficSign::setPosition(vertice pos) {
    TrafficSign::position = pos;
}

const std::vector<TrafficSignElement> TrafficSign::getTrafficSignElement() const {
    return trafficSignElement;
}

