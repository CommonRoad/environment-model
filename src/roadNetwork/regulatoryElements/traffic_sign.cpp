//
// Created by Sebastian Maierhofer on 31.10.20.
//

#include "traffic_sign.h"

void TrafficSign::setId(const int num) { id = num; }

void TrafficSign::addTrafficSignElement(const std::shared_ptr<TrafficSignElement> &elem) {
    trafficSignElement.push_back(elem);
}

int TrafficSign::getId() const { return id; }

void TrafficSign::setTrafficSignElement(const std::vector<std::shared_ptr<TrafficSignElement>> &newTrafficSignElement) {
    trafficSignElement = newTrafficSignElement;
}

bool TrafficSign::isVirtualElement() const {
    return virtualElement;
}

void TrafficSign::setVirtualElement(bool elem) {
    virtualElement = elem;
}

std::vector<std::shared_ptr<TrafficSignElement>> TrafficSign::getTrafficSignElements() const {
    return trafficSignElement;
}

void TrafficSign::setPosition(vertex pos) { position = pos; }

vertex TrafficSign::getPosition() const { return position; }