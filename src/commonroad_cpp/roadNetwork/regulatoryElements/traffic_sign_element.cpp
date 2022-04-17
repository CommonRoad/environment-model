//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "traffic_sign_element.h"
#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>

#include <utility>

TrafficSignElement::TrafficSignElement(std::string el_id, std::vector<std::string> values)
    : id(std::move(el_id)), additionalValues(std::move(values)) {}

std::string TrafficSignElement::getId() const { return id; }

std::vector<std::string> TrafficSignElement::getAdditionalValues() const { return additionalValues; }

void TrafficSignElement::addAdditionalValue(const std::string &value) { additionalValues.push_back(value); }

void TrafficSignElement::setAdditionalValues(const std::vector<std::string> &values) { additionalValues = values; }

std::string TrafficSignElement::convertGermanTrafficSignIdToString(TrafficSignTypes signId) {
    return TrafficSignIDGermany.at(signId);
}