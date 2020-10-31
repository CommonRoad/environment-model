//
// Created by sebastian on 31.10.20.
//

#include "traffic_sign_element.h"

#include <utility>

TrafficSignElement::TrafficSignElement(std::string el_id) : id(std::move(el_id)) {}

std::string TrafficSignElement::getId() const {
    return id;
}

std::vector<std::string> TrafficSignElement::getAdditionalValue() const {
    return additional_value;
}

void TrafficSignElement::addAdditionalValue(const std::string& additionalValue) {
    additional_value.push_back(additionalValue);
}
