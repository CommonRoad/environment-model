//
// Created by Sebastian Maierhofer on 31.10.20.
//

#include "traffic_sign_element.h"

#include <utility>

TrafficSignElement::TrafficSignElement(std::string el_id) : id(std::move(el_id)) {}

std::string TrafficSignElement::getId() const { return id; }

std::vector<std::string> TrafficSignElement::getAdditionalValues() const { return additionalValues; }

void TrafficSignElement::addAdditionalValue(const std::string &value) { additionalValues.push_back(value); }

void TrafficSignElement::setAdditionalValues(const std::vector<std::string> &values) { additionalValues = values; }

std::string TrafficSignElement::mapTrafficSignNameToCountryID(const std::string &trafficSignName,
                                                              SupportedTrafficSignCountry country) {
    if (trafficSignName == "green_arrow")
        return TrafficSignIDGermany.at(TrafficSignTypes::GREEN_ARROW);
    return "206"; // Stop sign in Germany
}

std::string TrafficSignElement::convertGermanTrafficSignIdToString(TrafficSignTypes signId) {
    return TrafficSignIDGermany.at(signId);
}