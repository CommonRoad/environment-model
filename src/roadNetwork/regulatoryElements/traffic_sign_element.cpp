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
        if (country == SupportedTrafficSignCountry::GERMANY)
            return convertGermanTrafficSignIdToString(TrafficSignIDGermany::GREEN_ARROW);
    return "206"; // Stop sign in Germany
}

std::string TrafficSignElement::convertGermanTrafficSignIdToString(TrafficSignIDGermany signId) {
    if (signId == TrafficSignIDGermany::GREEN_ARROW)
        return "720";
    else
        return "206"; // Stop sign in Germany
}