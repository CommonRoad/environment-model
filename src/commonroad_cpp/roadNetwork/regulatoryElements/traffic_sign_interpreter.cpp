//
// Created by Bowen Wu.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "traffic_sign_interpreter.h"
#include <set>
#include <vector>
TrafficSignInterpreter::TrafficSignInterpreter(SupportedTrafficSignCountry ruleCountry) : country(ruleCountry) {
    trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(ruleCountry);
}
double TrafficSignInterpreter::speedLimit(const Lanelet &lanelet) {
    double limit = 3e8; // Very large value as default
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet.getTrafficSigns();
    for (std::shared_ptr<TrafficSign> signPtr : trafficSigns) {
        for (std::shared_ptr<TrafficSignElement> elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getId() == trafficSignIDLookupTable->at(TrafficSignTypes::MAX_SPEED)) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit > signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double TrafficSignInterpreter::speedLimit(const std::set<size_t> &lanelet_ids, std::shared_ptr<RoadNetwork> network) {
    std::vector<double> speedLimits;
    for (const auto &laneletId : lanelet_ids) {
        std::shared_ptr<Lanelet> lanelet = network->findLaneletById(laneletId);
        speedLimits.push_back(speedLimit(*lanelet));
    }
    return *std::min_element(speedLimits.begin(), speedLimits.end());
}

double TrafficSignInterpreter::requiredSpeed(const std::set<size_t> &lanelet_ids,
                                             std::shared_ptr<RoadNetwork> network) {
    std::vector<double> speedLimits;
    for (const auto &laneletId : lanelet_ids) {
        std::shared_ptr<Lanelet> lanelet = network->findLaneletById(laneletId);
        speedLimits.push_back(speedLimit(*lanelet));
    }
    return *std::max_element(speedLimits.begin(), speedLimits.end());
}

double TrafficSignInterpreter::requiredSpeed(const Lanelet &lanelet) {
    double limit = 0;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet.getTrafficSigns();
    for (std::shared_ptr<TrafficSign> signPtr : trafficSigns) {
        for (std::shared_ptr<TrafficSignElement> elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getId() == trafficSignIDLookupTable->at(TrafficSignTypes::MIN_SPEED)) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit < signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}
