//
// Created by wilhelm on 5/16/21.
//

#include "traffic_sign_interpreter.h"
#include <set>
#include <vector>
TrafficSignInterpreter::TrafficSignInterpreter(SupportedTrafficSignCountry ruleCountry) : country(ruleCountry) {
    trafficSignIDLookupTable = TrafficSignLookupTableByCountry.at(ruleCountry);
    //    switch (ruleCountry) {
    //    case SupportedTrafficSignCountry::GERMANY:
    //        trafficSignIDLookupTable = &TrafficSignIDGermany;
    //        break;
    //    }
}
double TrafficSignInterpreter::speedLimit(const Lanelet &lanelet) {
    double limit = 3e8; // TODO maybe another return value?
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet.getTrafficSigns();
    for (std::shared_ptr<TrafficSign> signptr : trafficSigns) {
        for (std::shared_ptr<TrafficSignElement> elemptr : signptr->getTrafficSignElements()) {
            if (elemptr->getId() == trafficSignIDLookupTable->at(TrafficSignTypes::MAX_SPEED)) {
                int signLimit = std::stoi(elemptr->getAdditionalValues()[0]);
                if (limit > signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double TrafficSignInterpreter::speedLimit(const std::set<int> &lanelet_ids, std::shared_ptr<RoadNetwork> roadNetwork) {
    std::vector<double> speedLimits;
    for (int lanelet_id : lanelet_ids) {
        std::shared_ptr<Lanelet> lanelet = roadNetwork->findLaneletById(lanelet_id);
        speedLimits.push_back(speedLimit(*lanelet));
    }
    return *std::min_element(speedLimits.begin(), speedLimits.end());
}

double TrafficSignInterpreter::requiredSpeed(const std::set<int> &lanelet_ids, std::shared_ptr<RoadNetwork> network) {
    std::vector<double> speedLimits;
    for (int lanelet_id : lanelet_ids) {
        std::shared_ptr<Lanelet> lanelet = roadNetwork->findLaneletById(lanelet_id);
        speedLimits.push_back(speedLimit(*lanelet));
    }
    return *std::max_element(speedLimits.begin(), speedLimits.end());
}

double TrafficSignInterpreter::requiredSpeed(const Lanelet &lanelet) {

    double limit = 0;
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet.getTrafficSigns();
    for (std::shared_ptr<TrafficSign> signptr : trafficSigns) {
        for (std::shared_ptr<TrafficSignElement> elemptr : signptr->getTrafficSignElements()) {
            if (elemptr->getId() == trafficSignIDLookupTable->at(TrafficSignTypes::MIN_SPEED)) {
                int signLimit = std::stoi(elemptr->getAdditionalValues()[0]);
                if (limit < signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}
