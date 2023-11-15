//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <algorithm>

#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/predicates/predicate_config.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

std::set<std::shared_ptr<TrafficLight>>
regulatory_elements_utils::activeTrafficLights(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                               const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::set<std::shared_ptr<TrafficLight>> trafficLights;
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &lanelet : lanelets) {
        for (const auto &light : lanelet->getTrafficLights()) {
            if (light->isActive() and light->getElementAtTime(timeStep).color != TrafficLightState::inactive)
                trafficLights.insert(light);
        }
    }
    return trafficLights;
}

bool regulatory_elements_utils::atTrafficLightDirState(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                                       const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                       TurningDirection turnDir, TrafficLightState tlState) {
    if (!intersection_operations::onIncoming(timeStep, obs, roadNetwork))
        return false;
    std::unordered_set<TurningDirection> relevantTrafficLightDirections;
    switch (turnDir) {
    case TurningDirection::left:
        relevantTrafficLightDirections = {TurningDirection::left, TurningDirection::leftStraight,
                                          TurningDirection::leftRight, TurningDirection::all};
        break;
    case TurningDirection::right:
        relevantTrafficLightDirections = {TurningDirection::leftRight, TurningDirection::right,
                                          TurningDirection::straightRight, TurningDirection::all};
        break;
    case TurningDirection::straight:
        relevantTrafficLightDirections = {TurningDirection::straight, TurningDirection::straightRight,
                                          TurningDirection::leftStraight, TurningDirection::all};
        break;
    case TurningDirection::all:
        relevantTrafficLightDirections = {TurningDirection::left,      TurningDirection::leftStraight,
                                          TurningDirection::leftRight, TurningDirection::all,
                                          TurningDirection::right,     TurningDirection::straightRight,
                                          TurningDirection::straight};
        break;
    default:
        relevantTrafficLightDirections = {TurningDirection::left,      TurningDirection::leftStraight,
                                          TurningDirection::leftRight, TurningDirection::all,
                                          TurningDirection::right,     TurningDirection::straightRight,
                                          TurningDirection::straight};
    }
    auto activeTl{activeTrafficLights(timeStep, obs, roadNetwork)};
    for (const auto &light : activeTl) {
        if (std::any_of(relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                        [light](const TurningDirection &relevantDirection) {
                            return relevantDirection == light->getDirection();
                        }) and
            light->getElementAtTime(timeStep).color == tlState)
            return true;
    }
    return false;
}

double regulatory_elements_utils::speedLimit(const std::shared_ptr<Lanelet> &lanelet,
                                             const TrafficSignTypes &signType) {
    double limit = PredicateParameters().paramMap["maxPositiveDouble"];
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (const std::shared_ptr<TrafficSign> &signPtr : trafficSigns) {
        for (const std::shared_ptr<TrafficSignElement> &elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getTrafficSignType() == signType) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit > signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double regulatory_elements_utils::speedLimit(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                             const TrafficSignTypes &signType) {
    std::vector<double> speedLimits{std::numeric_limits<double>::max()};
    for (const auto &lanelet : lanelets) {
        speedLimits.push_back(speedLimit(lanelet, signType));
    }
    return *std::min_element(speedLimits.begin(), speedLimits.end());
}

double regulatory_elements_utils::speedLimitSuggested(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                      const TrafficSignTypes &signType) {
    double vMaxLane{speedLimit(lanelets, signType)};
    if (vMaxLane == PredicateParameters().paramMap["maxPositiveDouble"])
        return PredicateParameters().paramMap["desiredInterstateVelocity"];
    else
        return std::min(PredicateParameters().paramMap["desiredInterstateVelocity"], vMaxLane);
}

double regulatory_elements_utils::requiredVelocity(const std::shared_ptr<Lanelet> &lanelet,
                                                   const TrafficSignTypes &signType) {
    double limit = std::numeric_limits<double>::lowest();
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (const std::shared_ptr<TrafficSign> &signPtr : trafficSigns) {
        for (const std::shared_ptr<TrafficSignElement> &elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getTrafficSignType() == signType) {
                double signLimit = std::stod(elemPtr->getAdditionalValues()[0]);
                if (limit < signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double regulatory_elements_utils::requiredVelocity(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                   const TrafficSignTypes &signType) {
    std::vector<double> speedLimits{};
    for (const auto &lanelet : lanelets) {
        speedLimits.push_back(requiredVelocity(lanelet, signType));
    }
    if (speedLimits.empty())
        speedLimits.push_back(std::numeric_limits<double>::lowest()); // prevent error if no lanelet is provided
    return *std::max_element(speedLimits.begin(), speedLimits.end());
}

double regulatory_elements_utils::typeSpeedLimit(ObstacleType obstacleType) {
    switch (obstacleType) {
    case ObstacleType::truck:
        return 22.22;
    default:
        return std::numeric_limits<double>::max();
    }
}

std::vector<std::string> regulatory_elements_utils::getRelevantPrioritySignIDs() {
    std::vector<std::string> keys;
    keys.reserve(priorityTable.size());
    for (const auto &[key, value] : priorityTable) {
        keys.push_back(key);
    }
    return keys;
}

std::shared_ptr<TrafficSignElement>
regulatory_elements_utils::extractPriorityTrafficSign(const std::shared_ptr<Lanelet> &lanelet) {
    static const std::vector<std::string> relevantPrioritySignIds{getRelevantPrioritySignIDs()};
    std::vector<std::shared_ptr<TrafficSignElement>> relevantTrafficSignElements;
    for (const auto &trs : lanelet->getTrafficSigns()) {
        auto trafficSignElements{trs->getTrafficSignElements()};
        relevantTrafficSignElements.insert(relevantTrafficSignElements.end(), trafficSignElements.begin(),
                                           trafficSignElements.end());
    }
    std::shared_ptr<TrafficSignElement> tmpSign{
        std::make_shared<TrafficSignElement>(TrafficSignTypes::WARNING_RIGHT_BEFORE_LEFT)};
    for (const auto &tse : relevantTrafficSignElements) {
        if (!std::any_of(relevantTrafficSignElements.begin(), relevantTrafficSignElements.end(),
                         [tse](const std::shared_ptr<TrafficSignElement> &tel) {
                             return tel->getTrafficSignType() == tse->getTrafficSignType();
                         }))
            continue;
        if (tmpSign->getTrafficSignType() != TrafficSignTypes::WARNING_RIGHT_BEFORE_LEFT and
            (tse->getTrafficSignType() == TrafficSignTypes::PRIORITY or
             tse->getTrafficSignType() == TrafficSignTypes::YIELD))
            continue;
        tmpSign = tse;
    }
    return tmpSign;
}

int regulatory_elements_utils::extractPriorityTrafficSign(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                          TurningDirection dir) {
    std::shared_ptr<TrafficSignElement> tmpSign;
    int currentPriorityValue{std::numeric_limits<int>::min()};
    for (const auto &let : lanelets) {
        auto trs{regulatory_elements_utils::extractPriorityTrafficSign(let)};
        // TODO replace TrafficSignIDGermany.at(
        if (priorityTable.count(TrafficSignIDGermany.at(trs->getTrafficSignType())) == 1 and
            (currentPriorityValue == std::numeric_limits<int>::min() or
             priorityTable.at(TrafficSignIDGermany.at(trs->getTrafficSignType())).at(static_cast<size_t>(dir)) <
                 currentPriorityValue))
            currentPriorityValue =
                priorityTable.at(TrafficSignIDGermany.at(trs->getTrafficSignType())).at(static_cast<size_t>(dir));
    }
    return currentPriorityValue;
}

int regulatory_elements_utils::getPriority(size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                           const std::shared_ptr<Obstacle> &obs, TurningDirection dir) {
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lanelet>> relevantIncomingLanelets;
    for (const auto &let : lanelets) {
        if (let->hasLaneletType(LaneletType::incoming))
            relevantIncomingLanelets.push_back(let);
    }
    return regulatory_elements_utils::extractPriorityTrafficSign(relevantIncomingLanelets, dir);
}

TrafficSignTypes regulatory_elements_utils::extractTypeFromNationalID(const std::string &trafficSignId,
                                                                      SupportedTrafficSignCountry country,
                                                                      const std::string &country_string) {
    if (country == SupportedTrafficSignCountry::GERMANY or country == SupportedTrafficSignCountry::ZAMUNDA) {
        for (const auto &countrySign : TrafficSignIDGermany)
            if (countrySign.second == trafficSignId)
                return countrySign.first;
    } else if (country == SupportedTrafficSignCountry::USA) {
        for (const auto &countrySign : TrafficSignIDUSA)
            if (countrySign.second == trafficSignId)
                return countrySign.first;
    } else if (country == SupportedTrafficSignCountry::SPAIN) {
        for (const auto &countrySign : TrafficSignIDSpain)
            if (countrySign.second == trafficSignId)
                return countrySign.first;
    } else if (country == SupportedTrafficSignCountry::ARGENTINA) {
        for (const auto &countrySign : TrafficSignIDArgentina)
            if (countrySign.second == trafficSignId)
                return countrySign.first;
    } else if (country == SupportedTrafficSignCountry::BELGIUM) {
        for (const auto &countrySign : TrafficSignIDBelgium)
            if (countrySign.second == trafficSignId)
                return countrySign.first;
    } else
        throw std::runtime_error("ProtobufReader::createTrafficSignElementFromMessage: Unknown country ID " +
                                 country_string);
    throw std::runtime_error("ProtobufReader::createTrafficSignElementFromMessage: Unknown traffic sign ID " +
                             trafficSignId + " in country " + country_string);
}