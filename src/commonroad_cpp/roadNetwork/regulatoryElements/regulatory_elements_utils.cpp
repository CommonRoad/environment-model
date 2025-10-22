#include <algorithm>

#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/predicate_parameter_collection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <geometry/curvilinear_coordinate_system.h>
#include <unordered_set>

std::set<std::shared_ptr<TrafficLight>>
regulatory_elements_utils::activeTrafficLights(const size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                               const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::set<std::shared_ptr<TrafficLight>> trafficLights;
    const auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &lanelet : lanelets) {
        for (const auto &light : lanelet->getTrafficLights()) {
            if (light->isActive() and light->getElementAtTime(timeStep).color != TrafficLightState::inactive)
                trafficLights.insert(light);
        }
    }
    return trafficLights;
}

bool regulatory_elements_utils::atTrafficLightDirState(const size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                                       const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                       const Direction turnDir, const TrafficLightState tlState) {
    if (!intersection_operations::onIncoming(timeStep, obs, roadNetwork))
        return false;
    std::unordered_set<Direction> relevantTrafficLightDirections;
    switch (turnDir) {
    case Direction::left:
        relevantTrafficLightDirections = {Direction::left, Direction::leftStraight, Direction::leftRight,
                                          Direction::all};
        break;
    case Direction::right:
        relevantTrafficLightDirections = {Direction::leftRight, Direction::right, Direction::straightRight,
                                          Direction::all};
        break;
    case Direction::straight:
        relevantTrafficLightDirections = {Direction::straight, Direction::straightRight, Direction::leftStraight,
                                          Direction::all};
        break;
    case Direction::all:
        relevantTrafficLightDirections = {Direction::left,    Direction::leftStraight, Direction::leftRight,
                                          Direction::all,     Direction::right,        Direction::straightRight,
                                          Direction::straight};
        break;
    default:
        relevantTrafficLightDirections = {Direction::left,    Direction::leftStraight, Direction::leftRight,
                                          Direction::all,     Direction::right,        Direction::straightRight,
                                          Direction::straight};
    }
    const auto activeTl{activeTrafficLights(timeStep, obs, roadNetwork)};
    for (const auto &light : activeTl) {
        if (std::any_of(
                relevantTrafficLightDirections.begin(), relevantTrafficLightDirections.end(),
                [light](const Direction &relevantDirection) { return relevantDirection == light->getDirection(); }) and
            light->getElementAtTime(timeStep).color == tlState)
            return true;
    }
    return false;
}

double regulatory_elements_utils::speedLimit(const std::shared_ptr<Lanelet> &lanelet,
                                             const TrafficSignTypes &signType) {
    double limit = std::numeric_limits<double>::max();
    const std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (const std::shared_ptr<TrafficSign> &signPtr : trafficSigns) {
        for (const std::shared_ptr<TrafficSignElement> &elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getTrafficSignType() == signType) {
                if (const double signLimit = std::stod(elemPtr->getAdditionalValues()[0]); limit > signLimit)
                    limit = signLimit;
            }
        }
    }
    return limit;
}

double regulatory_elements_utils::speedLimit(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                             const TrafficSignTypes &signType) {
    std::vector speedLimits{std::numeric_limits<double>::max()};
    for (const auto &lanelet : lanelets) {
        speedLimits.push_back(speedLimit(lanelet, signType));
    }
    return *std::min_element(speedLimits.begin(), speedLimits.end());
}

double regulatory_elements_utils::speedLimitSuggested(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                      const TrafficSignTypes &signType,
                                                      const double desiredInterstateVelocity,
                                                      const double desiredUrbanVelocity) {
    const double vMaxLane{speedLimit(lanelets, signType)};
    if (vMaxLane == std::numeric_limits<double>::max()) {
        if (std::any_of(lanelets.begin(), lanelets.end(),
                        [](const std::shared_ptr<Lanelet> &let) { return let->hasLaneletType(LaneletType::urban); }))
            return desiredUrbanVelocity;
        if (std::any_of(lanelets.begin(), lanelets.end(), [](const std::shared_ptr<Lanelet> &let) {
                return let->hasLaneletType(LaneletType::interstate);
            }))
            return desiredInterstateVelocity;
        return desiredUrbanVelocity;
    }
    return vMaxLane;
}

double regulatory_elements_utils::requiredVelocity(const std::shared_ptr<Lanelet> &lanelet,
                                                   const TrafficSignTypes &signType) {
    double limit = std::numeric_limits<double>::lowest();
    const std::vector<std::shared_ptr<TrafficSign>> trafficSigns = lanelet->getTrafficSigns();
    for (const std::shared_ptr<TrafficSign> &signPtr : trafficSigns) {
        for (const std::shared_ptr<TrafficSignElement> &elemPtr : signPtr->getTrafficSignElements()) {
            if (elemPtr->getTrafficSignType() == signType) {
                if (const double signLimit = std::stod(elemPtr->getAdditionalValues()[0]); limit < signLimit)
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

double regulatory_elements_utils::typeSpeedLimit(const ObstacleType obstacleType) {
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
    static const std::vector relevantPrioritySignIds{getRelevantPrioritySignIDs()};
    std::vector<std::shared_ptr<TrafficSignElement>> relevantTrafficSignElements;
    for (const auto &trs : lanelet->getTrafficSigns()) {
        auto trafficSignElements{trs->getTrafficSignElements()};
        for (const auto &elem : trafficSignElements) {
            // Only insert if the traffic sign element type is in relevantPrioritySignIds
            if (std::find(relevantPrioritySignIds.begin(), relevantPrioritySignIds.end(),
                          TrafficSignIDGermany.at(elem->getTrafficSignType())) != relevantPrioritySignIds.end()) {
                relevantTrafficSignElements.push_back(elem);
            }
        }
    }
    auto tmpSign{std::make_shared<TrafficSignElement>(TrafficSignTypes::WARNING_RIGHT_BEFORE_LEFT)};
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
                                                          Direction dir) {
    std::shared_ptr<TrafficSignElement> tmpSign;
    int currentPriorityValue{std::numeric_limits<int>::min()};
    for (const auto &let : lanelets) {
        // TODO replace TrafficSignIDGermany.at(
        if (const auto trs{extractPriorityTrafficSign(let)};
            priorityTable.count(TrafficSignIDGermany.at(trs->getTrafficSignType())) == 1 and
            (currentPriorityValue == std::numeric_limits<int>::min() or
             priorityTable.at(TrafficSignIDGermany.at(trs->getTrafficSignType())).at(static_cast<size_t>(dir)) <
                 currentPriorityValue))
            currentPriorityValue =
                priorityTable.at(TrafficSignIDGermany.at(trs->getTrafficSignType())).at(static_cast<size_t>(dir));
    }
    return currentPriorityValue;
}

int regulatory_elements_utils::getPriority(const size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                           const std::shared_ptr<Obstacle> &obs, const Direction dir) {
    const auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lanelet>> relevantIncomingLanelets;
    for (const auto &let : lanelets) {
        if (let->hasLaneletType(LaneletType::incoming))
            relevantIncomingLanelets.push_back(let);
    }
    return extractPriorityTrafficSign(relevantIncomingLanelets, dir);
}

TrafficSignTypes regulatory_elements_utils::extractTypeFromNationalID(const std::string &trafficSignId,
                                                                      const SupportedTrafficSignCountry country,
                                                                      const std::string &country_string) {
    if (trafficSignId == "274") // some old scenarios use German ID for other countries.
        return TrafficSignTypes::MAX_SPEED;
    if (country == SupportedTrafficSignCountry::GERMANY or country == SupportedTrafficSignCountry::ZAMUNDA) {
        for (const auto &[fst, snd] : TrafficSignIDGermany)
            if (snd == trafficSignId)
                return fst;
    } else if (country == SupportedTrafficSignCountry::USA) {
        for (const auto &[fst, snd] : TrafficSignIDUSA)
            if (snd == trafficSignId)
                return fst;
    } else if (country == SupportedTrafficSignCountry::SPAIN) {
        for (const auto &[fst, snd] : TrafficSignIDSpain)
            if (snd == trafficSignId)
                return fst;
    } else if (country == SupportedTrafficSignCountry::ARGENTINA) {
        for (const auto &[fst, snd] : TrafficSignIDArgentina)
            if (snd == trafficSignId)
                return fst;
    } else if (country == SupportedTrafficSignCountry::BELGIUM) {
        for (const auto &[fst, snd] : TrafficSignIDBelgium)
            if (snd == trafficSignId)
                return fst;
    } else if (country == SupportedTrafficSignCountry::AUSTRALIA) {
        for (const auto &[fst, snd] : TrafficSignIDAustralia)
            if (snd == trafficSignId)
                return fst;
    } else
        throw std::runtime_error("regulatory_elements_utils::extractTypeFromNationalID: Unknown country ID " +
                                 country_string);
    throw std::runtime_error("regulatory_elements_utils::extractTypeFromNationalID: Unknown traffic sign ID " +
                             trafficSignId + " in country " + country_string);
}

bool regulatory_elements_utils::lineInFront(const std::vector<vertex> &line, const std::shared_ptr<Obstacle> &obs,
                                            const size_t timeStep, const std::shared_ptr<RoadNetwork> &roadNetwork) {
    const auto ccs{obs->getReferenceLane(roadNetwork, timeStep)->getCurvilinearCoordinateSystem()};
    if (!ccs->cartesianPointInProjectionDomain(line.at(0).x, line.at(0).y))
        return false;
    auto a{ccs->convertToCurvilinearCoords(line.at(0).x, line.at(0).y)};
    if (!ccs->cartesianPointInProjectionDomain(line.at(1).x, line.at(1).y))
        return false;
    auto b{ccs->convertToCurvilinearCoords(line.at(1).x, line.at(1).y)};
    const multi_polygon_type shapes{obs->getOccupancyPolygonShape(timeStep)};
    for (const auto &shape : shapes)
        for (size_t idx{0}; idx < shape.outer().size(); ++idx) {
            auto point{shape.outer().at(idx)};
            if (!ccs->cartesianPointInProjectionDomain(point.x(), point.y()))
                return false;
            auto c{ccs->convertToCurvilinearCoords(point.x(), point.y())};
            // ((b.X - a.X)*(c.Y - a.Y) > (b.Y - a.Y)*(c.X - a.X)) -> left of line a.y < b.y
            if (a.y() < 0) {
                if ((b.x() - a.x()) * (c.y() - a.y()) < (b.y() - a.y()) * (c.x() - a.x()))
                    return false;
            } else {
                if ((b.x() - a.x()) * (c.y() - a.y()) > (b.y() - a.y()) * (c.x() - a.x()))
                    return false;
            }
        }
    return true;
}

double regulatory_elements_utils::minDistance(const std::vector<vertex> &line, polygon_type shape) {
    const auto a{(line.at(0).y - line.at(1).y)};
    const auto b{line.at(1).x - line.at(0).x};
    const auto c{(line.at(0).x - line.at(1).x) * line.at(0).y + (line.at(1).y - line.at(0).y) * line.at(0).x};
    std::vector<double> distances;
    for (const auto &point : shape.outer()) {
        distances.push_back(std::fabs(a * point.x() + b * point.y() + c) / sqrt(a * a + b * b));
    }
    return *std::min_element(distances.begin(), distances.end());
}

Direction regulatory_elements_utils::matchDirections(const std::string &dir) {
    std::string str{dir};
    std::transform(str.begin(), str.end(), str.begin(), toupper);
    str.erase(remove(str.begin(), str.end(), '_'), str.end());
    if (DirectionNames.count(str) == 1)
        return DirectionNames.at(str);
    throw std::logic_error("regulatory_elements_utils::matchDirections: Invalid turning direction state '" + str +
                           "'!");
}
