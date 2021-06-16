#ifndef ENV_MODEL_TRAFFICSIGNINTERPRETER_H
#define ENV_MODEL_TRAFFICSIGNINTERPRETER_H

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <memory>
#include <set>

class TrafficSignInterpreter {
  public:
    TrafficSignInterpreter(SupportedTrafficSignCountry ruleCountry);

    /**
     * Extracts the maximum speed limit of provided lanelets
     * @note
     * @param lanelet_ids set of lanelets which should be considered
     * @param roadnetwork roadNetwork that should contain lanelets
     * @return speed limit of provided lanelets or 3e8 if no limit exists
     */
    double speedLimit(const std::set<int> &lanelet_ids, std::shared_ptr<RoadNetwork> network);

    /**
     * Extracts the maximum speed limit of provided lanelet
     * @param lanelet lanelet whose speed_limit is to be determined.
     * @return speed limit of provided lanelets or 3e8 if no limit exists
     */
    double speedLimit(const Lanelet &lanelet);

    /**
     * Extracts the required minimum speed of provided lanelets
     * @param lanelet_ids set of lanelets which should be considered
     * @param roadnetwork roadNetwork that should contain lanelets
     * @return speed limit of provided lanelets or 0 if no limit exists
     */
    double requiredSpeed(const std::set<int> &lanelet_ids, std::shared_ptr<RoadNetwork> network);

    /**
     * Extracts the required minimum speed of provided lanelet
     * @param lanelet lanelet whose speed_limit is to be determined.
     * @return speed limit of provided lanelets or 0 if no limit exists
     */
    double requiredSpeed(const Lanelet &lanelet);

  private:
    SupportedTrafficSignCountry country;
    const std::unordered_map<TrafficSignTypes, std::string> *trafficSignIDLookupTable;
    std::shared_ptr<RoadNetwork> roadNetwork;
};

#endif // ENV_MODEL_TRAFFICSIGNINTERPRETER_H
