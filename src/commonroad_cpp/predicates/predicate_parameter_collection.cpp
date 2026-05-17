#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <commonroad_cpp/predicates/predicate_parameter_collection.h>
#include <stdexcept>

void PredicateParameters::checkParameterValidity() {
    for (auto &[_, val] : parameterCollection) {
        val.checkParameterValidity();
    }
}

void PredicateParameters::updateParam(const std::string &name, double value) {
    // Single find() per map instead of count()+find() twice.
    if (const auto it = parameterCollection.find(name); it != parameterCollection.end()) {
        // tsl::robin_map: mutable value access requires it.value(), not it->second.
        it.value().updateValue(value);
        it.value().checkParameterValidity();
    } else if (const auto cit = constantMap.find(name); cit != constantMap.end()) {
        cit->second = value;
    } else {
        throw std::runtime_error("No predicate " + name + " found for update");
    }
}

double PredicateParameters::getParam(const std::string &name) {
    if (const auto it = parameterCollection.find(name); it != parameterCollection.end())
        return it->second.getValue();
    if (const auto it = constantMap.find(name); it != constantMap.end())
        return it->second;
    throw std::runtime_error("No predicate " + name + " found");
}

std::vector<std::string> PredicateParameters::getPredicateNames() const {
    // robin_map has no sorted iteration; sort explicitly here (not on hot path).
    std::vector<std::string> keys;
    keys.reserve(parameterCollection.size());
    for (const auto &[k, _] : parameterCollection)
        keys.push_back(k);
    std::sort(keys.begin(), keys.end());
    return keys;
}

std::vector<std::string> PredicateParameters::getConstantNames() const {
    std::vector<std::string> keys;
    boost::copy(constantMap | boost::adaptors::map_keys, std::back_inserter(keys));
    std::sort(keys.begin(), keys.end());
    return keys;
}

std::map<std::string, std::tuple<std::string, std::string, double, double, std::vector<std::string>, std::string,
                                 std::string, std::string, std::string, double>>
PredicateParameters::getParameterCollection() const {
    std::map<std::string, std::tuple<std::string, std::string, double, double, std::vector<std::string>, std::string,
                                     std::string, std::string, std::string, double>>
        parameterCollectionTuple;

    for (const auto &[key, value] : parameterCollection)
        parameterCollectionTuple.emplace(key, value.asTuple());

    return parameterCollectionTuple;
}
std::map<std::string, double> PredicateParameters::getParameterCollectionComplete() const {
    std::map<std::string, double> tmpMap(constantMap.begin(), constantMap.end());
    for (const auto &[k, v] : parameterCollection)
        tmpMap.emplace(k, v.getValue());
    return tmpMap;
}

// when using a parameter in a new predicate or adding a new parameter, add it to the yaml file and generate the
// cpp via the Python script
std::map<std::string, PredicateParam> paramMap = {
    {"aBrakingIntersection",
     PredicateParam("aBrakingIntersection", "acceleration of obstacle which we cause to brake [m/s^2]", 0.0, -20.0,
                    {"causes_braking_intersection"}, "acceleration", "greater", "float", "m/s^2", -1.0)},
    {"closeToBicycle", PredicateParam("closeToBicycle", "indicator if vehicle is close to a bicycle [m]", 10.0, 0.0,
                                      {"overtaking_bicycle_same_lane"}, "distance", "greater", "float", "m", 6.0)},
    {"closeToLaneBorder",
     PredicateParam("closeToLaneBorder", "indicator if vehicle is close to lane border [m]", 3.0, 0.0,
                    {"drives_leftmost", "drives_rightmost"}, "distance", "greater_equal", "float", "m", 0.2)},
    {"closeToOtherVehicle",
     PredicateParam("closeToOtherVehicle", "indicator if vehicle is close to another vehicle [m]", 3.0, 0.0,
                    {"drives_leftmost", "drives_rightmost"}, "distance", "greater", "float", "m", 0.5)},
    {"dBrakingIntersection",
     PredicateParam("dBrakingIntersection",
                    "maximum longitudinal distance to obstacle to be considered to cause braking [m]", 50.0, 0.0,
                    {"causes_braking_intersection"}, "distance", "greater_equal", "float", "m", 15.0)},
    {"dCauseBrakingIntersection",
     PredicateParam("dCauseBrakingIntersection",
                    "minimum longitudinal distance to obstacle to be considered to cause braking [m]", 30.0, -10.0,
                    {"causes_braking_intersection"}, "distance", "less_equal", "float", "m", -2.0)},
    {"dCloseToCrossing",
     PredicateParam("dCloseToCrossing", "maximum distance to be close to a intersection crosswalk [m]", 50.0, 0.0,
                    {"close_to_crosswalk"}, "distance", "greater", "float", "m", 20.0)},
    {"intersectionBrakingPossible",
     PredicateParam("intersectionBrakingPossible",
                    "threshold indicating whether braking is possible in an intersection [m/s^2]", 0.0, -20.0,
                    {"braking_at_intersection_possible"}, "acceleration", "none", "float", "m/s^2", -4.0)},
    {"laneMatchingOrientation",
     PredicateParam("laneMatchingOrientation",
                    "orientation threshold for evaluating whether lane-based orientation is similar [rad]", 1.0, 0.0,
                    {"lane_based_orientation_similar"}, "angle", "greater", "float", "rad", 0.35)},
    {"desiredInterstateVelocity",
     PredicateParam("desiredInterstateVelocity",
                    "Suggested maximum velocity on interstates if no speed limit exists [m/s]", 75.0, 15.0,
                    {"slow_leading_vehicle", "preserves_traffic_flow"}, "velocity", "none", "float", "m/s", 36.11)},
    {"desiredUrbanVelocity",
     PredicateParam("desiredUrbanVelocity", "Suggested maximum velocity on urban roads if no speed limit exists [m/s]",
                    75.0, 5.0, {"slow_leading_vehicle", "preserves_traffic_flow"}, "velocity", "none", "float", "m/s",
                    13.89)},
    {"minInterstateWidth",
     PredicateParam("minInterstateWidth", "minimum interstate width so that emergency lane can be created [m]", 20.0,
                    3.0, {"interstate_broad_enough"}, "distance", "less", "float", "m", 7.0)},
    {"minSafetyDistance",
     PredicateParam("minSafetyDistance",
                    "minimum safety distance between two vehicles even computed safe distance would be lower", 10.0,
                    0.0, {"safe_distance_gap_right_violated", "overtaking_bicycle_same_lane"}, "unknown", "none",
                    "float", "unknown", 5.0)},
    {"minVelocityDif",
     PredicateParam("minVelocityDif", "minimum velocity difference [m/s]", 50.0, 0.0,
                    {"preserves_traffic_flow", "slow_leading_vehicle"}, "velocity", "both", "float", "m/s", 15.0)},
    {"narrowRoad", PredicateParam("narrowRoad", "maximum width of road to be called narrow [m]", 10.0, 0.0,
                                  {"narrow_road"}, "distance", "greater_equal", "float", "m", 5.5)},
    {"slightlyHigherSpeedDifference",
     PredicateParam("slightlyHigherSpeedDifference", "indicator for slightly higher speed [m/s]", 10.0, 0.0,
                    {"drives_with_slightly_higher_speed"}, "velocity", "greater", "float", "m/s", 5.55)},
    {"standstillError",
     PredicateParam("standstillError",
                    "velocity deviation from zero which is still classified to be standstill [m/s^2]", 0.5, 0.0,
                    {"in_standstill", "reverses"}, "acceleration", "both", "float", "m/s^2", 0.001)},
    {"stopLineDistance",
     PredicateParam("stopLineDistance", "maximum distance vehicle is seen as in front of stop line [m]", 10.0, 0.0,
                    {"stop_line_in_front"}, "distance", "greater", "float", "m", 5.0)},
    {"closeStopLineDistance",
     PredicateParam("closeStopLineDistance", "maximum distance vehicle waiting in front of stop line has to wait [m]",
                    10.0, 0.0, {"stop_line_in_front"}, "distance", "greater", "float", "m", 1.0)},
    {"uTurnLower", PredicateParam("uTurnLower", "lower angle indicating u-turn on interstates [rad]", 3.14, -3.14,
                                  {"makes_u_turn"}, "angle", "less_equal", "float", "rad", 0.784)},
    {"uTurnUpper", PredicateParam("uTurnUpper", "upper angle indicating u-turn on interstates [rad]", 3.14, -3.14,
                                  {"makes_u_turn"}, "angle", "greater_equal", "float", "rad", 2.357)},
    {"globalInSameDirOrientation",
     PredicateParam("globalInSameDirOrientation",
                    "angle specifying global orientation threshold for not driving in the same direction  [rad]", 1, 0,
                    {"in_same_dir"}, "angle", "less_equal", "float", "rad", 0.3)},
    {"curvilinearInSameDirOrientation",
     PredicateParam("curvilinearInSameDirOrientation",
                    "angle specifying curvilinear orientation threshold for not driving in the same direction [rad]", 1,
                    0, {"in_same_dir"}, "angle", "less_equal", "float", "rad", 0.3)},
};
