//
// Created by Sebastian Maierhofer on 09.11.20.
//

#ifndef ENV_MODEL_LANELET_OPERATIONS_H
#define ENV_MODEL_LANELET_OPERATIONS_H

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "lane.h"
#include "lanelet.h"

/**
 * Matches a string to a lanelet type
 *
 * @param string for which lanelet type should be extracted
 * @return lanelet type which corresponds to string or unknown type if string does not match
 */
LaneletType matchStringToLaneletType(const std::string &type);

/**
 * Matches a string to a line marking
 *
 * @param string for which line marking should be extracted
 * @return line marking which corresponds to string or unknown type if string does not match
 */
LineMarking matchStringToLineMarking(const std::string &type);

/**
 * Iteratively concatenates lanelets with same lanelet type given a start lanelet and creates a lane out of them
 *
 * @param curLanelet start lanelet for which concatenation should be started
 * @param type specifies the lanelet type the concatenated lanelets should have in common
 * @return lane spanned by concatenated lanelets
 */
std::shared_ptr<Lane> combineLaneletAndSuccessorsWithSameTypeToLane(std::shared_ptr<Lanelet> curLanelet,
                                                                    LaneletType type);

#endif // ENV_MODEL_LANELET_OPERATIONS_H
