//
// Created by Sebastian Maierhofer on 09.11.20.
//

#ifndef ENV_MODEL_LANELET_OPERATIONS_H
#define ENV_MODEL_LANELET_OPERATIONS_H

#include "../../auxiliaryDefs/types.h"
#include "lanelet.h"
#include "lane.h"

LaneletType matchStringToLaneletType(const char *type);

LineMarking matchStringToLineMarking(const char *type);

Lane combineLaneletAndSuccessorsWithSameTypeToLane(std::shared_ptr<Lanelet> curLanelet, LaneletType type);

#endif //ENV_MODEL_LANELET_OPERATIONS_H
