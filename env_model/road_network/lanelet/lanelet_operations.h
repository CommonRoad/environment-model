//
// Created by Sebastian Maierhofer on 09.11.20.
//

#ifndef ENV_MODEL_LANELET_OPERATIONS_H
#define ENV_MODEL_LANELET_OPERATIONS_H

#include "../../auxiliaryDefs/types.h"
#include "lanelet.h"

LaneletType matchLaneletTypeToString(const char *type);

LineMarking matchLineMarkingToString(const char *type);

void combineLaneletAndSuccessors(std::vector<Lanelet> *laneList, Lanelet *curLanelet, size_t k);

#endif //ENV_MODEL_LANELET_OPERATIONS_H
