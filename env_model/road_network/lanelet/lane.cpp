//
// Created by sebastian on 08.11.20.
//

#include "lane.h"

#include <utility>

Lane::Lane(std::vector<std::shared_ptr<Lanelet>> containedLanelets, Lanelet &lanelet, CurvilinearCoordinateSystem &ccs) : containedLanelets(std::move(containedLanelets)),
                                                                             lanelet(lanelet), curvilinearCoordinateSystem(ccs) {}
