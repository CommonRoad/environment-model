#include "commonroad_cpp/roadNetwork/intersection/outgoing_group.h"

OutgoingGroup::OutgoingGroup(size_t groupID, std::vector<std::shared_ptr<Lanelet>> &outgoingLanelets)
    : groupID(groupID), outgoingLanelets(outgoingLanelets) {}
