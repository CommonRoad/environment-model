#include "commonroad_cpp/roadNetwork/intersection/outgoing_group.h"

OutgoingGroup::OutgoingGroup(size_t groupID, std::vector<std::shared_ptr<Lanelet>> &outgoingLanelets)
    : groupID(groupID), outgoingLanelets(outgoingLanelets) {}

void OutgoingGroup::setId(size_t outId) { groupID = outId; }

size_t OutgoingGroup::getId() const { return groupID; }

void OutgoingGroup::addOutgoingLanelet(std::shared_ptr<Lanelet> &lanelet) { outgoingLanelets.push_back(lanelet); }

std::vector<std::shared_ptr<Lanelet>> OutgoingGroup::getOutgoingLanelets() { return outgoingLanelets; }
