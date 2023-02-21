#pragma once



namespace obstacle_reference {

std::vector<std::shared_ptr<Lane>> computeRef(Obstacle& obstacle, const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep);

std::vector<std::shared_ptr<Lane>> computeRefGivenLanes(Obstacle& obstacle, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                      size_t timeStep,
                                                                      const std::vector<std::shared_ptr<Lane>> &lanes);
std::vector<std::shared_ptr<Lane>> countOccupanciesOverTime(Obstacle &obstacle,
                                            const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                                            std::vector<std::shared_ptr<Lane>> &relevantOccupiedLanes);
};
