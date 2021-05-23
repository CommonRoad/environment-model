//
// Created by Sebastian Maierhofer on 21.05.21.
//

#include "utils_predicate_test.h"

namespace utils_predicate_test {
std::shared_ptr<RoadNetwork> create_road_network() {
    // middle lanelet
    size_t idLaneletOne = 1;
    auto laneletTypeLaneletOne = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    auto userOneWayLaneletOne = std::vector<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletOne = std::vector<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto centerVerticesLaneletOne = std::vector<vertex>{vertex{0, 0.5}, vertex{1, 0.5}, vertex{2, 0.5},
                                                        vertex{3, 0.5}, vertex{4, 0.5}, vertex{5, 0.5}};
    auto leftBorderLaneletOne =
        std::vector<vertex>{vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4},
                            vertex{50, 4}, vertex{60, 4}, vertex{70, 4}, vertex{80, 4}};
    auto rightBorderLaneletOne =
        std::vector<vertex>{vertex{0, 0},  vertex{10, 0}, vertex{20, 0}, vertex{30, 0}, vertex{40, 0},
                            vertex{50, 0}, vertex{60, 0}, vertex{70, 0}, vertex{80, 0}};
    auto laneletOne =
        std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                          laneletTypeLaneletOne, userOneWayLaneletOne, userBidirectionalLaneletOne));

    return std::make_shared<RoadNetwork>(RoadNetwork({laneletOne}, {}, {}, {}));
}

} // namespace utils_predicate_test