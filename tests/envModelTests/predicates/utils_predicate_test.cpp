//
// Created by Sebastian Maierhofer on 21.05.21.
//

#include "utils_predicate_test.h"

namespace utils_predicate_test {
std::shared_ptr<RoadNetwork> create_road_network() {
    // right lanelet
    size_t idLaneletOne = 100;
    auto laneletTypeLaneletOne = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    auto userOneWayLaneletOne = std::vector<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletOne = std::vector<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletOne =
        std::vector<vertex>{vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4},
                            vertex{50, 4}, vertex{60, 4}, vertex{70, 4}, vertex{80, 4}};
    auto rightBorderLaneletOne =
        std::vector<vertex>{vertex{0, 0},  vertex{10, 0}, vertex{20, 0}, vertex{30, 0}, vertex{40, 0},
                            vertex{50, 0}, vertex{60, 0}, vertex{70, 0}, vertex{80, 0}};
    auto laneletOne =
        std::make_shared<Lanelet>(Lanelet(idLaneletOne, leftBorderLaneletOne, rightBorderLaneletOne,
                                          laneletTypeLaneletOne, userOneWayLaneletOne, userBidirectionalLaneletOne));

    // left lanelet
    size_t idLaneletTwo = 101;
    auto laneletTypeLaneletTwo = std::vector<LaneletType>{LaneletType::mainCarriageWay, LaneletType::interstate};
    auto userOneWayLaneletTwo = std::vector<ObstacleType>{ObstacleType::car, ObstacleType::bus};
    auto userBidirectionalLaneletTwo = std::vector<ObstacleType>{ObstacleType::truck, ObstacleType::pedestrian};
    auto leftBorderLaneletTwo =
        std::vector<vertex>{vertex{0, 8},  vertex{10, 8}, vertex{20, 8}, vertex{30, 8}, vertex{40, 8},
                            vertex{50, 8}, vertex{60, 8}, vertex{70, 8}, vertex{80, 8}};
    auto rightBorderLaneletTwo =
        std::vector<vertex>{vertex{0, 4},  vertex{10, 4}, vertex{20, 4}, vertex{30, 4}, vertex{40, 4},
                            vertex{50, 4}, vertex{60, 4}, vertex{70, 4}, vertex{80, 4}};
    auto laneletTwo =
        std::make_shared<Lanelet>(Lanelet(idLaneletTwo, leftBorderLaneletTwo, rightBorderLaneletTwo,
                                          laneletTypeLaneletTwo, userOneWayLaneletTwo, userBidirectionalLaneletTwo));

    return std::make_shared<RoadNetwork>(RoadNetwork({laneletOne, laneletTwo}));
}

} // namespace utils_predicate_test