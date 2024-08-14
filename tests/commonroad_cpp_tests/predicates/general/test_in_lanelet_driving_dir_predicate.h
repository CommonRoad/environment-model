#pragma once
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/predicates/general/in_lanelet_driving_dir_predicate.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <gtest/gtest.h>

class InLaneletDrivingDirPredicateTest : public testing::Test {
  protected:
    std::shared_ptr<Obstacle> vehicleOne;
    std::shared_ptr<Obstacle> vehicleTwo;
    std::shared_ptr<Obstacle> vehicleThree;
    std::shared_ptr<Obstacle> vehicleFour;
    std::shared_ptr<Obstacle> vehicleFive;
    InLaneletDrivingDirPredicate pred;
    std::shared_ptr<World> world;
    std::vector<std::string> opt;

    void initializeTestData(const std::set<ObstacleType> &userOneWayLanelet,
                            const std::set<ObstacleType> &userBidirectionalLanelet);

  private:
    void SetUp() override;
};
