#include "test_has_priority_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "../utils_predicate_test.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <commonroad_cpp/geometry/polygon.h>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/obstacle/occupancy.h>
#include <commonroad_cpp/predicates/regulatory/has_priority_must_predicate.h>
#include <commonroad_cpp/world.h>

TEST_F(HasPriorityPredicateTest, BooleanEvaluation) {
    // Todo
}

TEST_F(HasPriorityPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(HasPriorityPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, obstacleOne, obstacleTwo), std::runtime_error);
}

TEST_F(HasPriorityPredicateTest, SetBasedPrediction) {
    std::string scenarioName = "ZAM_TestTurnLeft-2_1_S-2";
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, scenarioName, boost::is_any_of("_"));
    auto dirName{pathSplit[0] + "_" + pathSplit[1]};
    std::string pathToTestXmlFile = TestUtils::getTestScenarioDirectory() + "/set_based/" + scenarioName + ".xml";
    const auto &scenarioXml = InputUtils::getDataFromCommonRoad(pathToTestXmlFile);

    auto obstacles{scenarioXml.obstacles};
    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, obstacles, {}, 0.1))};

    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(1001)};

    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs1, {"left", "straight"}, true));
}

TEST_F(HasPriorityPredicateTest, SetBasedMustMay) {
    HasPriorityMustPredicate predMust;
    // road network 2: lanelet 222 (incoming, yield sign) has priority 2, lanelet 333 (incoming) priority 3, lanelet 111
    // is no incoming lanelet
    const auto roadNetwork{utils_predicate_test::create_road_network_2()};
    // kth vehicle (state-based) on lanelet 333
    auto stateK0{std::make_shared<State>(0, 25, 7, 5, 0, 0)};
    auto stateK1{std::make_shared<State>(1, 30, 7, 5, 0, 0)};
    auto obsK{std::make_shared<Obstacle>(1, ObstacleRole::DYNAMIC, stateK0, ObstacleType::car, 50, 10, 3, -10, 0.3,
                                         state_map_t{{1, stateK1}}, 5, 1.8)};
    // pth vehicles with a set-based prediction at time step 1
    auto makeSetBased{[](size_t id, double yMin, double yMax) {
        auto obs{std::make_shared<Obstacle>(id, ObstacleRole::DYNAMIC, std::make_shared<State>(0, 25, 4.5, 5, 0, 0),
                                            ObstacleType::car, 50, 10, 3, -10, 0.3, state_map_t{}, 5, 1.8)};
        obs->appendOccupancyToSetBasedPrediction(std::make_shared<Occupancy>(
            1, std::make_shared<Polygon>(std::vector<vertex>{{27, yMin}, {33, yMin}, {33, yMax}, {27, yMax}})));
        return obs;
    }};
    auto obsInYield{makeSetBased(2, 3.5, 5.5)};     // only lanelet 222
    auto obsTwoLanelets{makeSetBased(3, 4.0, 7.0)}; // lanelets 222 and 333
    auto obsNotIncoming{makeSetBased(4, 1.0, 5.0)}; // lanelets 111 and 222
    auto world{std::make_shared<World>(
        World("testWorld", 0, roadNetwork,
              std::vector<std::shared_ptr<Obstacle>>{obsK, obsInYield, obsTwoLanelets, obsNotIncoming}, {}, 0.1))};
    const std::vector<std::string> dirs{"straight", "straight"};

    // all covered behaviors on the yield lanelet: the kth vehicle has priority over all of them
    EXPECT_TRUE(predMust.booleanEvaluation(1, world, obsK, obsInYield, dirs, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obsK, obsInYield, dirs, true));
    // some covered behaviors may be on lanelet 333 with the same priority as the kth vehicle
    EXPECT_FALSE(predMust.booleanEvaluation(1, world, obsK, obsTwoLanelets, dirs, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obsK, obsTwoLanelets, dirs, true));
    // some covered behaviors may be on lanelet 111 only, which is no incoming lanelet (no defined priority)
    EXPECT_FALSE(predMust.booleanEvaluation(1, world, obsK, obsNotIncoming, dirs, true));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, obsK, obsNotIncoming, dirs, true));
    // the set-based evaluation of has_priority is the may-version: the must-version implies it
    for (const auto &obsP : {obsInYield, obsTwoLanelets, obsNotIncoming})
        EXPECT_LE(predMust.booleanEvaluation(1, world, obsK, obsP, dirs, true),
                  pred.booleanEvaluation(1, world, obsK, obsP, dirs, true));
    // without an occupancy to evaluate, the must-version equals the base predicate
    EXPECT_EQ(predMust.booleanEvaluation(0, world, obsK, obsInYield, dirs, true),
              pred.booleanEvaluation(0, world, obsK, obsInYield, dirs, true));
}
