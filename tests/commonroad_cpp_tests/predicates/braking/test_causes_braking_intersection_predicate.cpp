#include "test_causes_braking_intersection_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/geometry/algorithms/append.hpp>
#include <commonroad_cpp/geometry/polygon.h>
#include <commonroad_cpp/obstacle/occupancy.h>
#include <commonroad_cpp/predicates/braking/causes_braking_intersection_may_predicate.h>
#include <commonroad_cpp/predicates/braking/causes_braking_intersection_must_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

void CausesBrakingIntersectionPredicateTest::SetUp() {}

TEST_F(CausesBrakingIntersectionPredicateTest, BooleanEvaluation) {
    // TODO
}

TEST_F(CausesBrakingIntersectionPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world1, obstacleOne), std::runtime_error);
}

TEST_F(CausesBrakingIntersectionPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world1, obstacleOne), std::runtime_error);
}

TEST_F(CausesBrakingIntersectionPredicateTest, SetBasedPrediction) {
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

    EXPECT_FALSE(pred.booleanEvaluation(0, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(20, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(29, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(38, world, obs1, ego, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(40, world, obs1, ego, {}, true));

    // the expectations at time steps 20, 38, and 40 depended on an uninitialized distance before
    EXPECT_FALSE(pred.booleanEvaluation(0, world, ego, obs1, {}, true));
    EXPECT_FALSE(pred.booleanEvaluation(20, world, ego, obs1, {}, true)); // occupancy front still ~27 m away
    EXPECT_TRUE(pred.booleanEvaluation(29, world, ego, obs1, {}, true));
    EXPECT_TRUE(pred.booleanEvaluation(38, world, ego, obs1, {}, true)); // occupancy front passed, rear part covered
    EXPECT_TRUE(pred.booleanEvaluation(40, world, ego, obs1, {}, true));
}

TEST_F(CausesBrakingIntersectionPredicateTest, SetBasedMustMay) {
    CausesBrakingIntersectionMustPredicate predMust;
    CausesBrakingIntersectionMayPredicate predMay;
    const auto &scenarioXml{InputUtils::getDataFromCommonRoad(TestUtils::getTestScenarioDirectory() +
                                                              "/set_based/ZAM_TestTurnLeft-2_1_S-2.xml")};
    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};
    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(1001)};

    // set-based prediction of the pth vehicle (obs1): the may-version holds if the braking interval
    // [dCauseBrakingIntersection, dBrakingIntersection] = [-2 m, 15 m] overlaps the covered distances
    EXPECT_FALSE(predMay.booleanEvaluation(20, world, ego, obs1, {}, true)); // front still ~27 m away
    EXPECT_TRUE(predMay.booleanEvaluation(29, world, ego, obs1, {}, true));
    EXPECT_TRUE(predMay.booleanEvaluation(38, world, ego, obs1, {}, true));
    // the occupancy of obs1 is longer than the braking interval, so not all covered behaviors are within it
    EXPECT_FALSE(predMust.booleanEvaluation(29, world, ego, obs1, {}, true));
    EXPECT_FALSE(predMust.booleanEvaluation(38, world, ego, obs1, {}, true));

    // the must-version holds only if the may-version holds, for both roles of the set-based vehicle
    for (size_t timeStep{0}; timeStep <= 40; ++timeStep) {
        EXPECT_LE(predMust.booleanEvaluation(timeStep, world, ego, obs1, {}, true),
                  predMay.booleanEvaluation(timeStep, world, ego, obs1, {}, true));
        EXPECT_LE(predMust.booleanEvaluation(timeStep, world, obs1, ego, {}, true),
                  predMay.booleanEvaluation(timeStep, world, obs1, ego, {}, true));
    }
}

TEST_F(CausesBrakingIntersectionPredicateTest, SetBasedMustMaySmallOccupancy) {
    CausesBrakingIntersectionMustPredicate predMust;
    CausesBrakingIntersectionMayPredicate predMay;
    const auto &scenarioXml{InputUtils::getDataFromCommonRoad(TestUtils::getTestScenarioDirectory() +
                                                              "/set_based/ZAM_TestTurnLeft-2_1_S-2.xml")};
    auto world{std::make_shared<World>(World("testWorld", 0, scenarioXml.roadNetwork, scenarioXml.obstacles, {}, 0.1))};
    auto ego{world->findObstacle(42)};
    auto obs1{world->findObstacle(1001)};
    const auto params{obs1->getActuatorParameters()};
    auto makeObstacle{[&](size_t id, const std::vector<vertex> &shape) {
        auto obs{std::make_shared<Obstacle>(id, ObstacleRole::DYNAMIC, obs1->getCurrentState(), ObstacleType::car,
                                            params.getVmax(), params.getAmax(), params.getAmaxLong(),
                                            params.getAminLong(), 0.3, state_map_t{}, 4.5, 1.8)};
        obs->appendOccupancyToSetBasedPrediction(std::make_shared<Occupancy>(29, std::make_shared<Polygon>(shape)));
        return obs;
    }};

    // small occupancy within the lane of obs1 (x in [56.5, 60]), about 10 m before the intersection points with the ego
    // lane: all covered fronts are within the braking interval, so some covered behavior is caused to brake (may). The
    // occupancy also covers behaviors which don't brake (upper acceleration bound a_max,long >= 0 >
    // aBrakingIntersection), so not all covered behaviors are caused to brake (not must).
    auto obsInLane{makeObstacle(2001, {{57.0, 71.0}, {59.5, 71.0}, {59.5, 73.0}, {57.0, 73.0}})};
    EXPECT_GE(obsInLane->getAcceleration(29, true, false), pred.getParameters().getParam("aBrakingIntersection"));
    EXPECT_FALSE(predMust.booleanEvaluation(29, world, ego, obsInLane, {}, true));
    EXPECT_TRUE(predMay.booleanEvaluation(29, world, ego, obsInLane, {}, true));

    // thin strip crossing the lane of obs1 with all vertices outside of it, about 14 m before the intersection points:
    // the clipped occupancy is within the braking interval (may)
    const std::vector<vertex> strip{{55.0, 75.0}, {61.5, 75.0}, {61.5, 75.5}, {55.0, 75.5}};
    auto obsStrip{makeObstacle(2002, strip)};
    const auto lane{obs1->getOccupiedLanes(world->getRoadNetwork(), 29, true).at(0)};
    for (const auto &vert : strip) {
        polygon_type point;
        boost::geometry::append(point, point_type{vert.x, vert.y});
        EXPECT_FALSE(lane->applyIntersectionTesting(point));
    }
    EXPECT_TRUE(predMay.booleanEvaluation(29, world, ego, obsStrip, {}, true));
    EXPECT_FALSE(predMust.booleanEvaluation(29, world, ego, obsStrip, {}, true));
}
