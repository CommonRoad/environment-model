#include "test_unobstructed_intersection_view_predicate.h"
#include "../../interfaces/utility_functions.h"
#include "commonroad_cpp/geometry/geometric_operations.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

void UnobstructedIntersectionViewPredicateTest::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 1000, 1006.9, 10, 0, 0);
    std::shared_ptr<State> stateOneEgoVehicle = std::make_shared<State>(1, 1010, 1006.5, 10, 0, 0);
    std::shared_ptr<State> stateTwoEgoVehicle = std::make_shared<State>(2, 1020, 1004.75, 10, 0, 0);
    std::shared_ptr<State> stateThreeEgoVehicle = std::make_shared<State>(3, 1030, 1004, 10, 0, 0);
    std::shared_ptr<State> stateFourEgoVehicle = std::make_shared<State>(4, 1040, 1006, 10, 0, 0);

    state_map_t trajectoryPredictionEgoVehicle{std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(1, stateOneEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(2, stateTwoEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(3, stateThreeEgoVehicle),
                                               std::pair<int, std::shared_ptr<State>>(4, stateFourEgoVehicle)};

    egoVehicle = std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, stateZeroEgoVehicle, ObstacleType::car,
                                                     50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));

    setUpIncoming();

    auto roadNetwork = std::make_shared<RoadNetwork>(
        RoadNetwork(lanelets, SupportedTrafficSignCountry::GERMANY, {}, {}, {intersection1}));
    world = std::make_shared<World>(World("testWorld", 0, roadNetwork, {egoVehicle}, {}, 0.1));
}

TEST_F(UnobstructedIntersectionViewPredicateTest, BooleanEvaluationObjects) {
    EXPECT_TRUE(pred.booleanEvaluation(0, world, egoVehicle));

    std::vector<vertex> fovMedium{{0.0, -100.0},   {70.71, -70.71}, {100.0, 0.0},     {70.71, 70.71}, {0.0, 100.0},
                                  {-70.71, 70.71}, {-100.0, 0.0},   {-70.71, -70.71}, {0.0, -100.0}};
    egoVehicle->setFov(geometric_operations::rotateAndTranslateVertices(
        fovMedium, {egoVehicle->getCurrentState()->getXPosition(), egoVehicle->getCurrentState()->getYPosition()}, 0));
    EXPECT_TRUE(pred.booleanEvaluation(1, world, egoVehicle));

    std::vector<vertex> fovSmall{{0.0, -5.0}, {3.535533906, -3.535533906}, {5.0, 0.0},  {3.535533906, 3.535533906},
                                 {0.0, 5.0},  {-3.535533906, 3.535533906}, {-5.0, 0.0}, {-3.535533906, -3.535533906},
                                 {0.0, -5.0}};
    egoVehicle->setFov(geometric_operations::rotateAndTranslateVertices(
        fovSmall, {egoVehicle->getCurrentState()->getXPosition(), egoVehicle->getCurrentState()->getYPosition()}, 0));
    // EXPECT_FALSE(pred.booleanEvaluation(2, world, egoVehicle)); TODO check why false
}

TEST_F(UnobstructedIntersectionViewPredicateTest, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, world, egoVehicle), std::runtime_error);
}

TEST_F(UnobstructedIntersectionViewPredicateTest, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, world, egoVehicle), std::runtime_error);
}
