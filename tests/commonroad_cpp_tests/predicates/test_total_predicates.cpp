// Predicates that rules use behind a guard (e.g., keeps_safe_distance_prec behind succeeds) must be total, i.e., return
// a documented value instead of throwing if they are evaluated outside the guard. The rank-based reordering of
// conjunctions and disjunctions relies on this.

#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include "utils_predicate_test.h"
#include <gtest/gtest.h>
#include <limits>

#include <commonroad_cpp/predicates/braking/keeps_safe_distance_prec_may_predicate.h>
#include <commonroad_cpp/predicates/braking/keeps_safe_distance_prec_must_predicate.h>
#include <commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h>
#include <commonroad_cpp/predicates/general/orientation_towards_predicate.h>
#include <commonroad_cpp/predicates/lane/close_to_lane_border_predicate.h>
#include <commonroad_cpp/predicates/lane/in_neighboring_lane_predicate.h>
#include <commonroad_cpp/predicates/lane/in_same_lane_may_predicate.h>
#include <commonroad_cpp/predicates/lane/in_same_lane_must_predicate.h>
#include <commonroad_cpp/predicates/lane/in_same_lane_predicate.h>
#include <commonroad_cpp/predicates/lane/in_single_lane_predicate.h>
#include <commonroad_cpp/predicates/lane/lane_based_orientation_similar_may_predicate.h>
#include <commonroad_cpp/predicates/lane/lane_based_orientation_similar_must_predicate.h>
#include <commonroad_cpp/predicates/lane/lane_based_orientation_similar_predicate.h>
#include <commonroad_cpp/predicates/lane/on_same_road_may_predicate.h>
#include <commonroad_cpp/predicates/lane/on_same_road_predicate.h>
#include <commonroad_cpp/predicates/position/in_front_of_may_predicate.h>
#include <commonroad_cpp/predicates/position/in_front_of_must_predicate.h>
#include <commonroad_cpp/predicates/position/in_front_of_predicate.h>
#include <commonroad_cpp/predicates/position/left_of_predicate.h>
#include <commonroad_cpp/predicates/position/right_of_predicate.h>

class TotalPredicatesTest : public testing::Test {
  protected:
    static constexpr size_t noStateTimeStep{7}; //**< time step at which no obstacle has a state */
    std::shared_ptr<Obstacle> ego;              //**< on the road */
    std::shared_ptr<Obstacle> ahead;            //**< on the road, in front of ego */
    std::shared_ptr<Obstacle> offRoad;          //**< outside of the road network: no reference lane */
    std::shared_ptr<World> world;

    static std::shared_ptr<Obstacle> createObstacle(size_t id, double x, double y) {
        const auto state0{std::make_shared<State>(0, x, y, 10, 0, 0)};
        const auto state1{std::make_shared<State>(1, x + 1, y, 10, 0, 0)};
        return std::make_shared<Obstacle>(id, ObstacleRole::DYNAMIC, state0, ObstacleType::car, 50, 10, 3, -10, 0.3,
                                          state_map_t{std::pair<int, std::shared_ptr<State>>(0, state0),
                                                      std::pair<int, std::shared_ptr<State>>(1, state1)},
                                          5, 2);
    }

    void SetUp() override {
        ego = createObstacle(1, 10, 2);
        ahead = createObstacle(2, 30, 2);
        offRoad = createObstacle(3, 1000, 1000);
        world = std::make_shared<World>("testWorld", 0, utils_predicate_test::create_road_network(),
                                        std::vector<std::shared_ptr<Obstacle>>{ego, ahead, offRoad},
                                        std::vector<std::shared_ptr<Obstacle>>{}, 0.1);
    }

    /** Unguarded situations: k or p has no reference lane, or no state at the time step. */
    template <typename F> void forEachUnguardedSituation(F &&check) {
        check(0, ego, offRoad);
        check(0, offRoad, ego);
        check(noStateTimeStep, ego, ahead);
    }
};

TEST_F(TotalPredicatesTest, SanityOnRoad) {
    // the predicates are not constant: with both vehicles on the road, they are evaluated as before
    EXPECT_TRUE(InFrontOfPredicate().booleanEvaluation(0, world, ego, ahead));
    EXPECT_FALSE(InFrontOfPredicate().booleanEvaluation(0, world, ahead, ego));
    EXPECT_TRUE(InSameLanePredicate().booleanEvaluation(0, world, ego, ahead));
}

TEST_F(TotalPredicatesTest, KeepsSafeDistancePrecIsTrue) {
    KeepsSafeDistancePrecPredicate pred;
    KeepsSafeDistancePrecMustPredicate predMust;
    KeepsSafeDistancePrecMayPredicate predMay;
    forEachUnguardedSituation([&](size_t timeStep, const auto &k, const auto &p) {
        EXPECT_NO_THROW(EXPECT_TRUE(pred.booleanEvaluation(timeStep, world, k, p)));
        EXPECT_NO_THROW(EXPECT_TRUE(predMust.booleanEvaluation(timeStep, world, k, p, {"0.0"}, true)));
        EXPECT_NO_THROW(EXPECT_TRUE(predMay.booleanEvaluation(timeStep, world, k, p, {"0.0"}, true)));
        EXPECT_NO_THROW(EXPECT_GT(pred.robustEvaluation(timeStep, world, k, p), 0));
        EXPECT_NO_THROW(EXPECT_GT(predMust.robustEvaluation(timeStep, world, k, p, {"0.0"}, true), 0));
        EXPECT_NO_THROW(EXPECT_GT(predMay.robustEvaluation(timeStep, world, k, p, {"0.0"}, true), 0));
        EXPECT_NO_THROW(EXPECT_GT(pred.constraintEvaluation(timeStep, world, k, p).realValuedConstraint, 0));
    });
}

TEST_F(TotalPredicatesTest, KeepsSafeDistancePrecNonPrecedingVehicle) {
    // p behind k on the road: safe distance not applicable, as documented
    EXPECT_TRUE(KeepsSafeDistancePrecPredicate().booleanEvaluation(0, world, ahead, ego));
}

TEST_F(TotalPredicatesTest, InFrontOfIsFalse) {
    InFrontOfPredicate pred;
    InFrontOfMustPredicate predMust;
    InFrontOfMayPredicate predMay;
    forEachUnguardedSituation([&](size_t timeStep, const auto &k, const auto &p) {
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(timeStep, world, k, p)));
        EXPECT_NO_THROW(EXPECT_FALSE(predMust.booleanEvaluation(timeStep, world, k, p, {"0.0"}, true)));
        EXPECT_NO_THROW(EXPECT_FALSE(predMay.booleanEvaluation(timeStep, world, k, p, {"0.0"}, true)));
        EXPECT_NO_THROW(EXPECT_LT(pred.robustEvaluation(timeStep, world, k, p), 0));
        EXPECT_NO_THROW(EXPECT_LT(predMay.robustEvaluation(timeStep, world, k, p, {"0.0"}, true), 0));
    });
}

TEST_F(TotalPredicatesTest, OrientationTowardsIsFalse) {
    OrientationTowardsPredicate pred;
    forEachUnguardedSituation([&](size_t timeStep, const auto &k, const auto &p) {
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(timeStep, world, k, p)));
    });
}

TEST_F(TotalPredicatesTest, InSameLaneIsFalse) {
    InSameLanePredicate pred;
    InSameLaneMustPredicate predMust;
    InSameLaneMayPredicate predMay;
    forEachUnguardedSituation([&](size_t timeStep, const auto &k, const auto &p) {
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(timeStep, world, k, p)));
        EXPECT_NO_THROW(EXPECT_FALSE(predMust.booleanEvaluation(timeStep, world, k, p, {}, true)));
        EXPECT_NO_THROW(EXPECT_FALSE(predMay.booleanEvaluation(timeStep, world, k, p, {}, true)));
    });
}

TEST_F(TotalPredicatesTest, OnSameRoadIsFalse) {
    OnSameRoadPredicate pred;
    OnSameRoadMayPredicate predMay;
    forEachUnguardedSituation([&](size_t timeStep, const auto &k, const auto &p) {
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(timeStep, world, k, p)));
        EXPECT_NO_THROW(EXPECT_FALSE(predMay.booleanEvaluation(timeStep, world, k, p, {}, true)));
    });
}

TEST_F(TotalPredicatesTest, InNeighboringLaneIsFalse) {
    InNeighboringLanePredicate pred;
    forEachUnguardedSituation([&](size_t timeStep, const auto &k, const auto &p) {
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(timeStep, world, k, p, {"left"})));
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(timeStep, world, k, p, {"right"})));
    });
}

TEST_F(TotalPredicatesTest, LaneBasedOrientationSimilarIsFalse) {
    LaneBasedOrientationSimilarPredicate pred;
    LaneBasedOrientationSimilarMustPredicate predMust;
    LaneBasedOrientationSimilarMayPredicate predMay;
    forEachUnguardedSituation([&](size_t timeStep, const auto &k, const auto &p) {
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(timeStep, world, k, p)));
        EXPECT_NO_THROW(EXPECT_FALSE(predMust.booleanEvaluation(timeStep, world, k, p, {}, true)));
        EXPECT_NO_THROW(EXPECT_FALSE(predMay.booleanEvaluation(timeStep, world, k, p, {}, true)));
    });
}

TEST_F(TotalPredicatesTest, LeftOfRightOfAreFalse) {
    LeftOfPredicate left;
    RightOfPredicate right;
    EXPECT_NO_THROW(EXPECT_FALSE(left.booleanEvaluation(noStateTimeStep, world, ego, ahead)));
    EXPECT_NO_THROW(EXPECT_FALSE(right.booleanEvaluation(noStateTimeStep, world, ego, ahead)));
}

TEST_F(TotalPredicatesTest, InSingleLaneDoesNotThrowOffRoad) {
    InSingleLanePredicate pred;
    EXPECT_NO_THROW(pred.booleanEvaluation(0, world, offRoad, ego));
}

TEST(TotalPredicatesScenarioTest, CloseToLaneBorderIsFalseWithoutStateOrProjection) {
    // a vehicle that is not projectable onto one of its lanes (verified in commonroad-monitor, R_G8) or has no state
    const auto scenario{
        InputUtils::getDataFromCommonRoad(TestUtils::getTestScenarioDirectory() +
                                          "/predicates/DEU_TestRightBeforeLeft-1/DEU_TestRightBeforeLeft-1_3_T-1.pb")};
    const auto world{std::make_shared<World>("testWorld", 0, scenario.roadNetwork, scenario.obstacles,
                                             std::vector<std::shared_ptr<Obstacle>>{}, scenario.timeStepSize)};
    CloseToLaneBorderPredicate pred;
    for (const auto &obstacle : scenario.obstacles)
        for (const auto &side : {"left", "right"})
            EXPECT_NO_THROW(pred.booleanEvaluation(0, world, obstacle, nullptr, {side}));
    for (const auto &obstacle : scenario.obstacles)
        EXPECT_NO_THROW(EXPECT_FALSE(pred.booleanEvaluation(1000, world, obstacle, nullptr, {"left"})));
}
