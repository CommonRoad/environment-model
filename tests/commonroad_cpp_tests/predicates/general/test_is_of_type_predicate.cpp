#include "test_is_of_type_predicate.h"

void TestIsOfTypePredicate::SetUp() {

    std::shared_ptr<State> stateZeroEgoVehicle = std::make_shared<State>(0, 0, 0, 10, 0, 0, 0, 0, 0);

    Obstacle::state_map_t trajectoryPredictionEgoVehicle{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroEgoVehicle)};

    egoVehicle =
        std::make_shared<Obstacle>(Obstacle(1, ObstacleRole::DYNAMIC, {stateZeroEgoVehicle}, ObstacleType::pedestrian,
                                            50, 10, 3, -10, 0.3, trajectoryPredictionEgoVehicle, 5, 2));
}

TEST_F(TestIsOfTypePredicate, VRU) {
    std::shared_ptr<OptionalPredicateParameters> opt{std::make_shared<OptionalPredicateParameters>()};
    opt->obstacleType = ObstacleType::vru;
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::bicycle);
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::motorcycle);
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
}

TEST_F(TestIsOfTypePredicate, NotVRU) {
    std::shared_ptr<OptionalPredicateParameters> opt{std::make_shared<OptionalPredicateParameters>()};
    opt->obstacleType = ObstacleType::vru;
    egoVehicle->setObstacleType(ObstacleType::bus);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::car);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::taxi);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::train);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::truck);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
}

TEST_F(TestIsOfTypePredicate, SpecialPurposeVehicle) {
    std::shared_ptr<OptionalPredicateParameters> opt{std::make_shared<OptionalPredicateParameters>()};
    opt->obstacleType = ObstacleType::special_purpose_vehicle;
    egoVehicle->setObstacleType(ObstacleType::bicycle);
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::bus);
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::taxi);
    EXPECT_TRUE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
}

TEST_F(TestIsOfTypePredicate, NotSpecialPurposeVehicle) {
    std::shared_ptr<OptionalPredicateParameters> opt{std::make_shared<OptionalPredicateParameters>()};
    opt->obstacleType = ObstacleType::special_purpose_vehicle;
    egoVehicle->setObstacleType(ObstacleType::car);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::vehicle);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
    egoVehicle->setObstacleType(ObstacleType::pedestrian);
    EXPECT_FALSE(pred.booleanEvaluation(0, nullptr, egoVehicle, {}, opt));
}

TEST_F(TestIsOfTypePredicate, RobustEvaluation) {
    EXPECT_THROW(pred.robustEvaluation(0, {}, egoVehicle), std::runtime_error);
}

TEST_F(TestIsOfTypePredicate, ConstraintEvaluation) {
    EXPECT_THROW(pred.constraintEvaluation(0, {}, egoVehicle), std::runtime_error);
}
