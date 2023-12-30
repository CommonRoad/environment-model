#include "test_obstacle.h"
#include "../interfaces/utility_functions.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include <cmath>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <map>
#include <memory>
#include <unordered_set>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/state.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>

void ObstacleTestInitialization::setUpObstacles() {
    size_t globalID{1234};
    auto globalIdRef{std::make_shared<size_t>(globalID)};

    obstacleOne = std::make_shared<Obstacle>(Obstacle());
    idObstacleOne = 1;
    roleObstacleOne = ObstacleRole::DYNAMIC;
    obstacleTypeObstacleOne = ObstacleType::car;
    vMaxObstacleOne = 50.0;
    aMaxObstacleOne = 5.0;
    aMaxLongObstacleOne = 4.9;
    aMinLongObstacleOne = -10.5;
    reactionTimeObstacleOne = 0.5;
    widthObstacleOne = 2.0;
    lengthObstacleOne = 4.0;
    occupiedLaneObstacleOne.insert(std::pair<int, std::shared_ptr<Lane>>(0, laneOne));
    trajectoryPredictionObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(1, stateTwo));
    trajectoryPredictionObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(2, stateThree));
    trajectoryPredictionObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(3, stateFour));
    historyObstacleOne.insert(std::pair<int, std::shared_ptr<State>>(0, stateOne));
    lengthObstacleOne = 4.0;
    widthObstacleOne = 3.0;
    geoShapeObstacleOne.setLength(lengthObstacleOne);
    geoShapeObstacleOne.setWidth(widthObstacleOne);
    occupiedLaneletsObstacleOne.insert(
        std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(0, std::vector<std::shared_ptr<Lanelet>>{laneletOne}));
    occupiedLaneletsObstacleOne.insert(
        std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(0, std::vector<std::shared_ptr<Lanelet>>{laneletFive}));
    obstacleOne->setId(idObstacleOne);
    obstacleOne->setCurrentState(stateOne);
    obstacleOne->setCurrentState(stateTwo);
    obstacleOne->setObstacleRole(roleObstacleOne);
    obstacleOne->setObstacleType(obstacleTypeObstacleOne);
    ActuatorParameters actuatorParamsObstacleOne(vMaxObstacleOne, aMaxObstacleOne, aMaxLongObstacleOne,
                                                 aMinLongObstacleOne, aMinLongObstacleOne);
    SensorParameters sensorParamsObstacleOne(reactionTimeObstacleOne);
    obstacleOne->setActuatorParameters(actuatorParamsObstacleOne);
    obstacleOne->setSensorParameters(sensorParamsObstacleOne);
    obstacleOne->appendStateToHistory(stateOne);
    obstacleOne->setTrajectoryPrediction(trajectoryPredictionObstacleOne);
    obstacleOne->setRectangleShape(lengthObstacleOne, widthObstacleOne);
    obstacleOne->computeLanes(roadNetwork, true);

    idObstacleTwo = 2;
    roleObstacleTwo = ObstacleRole::STATIC;
    obstacleTypeObstacleTwo = ObstacleType::bus;
    vMaxObstacleTwo = 30.0;
    aMaxObstacleTwo = 2.5;
    aMaxLongObstacleTwo = 2.0;
    aMinLongObstacleTwo = -8.0;
    // NOTE: Static obstacles have reactionTime = std::nullopt
    reactionTimeObstacleTwo = std::nullopt;
    widthObstacleTwo = 2.5;
    lengthObstacleTwo = 10.0;
    obstacleTwo =
        std::make_shared<Obstacle>(idObstacleTwo, roleObstacleTwo, stateFive, obstacleTypeObstacleTwo, vMaxObstacleTwo,
                                   aMaxObstacleTwo, aMaxLongObstacleTwo, aMinLongObstacleTwo, reactionTimeObstacleTwo,
                                   trajectoryPredictionObstacleTwo, lengthObstacleTwo, widthObstacleTwo);
    obstacleTwo->computeLanes(roadNetwork);

    // Obstacle three, four, and five
    /* State 0 */
    std::shared_ptr<State> stateZeroObstacleThree = std::make_shared<State>(0, 0, 2, 10, 0, 0);
    std::shared_ptr<State> stateZeroObstacleFour = std::make_shared<State>(0, 5, 6, 5, 0, 0);
    std::shared_ptr<State> stateZeroObstacleSeven = std::make_shared<State>(0, 10, 10, 0, 0, 0);
    /* State 1 */
    std::shared_ptr<State> stateOneObstacleThree = std::make_shared<State>(1, 10, 2, 10, 0, 0);
    std::shared_ptr<State> stateOneObstacleFour = std::make_shared<State>(1, 10, 6, 11, 0, 0);
    std::shared_ptr<State> stateOneObstacleSeven = std::make_shared<State>(1, -10, 10, 0, 0, M_PI / 2);
    /* State 2 */
    std::shared_ptr<State> stateTwoObstacleThree = std::make_shared<State>(2, 20, 2, 10, 0, 0);
    std::shared_ptr<State> stateTwoObstacleFour = std::make_shared<State>(2, 21, 6, 8, 0, 0);
    std::shared_ptr<State> stateTwoObstacleSeven = std::make_shared<State>(2, 10, -10, 0, 0, M_PI);
    /* State 3 */
    std::shared_ptr<State> stateThreeObstacleThree = std::make_shared<State>(3, 30, 2, 10, 0, 0);
    std::shared_ptr<State> stateThreeObstacleFour = std::make_shared<State>(3, 29, 6, 8, 0, 0);
    std::shared_ptr<State> stateThreeObstacleSeven = std::make_shared<State>(3, -10, -10, 0, 0, M_PI * 1.5);
    /* State 4 */
    std::shared_ptr<State> stateFourObstacleThree = std::make_shared<State>(4, 40, 2, 10, 0, 0);
    std::shared_ptr<State> stateFourObstacleFour = std::make_shared<State>(4, 1000, 2000, 10, 0, 0);
    std::shared_ptr<State> stateFourObstacleFive = std::make_shared<State>(4, 40, 6, 10, 0, 0);
    /* State 5 */
    std::shared_ptr<State> stateFiveObstacleThree = std::make_shared<State>(5, 50, 2, 10, 0, 0);
    std::shared_ptr<State> stateFiveObstacleFour = std::make_shared<State>(5, 55, 6, 10, 0, 0);
    std::shared_ptr<State> stateFiveObstacleFive = std::make_shared<State>(5, 1000, 1000, 10, 0, 0);
    /* State 6 */
    std::shared_ptr<State> stateSixObstacleThree = std::make_shared<State>(6, 60, 2, 10, 0, 0);
    std::shared_ptr<State> stateSixObstacleFour = std::make_shared<State>(6, 65, -2, 5, 0, 0);
    std::shared_ptr<State> stateSixObstacleFive = std::make_shared<State>(6, 1000, 1000, 10, 0, 0);
    /* State 7 */
    std::shared_ptr<State> stateSevenObstacleThree = std::make_shared<State>(7, 70, 2, 10, 0, 0);
    std::shared_ptr<State> stateSevenObstacleFour = std::make_shared<State>(7, 70, -2, 11, 0, 0);
    std::shared_ptr<State> stateSevenObstacleFive = std::make_shared<State>(7, 70, 6, 11, 0, 0);
    std::shared_ptr<State> stateSevenObstacleSix = std::make_shared<State>(7, 70, -5, 11, 0, 0);
    /* State 8 */
    std::shared_ptr<State> stateEightObstacleThree = std::make_shared<State>(8, 80, 2, 10, 0, 0);
    std::shared_ptr<State> stateEightObstacleFour = std::make_shared<State>(8, 81, -2, 8, 0, 0);
    std::shared_ptr<State> stateEightObstacleFive = std::make_shared<State>(8, 1000, 1000, 10, 0, 0);
    /* State 9 */
    std::shared_ptr<State> stateNineObstacleThree = std::make_shared<State>(9, 90, 2, 10, 0, 0);
    std::shared_ptr<State> stateNineObstacleFour = std::make_shared<State>(9, 89, -2, 10, 0, 0);
    std::shared_ptr<State> stateNineObstacleFive = std::make_shared<State>(9, 1000, 1000, 10, 0, 0);
    /* State 10 */
    std::shared_ptr<State> stateTenObstacleThree = std::make_shared<State>(10, 100, 2, 10, 0, 0);
    std::shared_ptr<State> stateTenObstacleFive = std::make_shared<State>(10, 100, -2, 10, 0, 0);

    Obstacle::state_map_t trajectoryPredictionObstacleThree{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleThree),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleThree),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleThree),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleThree),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleThree),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleThree),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleThree),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleThree),
        std::pair<int, std::shared_ptr<State>>(9, stateNineObstacleThree),
        std::pair<int, std::shared_ptr<State>>(10, stateTenObstacleThree)};

    Obstacle::state_map_t trajectoryPredictionObstacleFour{
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleFour),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleFour),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleFour),
        std::pair<int, std::shared_ptr<State>>(4, stateFourObstacleFour),
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFour),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleFour),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleFour),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleFour),
        std::pair<int, std::shared_ptr<State>>(9, stateNineObstacleFour)};

    Obstacle::state_map_t trajectoryPredictionObstacleFive{
        std::pair<int, std::shared_ptr<State>>(5, stateFiveObstacleFive),
        std::pair<int, std::shared_ptr<State>>(6, stateSixObstacleFive),
        std::pair<int, std::shared_ptr<State>>(7, stateSevenObstacleFive),
        std::pair<int, std::shared_ptr<State>>(8, stateEightObstacleFive),
        std::pair<int, std::shared_ptr<State>>(9, stateNineObstacleFive),
        std::pair<int, std::shared_ptr<State>>(10, stateTenObstacleFive)};

    Obstacle::state_map_t trajectoryPredictionObstacleSeven{
        std::pair<int, std::shared_ptr<State>>(0, stateZeroObstacleSeven),
        std::pair<int, std::shared_ptr<State>>(1, stateOneObstacleSeven),
        std::pair<int, std::shared_ptr<State>>(2, stateTwoObstacleSeven),
        std::pair<int, std::shared_ptr<State>>(3, stateThreeObstacleSeven)};

    obstacleThree =
        std::make_shared<Obstacle>(Obstacle(3, ObstacleRole::DYNAMIC, stateZeroObstacleThree, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleThree, 5, 2));
    obstacleFour =
        std::make_shared<Obstacle>(Obstacle(4, ObstacleRole::DYNAMIC, stateZeroObstacleFour, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleFour, 5, 2));
    obstacleFive =
        std::make_shared<Obstacle>(Obstacle(5, ObstacleRole::DYNAMIC, stateFourObstacleFive, ObstacleType::truck, 50,
                                            10, 3, -10, 0.3, trajectoryPredictionObstacleFive, 8, 2));
    obstacleSix = std::make_shared<Obstacle>(Obstacle(6, ObstacleRole::DYNAMIC, stateSevenObstacleSix,
                                                      ObstacleType::pedestrian, ActuatorParameters{50, 10, 3, -10, -10},
                                                      SensorParameters{250.0, 250.0, 0.3}, {},
                                                      std::make_unique<Circle>(Circle(1)), {}));
    obstacleSeven =
        std::make_shared<Obstacle>(Obstacle(7, ObstacleRole::DYNAMIC, stateZeroObstacleSeven, ObstacleType::car, 50, 10,
                                            3, -10, 0.3, trajectoryPredictionObstacleSeven, 5, 2));

    obstacleList.push_back(obstacleOne);
    obstacleList.push_back(obstacleTwo);
    obstacleList.push_back(obstacleThree);
    obstacleList.push_back(obstacleFour);
    obstacleList.push_back(obstacleFive);
    obstacleList.push_back(obstacleSix);
}

void ObstacleTest::SetUp() {
    setUpLanelets();
    setUpLane();
    setUpRoadNetwork();
    setUpSignalStates();
    setUpStates();
    setUpObstacles();
}

void ObstacleTestInitialization::compareStates(const std::shared_ptr<State> &stateOne,
                                               const std::shared_ptr<State> &stateTwo) {
    EXPECT_EQ(stateOne->getTimeStep(), stateTwo->getTimeStep());
    EXPECT_EQ(stateOne->getVelocity(), stateTwo->getVelocity());
    EXPECT_EQ(stateOne->getAcceleration(), stateTwo->getAcceleration());
    EXPECT_EQ(stateOne->getLatPosition(), stateTwo->getLatPosition());
    EXPECT_EQ(stateOne->getLonPosition(), stateTwo->getLonPosition());
    EXPECT_EQ(stateOne->getGlobalOrientation(), stateTwo->getGlobalOrientation());
    EXPECT_EQ(stateOne->getXPosition(), stateTwo->getXPosition());
    EXPECT_EQ(stateOne->getYPosition(), stateTwo->getYPosition());
    EXPECT_EQ(stateOne->getValidStates().acceleration, stateTwo->getValidStates().acceleration);
    EXPECT_EQ(stateOne->getValidStates().velocity, stateTwo->getValidStates().velocity);
    EXPECT_EQ(stateOne->getValidStates().xPosition, stateTwo->getValidStates().xPosition);
    EXPECT_EQ(stateOne->getValidStates().yPosition, stateTwo->getValidStates().yPosition);
    EXPECT_EQ(stateOne->getValidStates().globalOrientation, stateTwo->getValidStates().globalOrientation);
    EXPECT_EQ(stateOne->getValidStates().lonPosition, stateTwo->getValidStates().lonPosition);
    EXPECT_EQ(stateOne->getValidStates().latPosition, stateTwo->getValidStates().latPosition);
}

TEST_F(ObstacleTest, InitializationComplete) {
    EXPECT_EQ(obstacleOne->getId(), idObstacleOne);
    EXPECT_EQ(obstacleTwo->getId(), idObstacleTwo);
    EXPECT_EQ(obstacleOne->getObstacleRole(), roleObstacleOne);
    EXPECT_EQ(obstacleTwo->getObstacleRole(), roleObstacleTwo);
    compareStates(obstacleOne->getCurrentState(), stateTwo);
    compareStates(obstacleTwo->getCurrentState(), stateFive);
    EXPECT_EQ(obstacleOne->getObstacleType(), obstacleTypeObstacleOne);
    EXPECT_EQ(obstacleTwo->getObstacleType(), obstacleTypeObstacleTwo);
    EXPECT_EQ(obstacleOne->getObstacleType(), obstacleTypeObstacleOne);
    EXPECT_EQ(obstacleTwo->getObstacleType(), obstacleTypeObstacleTwo);
    EXPECT_EQ(obstacleOne->getVmax(), vMaxObstacleOne);
    EXPECT_EQ(obstacleTwo->getVmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmax(), aMaxObstacleOne);
    EXPECT_EQ(obstacleTwo->getAmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmaxLong(), aMaxLongObstacleOne);
    EXPECT_EQ(obstacleTwo->getAmaxLong(), 0.0);
    EXPECT_EQ(obstacleOne->getAminLong(), aMinLongObstacleOne);
    EXPECT_EQ(obstacleTwo->getAminLong(), 0.0);
    EXPECT_EQ(obstacleOne->getReactionTime(), reactionTimeObstacleOne);
    EXPECT_EQ(obstacleTwo->getReactionTime(), reactionTimeObstacleTwo);
    auto ref{obstacleOne->getReferenceLane(roadNetwork, obstacleOne->getCurrentState()->getTimeStep())};
    // also option: laneletSeven, laneletSix
    compareVerticesVector({ref->getCenterVertices().front(), ref->getCenterVertices().back()},
                          {laneletThree->getCenterVertices().front(), laneletTwo->getCenterVertices().back()});
    for (size_t i = 2; i <= 3; ++i)
        compareStates(trajectoryPredictionObstacleOne.at(i), obstacleOne->getTrajectoryPrediction().at(i));
    EXPECT_EQ(obstacleOne->getTrajectoryLength(), 3);
    EXPECT_EQ(obstacleTwo->getTrajectoryLength(), 0);
}

TEST_F(ObstacleTest, SetIsStatic) {
    obstacleOne->setIsStatic(true);
    EXPECT_EQ(obstacleOne->getVmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmax(), 0.0);
    EXPECT_EQ(obstacleOne->getAmaxLong(), 0.0);
    EXPECT_EQ(obstacleOne->getAminLong(), 0.0);
}

TEST_F(ObstacleTest, AppendStateToPrediction) {
    obstacleOne->appendStateToTrajectoryPrediction(stateFive);
    compareStates(obstacleOne->getStateByTimeStep(stateFive->getTimeStep()), stateFive);
}

TEST_F(ObstacleTest, GetOccupancyPolygonShape) {
    EXPECT_EQ(obstacleOne->getOccupancyPolygonShape(0).outer().at(0).x(), 0.5);
    EXPECT_EQ(obstacleOne->getOccupancyPolygonShape(0).outer().at(0).y(), 2);
    EXPECT_DOUBLE_EQ(obstacleOne->getOccupancyPolygonShape(1).outer().at(0).x(), 2);
    EXPECT_EQ(obstacleOne->getOccupancyPolygonShape(1).outer().at(0).y(), 0.0);
    EXPECT_EQ(obstacleSix->getOccupancyPolygonShape(7).outer().at(3).x(), 69.0);
    EXPECT_EQ(obstacleSix->getOccupancyPolygonShape(7).outer().at(1).y(), -4.0);
}

TEST_F(ObstacleTest, GetOccupiedLanelets) {
    EXPECT_EQ(obstacleOne->getOccupiedLaneletsByShape(roadNetwork, 0).size(), 1);
    EXPECT_EQ(obstacleOne->getOccupiedLaneletsByShape(roadNetwork, 1).size(), 1);
    EXPECT_EQ(obstacleTwo->getOccupiedLaneletsByShape(roadNetwork, 4).size(), 0);
}

TEST_F(ObstacleTest, FrontS) {
    EXPECT_EQ(obstacleOne->frontS(roadNetwork, 0), lonPositionStateOne + lengthObstacleOne / 2);
    EXPECT_EQ(obstacleOne->frontS(roadNetwork, 1), lonPositionStateTwo + widthObstacleOne / 2);
}

TEST_F(ObstacleTest, RearS) {
    EXPECT_EQ(obstacleOne->rearS(roadNetwork, 0), lonPositionStateOne - lengthObstacleOne / 2);
    EXPECT_EQ(obstacleOne->rearS(roadNetwork, 1), lonPositionStateTwo - widthObstacleOne / 2);
}

TEST_F(ObstacleTest, GetLonPosition) {
    EXPECT_EQ(obstacleOne->getLonPosition(roadNetwork, 0), lonPositionStateOne);
    EXPECT_EQ(obstacleOne->getLonPosition(roadNetwork, 1), lonPositionStateTwo);
    EXPECT_EQ(obstacleOne->getLonPosition(roadNetwork, 2), lonPositionStateThree);
}

TEST_F(ObstacleTest, GetLatPosition) {
    EXPECT_EQ(obstacleOne->getLatPosition(roadNetwork, 0), latPositionStateOne);
    EXPECT_EQ(obstacleOne->getLatPosition(roadNetwork, 1), latPositionStateTwo);
    EXPECT_EQ(obstacleOne->getLatPosition(roadNetwork, 2), latPositionStateThree);
}

TEST_F(ObstacleTest, GetStateByTimeStep) {
    EXPECT_EQ(obstacleOne->getStateByTimeStep(0)->getTimeStep(), 0);
    EXPECT_EQ(obstacleOne->getStateByTimeStep(1)->getTimeStep(), 1);
    EXPECT_EQ(obstacleOne->getStateByTimeStep(2)->getTimeStep(), 2);
    EXPECT_THROW(obstacleOne->getStateByTimeStep(2000)->getTimeStep(), std::logic_error);
}

TEST_F(ObstacleTest, TestObstacleRole) {
    EXPECT_EQ(obstacleOne->getObstacleRole(), ObstacleRole::DYNAMIC);
    EXPECT_EQ(obstacleTwo->getObstacleRole(), ObstacleRole::STATIC);
}

TEST_F(ObstacleTest, ConvertPointToCurvilinear) {
    stateOne->setXPosition(25.0);
    stateOne->setYPosition(1.5);
    obstacleOne->convertPointToCurvilinear(roadNetwork, 0);
    EXPECT_NEAR(stateOne->getLonPosition(), 75.0, 0.0005); // lane starts at x==-50
    EXPECT_EQ(stateOne->getLatPosition(), 0.0);
    stateOne->setXPosition(25.0);
    stateOne->setYPosition(-2.75);
    obstacleOne->convertPointToCurvilinear(roadNetwork, 0);
    EXPECT_NEAR(stateOne->getLonPosition(), 75.0, 0.0005);
    EXPECT_EQ(stateOne->getLatPosition(), -4.25);
}

TEST_F(ObstacleTest, SetOccupiedLaneletsDrivingDirectionByShape) {
    EXPECT_NO_THROW(obstacleOne->getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, 0));
}

TEST_F(ObstacleTest, SetReferenceGeneralScenario1) {
    size_t timeStep{0};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1/USA_Lanker-1_1_T-1.pb"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    roadNetworkScenario->setIdCounterRef(globalIdRef);
    auto obsOneScenario{obstacle_operations::getObstacleById(obstaclesScenario, 1219)};
    lanelet_id_set expRefLaneletsObsOneScenario{3570, 3632, 3652, 3616, 3456, 3462, 3470};
    EXPECT_EQ(expRefLaneletsObsOneScenario,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLaneletIDs());

    const auto obsTwoScenario{obstacle_operations::getObstacleById(obstaclesScenario, 1214)};
    obsTwoScenario->computeLanes(roadNetworkScenario);
    lanelet_id_set expRefLaneletsObsTwoScenario{3570, 3632, 3652, 3616, 3456, 3462, 3470};
    EXPECT_EQ(expRefLaneletsObsTwoScenario,
              obsTwoScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLaneletIDs());
}

TEST_F(ObstacleTest, SetReferenceGeneralScenario2) {
    size_t timeStep{0};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() +
                                  "/DEU_Guetersloh-25/DEU_Guetersloh-25_4_T-1.pb"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    roadNetworkScenario->setIdCounterRef(globalIdRef);
    auto obsOneScenario{obstacle_operations::getObstacleById(obstaclesScenario, 325)};
    auto ref{obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)};
    EXPECT_EQ(77062, ref->getContainedLanelets().front()->getId()); // 77695 and 77062 are possible
}

TEST_F(ObstacleTest, SetReferenceGeneralScenario3) {
    size_t timeStep{0};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/USA_Peach-2/USA_Peach-2_1_T-1.pb"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    roadNetworkScenario->setIdCounterRef(globalIdRef);
    auto obsOneScenario{obstacle_operations::getObstacleById(obstaclesScenario, 500)};
    EXPECT_EQ(53758,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(53786, obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)
                         ->getContainedLanelets()
                         .back()
                         ->getId()); // 53786 and 53788 are possible
}

TEST_F(ObstacleTest, SetReferenceGeneralScenario4) {
    size_t timeStep{0};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/USA_Peach-4/USA_Peach-4_1_T-1.pb"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    roadNetworkScenario->setIdCounterRef(globalIdRef);
    auto obsOneScenario{obstacle_operations::getObstacleById(obstaclesScenario, 88)};
    EXPECT_EQ(43349,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(43486,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().back()->getId());
    timeStep = 1;
    EXPECT_EQ(43349,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(43486,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().back()->getId());
}

TEST_F(ObstacleTest, SetReferenceGeneralScenario5) {
    size_t timeStep{37};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() +
                                  "/predicates/DEU_TestTurnLeft-1/DEU_TestTurnLeft-1_8_T-1.pb"};
    const auto &[obstaclesScenario, roadNetworkScenario, timeStepSize] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    size_t globalID{123456789};
    auto globalIdRef{std::make_shared<size_t>(globalID)};
    roadNetworkScenario->setIdCounterRef(globalIdRef);
    auto obsOneScenario{obstacle_operations::getObstacleById(obstaclesScenario, 1000)};
    EXPECT_EQ(1,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(402,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().back()->getId());
    timeStep = 39;
    EXPECT_EQ(1,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(402,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().back()->getId());
    timeStep = 49;
    EXPECT_EQ(1,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().front()->getId());
    EXPECT_EQ(402,
              obsOneScenario->getReferenceLane(roadNetworkScenario, timeStep)->getContainedLanelets().back()->getId());
}

TEST_F(ObstacleTest, TestGetFirstTrajectoryTimeStep) {
    EXPECT_EQ(obstacleOne->getFirstTrajectoryTimeStep(), 1);
    EXPECT_THROW(obstacleTwo->getFirstTrajectoryTimeStep(), std::runtime_error);
    EXPECT_EQ(obstacleThree->getFirstTrajectoryTimeStep(), 1);
    EXPECT_EQ(obstacleFour->getFirstTrajectoryTimeStep(), 1);
    EXPECT_EQ(obstacleFive->getFirstTrajectoryTimeStep(), 5);
}

TEST_F(ObstacleTest, testAppendSignalStateToHistory) {
    obstacleOne->appendSignalStateToHistory(signalStateOne);
    obstacleOne->appendSignalStateToHistory(signalStateTwo);
    EXPECT_EQ(obstacleOne->getSignalSeriesHistory().size(), 2);
}

TEST_F(ObstacleTest, testAppendSignalStateToSeries) {
    obstacleOne->appendSignalStateToSeries(signalStateOne);
    obstacleOne->appendSignalStateToSeries(signalStateTwo);
    EXPECT_EQ(obstacleOne->getSignalSeries().size(), 2);
}

TEST_F(ObstacleTest, testGetCurrentSignalState) {
    obstacleOne->setCurrentSignalState(signalStateOne);
    EXPECT_EQ(obstacleOne->getCurrentSignalState()->getTimeStep(), 0);
}

TEST_F(ObstacleTest, testGetSignalStateByTimeStep) {
    obstacleOne->appendSignalStateToHistory(signalStateOne);
    obstacleOne->appendSignalStateToSeries(signalStateTwo);
    EXPECT_TRUE(obstacleOne->getSignalStateByTimeStep(0)->isHorn());
    EXPECT_FALSE(obstacleOne->getSignalStateByTimeStep(1)->isHorn());
    EXPECT_THROW(obstacleOne->getSignalStateByTimeStep(2), std::logic_error);
}

TEST_F(ObstacleTest, testGetFieldOfViewRearDistance) {
    obstacleOne->setFieldOfViewRearDistance(77.5);
    EXPECT_EQ(obstacleOne->getFieldOfViewRearDistance(), 77.5);
    EXPECT_NE(obstacleOne->getFieldOfViewFrontDistance(), 77.5);
}

TEST_F(ObstacleTest, testGetFieldOfViewFrontDistance) {
    obstacleOne->setFieldOfViewFrontDistance(99.5);
    EXPECT_EQ(obstacleOne->getFieldOfViewFrontDistance(), 99.5);
    EXPECT_NE(obstacleOne->getFieldOfViewRearDistance(), 99.5);
}

TEST_F(ObstacleTest, testGetFrontXYCoordinate) {
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(0).at(0), 12.5);
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(0).at(1), 10);
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(1).at(0), -10.0);
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(1).at(1), 12.5);
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(2).at(0), 7.5);
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(2).at(1), -10.0);
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(3).at(0), -10.0);
    EXPECT_EQ(obstacleSeven->getFrontXYCoordinates(3).at(1), -12.5);
}

TEST_F(ObstacleTest, testGetBackXYCoordinate) {
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(0).at(0), 7.5);
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(0).at(1), 10);
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(1).at(0), -10.0);
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(1).at(1), 7.5);
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(2).at(0), 12.5);
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(2).at(1), -10.0);
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(3).at(0), -10.0);
    EXPECT_EQ(obstacleSeven->getBackXYCoordinates(3).at(1), -7.5);
}

TEST_F(ObstacleTest, testGetLateralDistanceToObstacle) {
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(0, obstacleFour, roadNetwork), 2.0);
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(1, obstacleFour, roadNetwork), 2.0);
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(2, obstacleFour, roadNetwork), 2.0);
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(3, obstacleFour, roadNetwork), 2.0);
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(0, obstacleSeven, roadNetwork), 6);
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(1, obstacleSeven, roadNetwork), 4.5);
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(2, obstacleSeven, roadNetwork), 10);
    EXPECT_EQ(obstacleThree->getLateralDistanceToObstacle(3, obstacleSeven, roadNetwork), 8.5);
}
