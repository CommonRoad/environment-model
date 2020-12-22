//
// Created by sebastian on 16.12.20.
//

#include "test_obstacle.h"
#include <map>

void ObstacleTest::setUpObstacles(){
    obstacleOne = std::make_shared<Obstacle>(Obstacle());
    idObstacleOne = 1;
    isStaticObstacleOne = false;

    obstacleTypeObstacleOne = ObstacleType::car;
    vMaxObstacleOne = 50.0;
    aMaxObstacleOne = 5.0;
    aMaxLongObstacleOne = 4.9;
    aMinLongObstacleOne = -10.5;
    reactionTimeObstacleOne = 0.5;
    widthObstacleOne = 2.0;
    lengthObstacleOne = 4.0;
    occupiedLaneObstacleOne.insert(std::pair<int, std::shared_ptr<Lane>>(0, laneOne));
    trajectoryPredictionObstacleOne.insert(std::pair<int, State>(2, stateThree));
    trajectoryPredictionObstacleOne.insert(std::pair<int, State>(3, stateFour));
    historyObstacleOne.insert(std::pair<int, State>(0, stateOne));
    lengthObstacleOne = 4.0;
    widthObstacleOne = 3.0;
    geoShapeObstacleOne.setLength(lengthObstacleOne);
    geoShapeObstacleOne.setWidth(widthObstacleOne);
    occupiedLaneletsObstacleOne.insert(std::pair<int,
            std::vector<std::shared_ptr<Lanelet>>>(0, std::vector<std::shared_ptr<Lanelet>>{laneletOne}));
    occupiedLaneletsObstacleOne.insert(std::pair<int,
            std::vector<std::shared_ptr<Lanelet>>>(0, std::vector<std::shared_ptr<Lanelet>>{laneletFive}));
    obstacleOne->setId(idObstacleOne);
    obstacleOne->setIsStatic(isStaticObstacleOne);
    obstacleOne->setCurrentState(stateTwo);
    obstacleOne->setObstacleType(obstacleTypeObstacleOne);
    obstacleOne->setVmax(vMaxObstacleOne);
    obstacleOne->setAmax(aMaxObstacleOne);
    obstacleOne->setAmaxLong(aMaxLongObstacleOne);
    obstacleOne->setAminLong(aMinLongObstacleOne);
    obstacleOne->setReactionTime(reactionTimeObstacleOne);
    obstacleOne->appendStateToHistory(stateOne);
    obstacleOne->setTrajectoryPrediction(trajectoryPredictionObstacleOne);
    obstacleOne->setOccupiedLane(std::vector<std::shared_ptr<Lane>>{laneOne}, 0);

    idObstacleTwo = 2;
    isStaticObstacleTwo = true;
    trajectoryPredictionObstacleTwo.insert(std::pair<int, State>(1, stateSix));
    obstacleTypeObstacleTwo = ObstacleType::bus;
    vMaxObstacleTwo = 30.0;
    aMaxObstacleTwo = 2.5;
    aMaxLongObstacleTwo = 2.0;
    aMinLongObstacleTwo = -8.0;
    reactionTimeObstacleTwo = 1.5;
    widthObstacleTwo = 2.5;
    lengthObstacleTwo = 10.0;

    obstacleTwo = std::make_shared<Obstacle>(Obstacle(idObstacleTwo, isStaticObstacleTwo, stateFive,
                                                      obstacleTypeObstacleTwo, vMaxObstacleTwo, aMaxObstacleTwo,
                                                      aMaxLongObstacleTwo, aMinLongObstacleTwo,
                                                      reactionTimeObstacleTwo,
                                                      trajectoryPredictionObstacleTwo,
                                                      lengthObstacleTwo, widthObstacleTwo));
}

void ObstacleTest::SetUp(){
    setUpLane();
    setUpObstacles();
}

void ObstacleTest::compareStates(State stateOne, State stateTwo){
    EXPECT_EQ(stateOne.getTimeStep(), stateTwo.getTimeStep());
    EXPECT_EQ(stateOne.getVelocity(), stateTwo.getVelocity());
    EXPECT_EQ(stateOne.getAcceleration(), stateTwo.getAcceleration());
    EXPECT_EQ(stateOne.getLatPosition(), stateTwo.getLatPosition());
    EXPECT_EQ(stateOne.getLonPosition(), stateTwo.getLonPosition());
    EXPECT_EQ(stateOne.getOrientation(), stateTwo.getOrientation());
    EXPECT_EQ(stateOne.getXPosition(), stateTwo.getXPosition());
    EXPECT_EQ(stateOne.getYPosition(), stateTwo.getYPosition());
    EXPECT_EQ(stateOne.getValidStates().acceleration, stateTwo.getValidStates().acceleration);
    EXPECT_EQ(stateOne.getValidStates().velocity, stateTwo.getValidStates().velocity);
    EXPECT_EQ(stateOne.getValidStates().xPosition, stateTwo.getValidStates().xPosition);
    EXPECT_EQ(stateOne.getValidStates().yPosition, stateTwo.getValidStates().yPosition);
    EXPECT_EQ(stateOne.getValidStates().orientation, stateTwo.getValidStates().orientation);
    EXPECT_EQ(stateOne.getValidStates().lonPosition, stateTwo.getValidStates().lonPosition);
    EXPECT_EQ(stateOne.getValidStates().latPosition, stateTwo.getValidStates().latPosition);
}

TEST_F(ObstacleTest, InitializationComplete){
    EXPECT_EQ(obstacleOne->getId(), idObstacleOne);
    EXPECT_EQ(obstacleTwo->getId(), idObstacleTwo);
    EXPECT_EQ(obstacleOne->getIsStatic(), isStaticObstacleOne);
    EXPECT_EQ(obstacleTwo->getIsStatic(), isStaticObstacleTwo);
    compareStates(obstacleOne->getCurrentState(), stateTwo);
    compareStates(obstacleTwo->getCurrentState(), stateFive);
    EXPECT_EQ(obstacleOne->getObstacleType(), obstacleTypeObstacleOne);
    EXPECT_EQ(obstacleTwo->getObstacleType(), obstacleTypeObstacleTwo);
    EXPECT_EQ(obstacleOne->getObstacleType(), obstacleTypeObstacleOne);
    EXPECT_EQ(obstacleTwo->getObstacleType(), obstacleTypeObstacleTwo);
    EXPECT_EQ(obstacleOne->getVmax(), vMaxObstacleOne);
    EXPECT_EQ(obstacleTwo->getVmax(), vMaxObstacleTwo);
    EXPECT_EQ(obstacleOne->getAmax(), aMaxObstacleOne);
    EXPECT_EQ(obstacleTwo->getAmax(), aMaxObstacleTwo);
    EXPECT_EQ(obstacleOne->getAmaxLong(), aMaxLongObstacleOne);
    EXPECT_EQ(obstacleTwo->getAmaxLong(), aMaxLongObstacleTwo);
    EXPECT_EQ(obstacleOne->getAminLong(), aMinLongObstacleOne);
    EXPECT_EQ(obstacleTwo->getAminLong(), aMinLongObstacleTwo);
    EXPECT_EQ(obstacleOne->getReactionTime(), reactionTimeObstacleOne);
    EXPECT_EQ(obstacleTwo->getReactionTime(), reactionTimeObstacleTwo);
    //compareLane(obstacleOne->getOccupiedLane(0), laneOne);

    for(int i = 2; i <= 3; ++i)
        compareStates(trajectoryPredictionObstacleOne.at(i),
                      obstacleOne->getTrajectoryPrediction().at(i));
    compareStates(trajectoryPredictionObstacleTwo.at(1),
                  obstacleTwo->getTrajectoryPrediction().at(1));
    EXPECT_EQ(obstacleOne->getTrajectoryLength(), 2);
    EXPECT_EQ(obstacleTwo->getTrajectoryLength(), 1);

}

//TEST_F(ObstacleTest, SetOccupiedLane){
//
//}
//
//TEST_F(ObstacleTest, AppendStateToPrediction){
//
//}
//
//TEST_F(ObstacleTest, GetOccupancyPolygonShape){
//
//}
//
//TEST_F(ObstacleTest, GetOccupiedLanelets){
//
//}
//
//TEST_F(ObstacleTest, FrontS){
//
//}
//
//TEST_F(ObstacleTest, RearS){
//
//}
//
//TEST_F(ObstacleTest, GetLonPosition){
//
//}
//
//TEST_F(ObstacleTest, GetLatPosition){
//
//}