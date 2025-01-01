#include "test_world.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/world.h"
#include "interfaces/utility_functions.h"
#include <array>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

TEST_F(WorldTest, TestScenariosValid) {
    std::array<std::string, 5> scenarioDirs{"ZAM_Urban-2_1_T-1", "USA_Peach-4_1_T-1", "USA_Peach-2_1_T-1",
                                            "ESP_Almansa-2_2_T-1", "ARG_Carcarana-6_5_T-1"};
    for (const auto &scen : scenarioDirs) {
        std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" + scen.substr(0, scen.size() - 6) +
                                      "/" + scen + ".pb"};
        const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
            InputUtils::getDataFromCommonRoad(pathToTestFileOne);
        EXPECT_NO_THROW(auto world{World(scen, 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)});
    }
}

TEST_F(WorldTest, TestSingleScenarioObstacle) {
    size_t obstacleId{325};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/BEL_Zwevegem-1/BEL_Zwevegem-1_5_T-1.pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_EQ(timeStepSizeOne, 0.1);
    auto world = World("BEL_Zwevegem-1_5_T-1", 0, roadNetworkScenarioOne,
                       {obstacle_operations::getObstacleById(obstaclesScenarioOne, obstacleId)}, {}, timeStepSizeOne);
    auto obs{world.findObstacle(obstacleId)};
    for (const auto &time : obs->getTimeSteps())
        EXPECT_NO_THROW(auto ref{obs->getReferenceLane(world.getRoadNetwork(), time)});
}

TEST_F(WorldTest, TestSingleScenario) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    EXPECT_NO_THROW(
        auto world{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)});
    for (const auto &obs : obstaclesScenarioOne)
        for (const auto &time : obs->getTimeSteps())
            EXPECT_NO_THROW(obs->getReferenceLane(roadNetworkScenarioOne, time));
}

TEST_F(WorldTest, FindObstacle) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)};
    EXPECT_EQ(world1.findObstacle(334)->getId(), 334);
    EXPECT_THROW(world1.findObstacle(1), std::logic_error);

    auto world2{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, {}, obstaclesScenarioOne, timeStepSizeOne)};
    EXPECT_EQ(world2.findObstacle(334)->getId(), 334);
    EXPECT_THROW(world2.findObstacle(1), std::logic_error);
}

TEST_F(WorldTest, GetTimeStep) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)};
    EXPECT_EQ(world1.getTimeStep(), 0);
    auto world2{World("USA_Peach-2_1_T-1", 1, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)};
    EXPECT_EQ(world2.getTimeStep(), 1);
}

TEST_F(WorldTest, GetTimeStepSize) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, 0.1)};
    EXPECT_EQ(world1.getDt(), 0.1);
    auto world2{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, 0.02)};
    EXPECT_EQ(world2.getDt(), 0.02);
}

TEST_F(WorldTest, FindObstacles) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)};
    auto obs{world1.findObstacles({334, 363})};
    EXPECT_EQ(obs.size(), 2);
    EXPECT_EQ(obs[0]->getId(), 334);
    EXPECT_EQ(obs[1]->getId(), 363);
    obs = world1.findObstacles({1, 334});
    EXPECT_EQ(obs.size(), 1);
    EXPECT_EQ(obs[0]->getId(), 334);

    auto world2{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, {}, obstaclesScenarioOne, timeStepSizeOne)};
    obs = world2.findObstacles({334, 363});
    EXPECT_EQ(obs.size(), 2);
    EXPECT_EQ(obs[0]->getId(), 334);
    EXPECT_EQ(obs[1]->getId(), 363);
    obs = world2.findObstacles({1, 334});
    EXPECT_EQ(obs.size(), 1);
    EXPECT_EQ(obs[0]->getId(), 334);
}

TEST_F(WorldTest, SetCurvilinearStates) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    RoadNetworkParameters roadParams;
    roadParams.numIntersectionsPerDirectionLaneGeneration = 2;
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, 0.1,
                      WorldParameters(roadParams, SensorParameters(250.0, 250.0), ActuatorParameters::egoDefaults(),
                                      TimeParameters::dynamicDefaults(), ActuatorParameters::vehicleDefaults()))};
    EXPECT_NO_THROW(world1.setCurvilinearStates());
}

TEST_F(WorldTest, GetEgoVehicles) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)};
    EXPECT_EQ(world1.getEgoVehicles().size(), 16);

    auto world2{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, {}, obstaclesScenarioOne, timeStepSizeOne)};
    EXPECT_EQ(world2.getEgoVehicles().size(), 0);
}

TEST_F(WorldTest, SetEgoVehicles) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);

    // world 1 (remove ego vehicles)
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)};
    EXPECT_EQ(world1.getEgoVehicles().size(), 16);
    std::vector<std::shared_ptr<Obstacle>> egos{};
    world1.setEgoVehicles(egos);
    EXPECT_EQ(world1.getEgoVehicles().size(), 0);

    // world 2 (add ego vehicles)
    auto world2{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, {}, obstaclesScenarioOne, timeStepSizeOne)};
    EXPECT_EQ(world2.getEgoVehicles().size(), 0);
    egos = {obstaclesScenarioOne.front()};
    world2.setEgoVehicles(egos);
    EXPECT_EQ(world2.getEgoVehicles().size(), 1);
    EXPECT_EQ(world2.getEgoVehicles()[0]->getId(), obstaclesScenarioOne.front()->getId());
}

TEST_F(WorldTest, IdCounterRef) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, timeStepSizeOne)};
    EXPECT_EQ(*world1.getIdCounterRef().get(), 53904);
}

TEST_F(WorldTest, UpdateObstacles) {
    std::string scenario{"DEU_TestSafeDistance-1_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/predicates/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto obsManip{obstaclesScenarioOne.at(0)};
    auto initialTimeStepTraj{obsManip->getTrajectoryPrediction().begin()->second->getTimeStep()};
    auto wp{WorldParameters(RoadNetworkParameters(), SensorParameters(), ActuatorParameters::egoDefaults(),
                            TimeParameters(5, 0.3), ActuatorParameters::vehicleDefaults())};

    auto world1{World("DEU_TestSafeDistance-1_1_T-1", 0, roadNetworkScenarioOne, {}, {obstaclesScenarioOne.at(0)},
                      timeStepSizeOne, wp)};

    // update obstacle and add new one
    auto obstacleCopyState{obsManip->getTrajectoryPrediction().begin()->second};
    auto obstacleCopyTraj{obsManip->getTrajectoryPrediction()};
    obstacleCopyTraj.erase(obstacleCopyTraj.begin());
    auto obstacleCopy{std::make_shared<Obstacle>(
        Obstacle(obsManip->getId(), obsManip->getObstacleRole(), obstacleCopyState, obsManip->getObstacleType(),
                 obsManip->getVmax(), obsManip->getAmax(), obsManip->getAmaxLong(), obsManip->getAminLong(),
                 obsManip->getReactionTime(), obstacleCopyTraj, obsManip->getGeoShape().getLength(),
                 obsManip->getGeoShape().getWidth()))};

    obstaclesScenarioOne.at(1)->setTimeParameters(wp.getTimeParams());
    std::vector<std::shared_ptr<Obstacle>> newObstacles{obstacleCopy, obstaclesScenarioOne.at(1)};
    EXPECT_NO_THROW(world1.updateObstacles(newObstacles));
    EXPECT_EQ(world1.getObstacles().size(), 2);
    EXPECT_EQ(world1.findObstacle(obsManip->getId())->getCurrentState()->getTimeStep(), initialTimeStepTraj);
    EXPECT_EQ(world1.findObstacle(obsManip->getId())->getTrajectoryPrediction().begin()->second->getTimeStep(),
              initialTimeStepTraj + 1);
    EXPECT_EQ(world1.findObstacle(obsManip->getId())->getTrajectoryHistory().size(), 1);

    // obstacle is not present anymore -> should be considered until relevant history size has passed
    for (size_t idx{0}; idx < 4; idx++) {
        obstacleCopyState = obsManip->getTrajectoryPrediction().begin()->second;
        obstacleCopyTraj = obsManip->getTrajectoryPrediction();
        obstacleCopyTraj.erase(obstacleCopyTraj.begin());
        obstacleCopy = std::make_shared<Obstacle>(
            Obstacle(obsManip->getId(), obsManip->getObstacleRole(), obstacleCopyState, obsManip->getObstacleType(),
                     obsManip->getVmax(), obsManip->getAmax(), obsManip->getAmaxLong(), obsManip->getAminLong(),
                     obsManip->getReactionTime(), obstacleCopyTraj, obsManip->getGeoShape().getLength(),
                     obsManip->getGeoShape().getWidth()));
        newObstacles = {obstacleCopy};
        EXPECT_NO_THROW(world1.updateObstacles(newObstacles));
        EXPECT_EQ(world1.getObstacles().size(), 2);
    }
    obstacleCopyState = obsManip->getTrajectoryPrediction().begin()->second;
    obstacleCopyTraj = obsManip->getTrajectoryPrediction();
    obstacleCopyTraj.erase(obstacleCopyTraj.begin());
    obstacleCopy = std::make_shared<Obstacle>(
        Obstacle(obsManip->getId(), obsManip->getObstacleRole(), obstacleCopyState, obsManip->getObstacleType(),
                 obsManip->getVmax(), obsManip->getAmax(), obsManip->getAmaxLong(), obsManip->getAminLong(),
                 obsManip->getReactionTime(), obstacleCopyTraj, obsManip->getGeoShape().getLength(),
                 obsManip->getGeoShape().getWidth()));
    newObstacles = {obstacleCopy};
    EXPECT_NO_THROW(world1.updateObstacles(newObstacles));
    EXPECT_EQ(world1.getObstacles().size(), 1);

    // check whether new obstacle with history is used
    // (coverage should also be verified via debugging)
    auto hist{obstacleCopyState};
    obstacleCopyState = obsManip->getTrajectoryPrediction().begin()->second;
    obstacleCopyTraj = obsManip->getTrajectoryPrediction();
    obstacleCopyTraj.erase(obstacleCopyTraj.begin());
    obstacleCopy = std::make_shared<Obstacle>(
        Obstacle(obsManip->getId(), obsManip->getObstacleRole(), obstacleCopyState, obsManip->getObstacleType(),
                 obsManip->getVmax(), obsManip->getAmax(), obsManip->getAmaxLong(), obsManip->getAminLong(),
                 obsManip->getReactionTime(), obstacleCopyTraj, obsManip->getGeoShape().getLength(),
                 obsManip->getGeoShape().getWidth()));
    obstacleCopy->setTrajectoryHistory({{hist->getTimeStep(), hist}});
    newObstacles = {obstacleCopy};
    EXPECT_NO_THROW(world1.updateObstacles(newObstacles));
    EXPECT_EQ(world1.getObstacles().size(), 1);
    EXPECT_EQ(world1.getObstacles().at(0)->getTrajectoryHistory().size(), 1);
    EXPECT_EQ(world1.getObstacles().at(0)->getTrajectoryHistory().begin().value<>()->getTimeStep(),
              hist->getTimeStep());
}

TEST_F(WorldTest, UpdateObstaclesTraj) {
    std::string scenario{"DEU_TestSafeDistance-1_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/predicates/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);
    auto obsManip{obstaclesScenarioOne.at(0)};
    auto initialTimeStepTraj{obsManip->getTrajectoryPrediction().begin()->second->getTimeStep()};
    auto wp{WorldParameters(RoadNetworkParameters(), SensorParameters(), ActuatorParameters::egoDefaults(),
                            TimeParameters(5, 0.3), ActuatorParameters::vehicleDefaults())};

    auto world1{World("DEU_TestSafeDistance-1_1_T-1", 0, roadNetworkScenarioOne, {}, {obstaclesScenarioOne.at(0)},
                      timeStepSizeOne, wp)};
    obstaclesScenarioOne.at(1)->setTimeParameters(wp.getTimeParams());

    // update obstacle and add new one
    auto obstacleCopyState{obsManip->getTrajectoryPrediction().begin()->second};
    auto obstacleCopyTraj{obsManip->getTrajectoryPrediction()};
    obstacleCopyTraj.erase(obstacleCopyTraj.begin());
    std::vector<std::shared_ptr<Obstacle>> newObstacles{obstaclesScenarioOne.at(1)};
    std::map<size_t, std::shared_ptr<State>> cstate{{{obsManip->getId(), obstacleCopyState}}};
    std::map<size_t, tsl::robin_map<time_step_t, std::shared_ptr<State>>> traj{{{obsManip->getId(), obstacleCopyTraj}}};
    EXPECT_NO_THROW(world1.updateObstaclesTraj(newObstacles, cstate, traj));
    EXPECT_EQ(world1.getObstacles().size(), 2);
    EXPECT_EQ(world1.findObstacle(obsManip->getId())->getCurrentState()->getTimeStep(), initialTimeStepTraj);
    EXPECT_EQ(world1.findObstacle(obsManip->getId())->getTrajectoryPrediction().begin()->second->getTimeStep(),
              initialTimeStepTraj + 1);
    EXPECT_EQ(world1.findObstacle(obsManip->getId())->getTrajectoryHistory().size(), 1);

    // obstacle is not present anymore -> should be considered until relevant history size has passed
    newObstacles.clear();
    for (size_t idx{0}; idx < 4; idx++) {
        obstacleCopyState = obsManip->getTrajectoryPrediction().begin()->second;
        obstacleCopyTraj = obsManip->getTrajectoryPrediction();
        obstacleCopyTraj.erase(obstacleCopyTraj.begin());
        cstate = {{obsManip->getId(), obstacleCopyState}};
        traj = {{obsManip->getId(), obstacleCopyTraj}};
        EXPECT_NO_THROW(world1.updateObstaclesTraj(newObstacles, cstate, traj));
        EXPECT_EQ(world1.getObstacles().size(), 2);
    }
    obstacleCopyState = obsManip->getTrajectoryPrediction().begin()->second;
    obstacleCopyTraj = obsManip->getTrajectoryPrediction();
    obstacleCopyTraj.erase(obstacleCopyTraj.begin());
    cstate = {{obsManip->getId(), obstacleCopyState}};
    traj = {{obsManip->getId(), obstacleCopyTraj}};
    EXPECT_NO_THROW(world1.updateObstaclesTraj(newObstacles, cstate, traj));
    EXPECT_EQ(world1.getObstacles().size(), 1);
}
