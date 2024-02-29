#include "test_world.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/predicates/braking/unnecessary_braking_predicate.h"
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
    UnnecessaryBrakingPredicate pred;
    auto world1{World("USA_Peach-2_1_T-1", 0, roadNetworkScenarioOne, obstaclesScenarioOne, {}, 0.1)};
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