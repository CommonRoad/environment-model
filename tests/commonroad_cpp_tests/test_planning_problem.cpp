#include "test_planning_problem.h"

#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/planning_problem.h"
#include "interfaces/utility_functions.h"

#include <array>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <optional>

using namespace boost::filesystem;

TEST_F(PlanningProblemTest, PlanningProblemProtobufReader) {
    std::string scenario{"USA_Peach-2_1_T-1"};
    std::string pathToTestFileOne{TestUtils::getTestScenarioDirectory() + "/" +
                                  scenario.substr(0, scenario.size() - 6) + "/" + scenario + ".pb"};
    const auto &[obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne, planningProblems] =
        InputUtils::getDataFromCommonRoad(pathToTestFileOne);

    EXPECT_EQ(planningProblems.size(), 1);

    auto planningProblem = planningProblems.front();
    EXPECT_EQ(planningProblem->getId(), 1500);
    EXPECT_EQ(planningProblem->getEgoId(), std::nullopt);

    auto goalStates = planningProblem->getGoalStates();
    EXPECT_EQ(goalStates.size(), 1);

    auto goalState = goalStates.front();

    auto goalPositions = goalState.getGoalPositions();
    EXPECT_EQ(goalPositions.size(), 0);

    EXPECT_EQ(goalState.getTime(), std::make_pair(100, 105));
    EXPECT_EQ(goalState.getOrientation(), std::nullopt);
    EXPECT_EQ(goalState.getVelocity(), std::nullopt);
}

TEST_F(PlanningProblemTest, PlanningProblemXMLReader) {
    std::string scenarioId{"DEU_Muc-2_1_T-1"};
    std::string scenarioPath = TestUtils::getTestScenarioDirectory() + "/" + scenarioId + ".xml";

    auto scenario = InputUtils::getDataFromCommonRoad(scenarioPath);
    auto planningProblems = scenario.planningProblems;

    EXPECT_EQ(planningProblems.size(), 1);

    auto planningProblem = planningProblems.front();
    EXPECT_EQ(planningProblem->getId(), 800);
    EXPECT_EQ(planningProblem->getEgoId(), std::nullopt);

    auto goalStates = planningProblem->getGoalStates();
    EXPECT_EQ(goalStates.size(), 1);

    auto goalState = goalStates.front();

    auto goalPositions = goalState.getGoalPositions();
    EXPECT_EQ(goalPositions.size(), 0);

    EXPECT_EQ(goalState.getTime(), std::make_pair(0, 30));
    EXPECT_EQ(goalState.getOrientation(), std::make_pair(2.5394, 3.3248));
    EXPECT_EQ(goalState.getVelocity(), std::nullopt);
}
