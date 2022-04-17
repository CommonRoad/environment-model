//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include "test_interfaces.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include "commonroad_cpp/interfaces/standalone/command_line_input.h"
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>

TEST_F(InterfacesTest, Read2018bFileSingleThread) {
    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc{5};
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/DEU_Muc-2_1_T-1.xml"};
    const char *array[5]{"myprogram", "--input-file", pathToTestFile.c_str(), "--t", "1"};
    char **argv{const_cast<char **>(array)};
    int error_code = InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);

    EXPECT_EQ(trafficSigns.size(), 2); // virtual speed limit signs for lanes who have a speed limit
    EXPECT_EQ(trafficLights.size(), 0);
    EXPECT_EQ(intersections.size(), 0);
    EXPECT_EQ(obstacles.size(), 5);
    EXPECT_EQ(lanelets.size(), 2);

    auto lanelet34782 =
        *std::find_if(lanelets.begin(), lanelets.end(), [](auto &lptr) { return lptr->getId() == 34782; });
    auto virtualSpeedLimitElem = lanelet34782->getTrafficSigns()[0]->getTrafficSignElements()[0];
    EXPECT_EQ(virtualSpeedLimitElem->getId(), TrafficSignIDGermany.at(TrafficSignTypes::MAX_SPEED));
    EXPECT_EQ(virtualSpeedLimitElem->getAdditionalValues()[0], "14");
}

TEST_F(InterfacesTest, Read2018bFileMultiThread) {
    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc{5};
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/DEU_Muc-2_1_T-1.xml"};
    const char *array[5]{"myprogram", "--input-file", pathToTestFile.c_str(), "--t", "4"};
    char **argv{const_cast<char **>(array)};
    int error_code = InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);

    EXPECT_EQ(trafficSigns.size(), 2);
    EXPECT_EQ(trafficLights.size(), 0);
    EXPECT_EQ(intersections.size(), 0);
    EXPECT_EQ(obstacles.size(), 5);
    EXPECT_EQ(lanelets.size(), 2);
}

TEST_F(InterfacesTest, Read2020aFileSingleThread) {
    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc{5};
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    const char *array[5]{"myprogram", "--input-file", pathToTestFile.c_str(), "--t", "1"};
    char **argv{const_cast<char **>(array)};
    int error_code = InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);

    EXPECT_EQ(trafficSigns.size(), 95);
    EXPECT_EQ(trafficLights.size(), 8);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(obstacles.size(), 24);
    EXPECT_EQ(lanelets.size(), 95);
}

TEST_F(InterfacesTest, Read2020aFileMultiThread) {
    // Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc{5};
    std::string pathToTestFile{TestUtils::getTestScenarioDirectory() + "/USA_Lanker-1_1_T-1.xml"};
    const char *array[5]{"myprogram", "--input-file", pathToTestFile.c_str(), "--t", "4"};
    char **argv{const_cast<char **>(array)};
    int error_code = InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);

    EXPECT_EQ(trafficSigns.size(), 95);
    EXPECT_EQ(trafficLights.size(), 8);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(obstacles.size(), 24);
    EXPECT_EQ(lanelets.size(), 95);
}

TEST_F(InterfacesTest, ReadCommandLineValuesValidDefault) {
    int num_threads;
    std::string xmlFilePath;
    int argc{1};
    char **argv{};
    int error_code{InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath)};

    EXPECT_EQ(error_code, 0);
    EXPECT_EQ(num_threads, 1);
    EXPECT_EQ(xmlFilePath, "../src/commonroad_cpp_tests/default_config.xml");
}

TEST_F(InterfacesTest, ReadCommandLineValuesValidParameters) {
    int num_threads;
    std::string xmlFilePath;
    int argc{5};
    const char *array[5]{"myprogram", "--input-file", "test", "--t", "4"};
    char **argv{const_cast<char **>(array)};
    int error_code{InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath)};

    EXPECT_EQ(error_code, 0);
    EXPECT_EQ(num_threads, 4);
    EXPECT_EQ(xmlFilePath, "test");
}

TEST_F(InterfacesTest, ReadCommandLineValuesHelp) {
    int num_threads;
    std::string xmlFilePath;
    int argc{2};
    const char *array[5]{"myprogram", "--help"};
    char **argv{const_cast<char **>(array)};
    int error_code{InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath)};

    EXPECT_EQ(error_code, 0);
}

TEST_F(InterfacesTest, ReadCommandLineValuesWrongNumberArguments) {
    int num_threads;
    std::string xmlFilePath;
    int argc{4};
    const char *array[5]{"myprogram", "--help"};
    char **argv{const_cast<char **>(array)};
    int error_code{InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath)};

    EXPECT_EQ(error_code, 1);
}

TEST_F(InterfacesTest, ReadCommandLineValuesUnrecognizedOption) {
    int num_threads;
    std::string xmlFilePath;
    int argc{2};
    const char *array[5]{"myprogram", "--abc"};
    char **argv{const_cast<char **>(array)};
    int error_code{InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath)};

    EXPECT_EQ(error_code, 1);
}

TEST_F(InterfacesTest, ReadCommandLineValuesMissingOption) {
    int num_threads;
    std::string xmlFilePath;
    int argc{2};
    const char *array[5]{"myprogram", "--input-file"};
    char **argv{const_cast<char **>(array)};
    int error_code{InputUtils::readCommandLineValues(argc, argv, num_threads, xmlFilePath)};
    EXPECT_EQ(error_code, 1);
}

TEST_F(InterfacesTest, InitializeSimulationParameters) {
    auto simulationParameters{InputUtils::initializeSimulationParameters(
        TestUtils::getTestScenarioDirectory() + "/../../src/commonroad_cpp/default_config.yaml")};
    EXPECT_EQ(simulationParameters.performanceMeasurement, true);
    EXPECT_EQ(simulationParameters.outputFileName, "predicate_satisfaction.csv");
    EXPECT_EQ(simulationParameters.outputDirectory, "src/commonroad_cpp/predicates");
    EXPECT_EQ(simulationParameters.benchmarkId, "DEU_test_safe_distance");
    EXPECT_EQ(simulationParameters.evaluationMode, EvaluationMode::directory);
    EXPECT_EQ(simulationParameters.directoryPaths.size(), 1);
    EXPECT_EQ(simulationParameters.directoryPaths.at(0), "./tests/scenarios/predicates");
    EXPECT_EQ(simulationParameters.egoVehicleId, 0);
}

TEST_F(InterfacesTest, StringToEvaluationMode) {
    std::string str{"directory"};
    EXPECT_EQ(InputUtils::stringToEvaluationMode(str), EvaluationMode::directory);
    str = "single_scenario";
    EXPECT_EQ(InputUtils::stringToEvaluationMode(str), EvaluationMode::singleScenario);
    str = "single_vehicle";
    EXPECT_EQ(InputUtils::stringToEvaluationMode(str), EvaluationMode::singleVehicle);
    str = "directory_single_vehicle";
    EXPECT_EQ(InputUtils::stringToEvaluationMode(str), EvaluationMode::directory_single_vehicle);
    str = "fail";
    EXPECT_THROW(InputUtils::stringToEvaluationMode(str), std::runtime_error);
}

TEST_F(InterfacesTest, FindRelevantScenarioFileNames) {
    EXPECT_EQ(InputUtils::findRelevantScenarioFileNames(TestUtils::getTestScenarioDirectory() + "/predicates").size(),
              4);
    EXPECT_EQ(InputUtils::findRelevantScenarioFileNames(TestUtils::getTestScenarioDirectory()).size(), 12);
}