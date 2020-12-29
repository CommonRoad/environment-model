//
// Created by sebastian on 26.12.20.
//

#include "test_reading_commonroad.h"
#include "interfaces/standalone/command_line_input.h"
#include "interfaces/commonroad/xml_reader.h"

TEST_F(ReadingCommonRoadTest, Read2018bFileSingleThread){
    //Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc { 5 };
    const char *array[5] {"myprogram", "--input-file", "./tests/testScenarios/DEU_Muc-2_1_T-1.xml", "--t", "1" };
    char **argv {const_cast<char **>(array)};
    int error_code = CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    //Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns,
                                                                                     trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections = XMLReader::createIntersectionFromXML(xmlFilePath,
                                                                                                    lanelets);

    EXPECT_EQ(trafficSigns.size(), 0);
    EXPECT_EQ(trafficLights.size(), 0);
    EXPECT_EQ(intersections.size(), 0);
    EXPECT_EQ(obstacles.size(), 5);
    EXPECT_EQ(lanelets.size(), 2);
}

TEST_F(ReadingCommonRoadTest, Read2018bFileMultiThread){
    //Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc { 5 };
    const char *array[5] {"myprogram", "--input-file", ".tests/testScenarios/DEU_Muc-2_1_T-1.xml", "--t", "4" };
    char **argv {const_cast<char **>(array)};
    int error_code = CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    //Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns,
                                                                                     trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections = XMLReader::createIntersectionFromXML(xmlFilePath,
                                                                                                    lanelets);

    EXPECT_EQ(trafficSigns.size(), 0);
    EXPECT_EQ(trafficLights.size(), 0);
    EXPECT_EQ(intersections.size(), 0);
    EXPECT_EQ(obstacles.size(), 5);
    EXPECT_EQ(lanelets.size(), 2);
}

TEST_F(ReadingCommonRoadTest, Read2020aFileSingleThread){
    //Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc { 5 };
    const char *array[5] {"myprogram", "--input-file", ".tests/testScenarios/USA_Lanker-1_1_T-1.xml", "--t", "1" };
    char **argv {const_cast<char **>(array)};
    int error_code = CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    //Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns,
                                                                                     trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections = XMLReader::createIntersectionFromXML(xmlFilePath,
                                                                                                    lanelets);

    EXPECT_EQ(trafficSigns.size(), 95);
    EXPECT_EQ(trafficLights.size(), 8);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(obstacles.size(), 24);
    EXPECT_EQ(lanelets.size(), 95);
}

TEST_F(ReadingCommonRoadTest, Read2020aFileMultiThread){
    //Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int argc { 5 };
    const char *array[5] {"myprogram", "--input-file", "./tests/testScenarios/USA_Lanker-1_1_T-1.xml", "--t", "4" };
    char **argv {const_cast<char **>(array)};
    int error_code = CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    EXPECT_EQ(error_code, 0);

    //Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns,
                                                                                     trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections = XMLReader::createIntersectionFromXML(xmlFilePath,
                                                                                                    lanelets);

    EXPECT_EQ(trafficSigns.size(), 95);
    EXPECT_EQ(trafficLights.size(), 8);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(obstacles.size(), 24);
    EXPECT_EQ(lanelets.size(), 95);
}