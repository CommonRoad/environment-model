//
// Created by sebastian on 28.10.20.
//

#include "../../auxiliaryDefs/structs.h"
#include "../commonroad/XMLReader.h"
#include "command_line_input.h"


int main(int argc, char **argv) {
    float timeStep;
    int num_threads;
    std::string xmlFilePath;

    int error_code = read_command_line_values(argc, argv, timeStep, num_threads, xmlFilePath);
    if (error_code != 0)
        return error_code;

    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);

    return 0;
}

