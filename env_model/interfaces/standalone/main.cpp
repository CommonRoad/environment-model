//
// Created by sebastian on 28.10.20.
//

#include "../../auxiliaryDefs/structs.h"
#include "../commonroad/XMLReader.h"
#include "command_line_input.h"


int main(int argc, char **argv) {
    //Read command line parameters; if none are provided, use default values (specified in read_command_line_values)
    int num_threads;
    std::string xmlFilePath;
    int error_code = CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath);
    if (error_code != 0)
        return error_code;

    //Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);

    return 0;
}
