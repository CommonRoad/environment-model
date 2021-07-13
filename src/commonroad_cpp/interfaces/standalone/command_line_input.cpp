//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <boost/program_options.hpp>
#include <iostream>

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

#include "command_line_input.h"

namespace CommandLine {

/**
 * Reads the arguments provided via command line.
 *
 * @param argc Argument count: Number of arguments.
 * @param argv Argument vector: List of arguments. (Argument at position 0 is the executable)
 * @param num_threads Reference where number of threads should be stored.
 * @param xmlFilePath Reference where file path to CommonRoad xml should be stored.
 */
int readCommandLineValues(int argc, char *const *argv, int &num_threads, std::string &xmlFilePath) {
    try {
        std::string xmlFileName;
        po::options_description desc;
        po::variables_map vm;
        desc.add_options()("help", "produce help message")(
            "input-file",
            boost::program_options::value<std::string>(&xmlFilePath)
                ->default_value("../testScenarios/USA_Lanker-1_1_T-1.xml")
                ->required(),
            "Input file")("threads,t", po::value<int>(&num_threads)->default_value(1),
                          "set number of threads to run with");
        po::positional_options_description p;
        p.add("input-file", -1);
        boost::program_options::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        po::notify(vm);

        if (vm.count("help") != 0u) {
            std::cout << desc << "\n";
            return 0;
        }

        std::cout << "[*] Using file " << xmlFilePath << std::endl;
        std::cout << "[*] Using that many threads: " << num_threads << std::endl;

        return 0;
    } catch (std::exception &e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Exception of unknown type!\n";
        return 1;
    }
}

/**
 * Loads and sets up CR scenario.
 * @param xmlFilePath Path to CommonRoad xml file
 * @return Tuple of obstacles and roadNetwork.
 */
std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>>
getDataFromCommonRoad(const std::string &xmlFilePath) {
    // Read and parse CommonRoad scenario file
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);
    auto country{XMLReader::extractCountryFromXML(xmlFilePath)};

    std::shared_ptr<RoadNetwork> roadNetwork{
        std::make_shared<RoadNetwork>(RoadNetwork(lanelets, country, trafficSigns, trafficLights, intersections))};

    return std::make_tuple(obstacles, roadNetwork);
}

} // namespace CommandLine
