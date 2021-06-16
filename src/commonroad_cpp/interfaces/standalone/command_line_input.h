//
// Created by Sebastian Maierhofer on 02.11.20.
//

#include <boost/program_options.hpp>
#include <iostream>

#include "commonroad_cpp/interfaces/commonroad/xml_reader.h"

namespace po = boost::program_options;

namespace CommandLine {
    /**
     * Reads the arguments provided via command line.
     *
     * @param argc Argument count: Number of arguments.
     * @param argv Argument vector: List of arguments. (Argument at position 0 is the executable)
     * @param num_threads Reference where number of threads should be stored.
     * @param xmlFilePath Reference where file path to CommonRoad xml should be stored.
     */
    int readCommandLineValues(int argc, char *const *argv, int &num_threads, std::string &xmlFilePath);

    /**
     * Loads and sets up CR scenario.
     * @param xmlFilePath Path to CommonRoad xml file
     * @return Tuple of obstacles and roadNetwork.
     */
    std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>>
    getDataFromCommonRoad(const std::string &xmlFilePath);

} // namespace CommandLine

