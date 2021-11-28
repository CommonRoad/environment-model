//
// Created by Sebastian Maierhofer.
// Technical University of Munich - Cyber-Physical Systems Group
// Copyright (c) 2021 Sebastian Maierhofer - Technical University of Munich. All rights reserved.
// Credits: BMW Car@TUM
//

#include <string>
#include <tuple>
#include <vector>

#include "../commonroad/xml_reader.h"
#include <boost/program_options.hpp>

class RoadNetwork;

namespace InputUtils {

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
 * @return Tuple of obstacles, roadNetwork, and time step size.
 */
std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double>
getDataFromCommonRoad(const std::string &xmlFilePath);

/**
 * Initializes CommonRoad evaluation.
 * @param path Path to config file.
 */
SimulationParameters initializeSimulationParameters(const std::string &path);

/**
 * Converts given string to evaluation mode enum.
 *
 * @param evaluationMode String containing evaluation mode (loaded from config file)
 * @return EvaluationMode enum.
 */
EvaluationMode stringToEvaluationMode(const std::string &evaluationMode);

/**
 * Extracts all XML-files within directory.
 *
 * @param dir Directory to search in.
 * @return List of paths to XML files within directory.
 */
std::vector<std::string> findRelevantScenarioFileNames(const std::string &dir);

} // namespace InputUtils
