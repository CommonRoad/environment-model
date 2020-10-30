//
// Created by sebastian on 28.10.20.
//

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "../../auxiliaryDefs/structs.h"
#include "../commonroad/XMLReader.h"
#include "../../road_network/lanelet/lanelet.h"
//#include "../../world/Journal.h"
//#include "../../world/lanelets/lanelet_operations.h"
//#include "../../world/obstacle/obstacle_operations.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char **argv) {
    // By default, logging is OFF. Use Journal class as depicted below to print the logs.
    // If compile configuration is DEBUG, then default value for logging mode is also ON(true).
    /*
    Journal::getJournal()->setLoggingMode(true);
     */
    float timeStep;
    int num_threads;

    std::string xmlFileName, xmlFilePath;
    try {

        po::options_description desc("Allowed options");
        desc.add_options()("help", "produce help message")(
                "timestep,s", po::value<float>(&timeStep)->default_value(0.1), "set time step for prediction")(
                "input-file",
                po::value<std::string>(&xmlFilePath)->default_value("unittest_data/DEU_Muc-3_1_T-1.xml")->required(),
                "Input file")("threads,t", po::value<int>(&num_threads)->default_value(4),
                                             "set number of threads to run with");
        po::positional_options_description p;
        p.add("input-file", -1);

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << "\n";
            return 0;
        }

        xmlFileName = fs::path(xmlFilePath).stem().string();

    } catch (std::exception &e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Exception of unknown type!\n";
        return 1;
    }

    std::cout << "[*] Using file " << xmlFilePath << std::endl;
    std::cout << "[*] Using timestep: " << timeStep << std::endl;
    std::cout << "[*] Using that many threads: " << num_threads << std::endl;

//    timeStruct timeHorizon{0.0, timeStep, 1.0};

    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath);

//    std::vector<std::shared_ptr<obstacle>> obstacleVector{};
//    int timeStamp = 0;
//    obstacleVector = XMLReader::createObstacleFromXML(xmlFilePath, timeStamp);
//
//    std::shared_ptr<EgoVehicle> tempEgo =
//            std::make_shared<EgoVehicle>(); // empty ego vehicle with default values from constructor
//
//    Spot *SpotInterface = Spot::getInstance();
    int ScenarioID = 1;

//    uint8_t returnValue =
//            SpotInterface->registerScenario(ScenarioID, std::move(lanelets), std::move(obstacleVector), std::move(tempEgo));
//    if (returnValue) {
//        std::cout << "Creation of Scenario terminated with error: " << returnValue << std::endl;
//        return 1;
//    }
//
//    returnValue = SpotInterface->triggerOccupancyPrediction(ScenarioID, timeHorizon.startingTime, timeHorizon.timeStep,
//                                                            timeHorizon.ending, num_threads);
//    if (returnValue) {
//        std::cout << "Prediction of Scenario terminated with error: " << returnValue << std::endl;
//        return 2;
//    }
//
//    SpotInterface->writeScenarioToXML(ScenarioID, xmlFileName);
//
//    returnValue = SpotInterface->removeScenario(ScenarioID);
//    if (returnValue) {
//        std::cout << "Removal of Scenario terminated with error: " << returnValue << std::endl;
//        return 3;
//    }

    return 0;
}
