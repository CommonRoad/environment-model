//
// Created by sebastian on 28.10.20.
//

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "../../auxiliaryDefs/structs.h"
#include "../commonroad/XMLReader.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

void read_command_line_values(int argc, char *const *argv, float &timeStep, int &num_threads, std::string &xmlFilePath,
                              po::options_description &desc, po::variables_map &vm);

int main(int argc, char **argv) {
    float timeStep;
    int num_threads;

    std::string xmlFileName, xmlFilePath;
    try {
        po::options_description desc;
        po::variables_map vm;
        read_command_line_values(argc, argv, timeStep, num_threads, xmlFilePath, desc, vm);

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

    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);

    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath);

    int timeStamp = 0;
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath, timeStamp);

    return 0;
}

void read_command_line_values(int argc, char *const *argv, float &timeStep, int &num_threads, std::string &xmlFilePath,
                              po::options_description &desc, po::variables_map &vm) {
    desc.add_options()("help", "produce help message")(
            "timestep,s", po::value<float>(&timeStep)->default_value(0.1), "set time step for prediction")(
            "input-file",
            boost::program_options::value<std::string>(&xmlFilePath)->default_value("../test_scenarios/USA_Lanker-1_1_T-1.xml")->required(),
            "Input file")("threads,t", po::value<int>(&num_threads)->default_value(4),
                                         "set number of threads to run with");
    po::positional_options_description p;
    p.add("input-file", -1);
    boost::program_options::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);
}
