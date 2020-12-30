//
// Created by Sebastian Maierhofer on 02.11.20.
//

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "command_line_input.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;


int CommandLine::readCommandLineValues(int argc, char *const *argv, int &num_threads, std::string &xmlFilePath) {
    try {
        std::string xmlFileName;
        po::options_description desc;
        po::variables_map vm;
        desc.add_options()("help", "produce help message")("input-file",
                                                           boost::program_options::value<std::string>(&xmlFilePath)->
                                                                   default_value(
                                                                   "../testScenarios/USA_Lanker-1_1_T-1.xml")->required(),
                                                           "Input file")("threads,t",
                                                                         po::value<int>(&num_threads)->default_value(1),
                                                                         "set number of threads to run with");
        po::positional_options_description p;
        p.add("input-file", -1);
        boost::program_options::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(),
                                      vm);
        po::notify(vm);

        if (vm.count("help")) {
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

