//
// Created by Sebastian Maierhofer on 28.10.20.
//

#include "../../auxiliaryDefs/structs.h"
#include "../commonroad/xml_reader.h"
#include "command_line_input.h"
#include "../../predicates/position_predicates.h"

#include <chrono>

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
    std::vector<std::shared_ptr<Lanelet>> lanelets = XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns,
                                                                                     trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections = XMLReader::createIntersectionFromXML(xmlFilePath,
                                                                                                    lanelets);

    std::shared_ptr<RoadNetwork> roadNetwork{std::make_shared<RoadNetwork>(RoadNetwork(lanelets))};

    for(const auto& obs : obstacles) {
        for (int i=0; i < obs->getTrajectoryLength(); ++i) {
            obs->setReferenceLane(roadNetwork->getLanes(), i);
        }
    }

    PositionPredicates posPred{PositionPredicates(roadNetwork)};

    // Start measuring time
    auto begin = std::chrono::high_resolution_clock::now();

    for(const auto& obs : obstacles) {
        std::cout << obs->getId() << '\n';
        for (int i=0; i < obs->getTrajectoryLength(); ++i) {
            if (posPred.onMainCarriageWay(i, obs))
                std::cout << "time step " << i << ": onMainCarriageWay: true \n";
            else
                std::cout << "time step " << i << ": onMainCarriageWay: false \n";
            if (posPred.onMainCarriageWayRightLane(i, obs))
                std::cout << "time step " << i << ": onMainCarriageWayRightLane: true \n";
            else
                std::cout << "time step " << i << ": onMainCarriageWayRightLane: false \n";
            if (posPred.onAccessRamp(i, obs))
                std::cout << "time step " << i << ": onAccessRamp: true \n";
            else
                std::cout << "time step " << i << ": onAccessRamp: false \n";
            for(const auto& obs2 : obstacles) {
                if(obs->getId() == obs2->getId())
                    continue;
                if (PositionPredicates::inFrontOf(i, obs, obs2))
                    std::cout << "time step " << i << ": obs " << obs2->getId() <<" inFrontOf: true \n";
                else
                    std::cout << "time step " << i << ": obs " << obs2->getId() <<" inFrontOf: false \n";
            }
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

    printf("Time measured: %.3f seconds.\n", elapsed.count() * 1e-9);

    return 0;
}

