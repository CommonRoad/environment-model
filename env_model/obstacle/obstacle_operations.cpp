#include "obstacle_operations.h"

#include "../interfaces/commonroad/pugi_xml/pugixml.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

Obstacle *getObstacleByIdAndRemove(std::vector<Obstacle *> *obstacleList, size_t id) {
    Obstacle *temp = nullptr;
    for (auto & i : *obstacleList) {
        if (i->getId() == id) {
            temp = i;
            i = nullptr;
            break;
        }
    }
    return temp;
}

Obstacle *getObstacleById(std::vector<Obstacle *> *obstacleList, size_t id) {
    Obstacle *temp = nullptr;
    for (auto & i : *obstacleList) {
        if (i->getId() == id) {
            temp = i;
            break;
        }
    }
    return temp;
}

bool obstacleInPreviousObstacles(std::vector<Obstacle *> *obstacleList, size_t id) {
    bool temp = false;
    for (auto & i : *obstacleList) {
        if (i->getId() == id) {
            temp = true;
            break;
        }
    }
    return temp;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief Determines whether the type of the obstacle is in the class hierarchy
///
///  Returns true if the obstacle can be dynamically casted to the ClassName (provided as string) and false otherwise
///
//bool isObstacleTypeOfClass(const std::shared_ptr<Obstacle> &Obstacle, const std::string &ClassName) {
//    if (ClassName == "obstacle" && dynamic_cast<Obstacle *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "vehicle" && dynamic_cast<vehicle *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "bicycle" && dynamic_cast<bicycle *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "bus" && dynamic_cast<bus *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "motorcycle" && dynamic_cast<motorcycle *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "passengerCar" && dynamic_cast<passengerCar *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "pedestrian" && dynamic_cast<pedestrian *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "priorityVehicle" && dynamic_cast<priorityVehicle *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "train" && dynamic_cast<train *>(Obstacle.get())) {
//        return true;
//    } else if (ClassName == "truck" && dynamic_cast<truck *>(Obstacle.get())) {
//        return true;
//    } else {
//        Journal::getJournal()->addLog("obstacle " + std::to_string(Obstacle->getId()) + " of type " +
//                                      typeid(*Obstacle).name() + " is not part of Class " + ClassName + '\n');
//        return false;
//    }
//}

