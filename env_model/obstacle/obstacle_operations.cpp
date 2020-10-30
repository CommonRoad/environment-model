#include "obstacle_operations.h"

#include "../../inputs/pugi_xml/pugixml.hpp"
#include "../Journal.h"
#include "bicycle.h"
#include "bus.h"
#include "motorcycle.h"
#include "passengerCar.h"
#include "pedestrian.h"
#include "priorityVehicle.h"
#include "train.h"
#include "truck.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

obstacle *getObstacleByIdAndRemove(std::vector<obstacle *> *obstacleList, size_t id) {
    obstacle *temp = 0;
    for (size_t i = 0; i < (*obstacleList).size(); i++) {
        if ((*obstacleList)[i]->getId() == id) {
            temp = (*obstacleList)[i];
            (*obstacleList)[i] = 0;
            break;
        }
    }
    return temp;
}

obstacle *getObstacleById(std::vector<obstacle *> *obstacleList, size_t id) {
    obstacle *temp = 0;
    for (size_t i = 0; i < (*obstacleList).size(); i++) {
        if ((*obstacleList)[i]->getId() == id) {
            temp = (*obstacleList)[i];
            break;
        }
    }
    return temp;
}

bool obstacleInPreviousObstacles(std::vector<obstacle *> *obstacleList, size_t id) {
    bool temp = false;
    for (size_t i = 0; i < (*obstacleList).size(); i++) {
        if ((*obstacleList)[i]->getId() == id) {
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
bool isObstacleTypeOfClass(const std::shared_ptr<obstacle> &Obstacle, const std::string &ClassName) {
    if (ClassName == "obstacle" && dynamic_cast<obstacle *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "vehicle" && dynamic_cast<vehicle *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "bicycle" && dynamic_cast<bicycle *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "bus" && dynamic_cast<bus *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "motorcycle" && dynamic_cast<motorcycle *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "passengerCar" && dynamic_cast<passengerCar *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "pedestrian" && dynamic_cast<pedestrian *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "priorityVehicle" && dynamic_cast<priorityVehicle *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "train" && dynamic_cast<train *>(Obstacle.get())) {
        return true;
    } else if (ClassName == "truck" && dynamic_cast<truck *>(Obstacle.get())) {
        return true;
    } else {
        Journal::getJournal()->addLog("obstacle " + std::to_string(Obstacle->getId()) + " of type " +
                                      typeid(*Obstacle).name() + " is not part of Class " + ClassName + '\n');
        return false;
    }
}

// in developement
void obstacleInteraction(std::vector<obstacle *> *obstacles) {
    size_t i, j, k, l;
    for (i = 0; i < obstacles->size(); i++) {
        if (!dynamic_cast<vehicle *>((*obstacles)[i])) {
            continue;
        }
        for (j = i + 1; j < obstacles->size(); j++) {
            if (!dynamic_cast<vehicle *>((*obstacles)[j])) {
                continue;
            }
            for (k = 0; k < (*obstacles)[i]->getInLane().size(); k++) {
                for (l = 0; l < (*obstacles)[j]->getInLane().size(); l++) {
                    if ((*obstacles)[i]->getInLane()[k]->getId() == (*obstacles)[i]->getInLane()[l]->getId()) {
                    } else if (mergingLanes((*obstacles)[i]->getInLane()[k], (*obstacles)[i]->getInLane()[l])) {
                    } else if (roadFork((*obstacles)[i]->getInLane()[k], (*obstacles)[i]->getInLane()[l])) {
                    }
                }
            }
        }
    }
}
