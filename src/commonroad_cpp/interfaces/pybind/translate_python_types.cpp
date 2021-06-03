//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "translate_python_types.h"
#include "pybind11/numpy.h"

#include "commonroad_cpp/geometry/circle.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"

std::vector<std::shared_ptr<TrafficSign>>
TranslatePythonTypes::convertTrafficSigns(const py::handle &py_laneletNetwork) {
    std::vector<std::shared_ptr<TrafficSign>> trafficSignContainer;
    const py::list &py_trafficSigns = py_laneletNetwork.attr("traffic_signs").cast<py::list>();
    trafficSignContainer.reserve(py_trafficSigns.size()); // Already know the size --> Faster memory allocation

    for (const auto &py_trafficSign : py_trafficSigns) {
        std::shared_ptr<TrafficSign> tempTrafficSign = std::make_shared<TrafficSign>();
        tempTrafficSign->setId(py_trafficSign.attr("traffic_sign_id").cast<int>());
        const py::list &py_trafficSignElements = py_trafficSign.attr("traffic_sign_elements").cast<py::list>();
        for (const py::handle &py_trafficSignElement : py_trafficSignElements) {
            std::string trafficSignElementId = py_trafficSignElement.attr("traffic_sign_element_id").cast<py::str>();
            std::shared_ptr<TrafficSignElement> newTrafficSignElement =
                std::make_shared<TrafficSignElement>(trafficSignElementId);
            const py::list &additionalValues = py_trafficSignElement.attr("additional_values").cast<py::list>();
            std::vector<std::string> additionalValuesList{};
            for (const auto &py_additional_value : additionalValues)
                additionalValuesList.push_back(py_additional_value.cast<std::string>());
            newTrafficSignElement->setAdditionalValues(additionalValuesList);
        }
        tempTrafficSign->setVirtualElement(py_trafficSign.attr("virtual").cast<bool>());
        py::array_t<double> py_trafficSignPosition = py::getattr(py_trafficSign, "position");
        tempTrafficSign->setPosition({py_trafficSignPosition.at(0), py_trafficSignPosition.at(1)});
        trafficSignContainer.emplace_back(tempTrafficSign);
    }

    return trafficSignContainer;
}

std::vector<std::shared_ptr<TrafficLight>>
TranslatePythonTypes::convertTrafficLights(const py::handle &py_laneletNetwork) {
    std::vector<std::shared_ptr<TrafficLight>> trafficLightContainer;
    const py::list &py_trafficLights = py_laneletNetwork.attr("traffic_lights").cast<py::list>();
    trafficLightContainer.reserve(py_trafficLights.size()); // Already know the size --> Faster memory allocation

    for (const auto &py_trafficLight : py_trafficLights) {
        std::shared_ptr<TrafficLight> tempTrafficLight = std::make_shared<TrafficLight>();
        tempTrafficLight->setId(py_trafficLight.attr("traffic_light_id").cast<int>());
        tempTrafficLight->setOffset(py_trafficLight.attr("time_offset").cast<int>());
        const py::list &py_trafficLightCycle = py_trafficLight.attr("cycle").cast<py::list>();
        std::vector<TrafficLightCycleElement> cycle;
        for (const py::handle &py_cycleElement : py_trafficLightCycle) {
            cycle.push_back({TrafficLight::matchTrafficLightState(py_cycleElement.attr("state").cast<py::str>()),
                             py_cycleElement.attr("duration").cast<int>()});
        }
        tempTrafficLight->setActive(py_trafficLight.attr("active").cast<bool>());
        py::array_t<double> py_trafficLightPosition = py::getattr(py_trafficLight, "position");
        tempTrafficLight->setPosition({py_trafficLightPosition.at(0), py_trafficLightPosition.at(1)});
        tempTrafficLight->setDirection(
            TrafficLight::matchTurningDirections(py_trafficLight.attr("direction").cast<py::str>()));
        trafficLightContainer.emplace_back(tempTrafficLight);
    }
    return trafficLightContainer;
}

std::shared_ptr<StopLine>
TranslatePythonTypes::convertStopLine(const py::handle &py_stopLine,
                                      const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                      const std::vector<std::shared_ptr<TrafficLight>> &trafficLights) {
    std::shared_ptr<StopLine> sl;
    sl->setLineMarking(matchStringToLineMarking(py::cast<std::string>(py_stopLine.attr("line_marking"))));
    py::object py_trafficSigns = py_stopLine.attr("_traffic_sign_ref");
    for (const auto &sign : trafficSigns) {
        if (sign->getId() == py_trafficSigns.cast<int>()) {
            sl->setTrafficSign(sign);
            break;
        }
    }
    py::object py_trafficLights = py_stopLine.attr("_traffic_light_ref");
    for (const auto &light : trafficLights) {
        if (light->getId() == py_trafficLights.cast<int>()) {
            sl->setTrafficLight(light);
            break;
        }
    }
    py::array_t<double> py_stopLineStartPosition = py::getattr(py_stopLine, "_start");
    py::array_t<double> py_stopLineEndPosition = py::getattr(py_stopLine, "_end");
    sl->setPoints({{py_stopLineStartPosition.at(0), py_stopLineStartPosition.at(1)},
                   {py_stopLineEndPosition.at(0), py_stopLineEndPosition.at(1)}});
    return sl;
}

std::vector<std::shared_ptr<Lanelet>>
TranslatePythonTypes::convertLanelets(const py::handle &py_laneletNetwork,
                                      const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                      const std::vector<std::shared_ptr<TrafficLight>> &trafficLights) {
    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    const py::list &py_lanelets = py_laneletNetwork.attr("lanelets").cast<py::list>();
    tempLaneletContainer.reserve(py_lanelets.size()); // Already know the size --> Faster memory allocation

    // all lanelets must be initialized first because they are referencing each other
    for (size_t i = 0; i < py_lanelets.size(); i++) {
        std::shared_ptr<Lanelet> tempLanelet = std::make_shared<Lanelet>();
        tempLaneletContainer.emplace_back(tempLanelet);
    }

    size_t arrayIndex{0};
    // set id of lanelets
    for (py::handle py_singleLanelet : py_lanelets) {
        tempLaneletContainer[arrayIndex]->setId(py::cast<size_t>(py_singleLanelet.attr("lanelet_id")));
        arrayIndex++;
    }

    arrayIndex = 0;
    for (py::handle py_singleLanelet : py_lanelets) {
        // add left vertices
        py::handle py_leftVertices = py_singleLanelet.attr("left_vertices");
        for (auto &el : py_leftVertices) {
            vertex newVertex{el.cast<py::array_t<double>>().at(0), el.cast<py::array_t<double>>().at(1)};
            tempLaneletContainer[arrayIndex]->addLeftVertex(newVertex);
        }
        // add right vertices
        py::array_t<double> py_rightVertices = py::getattr(py_singleLanelet, "right_vertices");
        for (auto &el : py_rightVertices) {
            vertex newVertex{el.cast<py::array_t<double>>().at(0), el.cast<py::array_t<double>>().at(1)};
            tempLaneletContainer[arrayIndex]->addRightVertex(newVertex);
        }
        // add users one way
        const py::list &py_laneletUserOneWay = py_singleLanelet.attr("user_one_way").cast<py::list>();
        std::set<ObstacleType> usersOneWay;
        for (py::handle py_user : py_laneletUserOneWay)
            usersOneWay.insert(matchStringToObstacleType(py::cast<std::string>(py_user.attr("value"))));
        tempLaneletContainer[arrayIndex]->setUserOneWay(usersOneWay);
        // add users bidirectional
        const py::list &py_laneletUserBidirectional = py_singleLanelet.attr("user_bidirectional").cast<py::list>();
        std::set<ObstacleType> usersBidirectional;
        for (py::handle py_user : py_laneletUserBidirectional)
            usersBidirectional.insert(matchStringToObstacleType(py::cast<std::string>(py_user.attr("value"))));
        tempLaneletContainer[arrayIndex]->setUserBidirectional(usersBidirectional);
        // add lanelet types
        const py::list &py_laneletTypes = py_singleLanelet.attr("lanelet_type").cast<py::list>();
        std::set<LaneletType> laneletTypes;
        for (py::handle py_type : py_laneletTypes)
            laneletTypes.insert(matchStringToLaneletType(py::cast<std::string>(py_type.attr("value"))));
        tempLaneletContainer[arrayIndex]->setLaneletType(laneletTypes);
        // set line markings
        tempLaneletContainer[arrayIndex]->setLineMarkingLeft(matchStringToLineMarking(
            py::cast<std::string>(py_singleLanelet.attr("line_marking_left_vertices").attr("value"))));
        tempLaneletContainer[arrayIndex]->setLineMarkingRight(matchStringToLineMarking(
            py::cast<std::string>(py_singleLanelet.attr("line_marking_left_vertices").attr("value"))));
        // set successors
        py::object py_successors = py_singleLanelet.attr("successor");
        for (py::handle py_item : py_successors) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == py_item.cast<size_t>()) {
                    tempLaneletContainer[arrayIndex]->addSuccessor(la);
                    break;
                }
            }
        }
        // set predecessors
        py::object py_predecessors = py_singleLanelet.attr("predecessor");
        for (py::handle py_item : py_predecessors) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == py_item.cast<size_t>()) {
                    tempLaneletContainer[arrayIndex]->addPredecessor(la);
                    break;
                }
            }
        }
        // add adjacent left
        py::object py_adjLeft = py_singleLanelet.attr("adj_left");
        if (py_adjLeft.get_type().attr("__name__").cast<std::string>() == "int") {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == py_adjLeft.cast<size_t>()) {
                    if (py_singleLanelet.attr("adj_left_same_direction").cast<bool>()) // same direction
                        tempLaneletContainer[arrayIndex]->setLeftAdjacent(la, DrivingDirection::same);
                    else // opposite direction
                        tempLaneletContainer[arrayIndex]->setLeftAdjacent(la, DrivingDirection::opposite);
                    break;
                }
            }
        }
        // add adjacent right
        py::object py_adjRight = py_singleLanelet.attr("adj_right");
        if (py_adjRight.get_type().attr("__name__").cast<std::string>() == "int") {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == py_adjRight.cast<size_t>()) {
                    if (py_singleLanelet.attr("adj_right_same_direction").cast<bool>()) // same direction
                        tempLaneletContainer[arrayIndex]->setRightAdjacent(la, DrivingDirection::same);
                    else // opposite direction
                        tempLaneletContainer[arrayIndex]->setRightAdjacent(la, DrivingDirection::opposite);
                    break;
                }
            }
        }
        // add traffic signs
        py::object py_trafficSigns = py_singleLanelet.attr("traffic_signs");
        for (const auto &sign : trafficSigns) {
            for (const auto &py_sign : py_trafficSigns)
                if (sign->getId() == py_sign.cast<int>()) {
                    tempLaneletContainer[arrayIndex]->addTrafficSign(sign);
                    break;
                }
        }
        // add traffic signs
        py::object py_trafficLights = py_singleLanelet.attr("traffic_lights");
        for (const auto &light : trafficLights) {
            for (const auto &py_light : py_trafficLights)
                if (light->getId() == py_light.cast<int>()) {
                    tempLaneletContainer[arrayIndex]->addTrafficLight(light);
                    break;
                }
        }
        py::handle py_stopLine = py_singleLanelet.attr("stop_line");
        if (py_stopLine != Py_None)
            tempLaneletContainer[arrayIndex]->setStopLine(convertStopLine(py_stopLine, trafficSigns, trafficLights));
        tempLaneletContainer[arrayIndex]->createCenterVertices();
        tempLaneletContainer[arrayIndex]->constructOuterPolygon();
        arrayIndex++;
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<Intersection>>
TranslatePythonTypes::convertIntersections(const py::handle &py_laneletNetwork,
                                           const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempIntersectionContainer{};
    const py::list &py_intersection_list = py_laneletNetwork.attr("intersections").cast<py::list>();
    size_t n = py_intersection_list.size();
    tempIntersectionContainer.reserve(n); // Already know the size --> Faster memory allocation
    // all intersections must be initialized first
    for (size_t i = 0; i < py_intersection_list.size(); i++) {
        std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
        tempIntersectionContainer.emplace_back(tempIntersection);
    }

    size_t intersectionIndex{0};
    for (const auto &py_intersection : py_intersection_list) {
        std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
        tempIntersection->setId(py_intersection.attr("intersection_id").cast<int>());
        std::vector<std::shared_ptr<Incoming>> incomings;
        incomings.reserve(py_intersection.attr("incomings").cast<py::list>().size());
        for (const auto &py_incoming : py_intersection.attr("incomings").cast<py::list>()) {
            std::shared_ptr<Incoming> tempIncoming = std::make_shared<Incoming>();
            tempIncoming->setId(py_incoming.attr("incoming_id").cast<int>());
            incomings.emplace_back(tempIncoming);
        }
        size_t incomignIndex{0};
        for (const auto &py_incoming : py_intersection.attr("incomings").cast<py::list>()) {
            // incoming lanelets
            auto py_incomingLanelets = py_incoming.attr("incoming_lanelets").cast<py::list>();
            std::vector<std::shared_ptr<Lanelet>> incomingLanelets;
            for (auto &incomingLaneletId : py_incomingLanelets) {
                size_t incId{incomingLaneletId.cast<size_t>()};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        incomingLanelets.push_back(la);
                        break;
                    }
                }
            }
            // successor right lanelets
            auto py_successorRight = py_incoming.attr("successors_right").cast<py::list>();
            std::vector<std::shared_ptr<Lanelet>> successorRightLanelets;
            for (auto &successorRightLaneletId : py_successorRight) {
                size_t incId{successorRightLaneletId.cast<size_t>()};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        successorRightLanelets.push_back(la);
                        break;
                    }
                }
            }
            incomings[incomignIndex]->setSuccessorsRight(successorRightLanelets);
            // successor left lanelets
            auto py_successorLeft = py_incoming.attr("successors_left").cast<py::list>();
            std::vector<std::shared_ptr<Lanelet>> successorLeftLanelets;
            for (auto &successorLeftLaneletId : py_successorLeft) {
                size_t incId{successorLeftLaneletId.cast<size_t>()};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        successorLeftLanelets.push_back(la);
                        break;
                    }
                }
            }
            incomings[incomignIndex]->setSuccessorsLeft(successorLeftLanelets);
            // successor straight lanelets
            auto py_successorStraight = py_incoming.attr("successors_straight").cast<py::list>();
            std::vector<std::shared_ptr<Lanelet>> successorStraightLanelets;
            for (auto &successorStraightLaneletId : py_successorStraight) {
                size_t incId{successorStraightLaneletId.cast<size_t>()};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        successorStraightLanelets.push_back(la);
                        break;
                    }
                }
            }
            incomings[incomignIndex]->setSuccessorsStraight(successorStraightLanelets);
            // left of
            if (py_incoming.attr("left_of").get_type().attr("__name__").cast<std::string>() == "int"){
                size_t py_isLeftOf = py_incoming.attr("left_of").cast<size_t>();
                for (auto &inc : incomings) {
                    if (inc->getId() == py_isLeftOf) {
                        incomings[incomignIndex]->setIsLeftOf(inc);
                        break;
                    }
                }
            }
            incomignIndex++;
        }
        tempIntersectionContainer[intersectionIndex]->setIncomings(incomings);
        // add crossings
        auto py_crossings = py_intersection.attr("crossings").cast<py::list>();
        std::vector<std::shared_ptr<Lanelet>> crossings;
        for (auto &crossing : py_crossings) {
            size_t crossingId{crossing.cast<size_t>()};
            for (const auto &la : lanelets) {
                if (la->getId() == crossingId) {
                    crossings.push_back(la);
                    break;
                }
            }
        }
        tempIntersectionContainer[intersectionIndex]->setCrossings(crossings);
    }
    return tempIntersectionContainer;
}

std::shared_ptr<State> createInitialState(py::handle py_singleObstacle) {
    // TODO add support for uncertain states
    std::shared_ptr<State> initialState = std::make_shared<State>();

    auto initialStateType =
        py_singleObstacle.attr("initial_state").attr("position").get_type().attr("__name__").cast<std::string>();
    if (initialStateType == "ndarray") {
        auto xPos = py_singleObstacle.attr("initial_state").attr("position").cast<py::list>()[0].cast<double>();
        auto yPos = py_singleObstacle.attr("initial_state").attr("position").cast<py::list>()[1].cast<double>();
        initialState->setXPosition(xPos);
        initialState->setYPosition(yPos);
        initialState->setGlobalOrientation(py_singleObstacle.attr("initial_state").attr("orientation").cast<double>());
        if (py::hasattr(py_singleObstacle.attr("initial_state"), "velocity"))
            initialState->setVelocity(py_singleObstacle.attr("initial_state").attr("velocity").cast<double>());
        if (py::hasattr(py_singleObstacle.attr("initial_state"), "acceleration"))
            initialState->setAcceleration(py_singleObstacle.attr("initial_state").attr("acceleration").cast<double>());
    }
    return initialState;
}

std::shared_ptr<Obstacle> createCommonObstaclePart(py::handle py_singleObstacle) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
    tempObstacle->setId(py_singleObstacle.attr("obstacle_id").cast<size_t>());
    tempObstacle->setObstacleType(
        matchStringToObstacleType(py_singleObstacle.attr("obstacle_type").attr("value").cast<std::string>()));
    py::handle py_obstacleShape = py_singleObstacle.attr("obstacle_shape");
    std::string commonroadShape{py_obstacleShape.get_type().attr("__name__").cast<std::string>()};

    if (commonroadShape == "Rectangle") { // TODO: add other shape types
        auto length{py_obstacleShape.attr("length").cast<double>()};
        auto width{py_obstacleShape.attr("width").cast<double>()};
        Rectangle &geoRectangle = dynamic_cast<Rectangle &>(tempObstacle->getGeoShape());
        geoRectangle.setLength(length);
        geoRectangle.setWidth(width);
    } else {
        std::cout << "Unknown obstacle shape (only circles, polygons and rectangles supported) \n";
    }
    tempObstacle->setCurrentState(createInitialState(py_singleObstacle));

    return tempObstacle;
}

std::shared_ptr<State> extractState(py::handle py_state) {
    auto state{std::make_shared<State>()};
    state->setXPosition(py_state.attr("position").cast<py::list>()[0].cast<double>());
    state->setYPosition(py_state.attr("position").cast<py::list>()[1].cast<double>());
    state->setGlobalOrientation(py_state.attr("orientation").cast<double>());
    state->setVelocity(py_state.attr("velocity").cast<double>());
    state->setTimeStep(py_state.attr("time_step").cast<size_t>());
    if (py::hasattr(py_state, "acceleration"))
        state->setAcceleration(py_state.attr("acceleration").cast<double>());

    return state;
}

std::shared_ptr<Obstacle> createDynamicObstacle(py::handle py_singleObstacle) {
    // TODO: add other prediction than trajectory prediction
    std::shared_ptr<Obstacle> tempObstacle = createCommonObstaclePart(py_singleObstacle);
    for (const auto &py_state :
         py_singleObstacle.attr("prediction").attr("trajectory").attr("state_list").cast<py::list>()) {
        tempObstacle->appendStateToTrajectoryPrediction(extractState(py_state));
    }
    return tempObstacle;
}

std::shared_ptr<Obstacle> createStaticObstacle(py::handle py_singleObstacle) {
    std::shared_ptr<Obstacle> tempObstacle = createCommonObstaclePart(py_singleObstacle);
    return tempObstacle;
}

std::vector<std::shared_ptr<Obstacle>> TranslatePythonTypes::convertObstacles(const py::list &py_obstacle_list) {
    std::vector<std::shared_ptr<Obstacle>> tempObstacleContainer{};
    tempObstacleContainer.reserve(py_obstacle_list.size()); // Already know the size --> Faster memory allocation
    // all obstacles must be initialized first
    for (size_t i = 0; i < py_obstacle_list.size(); i++) {
        std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
        tempObstacleContainer.emplace_back(tempObstacle);
    }

    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
    size_t arrayIndex{0};
    for (py::handle py_singleObstacle : py_obstacle_list) {
        std::string obstacleRole{py_singleObstacle.attr("obstacle_role").attr("value").cast<std::string>()};
        if (obstacleRole == "dynamic")
            tempObstacleContainer[arrayIndex] = createDynamicObstacle(py_singleObstacle);
        else if (obstacleRole == "static")
            tempObstacleContainer[arrayIndex] = createStaticObstacle(py_singleObstacle);
        arrayIndex++;
    }

    return tempObstacleContainer;
}
