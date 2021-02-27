//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "translate_python_types.h"
#include "../../obstacle/obstacle_operations.h"
#include "../../roadNetwork/lanelet/lanelet_operations.h"
#include "pybind11/numpy.h"


std::vector<std::shared_ptr<TrafficSign>> TranslatePythonTypes::convertTrafficSigns(const py::handle &py_laneletNetwork) {
    std::vector<std::shared_ptr<TrafficSign>> trafficSignContainer;
    const py::list &py_trafficSigns = py_laneletNetwork.attr("traffic_signs").cast<py::list>();
    trafficSignContainer.reserve(py_trafficSigns.size()); // Already know the size --> Faster memory allocation

    for (const auto &py_trafficSign : py_trafficSigns) {
        std::shared_ptr<TrafficSign> tempTrafficSign = std::make_shared<TrafficSign>();
        tempTrafficSign->setId(py_trafficSign.attr("traffic_sign_id").cast<int>());
        const py::list &py_trafficSignElements = py_trafficSign.attr("traffic_sign_elements").cast<py::list>();
        for (const py::handle &py_trafficSignElement : py_trafficSignElements) {
            std::string trafficSignElementId = py_trafficSignElement.attr("traffic_sign_element_id").cast<py::str>();
            std::shared_ptr<TrafficSignElement> newTrafficSignElement = std::make_shared<TrafficSignElement>(trafficSignElementId);
            const py::list &additionalValues = py_trafficSignElement.attr("additional_values").cast<py::list>();
            std::vector<std::string> additionalValuesList { additionalValues.attr("__str__")().cast<std::vector<std::string>>()};
            newTrafficSignElement->setAdditionalValues(additionalValuesList);
        }
        tempTrafficSign->setVirtualElement(py_trafficSign.attr("virtual").cast<bool>());
        py::array_t<double> py_trafficSignPosition = py::getattr(py_trafficSign, "position");
        tempTrafficSign->setPosition({py_trafficSignPosition.at(0), py_trafficSignPosition.at(1)});
        trafficSignContainer.emplace_back(tempTrafficSign);
    }

    return trafficSignContainer;
}

std::vector<std::shared_ptr<TrafficLight>> TranslatePythonTypes::convertTrafficLights(const py::handle &py_laneletNetwork) {
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
        tempTrafficLight->setDirection(TrafficLight::matchTurningDirections(py_trafficLight.attr("direction").cast<py::str>()));
        trafficLightContainer.emplace_back(tempTrafficLight);
    }
    return trafficLightContainer;
}

std::shared_ptr<StopLine> TranslatePythonTypes::convertStopLine(const py::handle& py_stopLine,
                                                                std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                                                std::vector<std::shared_ptr<TrafficLight>> trafficLights) {
    std::shared_ptr<StopLine> sl;
    sl->setLineMarking(matchStringToLineMarking(py::cast<const char*>(py_stopLine.attr("line_marking"))));
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
    sl->setPoints({ {py_stopLineStartPosition.at(0), py_stopLineStartPosition.at(1)},
                    {py_stopLineEndPosition.at(0), py_stopLineEndPosition.at(1)} });
    return sl;
}

std::vector<std::shared_ptr<Lanelet>>
TranslatePythonTypes::convertLanelets(const py::handle &py_laneletNetwork,
                                      std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                      std::vector<std::shared_ptr<TrafficLight>> trafficLights) {
    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer {};
    const py::list &py_lanelets = py_laneletNetwork.attr("lanelets").cast<py::list>();
    tempLaneletContainer.reserve(py_lanelets.size()); // Already know the size --> Faster memory allocation

    // all lanelets must be initialized first because they are referencing each other
    for (int i = 0; i < py_lanelets.size(); i++) {
        std::shared_ptr<Lanelet> tempLanelet = std::make_shared<Lanelet>();
        tempLaneletContainer.emplace_back(tempLanelet);
    }

    int arrayIndex = 0;
    // set id of lanelets
    for (py::handle py_singleLanelet : py_lanelets) {
        tempLaneletContainer[arrayIndex]->setId(py::cast<int>(py_singleLanelet.attr("lanelet_id")));
        arrayIndex++;
    }

    arrayIndex = 0;
    for (py::handle py_singleLanelet : py_lanelets) {
        // add left vertices
        py::handle py_leftVertices = py_singleLanelet.attr("left_vertices");
        for (auto &el : py_leftVertices) {
            vertex newVertex{ el.cast<py::array_t<double>>().at(0), el.cast<py::array_t<double>>().at(1) };
            tempLaneletContainer[arrayIndex]->addLeftVertex(newVertex);
        }
        // add right vertices
        py::array_t<double> py_rightVertices = py::getattr(py_singleLanelet, "right_vertices");
        for (auto &el : py_rightVertices) {
            vertex newVertex{ el.cast<py::array_t<double>>().at(0), el.cast<py::array_t<double>>().at(1) };
            tempLaneletContainer[arrayIndex]->addRightVertex(newVertex);
        }
        // add users one way
        const py::list &py_laneletUserOneWay = py_singleLanelet.attr("user_one_way").cast<py::list>();
        std::vector<ObstacleType> usersOneWay;
        for (py::handle py_user : py_laneletUserOneWay)
            usersOneWay.push_back(matchStringToObstacleType(py::cast<const char *>(py_user)));
        tempLaneletContainer[arrayIndex]->setUserOneWay(usersOneWay);
        // add users bidirectional
        const py::list &py_laneletUserBidirectional = py_singleLanelet.attr("user_bidirectional").cast<py::list>();
        std::vector<ObstacleType> usersBidirectional;
        for (py::handle py_user : py_laneletUserBidirectional)
            usersBidirectional.push_back(matchStringToObstacleType(py::cast<const char *>(py_user)));
        tempLaneletContainer[arrayIndex]->setUserBidirectional(usersBidirectional);
        // add lanelet types
        const py::list &py_laneletTypes = py_singleLanelet.attr("lanelet_type").cast<py::list>();
        std::vector<LaneletType> laneletTypes;
        for (py::handle py_type : py_laneletTypes)
            laneletTypes.push_back(matchStringToLaneletType(py::cast<const char *>(py_type)));
        tempLaneletContainer[arrayIndex]->setLaneletType(laneletTypes);
        // set line markings
        tempLaneletContainer[arrayIndex]->setLineMarkingLeft(matchStringToLineMarking(py::cast<const char*>(py_singleLanelet.attr("line_marking_left_vertices"))));
        tempLaneletContainer[arrayIndex]->setLineMarkingRight(matchStringToLineMarking(py::cast<const char*>(py_singleLanelet.attr("line_marking_left_vertices"))));
        // set successors
        py::object py_successors = py_singleLanelet.attr("successor");
        for (py::handle py_item : py_successors) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == py_item.cast<int>()) {
                    tempLaneletContainer[arrayIndex]->addSuccessor(la);
                    break;
                }
            }
        }
        // set predecessors
        py::object py_predecessors = py_singleLanelet.attr("predecessors");
        for (py::handle py_item : py_predecessors) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == py_item.cast<int>()) {
                    tempLaneletContainer[arrayIndex]->addPredecessor(la);
                    break;
                }
            }
        }
        // add adjacent left
        py::object py_adjLeft = py_singleLanelet.attr("adj_left");
        if (py_adjLeft.get_type().attr("__name__").cast<std::string>() == "int") {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == py_adjLeft.cast<int>()) {
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
                if (la->getId() == py_adjRight.cast<int>()) {
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
            if (sign->getId() == py_trafficSigns.cast<int>()) {
                tempLaneletContainer[arrayIndex]->addTrafficSign(sign);
                break;
            }
        }
        // add traffic signs
        py::object py_trafficLights = py_singleLanelet.attr("traffic_lights");
        for (const auto &light : trafficLights) {
            if (light->getId() == py_trafficLights.cast<int>()) {
                tempLaneletContainer[arrayIndex]->addTrafficLight(light);
                break;
            }
        }
        tempLaneletContainer[arrayIndex]->setStopLine(convertStopLine(py_singleLanelet.attr("stop_line"), trafficSigns, trafficLights));
        tempLaneletContainer[arrayIndex]->createCenterVertices();
        tempLaneletContainer[arrayIndex]->constructOuterPolygon();
        arrayIndex++;
    }
    return tempLaneletContainer;
}

//void TranslatePythonTypes::convertDynamicObstacles(const py::list &py_dynamicObstacles,
//                                                   std::vector<std::shared_ptr<Obstacle>> &obstacleList) {
//    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
//    for (py::handle py_singleObstacle : py_dynamicObstacles) {
//        tempObstacle->setId(py_singleObstacle.attr("obstacle_id").cast<int>());
//        tempObstacle->setObstacleType(matchObstacleTypeToString(py_singleObstacle.attr("obstacle_type").cast<py::str>()));
//
//
//
//
//
//        //-----------------------------------------------------------
//        // Get position: Different handling if position is uncertain (then instead of x,y value, we have a shape)
//        double xPos = 0, yPos = 0;
//        double uncertaintyAllDim = 0, uncertaintyLength = 0, uncertaintyWidth = 0;
//        std::string tempUncertainty =
//                py_singleObstacle.attr("initial_state").attr("position").get_type().attr("__name__").cast<std::string>();
//        if (tempUncertainty == "ndarray") {
//            // No uncertainty
//            int i = 0;
//            for (auto &elements : py_singleObstacle.attr("initial_state").attr("position")) {
//                if (i == 0) {
//                    xPos = elements.cast<double>();
//                } else {
//                    yPos = elements.cast<double>();
//                }
//                i++;
//            }
//        } else if (tempUncertainty == "Rectangle") {
//            // Uncertainty is given with shape of rectangle
//            int i = 0;
//            for (auto &elements : py_singleObstacle.attr("initial_state").attr("position").attr("center")) {
//                if (i == 0) {
//                    xPos = elements.cast<double>();
//                } else {
//                    yPos = elements.cast<double>();
//                }
//                i++;
//            }
//            UncertaintyLength = py_singleObstacle.attr("initial_state").attr("position").attr("length").cast<double>();
//            UncertaintyWidth = py_singleObstacle.attr("initial_state").attr("position").attr("width").cast<double>();
//        } else if (temp_Uncertainty == "Circle") {
//            // Uncertainty is given with shape of circle
//            int i = 0;
//            for (auto &elements : py_singleObstacle.attr("initial_state").attr("position").attr("center")) {
//                if (i == 0) {
//                    xPos = elements.cast<double>();
//                } else {
//                    yPos = elements.cast<double>();
//                }
//                i++;
//            }
//            // uncertainty of size is given by the diameter
//            UncertaintyAllDim =
//                    2 * py_singleObstacle.attr("initial_state").attr("position").attr("radius").cast<double>();
//        } else if (temp_Uncertainty == "Polygon") {
//            std::cout << "Uncertainty type not included yet" << std::endl;
//            // Todo: Implement old implementation from python (don't have shapely + how is ndaray stuctured
//            // Python code:
//            // minx, miny, maxx, maxy = item.initial_state.position._shapely_polygon.bounds
//            // length_x = abs(maxx-minx)
//            // length_y = abs(maxy-miny)
//            // dynamicObj.setPosition(minx+length_x/2.0, miny+length/2.0)
//            // position_uncertainty = math.sqrt(length_x**2+length_y**2)
//        } else {
//            std::cout << "Unknown uncertainty type for ID: " << temp_ID
//                      << " (only circles, polygons and rectangles supported)" << std::endl;
//            return 2; // Don't know uncertainty type
//        }
//        tempObstacle->setPosition(xPos, yPos); // set position
//
//        //-----------------------------------------------------------
//        // Use extracted uncertainties from above to
//        std::unique_ptr<shape> tempShape(
//                nullptr); // Empty pointer (specific object gets assigned depending on obstacle shape)
//
//        // Get the shape of the obstacle in commonRoad
//        std::string CommonroadShape =
//                py_singleObstacle.attr("obstacle_shape").get_type().attr("__name__").cast<std::string>();
//
//        // SPOT handles the shapes a little differently
//        // Independently from the shape in Commonroad --> Pedestrians are Circles in SPOT, Vehicles are Rectangles
//        if (tempObstacle->getGeoShape().getType() == "Rectangle") {
//            // This means, that this obstacle is some kind of vehicle
//            double RectLength = 0;
//            double RectWidth = 0;
//            double RawLength = 0;
//            double RawWidth = 0;
//
//            if (CommonroadShape == "Rectangle") {
//                RawLength = py_singleObstacle.attr("obstacle_shape").attr("length").cast<double>();
//                RawWidth = py_singleObstacle.attr("obstacle_shape").attr("width").cast<double>();
//
//                // If there were uncertainties, one has to add these to length and width
//                RectLength = RawLength + UncertaintyAllDim + UncertaintyLength;
//                RectWidth = RawWidth + UncertaintyAllDim + UncertaintyWidth;
//            } else if (CommonroadShape == "Circle") {
//                std::cout << "Commonroad vehicles of shape circle currently not supported" << std::endl;
//                return 5;
//            } else if (CommonroadShape == "Polygon") {
//                std::cout << "Obstacles with polygon shape currently not supported" << std::endl;
//                // Todo This would basically be a bounding box = rectangle?
//                return 4;
//            } else {
//                std::cout << "Unknown obstacle shape for ID: " << temp_ID
//                          << " (only circles, polygons and rectangles supported)" << std::endl;
//                return 3; // Don't know obstacle shape
//            }
//
//            tempObstacle->getGeoShape().setLength(RectLength);
//            tempObstacle->getGeoShape().setWidth(RectWidth);
//            tempObstacle->getGeoShape().setLength_raw(RawLength);
//            tempObstacle->getGeoShape().setWidth_raw(RawWidth);
//
//        } else if (tempObstacle->getGeoShape().getType() == "Circle") {
//            // This means that the obstacle is a pedestrian
//            // Small corrections for uncertainties needed if circular shape is used
//            UncertaintyAllDim = UncertaintyAllDim / 2.0; // Radius instead of diameter from before
//            double UncertaintyRadius = sqrt(pow(UncertaintyLength, 2) + pow(UncertaintyWidth, 2)) / 2.0;
//            double Radius = 0; // Radius assigned to circular shape
//
//            if (CommonroadShape == "Rectangle") {
//                double RawLength = py_singleObstacle.attr("obstacle_shape").attr("length").cast<double>();
//                double RawWidth = py_singleObstacle.attr("obstacle_shape").attr("width").cast<double>();
//
//                Radius = sqrt(pow(RawLength, 2) + pow(RawWidth, 2)) / 2.0 + UncertaintyAllDim + UncertaintyRadius;
//            } else if (CommonroadShape == "Circle") {
//                double RawRadius = py_singleObstacle.attr("obstacle_shape").attr("radius").cast<double>();
//
//                Radius = RawRadius + UncertaintyAllDim + UncertaintyRadius + UncertaintyAllDim;
//            } else if (CommonroadShape == "Polygon") {
//                std::cout << "Obstacles with polygon shape currently not supported" << std::endl;
//                // Todo This would basically be a bounding box = rectangle?
//                return 4;
//            } else {
//                std::cout << "Unknown obstacle shape for ID: " << temp_ID
//                          << " (only circles, polygons and rectangles supported)" << std::endl;
//                return 3; // Don't know obstacle shape
//            }
//
//            // Center point can be assinged independent from Commonroad Shape (all use center attribute
//            vertex CenterPoint; // Center point assigned to circular shape
//            py::object pyCenter = py_singleObstacle.attr("obstacle_shape").attr("center");
//            int XorY = 0; // Fo selection of x or y in struct
//            for (auto &el : pyCenter) {
//                // not possible to loop over struct --> use i as counter to differentiate
//                if (XorY == 0) {
//                    CenterPoint.x = el.cast<double>();
//                } else {
//                    CenterPoint.y = el.cast<double>();
//                }
//                XorY++;
//            }
//
//            tempObstacle->getGeoShape().setCenter(CenterPoint.x, CenterPoint.y);
//            tempObstacle->getGeoShape().setRadius(Radius);
//        }
//
//        //-----------------------------------------------------------
//        // Assign orientation (depending on uncertainty)
//        if (py::hasattr(py_singleObstacle.attr("initial_state").attr("orientation"), "start") &&
//            py::hasattr(py_singleObstacle.attr("initial_state").attr("orientation"), "end")) {
//            // Uncertain orientation: Intervall is given
//            double StartOrientation =
//                    py_singleObstacle.attr("initial_state").attr("orientation").attr("start").cast<double>();
//            double EndOrientation =
//                    py_singleObstacle.attr("initial_state").attr("orientation").attr("end").cast<double>();
//            double MeanOrientation = (StartOrientation + EndOrientation) / 2.0;
//
//            tempObstacle->setOrientation(MeanOrientation);
//            tempObstacle->setOrientationError(abs(MeanOrientation - StartOrientation));
//        } else {
//            // No uncertainty: get one value
//            tempObstacle->setOrientation(py_singleObstacle.attr("initial_state").attr("orientation").cast<double>());
//        }
//
//        //-----------------------------------------------------------
//        // Assign Velocity (depending on uncertainty)
//        if (py::hasattr(py_singleObstacle.attr("initial_state").attr("velocity"), "start") &&
//            py::hasattr(py_singleObstacle.attr("initial_state").attr("velocity"), "end")) {
//            // Uncertain orientation: Intervall is given
//            double StartVelocity =
//                    py_singleObstacle.attr("initial_state").attr("velocity").attr("start").cast<double>();
//            double EndVelocity = py_singleObstacle.attr("initial_state").attr("velocity").attr("end").cast<double>();
//            double MeanVelocity = (StartVelocity + EndVelocity) / 2.0;
//
//            tempObstacle->setVelocity(MeanVelocity);
//            tempObstacle->setVelocityError(abs(MeanVelocity - StartVelocity));
//        } else {
//            // No uncertainty: get one value
//            tempObstacle->setVelocity(py_singleObstacle.attr("initial_state").attr("velocity").cast<double>());
//        }
//    }
//
//    obstacleList.emplace_back(tempObstacle);
//}
