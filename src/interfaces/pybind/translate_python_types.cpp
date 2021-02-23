//
// Created by Sebastian Maierhofer on 23.02.21.
//

#include "translate_python_types.h"
#include "../../auxiliaryDefs/structs.h"
#include <limits>

inline double extractSpeedLimit2018(const py::handle &py_singleLanelet) {
    return py_singleLanelet.attr("speed_limit").cast<double>();
}

inline double extractSpeedLimit2020(const py::handle &py_singleLanelet, const std::map<int, double> &speedLimits) {
    const py::list py_trafficSigns = py_singleLanelet.attr("traffic_signs").cast<py::list>();
    for (const auto &py_trafficSignId : py_trafficSigns) {
        int trafficSignId = py::cast<int>(py_trafficSignId);
        const auto trafficSign = speedLimits.find(trafficSignId);
        if (trafficSign != speedLimits.end())
            return trafficSign->second;
    }
    return std::numeric_limits<double>::infinity();
}

std::vector<std::shared_ptr<Lanelet>>
TranslatePythonTypes::convertLanelets(const py::handle &py_laneletNetwork) {
    // todo add checks if laneletContainer is unused

    std::vector<std::shared_ptr<Lanelet>> laneletContainer;
    const py::list &py_lanelets = py_laneletNetwork.attr("lanelets").cast<py::list>();
    laneletContainer.reserve(py_lanelets.size()); // Already know the size --> Faster memory allocation

    std::map<int, double> speedLimits;

    const py::list &py_trafficSigns = py_laneletNetwork.attr("traffic_signs").cast<py::list>();
    for (const auto &py_trafficSign : py_trafficSigns) {
        int trafficSignId = py_trafficSign.attr("traffic_sign_id").cast<int>();
        const py::list &py_trafficSignElements = py_trafficSign.attr("traffic_sign_elements").cast<py::list>();
        for (const py::handle &py_trafficSignElement : py_trafficSignElements) {
            std::string trafficSignElementId =
                    py_trafficSignElement.attr("traffic_sign_element_id").cast<py::str>();
            if (trafficSignElementId.find("MAX_SPEED") != std::string::npos) {
                const py::list &additionalValues = py_trafficSignElement.attr("additional_values").cast<py::list>();

                const std::string py_speedLimit = additionalValues[0].attr("__str__")().cast<std::string>();
                double speedLimit = std::stod(py_speedLimit);
                speedLimits.insert({trafficSignId, speedLimit});
            }
        }
    }


    // iterate over items in python list
    for (py::handle py_SingleLanelet : py_lanelets) {
        // how to tell vehicular and pedestrian lanelets apart (later on if implemented in commonroad)
        // py_SingleLanelet.get_type().attr("__name__").cast<std::string>()

        //-----------------------------------------------------------


        //-----------------------------------------------------------
        size_t id = py::cast<size_t>(py_SingleLanelet.attr("lanelet_id"));

        //-----------------------------------------------------------
        // py_left_vertices is a numpy array with two dimensions
        // Possible alternative (needs numpy.h)
        // py::array_t<double> py_left_vertices2 = py::getattr(item, "left_vertices");
        py::object py_left_vertices = py_SingleLanelet.attr("left_vertices");

        std::vector<vertex> left_vertices;
        for (auto &el : py_left_vertices) {
            vertex temp_vert;
            int i = 0;
            for (auto &el2 : el) {
                // not possible to loop over struct --> use i as counter to differentiate
                if (i == 0) {
                    temp_vert.x = el2.cast<double>();
                } else {
                    temp_vert.y = el2.cast<double>();
                }
                i++;
            }
            left_vertices.emplace_back(temp_vert); // don't use std::move = trivially copyable
        }

        //-----------------------------------------------------------
        py::object py_right_vertices = py_SingleLanelet.attr("right_vertices");

        std::vector<vertex> right_vertices;
        for (auto &el : py_right_vertices) {
            vertex temp_vert;
            int i = 0;
            for (auto &el2 : el) {
                // not possible to loop over struct --> use i as counter to differentiate
                if (i == 0) {
                    temp_vert.x = el2.cast<double>();
                } else {
                    temp_vert.y = el2.cast<double>();
                }
                i++;
            }
            right_vertices.emplace_back(temp_vert);
        }

        //-----------------------------------------------------------
        py::object py_center_vertices = py_SingleLanelet.attr("center_vertices");

        std::vector<vertex> center_vertices;
        for (auto &el : py_center_vertices) {
            vertex temp_vert;
            int i = 0;
            for (auto &el2 : el) {
                // not possible to loop over struct --> use i as counter to differentiate
                if (i == 0) {
                    temp_vert.x = el2.cast<double>();
                } else {
                    temp_vert.y = el2.cast<double>();
                }
                i++;
            }
            center_vertices.emplace_back(temp_vert); // no std::move as trivially copyable
        }

        //-----------------------------------------------------------
        // Assign the information to a lanelet instance
        // For now it is assumed, that the lanelet is Lanelet (Like in the old interface)
        // Later: Compare what distinguishes the lanelets and create right instance (for addition of pedestrianLanelets)
        // Specific lanelet then should both be assignable to general pointer (polymorphism)
        // Alternative: Add container with pedestrianLanelets to Scenario
        std::shared_ptr<Lanelet> tempLanelet =
                std::make_shared<Lanelet>(); // make_shared is faster than (new Lanelet());
        tempLanelet->setId(id);
//        tempLanelet->setSpeedLimit(speed_lim);
//        tempLanelet->moveLeftBorder(std::move(left_vertices));
//        tempLanelet->moveRightBorder(std::move(right_vertices));
//        tempLanelet->moveCenterVertices(std::move(center_vertices));
        tempLanelet->constructOuterPolygon();

        // Add final Lanelet to container
        laneletContainer.emplace_back(tempLanelet);
    }

    // After creation of Lanelets still assign predecessor/successor pointers
    // Call the function below for this (keeps the single functions smaller)
//    TranslatePythonTypes::setLaneletPointer(py_lanelets, laneletContainer);

    return laneletContainer;
}
//
//// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
///// \brief This function adds the predecessor and successor lanelet pointers to the laneletObjects
/////
/////     This function should only be called by TranslatePythonTypes::convertLanelets() and is only meant to outsource
/////     some of the code.
/////     It is always assumed, that the laneletContainer is already initialized (holds all the lanelets).
/////     Thus this function only assigns the pointer values to predecessor/sucessor
/////
///// \param[in] py_lanelets = Commonroad version of lanelets
///// \param[in] laneletContainer = Container for SPOT version of lanelets (changes existing given lanelet instances)
///// \note The functions inside the namesapce "TranslatePythonTypes" are only used by the python-interface
//void TranslatePythonTypes::setLaneletPointer(const py::list &py_lanelets,
//                                             std::vector<std::shared_ptr<Lanelet>> &laneletContainer) {
//    // Second loop through lanelets to set pointers of predecessor/successors correctly
//    // In commonroad only IDs are given as Integer values --> Search ID in C++ container and store pointer value
//    // Todo: Check if this can be done more efficiently (don't loop two times through lanelets, Use std::map?)
//    int IndexPointer = 0;
//    for (py::handle py_SingleLanelet : py_lanelets) {
//        //-----------------------------------------------------------
//        // All predecessors
//        py::object py_predecessors = py_SingleLanelet.attr("predecessor");
//        for (py::handle py_item : py_predecessors) {
//            // One lanelet can have multiple predecessors
//            for (auto it = laneletContainer.begin(); it != laneletContainer.end(); ++it) {
//                // Search specified predecessor id and store its pointer
//                if ((*it)->getId() == py_item.cast<size_t>()) {
//                    // give raw pointer (no ownership only access)
//                    laneletContainer.at(IndexPointer)->addPredecessor((*it).get());
//                    break; // Id's are unique: Stop if found
//                }
//            }
//        }
//
//        //-----------------------------------------------------------
//        // All Successors
//        py::object py_successors = py_SingleLanelet.attr("successor");
//        for (py::handle py_item : py_successors) {
//            // One lanelet can have multiple sucessors
//            // for(std::size_t i=0; i<tempLaneletContainer.size(); ++i)
//            for (auto it = laneletContainer.begin(); it != laneletContainer.end(); ++it) {
//                // Search specified successor id and store its pointer
//                if ((*it)->getId() == py_item.cast<size_t>()) {
//                    // give raw pointer (no ownership only access)
//                    laneletContainer.at(IndexPointer)->addSuccessor((*it).get());
//                    break; // Id's are unique
//                }
//            }
//        }
//
//        //-----------------------------------------------------------
//        // Adjacent left
//        py::object py_AdjLeft = py_SingleLanelet.attr("adj_left");
//        if (py_AdjLeft.get_type().attr("__name__").cast<std::string>() == "int") {
//            // One adjacent lanelet on the left (if not the type is "NoneType" not "int")
//            for (auto it = laneletContainer.begin(); it != laneletContainer.end(); ++it) {
//                // Search specified predecessor id and store its pointer
//                if ((*it)->getId() == py_AdjLeft.cast<size_t>()) {
//                    // Found lanelet which is adjacent left
//                    // Same or opposite direction:
//                    py::object py_AdjLeftSameDir = py_SingleLanelet.attr("adj_left_same_direction");
//                    if (py_AdjLeftSameDir.cast<bool>()) {
//                        // Same direction
//                        // give raw pointer (no ownership only access)
//                        laneletContainer.at(IndexPointer)->setLeftAdjacent((*it).get(), "same");
//                    } else {
//                        // opposite direction
//                        // give raw pointer (no ownership only access)
//                        laneletContainer.at(IndexPointer)->setLeftAdjacent((*it).get(), "opposite");
//                    }
//                    break; // Id's are unique
//                }
//            }
//        }
//
//        //-----------------------------------------------------------
//        // Adjacent right
//        py::object py_AdjRight = py_SingleLanelet.attr("adj_right");
//        if (py_AdjRight.get_type().attr("__name__").cast<std::string>() == "int") {
//            // One adjacent lanelet on the right (if not the type is "NoneType" not "int")
//            for (auto it = laneletContainer.begin(); it != laneletContainer.end(); ++it) {
//                // Search specified predecessor id and store its pointer
//                if ((*it)->getId() == py_AdjRight.cast<size_t>()) {
//                    // Found lanelet which is adjacent left
//                    // Same or opposite direction:
//                    py::object py_AdjRightSameDir = py_SingleLanelet.attr("adj_right_same_direction");
//                    if (py_AdjRightSameDir.cast<bool>()) {
//                        // Same direction
//                        // give raw pointer (no ownership only access)
//                        laneletContainer.at(IndexPointer)->setRightAdjacent((*it).get(), "same");
//                    } else {
//                        // opposite direction
//                        // give raw pointer (no ownership only access)
//                        laneletContainer.at(IndexPointer)->setRightAdjacent((*it).get(), "opposite");
//                    }
//                    break; // Id's are unique
//                }
//            }
//        }
//        IndexPointer++;
//    }
//}
//
//// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
///// \brief Convert the obstacles from Commonroad to the C++ counterpart
/////
/////     This function is called from the PythonInterface.cpp
/////
///// \param[in] py_dynamicObstacles = Commonroad version of obstacles
///// \param[in] ObstacleContainer = Container for SPOT version of the obstacles (filled here and should be empty before)
///// \note The functions inside the namesapce "TranslatePythonTypes" are only used by the python-interface
//uint8_t TranslatePythonTypes::convertObstacles(const py::list &py_dynamicObstacles,
//                                               std::vector<std::shared_ptr<Obstacle>> &ObstacleContainer) {
//    // loop through all the given dynamic obstacles and differentiate between different commonroad types
//    // assign the corresponding C++ version of the obstacles
//    for (py::handle py_singleObstacle : py_dynamicObstacles) {
//        //-----------------------------------------------------------
//        // ToDo: Additionally revise how the default parameters of each class are set
//        py::object py_ObstacleType = py_singleObstacle.attr("obstacle_type");
//        std::shared_ptr<Obstacle> tempObstacle(
//                nullptr); // Empty pointer (specific object gets assigned depending on obstacle type)
//
//        // Vehicles get the shape rectangular and pedestrian a circle in SPOT
//        // The the subclasses of dynamicObstacle: vehicle and pedestrian have their own specific
//        // implementations of "Compute occupancy core --> There either a rectangle or a circle is extracted from shape
//        // Todo this implicit handling might not be advisable for the future (needs assumption, that here the corect
//        // subclass of shape is assigned)
//        // Is this assumption even needed? (Why not let also pedestrian be rectangles
//        // if they were like this in Commonroad)
//        // might be interesting to look at with level 2 architecture change
//
//        if (py_ObstacleType.is(py_ObstacleType.attr("CAR"))) {
//            tempObstacle = std::make_shared<passengerCar>();
//        } else if (py_ObstacleType.is(py_ObstacleType.attr("TRUCK"))) {
//            tempObstacle = std::make_shared<truck>();
//        } else if (py_ObstacleType.is(py_ObstacleType.attr("BUS"))) {
//            tempObstacle = std::make_shared<bus>();
//        } else if (py_ObstacleType.is(py_ObstacleType.attr("PRIORITY_VEHICLE"))) {
//            tempObstacle = std::make_shared<priorityVehicle>();
//            //        ToDo: uncomment the lines below when commonroad-io supports motorcycles
//            //        } else if (py_ObstacleType.is(py_ObstacleType.attr("MOTORCYCLE"))) {
//            //            tempObstacle = std::make_shared<motorcycle>();
//        } else if (py_ObstacleType.is(py_ObstacleType.attr("BICYCLE"))) {
//            tempObstacle = std::make_shared<bicycle>();
//        } else if (py_ObstacleType.is(py_ObstacleType.attr("TRAIN"))) {
//            tempObstacle = std::make_shared<train>();
//        } else if (py_ObstacleType.is(py_ObstacleType.attr("PEDESTRIAN"))) {
//            tempObstacle = std::make_shared<pedestrian>();
//        } else if (py_ObstacleType.is(py_ObstacleType.attr("UNKNOWN"))) {
//            std::cout << "Obstacle type is " << py_ObstacleType << std::endl;
//            return 1;
//        } else {
//            std::cout << "Unknown obstacle type. Only car, bicycle and pedestrian are supported for now, but received "
//                      << py_ObstacleType << std::endl;
//            return 1; // Don't know obstacle type
//        }
//
//        //-----------------------------------------------------------
//        // Generate values which are handled the same for all obstacle types
//        // Note: Specific assignments might have to be added into if-structure above
//        size_t temp_ID = py::cast<size_t>(py_singleObstacle.attr("obstacle_id"));
//        tempObstacle->setId(temp_ID);
//
//        //-----------------------------------------------------------
//        // Get position: Different handling if position is uncertain (then instead of x,y value, we have a shape)
//        double xPos = 0, yPos = 0;
//        double UncertaintyAllDim = 0, UncertaintyLength = 0, UncertaintyWidth = 0;
//        std::string temp_Uncertainty =
//                py_singleObstacle.attr("initial_state").attr("position").get_type().attr("__name__").cast<std::string>();
//        if (temp_Uncertainty == "ndarray") {
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
//        } else if (temp_Uncertainty == "Rectangle") {
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
//
//        //-----------------------------------------------------------
//        // Set values which are handled the same for all obstacle types
//        ObstacleContainer.emplace_back(tempObstacle);
//    }
//
//    return 0;
//}
//
//// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
///// \brief Use the information from the planning problem from Commonroad to define the egoVehicle
/////
/////     This function is called from the PythonInterface.cpp
/////
///// \param[in] py_planningProblem = Commonroad version of a planningProblem
///// \param[in] tempEgo = create the egoVehicle and store it in this pointer
///// \note The functions inside the namesapce "TranslatePythonTypes" are only used by the python-interface
//void TranslatePythonTypes::convertEgoVehicle(const py::list &py_planningProblem, std::shared_ptr<EgoVehicle> tempEgo) {
//    // The planning problem contains information about position, velocity and orientation of the egoVehicle
//    // Note, that the information about the size of the rectangular shape is curently by default: width = 1,8 and length
//    // = 4,5 To change these values: Use update function Also information about comfortable accelerations min/max should
//    // be updated with updateScenario
//    for (py::handle pyEgo : py_planningProblem) {
//        // List should contain only one element (loop only one time)
//        vehicle tempEgoVehicleObj = tempEgo->getVehicleObj();
//        tempEgoVehicleObj.setId(pyEgo.attr("planning_problem_id").cast<size_t>());
//        tempEgoVehicleObj.setVelocity(pyEgo.attr("initial_state").attr("velocity").cast<double>());
//        tempEgoVehicleObj.setOrientation(pyEgo.attr("initial_state").attr("orientation").cast<double>());
//        // py_planningProblem
//
//        double xPos = 0, yPos = 0;
//        int i = 0;
//        for (auto &elements : pyEgo.attr("initial_state").attr("position")) {
//            if (i == 0) {
//                xPos = elements.cast<double>();
//            } else {
//                yPos = elements.cast<double>();
//            }
//            i++;
//        }
//        tempEgoVehicleObj.setPosition(xPos, yPos);
//
//        tempEgoVehicleObj.getGeoShape().setLength(4.5);
//        tempEgoVehicleObj.getGeoShape().setWidth(1.8);
//
//        // Acc_Comfort min and max also have to be change with updateScenario (default = 4 and -1)
//    }
//}
