#include <geometry/curvilinear_coordinate_system.h>

#include <algorithm> // for max, min
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <cmath>
#include <stdexcept> // for logic_error
#include <string>    // for operator+
#include <utility>

#include <Eigen/Core> // for Vector2d

#include <boost/geometry/geometries/ring.hpp> // for ring

#include <commonroad_cpp/auxiliaryDefs/structs.h>
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/obstacle/obstacle_reference.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <spdlog/spdlog.h>

static std::array<double, 4> rotatedCornerLatitudes(const Rectangle &rect, const double theta) {
    const double width = rect.getWidth();
    const double length = rect.getLength();

    return {(width / 2) * cos(theta) - (length / 2) * sin(theta), (width / 2) * cos(theta) - (-length / 2) * sin(theta),
            (-width / 2) * cos(theta) - (length / 2) * sin(theta),
            (-width / 2) * cos(theta) - (-length / 2) * sin(theta)};
}

static std::array<double, 4> rotatedCornerLongitudes(const Rectangle &rect, const double theta) {
    const double width = rect.getWidth();
    const double length = rect.getLength();

    return {(length / 2) * cos(theta) - (width / 2) * sin(theta), (length / 2) * cos(theta) - (-width / 2) * sin(theta),
            (-length / 2) * cos(theta) - (width / 2) * sin(theta),
            (-length / 2) * cos(theta) - (-width / 2) * sin(theta)};
}

static double rotatedMinimumLatitude(const Rectangle &rect, const double theta) {
    const auto latitudes = rotatedCornerLatitudes(rect, theta);
    return *std::min_element(latitudes.cbegin(), latitudes.cend());
}

static double rotatedMaximumLatitude(const Rectangle &rect, const double theta) {
    const auto latitudes = rotatedCornerLatitudes(rect, theta);
    return *std::max_element(latitudes.cbegin(), latitudes.cend());
}

static double rotatedMinimumLongitude(const Rectangle &rect, const double theta) {
    const auto longitudes = rotatedCornerLongitudes(rect, theta);
    return *std::min_element(longitudes.cbegin(), longitudes.cend());
}

static double rotatedMaximumLongitude(const Rectangle &rect, const double theta) {
    const auto longitudes = rotatedCornerLongitudes(rect, theta);
    return *std::max_element(longitudes.cbegin(), longitudes.cend());
}

Obstacle::Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
                   ObstacleType obstacleType, double vMax, double aMax, double aMaxLong, double aMinLong,
                   std::optional<double> reactionTime, Obstacle::state_map_t trajectoryPrediction, double length,
                   double width, const std::vector<vertex> &fov)
    : Obstacle{obstacleId,
               obstacleRole,
               std::move(currentState),
               obstacleType,
               ActuatorParameters{vMax, aMax, aMaxLong, aMinLong, aMinLong},
               SensorParameters{250.0, 250.0, reactionTime},
               std::move(trajectoryPrediction),
               std::make_unique<Rectangle>(length, width),
               fov} {}

Obstacle::Obstacle(size_t obstacleId, ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
                   ObstacleType obstacleType, ActuatorParameters actuatorParameters, SensorParameters sensorParameters,
                   Obstacle::state_map_t trajectoryPrediction, std::unique_ptr<Shape> shape,
                   const std::vector<vertex> &fov)
    : obstacleId(obstacleId), obstacleRole(obstacleRole), currentState(std::move(currentState)),
      obstacleType(obstacleType), actuatorParameters(actuatorParameters), sensorParameters(sensorParameters),
      trajectoryPrediction(std::move(trajectoryPrediction)), geoShape(std::move(shape)) {
    if (obstacleRole == ObstacleRole::STATIC)
        setIsStatic(true);

    if (this->trajectoryPrediction.empty())
        finalTimeStep = this->currentState->getTimeStep();
    else
        finalTimeStep = std::max_element(this->trajectoryPrediction.begin(), this->trajectoryPrediction.end())->first;
    if (trajectoryHistory.empty())
        firstTimeStep = this->currentState->getTimeStep();
    else
        firstTimeStep = std::min_element(this->trajectoryHistory.begin(), this->trajectoryHistory.end())->first;

    if (fov.empty()) {
        // TODO update default fov values
        std::vector<vertex> fovVertices{{0.0, -400.0},        {282.8427, -282.8427},  {400.0, 0.0},
                                        {282.8427, 282.8427}, {0.0, 400.0},           {-282.8427, 282.8427},
                                        {-400.0, 0.0},        {-282.8427, -282.8427}, {0.0, -400.0}};
        setFov(geometric_operations::rotateAndTranslateVertices(
            fovVertices, {this->currentState->getXPosition(), this->currentState->getYPosition()}, 0));
    }
}

void Obstacle::setId(const size_t oId) { obstacleId = oId; }

void Obstacle::setIsStatic(bool staticObstacle) {
    if (staticObstacle) {
        actuatorParameters = ActuatorParameters::staticDefaults();
    }
}

void Obstacle::setCurrentState(const std::shared_ptr<State> &state) {
    if (!currentState and trajectoryHistory.empty())
        firstTimeStep = state->getTimeStep();
    if (trajectoryPrediction.empty())
        finalTimeStep = state->getTimeStep();
    currentState = state;
}

void Obstacle::setObstacleType(ObstacleType type) { obstacleType = type; }

void Obstacle::setActuatorParameters(ActuatorParameters params) { actuatorParameters = params; }

void Obstacle::setSensorParameters(SensorParameters params) { sensorParameters = params; }

void Obstacle::setTrajectoryPrediction(const Obstacle::state_map_t &trajPrediction) {
    trajectoryPrediction = trajPrediction;
    finalTimeStep = std::max_element(trajectoryPrediction.begin(), trajectoryPrediction.end())->first;
}

void Obstacle::setTrajectoryHistory(const Obstacle::state_map_t &trajHistory) {
    trajectoryHistory = trajHistory;
    firstTimeStep = std::min_element(trajectoryHistory.begin(), trajectoryHistory.end())->first;
}

void Obstacle::setRectangleShape(double length, double width) { geoShape = std::make_unique<Rectangle>(length, width); }

void Obstacle::setCircleShape(double radius, vertex center) { geoShape = std::make_unique<Circle>(radius, center); }

void Obstacle::setGeoShape(std::unique_ptr<Shape> shape) { geoShape = std::move(shape); }

void Obstacle::appendStateToTrajectoryPrediction(const std::shared_ptr<State> &state) {
    trajectoryPrediction.insert(std::pair<size_t, std::shared_ptr<State>>(state->getTimeStep(), state));
    if (state->getTimeStep() > finalTimeStep)
        finalTimeStep = state->getTimeStep();
}

void Obstacle::appendStateToHistory(const std::shared_ptr<State> &state) {
    trajectoryHistory.insert(std::pair<size_t, std::shared_ptr<State>>(state->getTimeStep(), state));
}

size_t Obstacle::getId() const { return obstacleId; }

const std::shared_ptr<State> &Obstacle::getCurrentState() const { return currentState; }

bool Obstacle::timeStepExists(size_t timeStep) const {
    // for computational reasons we assume an obstacle trajectory has equidistant time steps from the initial/current
    // state or first history state until the last trajectory prediction state
    return isStatic() or (firstTimeStep <= timeStep and timeStep <= finalTimeStep);
}

std::shared_ptr<State> Obstacle::getStateByTimeStep(size_t timeStep) const {
    if (isStatic())
        return currentState;
    else if (trajectoryPrediction.count(timeStep) == 1)
        return trajectoryPrediction.at(timeStep);
    else if (currentState->getTimeStep() == timeStep)
        return currentState;
    else if (trajectoryHistory.count(timeStep) == 1)
        return trajectoryHistory.at(timeStep);
    else
        throw std::logic_error("Time step does not exist. Obstacle ID: " + std::to_string(this->getId()) +
                               " - Time step: " + std::to_string(timeStep));
}

std::shared_ptr<SignalState> Obstacle::getSignalStateByTimeStep(size_t timeStep) const {
    if (isStatic())
        return currentSignalState;
    else if (signalSeries.count(timeStep) == 1)
        return signalSeries.at(timeStep);
    else if (currentSignalState && currentSignalState->getTimeStep() == timeStep)
        return currentSignalState;
    else if (signalSeriesHistory.count(timeStep) == 1)
        return signalSeriesHistory.at(timeStep);
    spdlog::info("Obstacle::getSignalStateByTimeStep: No signal state found. Returning default signal state.");
    return std::make_shared<SignalState>(false, false, false, false, false, false, timeStep);
}

ObstacleType Obstacle::getObstacleType() const { return obstacleType; }

ObstacleRole Obstacle::getObstacleRole() const { return obstacleRole; }

ActuatorParameters Obstacle::getActuatorParameters() const { return *actuatorParameters; }

SensorParameters Obstacle::getSensorParameters() const { return *sensorParameters; }

double Obstacle::getVmax() const {
    assert(actuatorParameters);
    return actuatorParameters->getVmax();
}

double Obstacle::getAmax() const {
    assert(actuatorParameters);
    return actuatorParameters->getAmax();
}

double Obstacle::getAmaxLong() const {
    assert(actuatorParameters);
    return actuatorParameters->getAmaxLong();
}

double Obstacle::getAminLong() const {
    assert(actuatorParameters);
    return actuatorParameters->getAminLong();
}

std::optional<double> Obstacle::getReactionTime() const {
    assert(sensorParameters);
    return sensorParameters->getReactionTime();
}

std::vector<size_t> Obstacle::getPredictionTimeSteps() {
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryPrediction | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getHistoryTimeSteps() {
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryHistory | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getTimeSteps() {
    std::vector<size_t> timeSteps{getPredictionTimeSteps()};
    if (timeSteps.empty())
        timeSteps.insert(timeSteps.begin(), currentState->getTimeStep());
    else {
        if (timeSteps.front() != currentState->getTimeStep())
            timeSteps.insert(timeSteps.begin(), currentState->getTimeStep());
    }
    return timeSteps;
}

Obstacle::state_map_t Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction; }

Obstacle::state_map_t Obstacle::getTrajectoryHistory() const { return trajectoryHistory; }

size_t Obstacle::getTrajectoryLength() const { return trajectoryPrediction.size(); }

polygon_type Obstacle::getOccupancyPolygonShape(size_t timeStep) { return setOccupancyPolygonShape(timeStep); }

polygon_type Obstacle::setOccupancyPolygonShape(size_t timeStep) {
    if (shapeAtTimeStep.count(timeStep) == 1)
        return shapeAtTimeStep[timeStep];
    std::vector<vertex> boundingVertices;
    polygon_type polygonShape;
    std::shared_ptr<State> state{this->getStateByTimeStep(timeStep)};

    if (this->getGeoShape().getType() == ShapeType::rectangle) {
        boundingVertices = geometric_operations::addObjectDimensionsRectangle(
            std::vector<vertex>{vertex{0.0, 0.0}}, this->getGeoShape().getLength(), this->getGeoShape().getWidth());
    } else if (this->getGeoShape().getType() == ShapeType::circle) {
        boundingVertices =
            geometric_operations::addObjectDimensionsCircle(vertex{0.0, 0.0}, this->getGeoShape().getRadius());
    } else {
        throw std::runtime_error{"obstacle shapes other than rectangle not supported by setOccupancyPolygonShape"};
    }

    /*
     * rotate and translate the vertices of the occupancy set in local
     * coordinates to the object's reference position and rotation
     */
    std::vector<vertex> adjustedBoundingVertices = geometric_operations::rotateAndTranslateVertices(
        boundingVertices, vertex{state->getXPosition(), state->getYPosition()}, state->getGlobalOrientation());

    polygonShape.outer().resize(adjustedBoundingVertices.size() + 1);

    // make polygon shape from previously created vertices
    for (size_t i{0}; i < adjustedBoundingVertices.size(); i++) {
        polygonShape.outer()[i] = point_type{adjustedBoundingVertices[i].x, adjustedBoundingVertices[i].y};
    }

    // add first point once again at the end
    if (!adjustedBoundingVertices.empty()) {
        polygonShape.outer().back() = point_type{adjustedBoundingVertices[0].x, adjustedBoundingVertices[0].y};
    }
    shapeAtTimeStep[timeStep] = polygonShape;
    return polygonShape;
}

Shape &Obstacle::getGeoShape() { return *geoShape; }

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (occupiedLanelets.find(timeStep) != occupiedLanelets.end())
        return occupiedLanelets.at(timeStep);
    polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
    std::vector<std::shared_ptr<Lanelet>> occupied{roadNetwork->findOccupiedLaneletsByShape(polygonShape)};
    occupiedLanelets.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occupied));
    return occupied;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    return setOccupiedLaneletsByShape(roadNetwork, timeStep);
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByFront(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (occupiedLaneletsFront.find(timeStep) != occupiedLaneletsFront.end())
        return occupiedLaneletsFront.at(timeStep);
    std::vector<double> front = getFrontXYCoordinates(timeStep);
    std::vector<std::shared_ptr<Lanelet>> occ{roadNetwork->findLaneletsByPosition(front[0], front[1])};
    occupiedLaneletsFront.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occ));
    return occ;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByBack(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (occupiedLaneletsBack.find(timeStep) != occupiedLaneletsBack.end())
        return occupiedLaneletsBack.at(timeStep);
    std::vector<double> back = getBackXYCoordinates(timeStep);
    std::vector<std::shared_ptr<Lanelet>> occ{roadNetwork->findLaneletsByPosition(back[0], back[1])};
    occupiedLaneletsBack.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occ));
    return occ;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    return setOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep);
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsNotDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                        size_t timeStep) {
    return setOccupiedLaneletsNotDrivingDirectionByShape(roadNetwork, timeStep);
}

void Obstacle::convertPointToCurvilinear(time_step_t timeStep, const std::shared_ptr<Lane> &refLane) {
    convertPointToCurvilinear(timeStep, refLane->getCurvilinearCoordinateSystem());
}

void Obstacle::convertPointToCurvilinear(size_t timeStep,
                                         const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs) {
    auto state{getStateByTimeStep(timeStep)};
    Eigen::Vector2d convertedPoint{ccs->convertToCurvilinearCoords(state->getXPosition(), state->getYPosition())};
    auto ccsTangent{ccs->tangent(convertedPoint.x())};
    double ccsOrientation = atan2(ccsTangent.y(), ccsTangent.x());
    double theta = geometric_operations::subtractOrientations(state->getGlobalOrientation(), ccsOrientation);
    convertedPositions[timeStep][ccs] = {
        convertedPoint.x() - RoadNetworkParameters::numAdditionalSegmentsCCS * ccs->eps2(), convertedPoint.y(), theta};
}

std::string Obstacle::ccsErrorMsg(size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                  const std::string &func) const {
    std::string refInfo;
    for (const auto &ref : ccs->referencePath())
        refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}; ";
    return "Obstacle::" + func +
           "Custom CCS - Curvilinear Projection Error - Obstacle ID: " + std::to_string(obstacleId) +
           " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " + refInfo +
           " - x-position: " + std::to_string(getStateByTimeStep(timeStep)->getXPosition()) +
           " - y-position: " + std::to_string(getStateByTimeStep(timeStep)->getYPosition());
}

double Obstacle::frontS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double lonPosition = getLonPosition(roadNetwork, timeStep);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use maximum of all corners
    return lonPosition + rotatedMaximumLongitude(rect, theta);
}

double Obstacle::frontS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    return frontS(timeStep, refLane->getCurvilinearCoordinateSystem());
}

double Obstacle::frontS(size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs) {
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "frontS"));
        }
    }
    double lonPosition = convertedPositions[timeStep][ccs][0];
    double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use maximum of all corners
    return lonPosition + rotatedMaximumLongitude(rect, theta);
}

double Obstacle::rearS(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double lonPosition = getLonPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    // use minimum of all corners
    return lonPosition + rotatedMinimumLongitude(rect, theta);
}

double Obstacle::rearS(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    return rearS(timeStep, refLane->getCurvilinearCoordinateSystem());
}

double Obstacle::rearS(size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs) {
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "frontS"));
        }
    }
    double lonPosition = convertedPositions[timeStep][ccs][0];
    double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use minimum of all corners
    return lonPosition + rotatedMinimumLongitude(rect, theta);
}

double Obstacle::rightD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double latPos = getLatPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    return latPos + rotatedMinimumLatitude(rect, theta);
}

double Obstacle::rightD(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    return rightD(timeStep, refLane->getCurvilinearCoordinateSystem());
}

double Obstacle::rightD(size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs) {
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "rightD"));
        }
    }
    double latPosition = convertedPositions[timeStep][ccs][1];
    double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    return latPosition + rotatedMinimumLatitude(rect, theta);
}

double Obstacle::leftD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    double latPos = getLatPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    return latPos + rotatedMaximumLatitude(rect, theta);
}

double Obstacle::leftD(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    return leftD(timeStep, refLane->getCurvilinearCoordinateSystem());
}

double Obstacle::leftD(size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs) {
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "leftD"));
        }
    }
    double latPosition = convertedPositions[timeStep][ccs][1];
    double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    return latPosition + rotatedMaximumLatitude(rect, theta);
}

double Obstacle::getLonPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().lonPosition)
        return getStateByTimeStep(timeStep)->getLonPosition();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getLonPosition();
}

double Obstacle::getLatPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().latPosition)
        return getStateByTimeStep(timeStep)->getLatPosition();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getLatPosition();
}

double Obstacle::getLonPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    auto ccs = refLane->getCurvilinearCoordinateSystem();
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "getLonPosition"));
        }
    }
    return convertedPositions[timeStep][ccs][0];
}

double Obstacle::getLatPosition(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    auto ccs = refLane->getCurvilinearCoordinateSystem();
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : ccs->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}; ";
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "getLatPosition"));
        }
    }
    return convertedPositions[timeStep][ccs][1];
}

double Obstacle::getCurvilinearOrientation(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().curvilinearOrientation)
        return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
}

double Obstacle::getCurvilinearOrientation(size_t timeStep, const std::shared_ptr<Lane> &refLane) {
    auto ccs = refLane->getCurvilinearCoordinateSystem();
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "getCurvilinearOrientation"));
        }
    }
    return geometric_operations::constrainAngle(convertedPositions[timeStep][ccs][2]);
}

size_t Obstacle::getFirstTrajectoryTimeStep() const {
    // NOTE: trajectoryPrediction is an unordered_map, so the first element (.begin()) is not
    // necessarily the minimum element!
    if (!trajectoryPrediction.empty())
        return std::min_element(trajectoryPrediction.begin(), trajectoryPrediction.end())->first;
    else
        throw std::runtime_error("Obstacle::getFirstTrajectoryTimeStep: Obstacle with ID " +
                                 std::to_string(obstacleId) + " has not trajectoryPrediction");
}

std::shared_ptr<Lane> Obstacle::getReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    return setReferenceLane(roadNetwork, timeStep);
}

std::shared_ptr<Lane> Obstacle::setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (referenceLane.count(timeStep) == 1 and referenceLane.at(timeStep) != nullptr)
        return referenceLane.at(timeStep);

    auto refLaneTmp{obstacle_reference::computeRef(*this, roadNetwork, timeStep)};

    if (!refLaneTmp.empty())
        referenceLane[timeStep] = refLaneTmp.at(0);

    if (referenceLane.count(timeStep) == 0 or referenceLane.at(timeStep) == nullptr)
        throw std::runtime_error("Obstacle::setReferenceLane: No matching referenceLane found! Obstacle ID " +
                                 std::to_string(getId()) + " at time step " + std::to_string(timeStep));
    return referenceLane.at(timeStep);
}

void Obstacle::convertPointToCurvilinear(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    auto curRefLaneCCS{getReferenceLane(roadNetwork, timeStep)->getCurvilinearCoordinateSystem()};
    try {
        convertPointToCurvilinear(timeStep, curRefLaneCCS);
        getStateByTimeStep(timeStep)->setLonPosition(convertedPositions[timeStep][curRefLaneCCS][0]);
        getStateByTimeStep(timeStep)->setLatPosition(convertedPositions[timeStep][curRefLaneCCS][1]);
        getStateByTimeStep(timeStep)->setCurvilinearOrientation(convertedPositions[timeStep][curRefLaneCCS][2]);
    } catch (...) {
        throw std::runtime_error(ccsErrorMsg(timeStep, curRefLaneCCS, "convertPointToCurvilinear"));
    }
}

void Obstacle::interpolateAcceleration(size_t timeStep, double timeStepSize) const {
    if (getStateByTimeStep(timeStep)->getValidStates().acceleration)
        return;
    if (!timeStepExists(timeStep - 1)) {
        getStateByTimeStep(timeStep)->setAcceleration(0);
        return;
    }
    double curVelocity{getStateByTimeStep(timeStep)->getVelocity()};
    double prevVelocity{getStateByTimeStep(timeStep - 1)->getVelocity()};
    getStateByTimeStep(timeStep)->setAcceleration((curVelocity - prevVelocity) / timeStepSize);
}

void Obstacle::setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes, size_t timeStep) {
    if (occupiedLanes.count(timeStep) == 0)
        occupiedLanes[timeStep] = lanes;
}

void Obstacle::setObstacleRole(ObstacleRole type) { obstacleRole = type; }

void Obstacle::setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    auto lanelets{getOccupiedLaneletsRoadByShape(roadNetwork, timeStep)};
    assert(sensorParameters);
    std::vector<std::shared_ptr<Lane>> occLanes{lane_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters->getFieldOfViewRear(), sensorParameters->getFieldOfViewFront(), 2,
        {getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition()})};
    occupiedLanes[timeStep] = occLanes;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsRoadByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {

    std::set<size_t> relevantLanelets1;
    std::set<size_t> relevantLanelets2;
    auto lets{getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &let : lets) {
        relevantLanelets1.insert(let->getId());
    }

    // special case where obstacle drives mostly on lanelets in other driving direction (see
    // SetReferenceGeneralScenario4 test case)
    size_t startTime{0};
    if (timeStep < relevantTimeIntervalSize)
        startTime = firstTimeStep;
    else
        startTime = std::max(firstTimeStep, timeStep - relevantTimeIntervalSize);

    // find all paths for all occupied initial lanelets to all occupied final lanelets
    for (const auto &initialLet : getOccupiedLaneletsByShape(roadNetwork, startTime))
        for (const auto &finalLet :
             getOccupiedLaneletsByShape(roadNetwork, std::min(finalTimeStep, timeStep + relevantTimeIntervalSize))) {
            auto path{roadNetwork->getTopologicalMap()->findPaths(initialLet->getId(), finalLet->getId(), true)};
            relevantLanelets2.insert(path.begin(), path.end());
        }

    // get lanelet objects for relevant lanelets and also consider adjacent lanelets
    for (const auto &letBase : relevantLanelets2)
        for (const auto &adj : lanelet_operations::adjacentLanelets(roadNetwork->findLaneletById(letBase), true))
            relevantLanelets2.insert(adj->getId());

    // todo part above could be extracted and only computed on demand (usually when trajectoryPrediction changes)
    // restrict lanelets to the ones which are currently occupied
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &letBase : getOccupiedLaneletsByShape(roadNetwork, timeStep))
        if (relevantLanelets1.find(letBase->getId()) != relevantLanelets1.end() and
            relevantLanelets2.find(letBase->getId()) != relevantLanelets2.end())
            lanelets.push_back(letBase);

    return lanelets;
}

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedRoadLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                  size_t timeStep) {
    auto occLanes{getOccupiedLanes(roadNetwork, timeStep)};
    if (occLanes.size() == 1)
        return occLanes;
    else {
        auto occLanelets{getOccupiedLaneletsRoadByShape(roadNetwork, timeStep)};
        std::vector<std::shared_ptr<Lane>> relevantLanes;
        for (const auto &lane : occLanes) {
            if (lane->getId() == getReferenceLane(roadNetwork, timeStep)->getId()) {
                relevantLanes.push_back(lane);
                continue;
            }
            // check whether lanelet is adjacent to reference lane
            if (lanelet_operations::areLaneletsInDirectlyAdjacentLanes(getReferenceLane(roadNetwork, timeStep), lane,
                                                                       occLanelets))
                relevantLanes.push_back(lane);
        }
        return relevantLanes;
    }
}

std::vector<std::shared_ptr<Lane>>
Obstacle::getOccupiedLanesDrivingDirection(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    auto occLanes{getOccupiedLanes(roadNetwork, timeStep)};
    if (occLanes.size() == 1)
        return occLanes;
    else {
        std::vector<std::shared_ptr<Lane>> relevantLanes;
        auto occLanelets{getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep)};
        for (const auto &lanelet : occLanes) {
            if (lanelet->getId() == getReferenceLane(roadNetwork, timeStep)->getId()) {
                relevantLanes.push_back(lanelet);
                continue;
            }
            // check whether lanelet is adjacent to reference lane
            if (lanelet_operations::areLaneletsInDirectlyAdjacentLanes(getReferenceLane(roadNetwork, timeStep), lanelet,
                                                                       occLanelets))
                relevantLanes.push_back(lanelet);
        }
        return relevantLanes;
    }
}

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                              size_t timeStep) {
    if (occupiedLanes[timeStep].empty())
        setOccupiedLanes(roadNetwork, timeStep);
    return occupiedLanes[timeStep];
}

std::vector<std::shared_ptr<Lane>>
Obstacle::getOccupiedLanesAndAdjacent(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    std::set<size_t> relevantLanelets;
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    auto lets{getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep)};
    if (lets.empty())
        lets = getOccupiedLaneletsByShape(roadNetwork, timeStep);
    for (const auto &let : lets) {
        relevantLanelets.insert(let->getId());
        for (const auto &let2 : lanelet_operations::adjacentLanelets(let, false))
            relevantLanelets.insert(let2->getId());
    }
    for (const auto &id : relevantLanelets)
        lanelets.push_back(roadNetwork->findLaneletById(id));

    assert(sensorParameters);
    std::vector<std::shared_ptr<Lane>> occLanes{lane_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters->getFieldOfViewRear(), sensorParameters->getFieldOfViewFront(), 2,
        {getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition()})};

    return occLanes;
}

void Obstacle::computeLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, bool considerHistory) {
    const size_t timeStamp{currentState->getTimeStep()};
    auto lanelets{getOccupiedLaneletsByShape(roadNetwork, timeStamp)};
    auto lanes{lane_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters->getFieldOfViewRear(), sensorParameters->getFieldOfViewFront())};
    setOccupiedLanes(lanes, timeStamp);
    if (!isStatic()) {
        for (const auto &time : getPredictionTimeSteps())
            setOccupiedLanes(roadNetwork, time);
        if (considerHistory)
            for (const auto &time : getHistoryTimeSteps())
                setOccupiedLanes(roadNetwork, time);
    }
}

void Obstacle::setCurvilinearStates(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (!currentState->getValidStates().lonPosition)
        convertPointToCurvilinear(roadNetwork, currentState->getTimeStep());
    if (!isStatic())
        for (const auto &timeStep : getPredictionTimeSteps())
            if (!getStateByTimeStep(timeStep)->getValidStates().lonPosition)
                convertPointToCurvilinear(roadNetwork, timeStep);
}

const polygon_type &Obstacle::getFov() const { return fov; }

void Obstacle::setFov(const std::vector<vertex> &fovVertices) {
    polygon_type polygon;
    polygon.outer().resize(fovVertices.size());
    size_t idx{0};
    for (const auto &left : fovVertices) {
        polygon.outer()[idx] = point_type{left.x, left.y};
        idx++;
    }

    fov = polygon;
    boost::geometry::simplify(polygon, fov, 0.01);
    boost::geometry::unique(fov);
    boost::geometry::correct(fov);
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                     time_step_t timeStep) {
    if (occupiedLaneletsDrivingDir.find(timeStep) != occupiedLaneletsDrivingDir.end())
        return occupiedLaneletsDrivingDir[timeStep];

    std::set<size_t> relevantLanelets1;
    std::set<size_t> relevantLanelets2;

    auto occLanelets{getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &la : occLanelets)
        if (std::abs(geometric_operations::subtractOrientations(
                la->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                             getStateByTimeStep(timeStep)->getYPosition()),
                getStateByTimeStep(timeStep)->getGlobalOrientation())) < 0.785)
            relevantLanelets1.insert(la->getId());

    // special case where obstacle drives mostly on lanelets in other driving direction (see
    // SetReferenceGeneralScenario4 test case)
    size_t startTime{0};
    if (timeStep < relevantTimeIntervalSize)
        startTime = firstTimeStep;
    else
        startTime = std::max(firstTimeStep, timeStep - relevantTimeIntervalSize);

    // find all paths for all occupied initial lanelets to all occupied final lanelets
    for (const auto &initialLet : getOccupiedLaneletsByShape(roadNetwork, startTime))
        for (const auto &finalLet :
             getOccupiedLaneletsByShape(roadNetwork, std::min(finalTimeStep, timeStep + relevantTimeIntervalSize))) {
            auto path{roadNetwork->getTopologicalMap()->findPaths(initialLet->getId(), finalLet->getId(), true)};
            relevantLanelets2.insert(path.begin(), path.end());
        }

    // todo part above could be extracted and only computed on demand (usually when trajectoryPrediction changes)
    // restrict lanelets to the ones which are currently occupied
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &letBase : getOccupiedLaneletsByShape(roadNetwork, timeStep))
        if (relevantLanelets1.find(letBase->getId()) != relevantLanelets1.end() and
            relevantLanelets2.find(letBase->getId()) != relevantLanelets2.end())
            lanelets.push_back(letBase);

    occupiedLaneletsDrivingDir[timeStep] = lanelets;
    return occupiedLaneletsDrivingDir[timeStep];
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsNotDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                        time_step_t timeStep) {
    if (occupiedLaneletsNotDrivingDir.find(timeStep) != occupiedLaneletsNotDrivingDir.end())
        return occupiedLaneletsNotDrivingDir[timeStep];

    auto occ = getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep);
    auto all = getOccupiedLaneletsByShape(roadNetwork, timeStep);

    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &lanelet : all) {
        if (std::none_of(occ.begin(), occ.end(),
                         [lanelet](std::shared_ptr<Lanelet> &occL) { return lanelet->getId() == occL->getId(); }))
            lanelets.emplace_back(lanelet);
    }

    occupiedLaneletsNotDrivingDir[timeStep] = lanelets;
    return occupiedLaneletsNotDrivingDir[timeStep];
}

void Obstacle::setCurrentSignalState(const std::shared_ptr<SignalState> &state) { currentSignalState = state; }

void Obstacle::appendSignalStateToSeries(const std::shared_ptr<SignalState> &state) {
    signalSeries.insert(std::pair<size_t, std::shared_ptr<SignalState>>(state->getTimeStep(), state));
}

void Obstacle::appendSignalStateToHistory(const std::shared_ptr<SignalState> &state) {
    signalSeriesHistory.insert(std::pair<size_t, std::shared_ptr<SignalState>>(state->getTimeStep(), state));
}
const Obstacle::signal_state_map_t &Obstacle::getSignalSeries() const { return signalSeries; }

const Obstacle::signal_state_map_t &Obstacle::getSignalSeriesHistory() const { return signalSeriesHistory; }

const std::shared_ptr<SignalState> &Obstacle::getCurrentSignalState() const { return currentSignalState; }

bool Obstacle::isStatic() const { return obstacleRole == ObstacleRole::STATIC; }

double Obstacle::drivenTrajectoryDistance() const {
    std::vector<vertex> polyline{{currentState->getXPosition(), currentState->getYPosition()}};
    for (const auto &state : trajectoryAsVector())
        polyline.push_back({state->getXPosition(), state->getYPosition()});
    return geometric_operations::computePathLengthFromPolyline(polyline).back();
}
std::vector<std::shared_ptr<State>> Obstacle::trajectoryAsVector() const {
    std::vector<std::shared_ptr<State>> trajectory;
    for (size_t timeStep{getFirstTrajectoryTimeStep()}; timeStep <= finalTimeStep; ++timeStep)
        if (trajectoryPrediction.find(timeStep) != trajectoryPrediction.end())
            trajectory.push_back(trajectoryPrediction.at(timeStep));
    return trajectory;
}

double Obstacle::getFieldOfViewRearDistance() const { return fieldOfViewRear; }

double Obstacle::getFieldOfViewFrontDistance() const { return fieldOfViewFront; }

void Obstacle::setFieldOfViewRearDistance(double distance) { fieldOfViewRear = distance; }

void Obstacle::setFieldOfViewFrontDistance(double distance) { fieldOfViewFront = distance; }

size_t Obstacle::getFirstTimeStep() const { return firstTimeStep; }

size_t Obstacle::getFinalTimeStep() const { return finalTimeStep; }

std::vector<double> Obstacle::getFrontXYCoordinates(time_step_t timeStep) {
    if (frontXYPositions.find(timeStep) != frontXYPositions.end())
        return frontXYPositions[timeStep];

    std::shared_ptr<State> state = getStateByTimeStep(timeStep);
    double frontX = getGeoShape().getLength() / 2 * cos(state->getGlobalOrientation()) + state->getXPosition();
    double frontY = getGeoShape().getLength() / 2 * sin(state->getGlobalOrientation()) + state->getYPosition();
    std::vector<double> result{frontX, frontY};
    frontXYPositions[timeStep] = result;
    return frontXYPositions[timeStep];
}

std::vector<double> Obstacle::getBackXYCoordinates(time_step_t timeStep) {
    if (backXYPositions.find(timeStep) != backXYPositions.end())
        return backXYPositions[timeStep];

    std::shared_ptr<State> state = getStateByTimeStep(timeStep);
    double backX = getGeoShape().getLength() / 2 * cos(state->getGlobalOrientation() + M_PI) + state->getXPosition();
    double backY = getGeoShape().getLength() / 2 * sin(state->getGlobalOrientation() + M_PI) + state->getYPosition();
    std::vector<double> result{backX, backY};
    backXYPositions[timeStep] = result;
    return backXYPositions[timeStep];
}

double Obstacle::getLateralDistanceToObstacle(time_step_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                              const std::shared_ptr<RoadNetwork> &roadnetwork) {
    if (lateralDistanceToObjects.find(timeStep) != lateralDistanceToObjects.end() &&
        lateralDistanceToObjects[timeStep].find(obs->getId()) != lateralDistanceToObjects[timeStep].end())
        return lateralDistanceToObjects[timeStep][obs->getId()];

    double leftThis = leftD(timeStep, getReferenceLane(roadnetwork, timeStep));
    double rightThis = rightD(timeStep, getReferenceLane(roadnetwork, timeStep));

    double leftOther = obs->leftD(timeStep, getReferenceLane(roadnetwork, timeStep));
    double rightOther = obs->rightD(timeStep, getReferenceLane(roadnetwork, timeStep));

    double min = std::min(abs(rightThis - leftOther), abs(leftThis - rightOther));

    lateralDistanceToObjects[timeStep][obs->getId()] = min;

    return min;
}
