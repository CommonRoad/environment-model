#include "obstacle.h"
#include "../geometry/geometricOperations.h"
//#include "../../geometry/rectangle.h"
//#include "../Journal.h"
//#include "../lanelets/lanelet_operations.h"
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define max(a, b)                                                                                                      \
    ({                                                                                                                 \
        __typeof__(a) _a = (a);                                                                                        \
        __typeof__(b) _b = (b);                                                                                        \
        _a > _b ? _a : _b;                                                                                             \
    })

#define min(a, b)                                                                                                      \
    ({                                                                                                                 \
        __typeof__(a) _a = (a);                                                                                        \
        __typeof__(b) _b = (b);                                                                                        \
        _a < _b ? _a : _b;                                                                                             \
    })

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief Update this obstacle's properties according to an update map.
///
///     This will go through the keys in the given update map and update the corresponding obstacle property
///     with the corresponding update map value by calling updateProperty().
///
///     Each obstacle subclass supports different properties:
///         - EgoVehicle::updateProperty()
///         - vehicle::updateProperty()
///         - pedestrian::updateProperty()
///         - obstacle::updateProperty()
///
///     This function should probably not be overridden.
///
///     See Scenario::updateProperties() for more information.
///
/// \param[in] UpdateMap Map where keys denote a property to be updated and value the according values to set.
/// \returns Number that indicates if run was successful
///          0 = success
///          2 = property not found
uint8_t obstacle::updateProperties(const std::map<std::string, std::variant<bool, float>> &UpdateMap) {
    for (auto const &[key, val] : UpdateMap) {
        uint8_t res = updateProperty(key, val);
        if (res) {
            return res;
        }
    }
    return 0;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief Update a specific property of this obstacle.
///
///     Classes inheriting from obstacle can implement their own version of `updateProperty` and add their custom
///     supported update operations. **Always call <parent>::updateProperty last if no corresponding update property
///     was found in the child classes function.**
///
///     See Scenario::updateProperties() for more information.
///
/// \param[in] name = Name of the property to be updated. The supported properties are:
///        - NULL
/// \param[in] value = variant<bool, float> of the value that the property should be set to. Should contain a value
///                    that matches the field type.
/// \returns Number that indicates if run was successful
///          0 = success
///          2 = property not found
uint8_t obstacle::updateProperty(const std::string &name, const std::variant<bool, float> &value) {
    if (name == "v_max") {
        setVmax(std::get<float>(value));
    } else if (name == "a_max") {
        setAmax(std::get<float>(value));
    } else if (name == "a_max_long") {
        setAmaxLong(std::get<float>(value));
    } else if (name == "a_min_long") {
        setAminLong(std::get<float>(value));
    } else if (name == "violated_cvmax") {
        setViolatedCvmax(std::get<bool>(value));
    } else if (name == "violated_camax") {
        setViolatedCamax(std::get<bool>(value));
    } else if (name == "offset") {
        setOffset(std::get<float>(value));
    } else {
        std::cout << "Obstacle " << std::to_string(obstacle::id) << " does not support updating property \"" << name
                  << "\"" << std::endl;
        return 2;
    }
    return 0;
}

// all constraints are only applicable for dynamic obstacles
//void obstacle::manageConstraints() {
//
//    if (isStatic) {
//        std::cout << "Empty object has no constraints" << std::endl;
//    } else {
//
//        // if speed of obstacle is higher than its v_max, set v_max to actual speed
//        if (std::abs(this->getVelocity()) + this->getVelocityError() > this->getVmax()) {
//            Journal::getJournal()->addLog(
//                "The speed of obstacle " + std::to_string(this->getId()) +
//                " (v = " + std::to_string(std::abs(this->getVelocity()) + this->getVelocityError()) +
//                ") is higher than its parameterized maximum speed (v_max = " + std::to_string(this->getVmax()) + ").");
//
//            this->setVmax(std::abs(this->getVelocity()) + this->getVelocityError());
//
//            this->setViolatedCvmax(true); // set flag for violation of assumption
//        }
//
//        /* manage constr_max_acc (maximum absolute acceleration of dynamic obstacle)
//         * if acceleration of dynamic obstacle is higher than its a_max, set a_max
//         * to actual acceleration
//         */
//        if (std::abs(this->getAcceleration()) + this->getAccelerationError() > this->getAmax()) {
//            Journal::getJournal()->addLog(
//                "The acceleration of obstacle " + std::to_string(this->getId()) +
//                " (a = " + std::to_string((std::abs(this->getAcceleration()) + this->getAccelerationError())) +
//                " is higher than its " + "maximum allowed acceleration (a_max = " + std::to_string(this->getAmax()) +
//                ").");
//
//            this->setAmax(std::abs(this->getAcceleration()) + this->getAccelerationError());
//
//            this->setViolatedCamax(true); // set flag for violation of assumption
//        }
//    }
//}

void obstacle::setNewObst(const bool val) { newObst = val; }

bool obstacle::getNewObst() const { return newObst; }

void obstacle::setId(const size_t &num) { id = num; }

void obstacle::setPosition(const double &x, const double &y) {
    xPosition = x;
    yPosition = y;
}

void obstacle::setOrientation(const double &value) { orientation = wrapToPi(value); }

void obstacle::setOrientationError(const double &value) { orientation_error = value; }
//
//void obstacle::addInLane(lane *l) { inLanes.emplace_back(l); }
//
//void obstacle::updateInLane(std::vector<lane *> &lanes) {
//    inLanes = this->getInTracks(lanes);
//    this->updateInLanelets();
//}

//void obstacle::updateInLanelets() {
//
//    std::vector<vehicularLanelet *> laneletsOfOneLane;
//
//    for (const auto &it : inLanes) {
//        laneletsOfOneLane.clear();
//        laneletsOfOneLane = this->getInTracks(it->getAssLanelets());
//        inLanelets.insert(std::end(inLanelets), std::begin(laneletsOfOneLane), std::end(laneletsOfOneLane));
//    }
//}

void obstacle::setOccType(const size_t type) { occType = type; }

void obstacle::setTimeStamp(const double stamp) { timeStamp = stamp; }

void obstacle::useShapeAsRef(const bool val) { useShape = val; }

size_t obstacle::getOccType() const { return occType; }

size_t obstacle::getId() const { return id; }

double obstacle::getXpos() const { return xPosition; }

double obstacle::getYpos() const { return yPosition; }

double obstacle::getOrientation() const { return orientation; }

double obstacle::getOrientationError() const { return orientation_error; }

double obstacle::getTimeStamp() const { return timeStamp; }

//const std::vector<lane *> &obstacle::getInLane() const { return inLanes; }
//
//const std::vector<vehicularLanelet *> &obstacle::getInLanelets() const { return inLanelets; }

bool obstacle::getUseShape() const { return useShape; }

//const std::vector<std::vector<occTypes>> &obstacle::getOccupancyMatrix() const { return occupancyMatrix; }

//const std::vector<std::vector<occTypes>> *obstacle::getOccupancyMatrixPtr() { return &occupancyMatrix; }

//void obstacle::setOccupancyMatrix(const std::vector<std::vector<occTypes>> &occMatrix) { occupancyMatrix = occMatrix; }

//void obstacle::setOccupancyMatrix(const std::vector<std::vector<occTypes>> &&occMatrix) { occupancyMatrix = occMatrix; }

void obstacle::setVelocity(const double velo) { velocity = isStatic ? 0.0 : velo; }

void obstacle::setVelocityError(const double v_error) { velocity_error = isStatic ? 0.0 : v_error; }

void obstacle::setAcceleration(const double acc) { acceleration = isStatic ? 0.0 : acc; }

void obstacle::setAccelerationError(const double acc_error) { acceleration_error = isStatic ? 0.0 : acc_error; }

void obstacle::setVmax(const double vmax) { v_max = isStatic ? 0.0 : vmax; }

void obstacle::setAmax(const double amax) { a_max = isStatic ? 0.0 : amax; }

void obstacle::setAmaxLong(const double amax_long) { a_max_long = isStatic ? 0.0 : amax_long; }

void obstacle::setAminLong(const double amin_long) { a_min_long = isStatic ? 0.0 : amin_long; }

//void obstacle::setReachableLanes(std::vector<lane *> lanes) {
//    reachableLanes.clear();
//    for (size_t i = 0; i < lanes.size(); i++) {
//        reachableLanes.emplace_back(lanes[i]->getId());
//    }
//}

void obstacle::setOffset(const double val) { offset = val; }

void obstacle::setViolatedCvmax(const bool val) { violated_cvmax = val; }

void obstacle::setViolatedCamax(const bool val) { violated_camax = val; }

double obstacle::getOffset() const { return offset; }

double obstacle::getVelocity() const { return velocity; }

double obstacle::getVelocityError() const { return velocity_error; }

double obstacle::getAcceleration() const { return acceleration; }

double obstacle::getAccelerationError() const { return acceleration_error; }

double obstacle::getVmax() const { return v_max; }

double obstacle::getAmax() const { return a_max; }

double obstacle::getAmaxLong() const { return a_max_long; }

double obstacle::getAminLong() const { return a_min_long; }

std::vector<size_t> obstacle::getReachableLanes() { return reachableLanes; }

bool obstacle::getViolatedCvmax() const { return violated_cvmax; }

bool obstacle::getViolatedCamax() const { return violated_camax; }

const polygon_type obstacle::getOccupancyPolygonShape() {

    std::vector<vertice> boundingRectangleVertices;
    polygon_type polygonShape;
    // size_t i;

    if (this->getGeoShape().getType() == "Rectangle") {

        // p are vertices of the bounding rectangle
        // vertices p represent the occupancy with vehicle dimensions (Theorem 1)
        boundingRectangleVertices = addObjectDimensions(
            std::vector<vertice>{vertice{0.0, 0.0}}, this->getGeoShape().getLength(), this->getGeoShape().getWidth());

        /*
         * rotate and translate the vertices of the occupancy set in local
         * coordinates to the object's reference position and rotation
         */
        std::vector<vertice> adjustedBoundingRectangleVertices = rotateAndTranslateVertices(
            boundingRectangleVertices, vertice{this->getXpos(), this->getYpos()}, this->getOrientation());

        polygonShape.outer().resize(adjustedBoundingRectangleVertices.size() + 1);

        // make polygonshape from previously created vertices
        for (size_t i = 0; i < adjustedBoundingRectangleVertices.size(); i++) {
            polygonShape.outer()[i] =
                point_type{adjustedBoundingRectangleVertices[i].x, adjustedBoundingRectangleVertices[i].y};
        }

        // add first point once again at the end
        if (adjustedBoundingRectangleVertices.size() > 0) {
            polygonShape.outer().back() =
                point_type{adjustedBoundingRectangleVertices[0].x, adjustedBoundingRectangleVertices[0].y};
        }
    }
    return polygonShape;
}

void obstacle::setIsStatic(const bool isStatic) {
    this->isStatic = isStatic;
    if (isStatic) {
        velocity = 0.0;
        acceleration = 0.0;
        v_max = 0.0;
        a_max = 0.0;
        a_min_long = 0.0;
        a_max_long = 0.0;
        velocity_error = 0.0;
        acceleration_error = 0.0;
    }
}

bool obstacle::getIsStatic() const { return isStatic; }

//void obstacle::computeOccupancyCore(const std::vector<vehicularLanelet *> &vehLanelets,
//                                    const std::vector<pedestrianLanelet *> &pedLanelets, std::vector<lane *> &lanes,
//                                    timeStruct &timeInterval, EgoVehicle *egoVehicle, mpolygon_t &unionLanelets) {
//    return this->computeOccupancyCore(vehLanelets, lanes, timeInterval, egoVehicle, unionLanelets);
//}
//
//void obstacle::computeOccupancyCore(const std::vector<vehicularLanelet *> &lanelets, std::vector<lane *> &lanes,
//                                    timeStruct &timeInterval, EgoVehicle *egoVehicle, mpolygon_t &unionLanelets) {
//    size_t timeLength = round((timeInterval.ending - timeInterval.startingTime) / timeInterval.timeStep);
//    std::vector<std::vector<vertice>> verticesM1;
//
//    std::vector<vertice> p;
//    std::vector<vertice> q;
//    std::vector<std::vector<vertice>> occM1;
//    // compute the occupancy for static and dynamic objects
//    // std::cout << this->getShape() << std::endl;
//    polygon_type polygon;
//    if (this->getGeoShape().getType() == "Rectangle") {
//        vertice q1;
//
//        q1.x = 0;
//        q1.y = 0;
//        q.push_back(q1);
//        // vertices p represent the occupancy with vehicle dimensions (Theorem 1)
//        p = addObjectDimensions(std::vector<vertice>{q}, this->getGeoShape().getLength(),
//                                this->getGeoShape().getWidth());
//        /*
//         * rotate and translate the vertices of the occupancy set in local
//         * coordinates to the object's reference position and rotation
//         */
//        vertice position;
//        position.x = this->getXpos();
//        position.y = this->getYpos();
//        verticesM1.push_back(rotateAndTranslateVertices(p, position, this->getOrientation()));
//        for (size_t i = 0; i < verticesM1.front().size(); i++) {
//            boost::geometry::append(polygon, point_type{verticesM1.front()[i].x, verticesM1.front()[i].y});
//        }
//        if (verticesM1.front().size() > 0) {
//            boost::geometry::append(polygon, point_type{verticesM1.front()[0].x, verticesM1.front()[0].y});
//        }
//    }
//
//    // initialize occupancy matrix
//    std::vector<std::vector<occTypes>> occMatrix(timeLength);
//
//    for (size_t k = 0; k < timeLength; k++) {
//        occMatrix[k].resize(1);
//        for (size_t j = 0; j < this->getInLane().size(); j++) {
//            occMatrix[k][0].forLane.push_back(this->getInLane()[j]);
//        }
//        occMatrix[k][0].timeInterval.startingTime = timeInterval.startingTime + k * timeInterval.timeStep;
//        occMatrix[k][0].timeInterval.timeStep = timeInterval.timeStep;
//        occMatrix[k][0].timeInterval.ending = timeInterval.startingTime + (k + 1) * timeInterval.timeStep;
//        occMatrix[k][0].vertices.push_back(polygon);
//        occMatrix[k][0].minVelo = 0.0;
//        occMatrix[k][0].maxVelo = 0.0;
//    }
//
//    this->setOccupancyMatrix(occMatrix);
//}
//
//bool obstacle::possibleIntersection(polygon_type *egoM1Polygon, timeStruct &timeInterval) {
//    bool interaction;
//    std::vector<vertice> p;
//    std::vector<vertice> q;
//
//    polygon_type polygon;
//    if (this->getGeoShape().getType() == "Rectangle") {
//        vertice q1;
//
//        vertice position;
//        position.x = this->getXpos();
//        position.y = this->getYpos();
//
//        q1.x = 0;
//        q1.y = 0;
//        q.push_back(q1);
//        // vertices p represent the occupancy with vehicle dimensions (Theorem 1)
//        p = addObjectDimensions(std::vector<vertice>{q}, this->getGeoShape().getLength(),
//                                this->getGeoShape().getWidth());
//
//        q = rotateAndTranslateVertices(p, position, this->getOrientation());
//
//        for (size_t i = 0; i < q.size(); i++) {
//            boost::geometry::append(polygon, point_type{q[i].x, q[i].y});
//        }
//        if (!q.empty()) {
//            boost::geometry::append(polygon, point_type{q[0].x, q[0].y});
//        }
//    }
//
//    boost::geometry::correct(polygon);
//
//    // omit obstacle if there is no possible interection
//    if (boost::geometry::intersects(polygon, *egoM1Polygon)) {
//        interaction = true;
//    } else {
//        interaction = false;
//    }
//
//    return interaction;
//}
//
//bool obstacle::findLaneletsCorrespondingToObstacle(const std::vector<vehicularLanelet *> &intersectionLanelets,
//                                                   std::vector<vehicularLanelet *> &inLanelet) {
//
//    size_t i;
//    std::vector<vehicularLanelet *> oppositeLanelets;
//
//    double laneletOrientation; // orientation of lanelet at obstacle position
//    double obstacleOrientation = this->getOrientation();
//    // if obstacle intersects with any lanelet --> obstacle is in road network
//
//    vertice pos = vertice{this->getXpos(), this->getYpos()};
//    std::pair<double, double> validOrientations = config::getValidTrackOrientations();
//    for (i = 0; i < intersectionLanelets.size(); i++) {
//        // calculate orientation of lanelet at obstacle position
//        laneletOrientation = calcAngleOfVerticesAtPosition((intersectionLanelets)[i]->getCenterVertices(), pos);
//
//        auto relationToLanelet =
//            orientationToTrack(obstacleOrientation, laneletOrientation, this->getOrientationError(), validOrientations);
//        // obstacle is only in a lanelet if it has approx. the same orientation
//        if (relationToLanelet) {
//            double orientationToLanelet = relationToLanelet.value();
//            if (orientationToLanelet < 0) {
//                oppositeLanelets.emplace_back(intersectionLanelets[i]);
//            } else {
//                inLanelet.emplace_back(intersectionLanelets[i]);
//            }
//        }
//    }
//
//    if (inLanelet.empty()) {
//        inLanelet = std::move(oppositeLanelets);
//        return true; // lanelets with opposite direction
//    }
//    return false;
//}

shape &obstacle::getGeoShape() { return geoShape; }
