//
// Created by sebastian on 23.10.20.
//

#include "lanelet_operations.h"

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/// \brief Search lanelets from a vector that don't have any predecessors
///
///  This function mirrors the functionality from the private function
///  "Scenario::findLaneStarts" and is only used for the test cases to test if the lanelets were
///  read in correctly
///
/// \param lanelets
/// \return lanelets which have no predecssor as raw pointers (indicates no responsibility for memory)
/// \note This function may become obsolete with some restructuring of the unit tests
std::vector<vehicularLanelet *> noPredecessorLanelets(const std::vector<std::shared_ptr<vehicularLanelet>> &lanelets) {
    // Potentially also use: const std::vector< std::shared_ptr<const vehicularLanelet> >
    // But then have to use const vehicularLanelet* as return
    std::vector<vehicularLanelet *> noPredLanelets;
    for (size_t i = 0; i < lanelets.size(); i++) {
        if (lanelets[i]->getPredecessors().empty()) // lanelet has no predecessor
        {
            noPredLanelets.emplace_back(lanelets[i].get());
        }
    }
    return noPredLanelets;
}

std::vector<vehicularLanelet *> findAllSuccessorLanelets(vehicularLanelet *lanelets) {
    std::vector<vehicularLanelet *> allSuccessorLanelets;
    for (size_t i = 0; i < lanelets->getSuccessors().size(); i++) {
        allSuccessorLanelets.push_back(lanelets->getSuccessors()[i]);
    }
    return allSuccessorLanelets;
}

std::vector<vehicularLanelet *> findInLaneletByShapeAndDelete(std::vector<vehicularLanelet *> &lanelets,
                                                              const polygon_type &polygonShape) {

    std::vector<vehicularLanelet *> intersectionLanelets;

    intersectionLanelets.reserve(lanelets.size());

    // check if obstacle intersects with any lanelet, add this lanelet to intersectionLanelets
    // also delete lanelet from lanelets, it is not considered anymore

    for (auto it = lanelets.begin(); it != lanelets.end();) {
        if ((*it)->checkIntersection(polygonShape, PARTIALLY_CONTAINED)) {
            intersectionLanelets.emplace_back(*it);
            it = lanelets.erase(it);
        } else {
            ++it;
        }
    }

    return intersectionLanelets;
}

std::vector<vertice> resample_polyline(const std::vector<vertice> &polyline, size_t step) {
    std::vector<vertice> newPolyline{polyline[0]};
    double current_position = 0 + step;
    double current_length =
            sqrt(std::pow(polyline[0].x - polyline[1].x, 2.0) + std::pow(polyline[0].y - polyline[1].y, 2.0));
    size_t current_idx = 0;

    while (current_idx < polyline.size() - 1) {
        if (current_position >= current_length) {
            current_position = current_position - current_length;

            if (current_idx > polyline.size() - 2) {
                break;
            }
            current_idx++;
            current_length = sqrt(std::pow(polyline[current_idx].x - polyline[current_idx + 1].x, 2.0) +
                                  std::pow(polyline[current_idx].y - polyline[current_idx + 1].y, 2.0));
        } else {
            double rel = current_position / current_length;
            vertice temp;
            temp.x = (1 - rel) * polyline[current_idx].x + rel * polyline[current_idx + 1].x;
            temp.y = (1 - rel) * polyline[current_idx].y + rel * polyline[current_idx + 1].y;
            newPolyline.emplace_back(temp);

            current_position += step;
        }
    }

    return newPolyline;
}

std::vector<vertice> chaikins_corner_cutting(const std::vector<vertice> &polyline) {
    std::vector<vertice> newPolyline;
    if (!polyline.empty()) {
        newPolyline.emplace_back(polyline[0]);
    }

    for (int i = 0; i < (int)polyline.size() - 1; i++) {
        newPolyline.emplace_back(
                vertice{0.75 * polyline[i].x + 0.25 * polyline[i + 1].x, 0.75 * polyline[i].y + 0.25 * polyline[i + 1].y});
        newPolyline.emplace_back(
                vertice{0.25 * polyline[i].x + 0.75 * polyline[i + 1].x, 0.25 * polyline[i].y + 0.75 * polyline[i + 1].y});
    }
    if (!polyline.empty()) {
        newPolyline.emplace_back(polyline.back());
    }
    return newPolyline;
}

mpolygon_t unify_lanelets(const std::vector<vehicularLanelet *> &lanelets) {
    // Declare strategies
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(0.01);
    boost::geometry::strategy::buffer::join_round join_strategy;
    boost::geometry::strategy::buffer::end_round end_strategy;
    boost::geometry::strategy::buffer::point_circle circle_strategy;
    boost::geometry::strategy::buffer::side_straight side_strategy;

    mpolygon_t unionLanelets;

    size_t i;

    for (i = 0; i < lanelets.size(); i++) {
        // Declare output
        mpolygon_t result;

        // Create the buffer of a linestring
        boost::geometry::buffer(lanelets[i]->getOuterPolygon(), result, distance_strategy, side_strategy, join_strategy,
                                end_strategy, circle_strategy);

        mpolygon_t temp;

        boost::geometry::union_(unionLanelets, result[0], temp);

        unionLanelets = temp;
    }

    return unionLanelets;
}
