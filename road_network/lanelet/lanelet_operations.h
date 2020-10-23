//
// Created by sebastian on 23.10.20.
//

#ifndef ENVIRONMENT_MODEL_LANELET_OPERATIONS_H
#define ENVIRONMENT_MODEL_LANELET_OPERATIONS_H


// find the lanelets which have no predecessor
std::vector<vehicularLanelet *> noPredecessorLanelets(const std::vector<std::shared_ptr<vehicularLanelet>> &lanelets);

// returns all successor lanelets of a lanelet
std::vector<vehicularLanelet *> findAllSuccessorLanelets(vehicularLanelet *lanelets);

std::vector<vehicularLanelet *> findInLaneletByShapeAndDelete(std::vector<vehicularLanelet *> &lanelets,
                                                              const polygon_type &polygonShape);

// algorithms for lanelet smoothing
std::vector<vertice> resample_polyline(const std::vector<vertice> &polyline, size_t step = 2);
std::vector<vertice> chaikins_corner_cutting(const std::vector<vertice> &polyline);

// union of all lanelets
mpolygon_t unify_lanelets(const std::vector<vehicularLanelet *> &lanelets);



#endif //ENVIRONMENT_MODEL_LANELET_OPERATIONS_H
