//
// Created by sebastian on 07.05.22.
//

#pragma once

// class to get all paths from a source to destination.
#include "dijkstra.h"
#include "lanelet.h"
#include <map>
#include <vector>

// A directed graph using
// adjacency list representation
class LaneletGraph {
    std::unordered_map<size_t, size_t> verticesAdjSuc;
    std::unordered_map<size_t, size_t> verticesSuc;
    std::unordered_map<size_t, size_t> verticesAdjSucRes;
    std::unordered_map<size_t, size_t> verticesSucRes;
    graph<std::size_t, size_t> graphAdjSuc;
    graph<std::size_t, size_t> graphSuc;

  public:
    LaneletGraph(const std::vector<std::shared_ptr<Lanelet>> &lanelets);
    std::vector<size_t> findPaths(size_t src, size_t dst, bool considerAdjacency);
};
