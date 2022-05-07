//
// Created by sebastian on 07.05.22.
//

#pragma once

// class to get all paths from a source to destination.
#include "lanelet.h"
#include <map>
#include <vector>

// A directed graph using
// adjacency list representation
class LaneletGraph {
    std::map<size_t, std::vector<size_t>> graphAdjSuc;
    std::map<size_t, std::vector<size_t>> graphSuc;
    static void printPath(std::vector<size_t> &path);
    static bool isNotVisited(size_t node, std::vector<size_t> &path);
    static bool exists(const std::vector<std::vector<size_t>> &paths, const std::vector<size_t> &newPath);
    static std::vector<std::vector<size_t>> findPaths(size_t src, size_t dst,
                                                      const std::map<size_t, std::vector<size_t>> &graphAdjSuc);

  public:
    LaneletGraph(const std::vector<std::shared_ptr<Lanelet>> &lanelets);
    std::vector<std::vector<size_t>> findPaths(size_t src, size_t dst, bool considerAdjacency);
};
