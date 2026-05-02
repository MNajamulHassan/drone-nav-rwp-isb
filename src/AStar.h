#pragma once

// Declares the A* pathfinding interface.

#include "Graph.h"

#include <vector>

struct AStarNode {
    long long nodeId;
    double g;
    double h;
    double f;
    long long parentId;

    bool operator>(const AStarNode& other) const { return f > other.f; }
};

class AStar {
public:
    explicit AStar(const Graph& graph);

    int nodesExplored = 0;
    double pathCostTotal = 0.0;
    double executionTimeMs = 0.0;
    std::vector<long long> exploredOrder;

    // Drone flight metrics (DJI Mavic 3 reference specs)
    double totalElevationGain = 0.0;    // meters climbed
    double totalElevationLoss = 0.0;    // meters descended
    double estimatedFlightTimeSec = 0.0;

    std::vector<long long> findPath(long long startId, long long endId);

private:
    const Graph& graph;

    double heuristic(long long nodeId, long long goalId) const;
    std::vector<long long> reconstructPath(
        long long startId,
        long long endId,
        const std::unordered_map<long long, long long>& parent) const;
};