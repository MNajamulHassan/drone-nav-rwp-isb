// Implements the A* pathfinding algorithm.

#include "AStar.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace {
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

double toRadians(double degrees) {
    return degrees * kPi / 180.0;
}
}  // namespace

AStar::AStar(const Graph& graph) : graph(graph) {}

// Time Complexity: O((V + E) log V)
// Space Complexity: O(V)
// Recurrence (path reconstruction): T(n) = T(n-1) + O(1) => O(n)
// Big-Theta: Theta((V+E) log V) average case
// Big-Omega: Omega(V) best case (goal is immediate neighbor)
std::vector<long long> AStar::findPath(long long startId, long long endId) {
    nodesExplored = 0;
    pathCostTotal = 0.0;
    executionTimeMs = 0.0;
    exploredOrder.clear();

    const auto startTime = std::chrono::high_resolution_clock::now();

    try {
        graph.getNode(startId);
        graph.getNode(endId);
    } catch (const std::exception& e) {
        std::cerr << "AStar error: " << e.what() << std::endl;
        const auto endTime = std::chrono::high_resolution_clock::now();
        executionTimeMs =
            std::chrono::duration<double, std::milli>(endTime - startTime).count();
        return {};
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openList;
    std::unordered_map<long long, double> gCost;
    std::unordered_map<long long, long long> parent;
    std::unordered_set<long long> visited;

    for (const auto& entry : graph.getAllNodes()) {
        gCost[entry.first] = std::numeric_limits<double>::infinity();
    }

    gCost[startId] = 0.0;
    openList.push({startId, 0.0, heuristic(startId, endId), heuristic(startId, endId), -1});

    while (!openList.empty()) {
        const AStarNode current = openList.top();
        openList.pop();

        if (visited.find(current.nodeId) != visited.end()) {
            continue;
        }

        visited.insert(current.nodeId);
        exploredOrder.push_back(current.nodeId);
        ++nodesExplored;

        if (current.nodeId == endId) {
            pathCostTotal = gCost[endId];
            const auto endTime = std::chrono::high_resolution_clock::now();
            executionTimeMs =
                std::chrono::duration<double, std::milli>(endTime - startTime).count();
            return reconstructPath(startId, endId, parent);
        }

        const std::vector<GraphEdge>& neighbors = graph.getNeighbors(current.nodeId);
        for (const GraphEdge& edge : neighbors) {
            const GraphNode& neighborNode = graph.getNode(edge.toNodeId);
            if (neighborNode.isNoFly) {
                continue;
            }
            if (visited.find(edge.toNodeId) != visited.end()) {
                continue;
            }

            const double newG = gCost[current.nodeId] + edge.weight;
            if (newG < gCost[edge.toNodeId]) {
                const double h = heuristic(edge.toNodeId, endId);
                gCost[edge.toNodeId] = newG;
                parent[edge.toNodeId] = current.nodeId;
                openList.push({edge.toNodeId, newG, h, newG + h, current.nodeId});
            }
        }
    }

    const auto endTime = std::chrono::high_resolution_clock::now();
    executionTimeMs =
        std::chrono::duration<double, std::milli>(endTime - startTime).count();
    return {};
}

double AStar::heuristic(long long nodeId, long long goalId) const {
    const GraphNode& node = graph.getNode(nodeId);
    const GraphNode& goal = graph.getNode(goalId);

    const double lat1Rad = toRadians(node.lat);
    const double lat2Rad = toRadians(goal.lat);
    const double deltaLat = toRadians(goal.lat - node.lat);
    const double deltaLon = toRadians(goal.lon - node.lon);

    const double sinHalfLat = std::sin(deltaLat / 2.0);
    const double sinHalfLon = std::sin(deltaLon / 2.0);
    const double a = sinHalfLat * sinHalfLat +
                     std::cos(lat1Rad) * std::cos(lat2Rad) * sinHalfLon * sinHalfLon;
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return kEarthRadiusMeters * c;
}

std::vector<long long> AStar::reconstructPath(
    long long startId,
    long long endId,
    const std::unordered_map<long long, long long>& parent) const {
    std::vector<long long> path;
    long long currentId = endId;
    path.push_back(currentId);

    while (currentId != startId) {
        const auto it = parent.find(currentId);
        if (it == parent.end()) {
            return {};
        }

        currentId = it->second;
        path.push_back(currentId);
    }

    std::reverse(path.begin(), path.end());
    return path;
}