/*
 * DIJKSTRA vs A* COMPARISON:
 * Dijkstra: no heuristic, expands uniformly in all directions
 * A*: guided by Haversine heuristic toward goal
 * Both guarantee optimal path
 * Time Complexity: O((V+E) log V) — Big-Theta: Θ((V+E) log V)
 * Space Complexity: O(V) — Big-Omega: Ω(V) best case
 * A* explores fewer nodes in practice due to heuristic guidance
 * Recurrence (path reconstruction): T(n) = T(n-1) + O(1) => O(n)
 */

#include "Dijkstra.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include "Utils.h"

Dijkstra::Dijkstra(const Graph& graph) : graph(graph) {}

std::vector<long long> Dijkstra::findPath(long long startId, long long endId) {
    nodesExplored = 0;
    pathCostTotal = 0.0;
    executionTimeMs = 0.0;
    totalElevationGain = 0.0;
    totalElevationLoss = 0.0;
    estimatedFlightTimeSec = 0.0;
    exploredOrder.clear();

    const auto startTime = std::chrono::high_resolution_clock::now();

    try {
        graph.getNode(startId);
        graph.getNode(endId);
    } catch (const std::exception& e) {
        std::cerr << "Dijkstra error: " << e.what() << std::endl;
        const auto endTime = std::chrono::high_resolution_clock::now();
        executionTimeMs =
            std::chrono::duration<double, std::milli>(endTime - startTime).count();
        return {};
    }

    std::priority_queue<
        std::pair<double, long long>,
        std::vector<std::pair<double, long long>>,
        std::greater<std::pair<double, long long>>> pq;

    std::unordered_map<long long, double> dist;
    std::unordered_map<long long, long long> parent;
    std::unordered_set<long long> visited;

    for (const auto& entry : graph.getAllNodes()) {
        dist[entry.first] = std::numeric_limits<double>::infinity();
    }

    dist[startId] = 0.0;
    pq.push({0.0, startId});

    while (!pq.empty()) {
        const double currentDist = pq.top().first;
        const long long currentId = pq.top().second;
        pq.pop();

        if (visited.find(currentId) != visited.end()) {
            continue;
        }

        visited.insert(currentId);
        exploredOrder.push_back(currentId);
        ++nodesExplored;

        if (currentId == endId) {
            pathCostTotal = currentDist;
            const auto endTime = std::chrono::high_resolution_clock::now();
            executionTimeMs =
                std::chrono::duration<double, std::milli>(endTime - startTime).count();

            // Calculate drone flight metrics along the found path
            std::vector<long long> path = reconstructPath(startId, endId, parent);
            double pathDistanceMeters = 0.0;
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                const GraphNode& curr = graph.getNode(path[i]);
                const GraphNode& next = graph.getNode(path[i + 1]);
                pathDistanceMeters += DroneUtils::haversineDistanceMeters(curr.lat, curr.lon, next.lat, next.lon);
                const double diff = next.elevation - curr.elevation;
                if (diff > 0.0) totalElevationGain += diff;
                else if (diff < 0.0) totalElevationLoss += (-diff);
            }

            // DJI Mavic 3 reference: 15 m/s cruise, 6 m/s ascent, 3 m/s descent
            constexpr double cruiseSpeed = 15.0;  // m/s
            constexpr double ascentRate = 6.0;    // m/s
            constexpr double descentRate = 3.0;   // m/s
            const double horizontalTime = pathDistanceMeters / cruiseSpeed;
            const double climbTime = totalElevationGain / ascentRate;
            const double descentTime = totalElevationLoss / descentRate;
            estimatedFlightTimeSec = horizontalTime + climbTime + descentTime;

            return path;
        }

        const std::vector<GraphEdge>& neighbors = graph.getNeighbors(currentId);
        for (const GraphEdge& edge : neighbors) {
            // No-fly zones handled as cost penalties in Graph, not hard blocks
            // Drone can physically fly anywhere but avoids restricted airspace
            if (visited.find(edge.toNodeId) != visited.end()) {
                continue;
            }

            const double newDist = currentDist + edge.weight;
            if (newDist < dist[edge.toNodeId]) {
                dist[edge.toNodeId] = newDist;
                parent[edge.toNodeId] = currentId;
                pq.push({newDist, edge.toNodeId});
            }
        }
    }

    const auto endTime = std::chrono::high_resolution_clock::now();
    executionTimeMs =
        std::chrono::duration<double, std::milli>(endTime - startTime).count();
    return {};
}

std::vector<long long> Dijkstra::reconstructPath(
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