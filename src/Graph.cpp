// Implements graph-related functionality for drone navigation.

#include "Graph.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "Utils.h"

void Graph::build(const OSMParser& parser, ElevationMap& elevation, const NoFlyZoneManager& noFlyZones) {
    nodes.clear();
    adjacency.clear();

    for (const auto& entry : parser.getNodes()) {
        const OSMNode& osmNode = entry.second;
        nodes[osmNode.id] = GraphNode{
            osmNode.id,
            osmNode.lat,
            osmNode.lon,
            elevation.getElevation(osmNode.lat, osmNode.lon),
            noFlyZones.isInNoFlyZone(osmNode.lat, osmNode.lon)
        };
        adjacency[osmNode.id] = {};
    }

    std::size_t edgeCount = 0;

    for (const OSMWay& way : parser.getWays()) {
        const double roadWeight = parser.getWeightForHighway(way.highwayType);
        for (std::size_t i = 0; i + 1 < way.nodeRefs.size(); ++i) {
            const long long fromId = way.nodeRefs[i];
            const long long toId = way.nodeRefs[i + 1];

            const auto fromIt = nodes.find(fromId);
            const auto toIt = nodes.find(toId);
            if (fromIt == nodes.end() || toIt == nodes.end()) {
                continue;
            }

            const GraphNode& fromNode = fromIt->second;
            const GraphNode& toNode = toIt->second;

            const double distance = haversineDistance(fromNode.lat, fromNode.lon, toNode.lat, toNode.lon);
            const double elevationCost =
                elevation.getElevationCost(fromNode.lat, fromNode.lon, toNode.lat, toNode.lon);
            const double weight = distance * roadWeight + elevationCost;

            adjacency[fromId].push_back({toId, weight});
            adjacency[toId].push_back({fromId, weight});
            edgeCount += 2;
        }
    }

    std::cout << "Graph built: " << nodes.size() << " nodes, " << edgeCount << " edges" << std::endl;
}

double Graph::haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    return DroneUtils::haversineDistanceMeters(lat1, lon1, lat2, lon2);
}

long long Graph::findNearestNode(double lat, double lon) const {
    long long nearestId = -1;
    double bestDistance = std::numeric_limits<double>::max();

    for (const auto& entry : nodes) {
        const GraphNode& node = entry.second;
        const double distance = DroneUtils::haversineDistanceMeters(lat, lon, node.lat, node.lon);

        if (distance < bestDistance) {
            bestDistance = distance;
            nearestId = node.osmId;
        }
    }

    return nearestId;
}

const GraphNode& Graph::getNode(long long id) const {
    const auto it = nodes.find(id);
    if (it == nodes.end()) {
        throw std::out_of_range("Graph node ID not found");
    }
    return it->second;
}

const std::vector<GraphEdge>& Graph::getNeighbors(long long id) const {
    const auto it = adjacency.find(id);
    if (it == adjacency.end()) {
        throw std::out_of_range("Graph adjacency list for node ID not found");
    }
    return it->second;
}

const std::unordered_map<long long, GraphNode>& Graph::getAllNodes() const {
    return nodes;
}