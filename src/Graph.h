#pragma once

// Declares the graph data structures for drone navigation.

#include "OSMParser.h"
#include "ElevationMap.h"
#include "NoFlyZone.h"

#include <unordered_map>
#include <vector>

struct GraphNode {
    long long osmId;
    double lat;
    double lon;
    double elevation;
    bool isNoFly;
};

struct GraphEdge {
    long long toNodeId;
    double weight;
};

class Graph {
public:
    void build(const OSMParser& parser, ElevationMap& elevation, const NoFlyZoneManager& noFlyZones);

    double haversineDistance(double lat1, double lon1, double lat2, double lon2);
    long long findNearestNode(double lat, double lon) const;

    const GraphNode& getNode(long long id) const;
    const std::vector<GraphEdge>& getNeighbors(long long id) const;
    const std::unordered_map<long long, GraphNode>& getAllNodes() const;

private:
    std::unordered_map<long long, GraphNode> nodes;
    std::unordered_map<long long, std::vector<GraphEdge>> adjacency;
};