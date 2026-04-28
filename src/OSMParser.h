#pragma once

// Declares the OpenStreetMap XML parser interface.

#include <string>
#include <unordered_map>
#include <vector>

struct OSMNode {
    long long id;
    double lat;
    double lon;
};

struct OSMWay {
    long long id;
    std::vector<long long> nodeRefs;
    std::string highwayType;
};

class OSMParser {
public:
    bool parse(const std::string& filepath);

    const std::unordered_map<long long, OSMNode>& getNodes() const;
    const std::vector<OSMWay>& getWays() const;
    double getWeightForHighway(const std::string& type) const;

private:
    std::unordered_map<long long, OSMNode> nodes;
    std::vector<OSMWay> ways;
};