// Implements OpenStreetMap XML parsing functionality.

#include "OSMParser.h"

#include "tinyxml2.h"

#include <cstdint>
#include <iostream>
#include <utility>

bool OSMParser::parse(const std::string& filepath) {
    nodes.clear();
    ways.clear();

    tinyxml2::XMLDocument document;
    const tinyxml2::XMLError loadResult = document.LoadFile(filepath.c_str());
    if (loadResult != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to open or parse OSM file: " << filepath;
        const char* errorText = document.ErrorStr();
        if (errorText != nullptr && errorText[0] != '\0') {
            std::cerr << " (" << errorText << ")";
        }
        std::cerr << std::endl;
        return false;
    }

    const tinyxml2::XMLElement* osm = document.FirstChildElement("osm");
    if (osm == nullptr) {
        std::cerr << "Invalid OSM file: missing <osm> root element in " << filepath << std::endl;
        return false;
    }

    for (const tinyxml2::XMLElement* nodeElement = osm->FirstChildElement("node");
         nodeElement != nullptr;
         nodeElement = nodeElement->NextSiblingElement("node")) {
        std::int64_t id = 0;
        double lat = 0.0;
        double lon = 0.0;

        const tinyxml2::XMLError idResult = nodeElement->QueryInt64Attribute("id", &id);
        const tinyxml2::XMLError latResult = nodeElement->QueryDoubleAttribute("lat", &lat);
        const tinyxml2::XMLError lonResult = nodeElement->QueryDoubleAttribute("lon", &lon);

        if (idResult != tinyxml2::XML_SUCCESS ||
            latResult != tinyxml2::XML_SUCCESS ||
            lonResult != tinyxml2::XML_SUCCESS) {
            std::cerr << "Skipping malformed <node> element in " << filepath << std::endl;
            continue;
        }

        nodes[static_cast<long long>(id)] = OSMNode{
            static_cast<long long>(id),
            lat,
            lon
        };
    }

    for (const tinyxml2::XMLElement* wayElement = osm->FirstChildElement("way");
         wayElement != nullptr;
         wayElement = wayElement->NextSiblingElement("way")) {
        std::int64_t wayId = 0;
        if (wayElement->QueryInt64Attribute("id", &wayId) != tinyxml2::XML_SUCCESS) {
            std::cerr << "Skipping malformed <way> element without a valid id in " << filepath << std::endl;
            continue;
        }

        std::string highwayType;
        for (const tinyxml2::XMLElement* tagElement = wayElement->FirstChildElement("tag");
             tagElement != nullptr;
             tagElement = tagElement->NextSiblingElement("tag")) {
            const char* key = tagElement->Attribute("k");
            if (key == nullptr || std::string(key) != "highway") {
                continue;
            }

            const char* value = tagElement->Attribute("v");
            if (value != nullptr) {
                highwayType = value;
            }
            break;
        }

        if (highwayType.empty()) {
            continue;
        }

        OSMWay way;
        way.id = static_cast<long long>(wayId);
        way.highwayType = std::move(highwayType);

        for (const tinyxml2::XMLElement* nodeRefElement = wayElement->FirstChildElement("nd");
             nodeRefElement != nullptr;
             nodeRefElement = nodeRefElement->NextSiblingElement("nd")) {
            std::int64_t nodeRef = 0;
            if (nodeRefElement->QueryInt64Attribute("ref", &nodeRef) != tinyxml2::XML_SUCCESS) {
                std::cerr << "Skipping malformed <nd> reference in way " << way.id << std::endl;
                continue;
            }

            way.nodeRefs.push_back(static_cast<long long>(nodeRef));
        }

        ways.push_back(std::move(way));
    }

    std::cout << "Parsed " << nodes.size() << " nodes, " << ways.size() << " road ways" << std::endl;
    return true;
}

const std::unordered_map<long long, OSMNode>& OSMParser::getNodes() const {
    return nodes;
}

const std::vector<OSMWay>& OSMParser::getWays() const {
    return ways;
}

double OSMParser::getWeightForHighway(const std::string& type) const {
    if (type == "motorway" || type == "trunk") {
        return 1.0;
    }
    if (type == "primary" || type == "secondary") {
        return 1.2;
    }
    if (type == "residential" || type == "tertiary") {
        return 1.5;
    }
    if (type == "footway" || type == "path" || type == "cycleway") {
        return 2.0;
    }
    return 1.5;
}