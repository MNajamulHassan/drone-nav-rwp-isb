#include "OSMParser.h"
#include "ElevationMap.h"
#include "NoFlyZone.h"
#include "Graph.h"
#include "AStar.h"
#include "Dijkstra.h"
#include "GUI.h"

#include <iostream>

int main() {
    std::cout << "========================================\n"
              << "  DroneNavReal - Drone Path Planning\n"
              << "  Rawalpindi / Islamabad, Pakistan\n"
              << "========================================\n\n";

    OSMParser parser;
    if (!parser.parse("data/rawalpindi.osm")) {
        std::cerr << "ERROR: data/rawalpindi.osm not found!\n"
                  << "1. Go to https://overpass-turbo.eu\n"
                  << "2. Run query: [out:xml][timeout:90][bbox:33.55,72.95,33.80,73.20];\n"
                  << "   (way[highway];); out body; >; out skel qt;\n"
                  << "3. Export as OSM -> save to data/rawalpindi.osm\n";
        return 1;
    }

    ElevationMap elevation;
    elevation.printInfo();

    NoFlyZoneManager noFlyZones;
    noFlyZones.printZones();

    Graph graph;
    graph.build(parser, elevation, noFlyZones);

    AStar astar(graph);
    Dijkstra dijkstra(graph);

    GUI gui(graph, noFlyZones);
    if (!gui.init()) {
        std::cerr << "GUI failed\n";
        return 1;
    }
    gui.run(astar, dijkstra);

    std::cout << "DroneNavReal exited cleanly.\n";
    return 0;
}