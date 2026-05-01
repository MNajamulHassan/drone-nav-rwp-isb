#pragma once

#include "Graph.h"
#include "NoFlyZone.h"
#include "AStar.h"
#include "Dijkstra.h"

#include <SFML/Graphics.hpp>
#include <string>
#include <vector>
#include <utility>

class GUI {
public:
    static constexpr double MIN_LAT = 33.55;
    static constexpr double MAX_LAT = 33.80;
    static constexpr double MIN_LON = 72.95;
    static constexpr double MAX_LON = 73.20;

    GUI(Graph& graph, const NoFlyZoneManager& noFlyZones);

    bool init();
    void run(AStar& astar, Dijkstra& dijkstra);

private:
    enum class Mode { VIEW, SET_START, SET_END };

    Graph& graph;
    const NoFlyZoneManager& noFlyZones;

    sf::RenderWindow window;
    sf::Texture mapTexture;
    sf::Sprite mapSprite;
    sf::Font font;

    bool hasMapImage;

    Mode currentMode = Mode::VIEW;

    long long startNodeId = -1;
    long long endNodeId = -1;
    std::vector<long long> currentPath;
    std::vector<long long> exploredOrder;

    // Stats
    std::string lastAlgorithm = "-";
    std::string pathFoundStatus = "-";
    double lastDistance = 0.0;
    double lastCost = 0.0;
    int lastExplored = 0;
    double lastTimeMs = 0.0;

    sf::Vector2f gpsToScreen(double lat, double lon) const;
    std::pair<double, double> screenToGps(float x, float y) const;

    void handleEvents(AStar& astar, Dijkstra& dijkstra);
    void handleMouseClick(float x, float y, AStar& astar, Dijkstra& dijkstra);
    
    void drawMap();
    void drawNoFlyZones();
    void drawGraphNodes();
    void drawExploredNodes();
    void drawPath();
    void drawMarkers();
    void drawSidebar();
};