#include "GUI.h"
#include <iostream>
#include <cmath>
#include <cstdio>

GUI::GUI(Graph& graph, const NoFlyZoneManager& noFlyZones)
    : graph(graph), noFlyZones(noFlyZones), hasMapImage(false) {}

bool GUI::init() {
    window.create(sf::VideoMode(1280, 800), "DroneNav \xE2\x80\x94 Drone Path Planning System");
    window.setFramerateLimit(60);

    if (!font.loadFromFile("C:/Windows/Fonts/arial.ttf")) {
        std::cerr << "Warning: Failed to load arial.ttf. Text may not render correctly." << std::endl;
    }

    if (mapTexture.loadFromFile("data/map.png")) {
        hasMapImage = true;
        mapSprite.setTexture(mapTexture);
        mapSprite.setScale(950.0f / mapTexture.getSize().x, 800.0f / mapTexture.getSize().y);
    } else {
        hasMapImage = false;
        std::cerr << "Notice: map.png not found. Using fallback green grid." << std::endl;
    }

    return true;
}

void GUI::run(AStar& astar, Dijkstra& dijkstra) {
    while (window.isOpen()) {
        handleEvents(astar, dijkstra);

        window.clear();
        drawMap();
        drawNoFlyZones();
        drawGraphNodes();
        drawExploredNodes();
        drawPath();
        drawMarkers();
        drawSidebar();
        window.display();
    }
}

sf::Vector2f GUI::gpsToScreen(double lat, double lon) const {
    float x = static_cast<float>((lon - MIN_LON) / (MAX_LON - MIN_LON) * 950.0);
    float y = static_cast<float>((MAX_LAT - lat) / (MAX_LAT - MIN_LAT) * 800.0);
    return sf::Vector2f(x, y);
}

std::pair<double, double> GUI::screenToGps(float x, float y) const {
    double lon = MIN_LON + (x / 950.0) * (MAX_LON - MIN_LON);
    double lat = MAX_LAT - (y / 800.0) * (MAX_LAT - MIN_LAT);
    return {lat, lon};
}

void GUI::handleEvents(AStar& astar, Dijkstra& dijkstra) {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        } else if (event.type == sf::Event::MouseButtonPressed) {
            if (event.mouseButton.button == sf::Mouse::Left) {
                handleMouseClick(static_cast<float>(event.mouseButton.x), 
                                 static_cast<float>(event.mouseButton.y), 
                                 astar, dijkstra);
            }
        }
    }
}

void GUI::handleMouseClick(float x, float y, AStar& astar, Dijkstra& dijkstra) {
    if (x < 950) {
        if (currentMode == Mode::SET_START) {
            auto gps = screenToGps(x, y);
            startNodeId = graph.findNearestNode(gps.first, gps.second);
            currentMode = Mode::VIEW;
        } else if (currentMode == Mode::SET_END) {
            auto gps = screenToGps(x, y);
            endNodeId = graph.findNearestNode(gps.first, gps.second);
            currentMode = Mode::VIEW;
        }
    } else {
        sf::FloatRect runAStarRect(975, 120, 280, 45);
        sf::FloatRect runDijkstraRect(975, 180, 280, 45);
        sf::FloatRect clearRect(975, 240, 280, 45);
        sf::FloatRect startRect(975, 300, 280, 45);
        sf::FloatRect endRect(975, 360, 280, 45);

        if (runAStarRect.contains(x, y)) {
            if (startNodeId != -1 && endNodeId != -1) {
                currentPath = astar.findPath(startNodeId, endNodeId);
                exploredOrder = astar.exploredOrder;
                lastAlgorithm = "A*";
                lastCost = astar.pathCostTotal;
                lastExplored = astar.nodesExplored;
                lastTimeMs = astar.executionTimeMs;
                
                if (!currentPath.empty()) {
                    pathFoundStatus = "Yes";
                    lastDistance = 0.0;
                    for (size_t i = 0; i + 1 < currentPath.size(); ++i) {
                        try {
                            const GraphNode& n1 = graph.getNode(currentPath[i]);
                            const GraphNode& n2 = graph.getNode(currentPath[i+1]);
                            lastDistance += graph.haversineDistance(n1.lat, n1.lon, n2.lat, n2.lon);
                        } catch (...) {}
                    }
                } else {
                    pathFoundStatus = "No";
                    lastDistance = 0.0;
                }
            }
        } else if (runDijkstraRect.contains(x, y)) {
            if (startNodeId != -1 && endNodeId != -1) {
                currentPath = dijkstra.findPath(startNodeId, endNodeId);
                exploredOrder = dijkstra.exploredOrder;
                lastAlgorithm = "Dijkstra";
                lastCost = dijkstra.pathCostTotal;
                lastExplored = dijkstra.nodesExplored;
                lastTimeMs = dijkstra.executionTimeMs;
                
                if (!currentPath.empty()) {
                    pathFoundStatus = "Yes";
                    lastDistance = 0.0;
                    for (size_t i = 0; i + 1 < currentPath.size(); ++i) {
                        try {
                            const GraphNode& n1 = graph.getNode(currentPath[i]);
                            const GraphNode& n2 = graph.getNode(currentPath[i+1]);
                            lastDistance += graph.haversineDistance(n1.lat, n1.lon, n2.lat, n2.lon);
                        } catch (...) {}
                    }
                } else {
                    pathFoundStatus = "No";
                    lastDistance = 0.0;
                }
            }
        } else if (clearRect.contains(x, y)) {
            currentPath.clear();
            exploredOrder.clear();
            lastAlgorithm = "-";
            pathFoundStatus = "-";
            lastDistance = 0.0;
            lastCost = 0.0;
            lastExplored = 0;
            lastTimeMs = 0.0;
        } else if (startRect.contains(x, y)) {
            currentMode = Mode::SET_START;
        } else if (endRect.contains(x, y)) {
            currentMode = Mode::SET_END;
        }
    }
}

void GUI::drawMap() {
    if (hasMapImage) {
        window.draw(mapSprite);
    } else {
        sf::RectangleShape bg(sf::Vector2f(950, 800));
        bg.setFillColor(sf::Color(26, 58, 26)); // #1a3a1a
        window.draw(bg);
        
        sf::VertexArray grid(sf::Lines);
        for (int i = 0; i < 950; i += 50) {
            grid.append(sf::Vertex(sf::Vector2f(static_cast<float>(i), 0.0f), sf::Color(255, 255, 255, 30)));
            grid.append(sf::Vertex(sf::Vector2f(static_cast<float>(i), 800.0f), sf::Color(255, 255, 255, 30)));
        }
        for (int i = 0; i < 800; i += 50) {
            grid.append(sf::Vertex(sf::Vector2f(0.0f, static_cast<float>(i)), sf::Color(255, 255, 255, 30)));
            grid.append(sf::Vertex(sf::Vector2f(950.0f, static_cast<float>(i)), sf::Color(255, 255, 255, 30)));
        }
        window.draw(grid);
    }
}

void GUI::drawNoFlyZones() {
    for (const auto& zone : noFlyZones.getAllZones()) {
        double distPerPxY = graph.haversineDistance(
            zone.centerLat, zone.centerLon, 
            zone.centerLat + (MAX_LAT - MIN_LAT) / 800.0, zone.centerLon
        );
        if (distPerPxY <= 0.01) distPerPxY = 34.78; 
        
        float radiusPx = static_cast<float>(zone.radiusMeters / distPerPxY);
        sf::Vector2f pos = gpsToScreen(zone.centerLat, zone.centerLon);
        
        sf::CircleShape circle(radiusPx);
        circle.setOrigin(radiusPx, radiusPx);
        circle.setPosition(pos);
        circle.setFillColor(sf::Color(255, 0, 0, 100));
        window.draw(circle);
        
        sf::Text label(zone.name, font, 14);
        label.setFillColor(sf::Color::White);
        label.setOutlineThickness(1.0f);
        label.setOutlineColor(sf::Color::Black);
        
        sf::FloatRect bounds = label.getLocalBounds();
        label.setOrigin(bounds.left + bounds.width / 2.0f, bounds.top + bounds.height / 2.0f);
        label.setPosition(pos.x, pos.y);
        window.draw(label);
    }
}

void GUI::drawGraphNodes() {
    const auto& allNodes = graph.getAllNodes();
    if (allNodes.size() <= 50000) {
        sf::VertexArray nodesArray(sf::Points);
        for (const auto& entry : allNodes) {
            sf::Vector2f pos = gpsToScreen(entry.second.lat, entry.second.lon);
            nodesArray.append(sf::Vertex(pos, sf::Color(128, 128, 128)));
        }
        window.draw(nodesArray);
    }
}

void GUI::drawExploredNodes() {
    if (exploredOrder.empty()) return;
    
    sf::VertexArray exploredArray(sf::Quads);
    sf::Color col(255, 255, 0); // yellow
    
    for (long long id : exploredOrder) {
        try {
            const GraphNode& node = graph.getNode(id);
            sf::Vector2f pos = gpsToScreen(node.lat, node.lon);
            exploredArray.append(sf::Vertex(sf::Vector2f(pos.x - 1, pos.y - 1), col));
            exploredArray.append(sf::Vertex(sf::Vector2f(pos.x + 1, pos.y - 1), col));
            exploredArray.append(sf::Vertex(sf::Vector2f(pos.x + 1, pos.y + 1), col));
            exploredArray.append(sf::Vertex(sf::Vector2f(pos.x - 1, pos.y + 1), col));
        } catch (...) {}
    }
    window.draw(exploredArray);
}

void GUI::drawPath() {
    if (currentPath.size() < 2) return;
    
    for (size_t i = 0; i + 1 < currentPath.size(); ++i) {
        try {
            const GraphNode& n1 = graph.getNode(currentPath[i]);
            const GraphNode& n2 = graph.getNode(currentPath[i+1]);
            
            sf::Vector2f p1 = gpsToScreen(n1.lat, n1.lon);
            sf::Vector2f p2 = gpsToScreen(n2.lat, n2.lon);
            
            sf::Vector2f diff = p2 - p1;
            float len = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            
            if (len > 0) {
                sf::RectangleShape line(sf::Vector2f(len, 4));
                line.setOrigin(0, 2);
                line.setPosition(p1);
                line.setRotation(std::atan2(diff.y, diff.x) * 180.0f / 3.14159265f);
                line.setFillColor(sf::Color::Blue);
                window.draw(line);
            }
        } catch (...) {}
    }
}

void GUI::drawMarkers() {
    auto drawCircle = [&](long long id, sf::Color col, const std::string& labelText) {
        if (id == -1) return;
        try {
            const GraphNode& node = graph.getNode(id);
            sf::Vector2f pos = gpsToScreen(node.lat, node.lon);
            
            sf::CircleShape circle(8);
            circle.setOrigin(8, 8);
            circle.setPosition(pos);
            circle.setFillColor(col);
            window.draw(circle);
            
            sf::Text text(labelText, font, 12);
            text.setFillColor(sf::Color::White);
            sf::FloatRect bounds = text.getLocalBounds();
            text.setOrigin(bounds.left + bounds.width / 2.0f, bounds.top + bounds.height / 2.0f);
            text.setPosition(pos);
            window.draw(text);
        } catch (...) {}
    };

    drawCircle(startNodeId, sf::Color::Green, "S");
    drawCircle(endNodeId, sf::Color::Red, "E");
}

void GUI::drawSidebar() {
    sf::RectangleShape sidebar(sf::Vector2f(330, 800));
    sidebar.setPosition(950, 0);
    sidebar.setFillColor(sf::Color(26, 26, 46));
    window.draw(sidebar);

    sf::Text title("DroneNav", font, 36);
    title.setFillColor(sf::Color::White);
    title.setPosition(970, 20);
    window.draw(title);

    sf::Text subtitle("Rawalpindi / Islamabad", font, 18);
    subtitle.setFillColor(sf::Color(150, 150, 150));
    subtitle.setPosition(970, 65);
    window.draw(subtitle);

    struct Btn { sf::FloatRect rect; std::string text; };
    Btn btns[] = {
        {{975, 120, 280, 45}, "Run A*"},
        {{975, 180, 280, 45}, "Run Dijkstra"},
        {{975, 240, 280, 45}, "Clear Path"},
        {{975, 300, 280, 45}, "Set Start"},
        {{975, 360, 280, 45}, "Set End"}
    };

    sf::Vector2i mousePos = sf::Mouse::getPosition(window);

    for (const auto& btn : btns) {
        sf::RectangleShape rect(sf::Vector2f(btn.rect.width, btn.rect.height));
        rect.setPosition(btn.rect.left, btn.rect.top);
        if (rect.getGlobalBounds().contains(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y))) {
            rect.setFillColor(sf::Color(233, 69, 96));
        } else {
            rect.setFillColor(sf::Color(15, 52, 96));
        }
        window.draw(rect);
        
        sf::Text text(btn.text, font, 20);
        text.setFillColor(sf::Color::White);
        sf::FloatRect bounds = text.getLocalBounds();
        text.setOrigin(bounds.left + bounds.width / 2.0f, bounds.top + bounds.height / 2.0f);
        text.setPosition(btn.rect.left + btn.rect.width / 2.0f,
                         btn.rect.top + btn.rect.height / 2.0f);
        window.draw(text);
    }

    std::string modeStr = "Mode: View";
    if (currentMode == Mode::SET_START) modeStr = "Mode: Set Start";
    else if (currentMode == Mode::SET_END) modeStr = "Mode: Set End";

    sf::Text modeText(modeStr, font, 20);
    modeText.setFillColor(sf::Color::White);
    modeText.setPosition(970, 430);
    window.draw(modeText);

    float statsY = 480;
    auto drawStat = [&](const std::string& label, const std::string& val, sf::Color valColor = sf::Color::White) {
        sf::Text lbl(label + ": ", font, 16);
        lbl.setFillColor(sf::Color(200, 200, 200));
        lbl.setPosition(970, statsY);
        window.draw(lbl);
        
        sf::FloatRect lblBounds = lbl.getLocalBounds();

        sf::Text value(val, font, 16);
        value.setFillColor(valColor);
        value.setPosition(970 + lblBounds.left + lblBounds.width, statsY);
        window.draw(value);
        statsY += 30;
    };

    drawStat("Algorithm", lastAlgorithm);
    if (pathFoundStatus == "No") {
        drawStat("Path Found", "No path found", sf::Color::Red);
    } else {
        drawStat("Path Found", pathFoundStatus);
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "%.2f m", lastDistance);
    drawStat("Distance", buf);

    snprintf(buf, sizeof(buf), "%.2f", lastCost);
    drawStat("Cost", buf);

    drawStat("Explored", std::to_string(lastExplored));

    snprintf(buf, sizeof(buf), "%.2f ms", lastTimeMs);
    drawStat("Time", buf);
}