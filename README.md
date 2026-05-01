# DroneNavReal

## What This Project Does

DroneNavReal is a C++ drone path planning simulation for the Rawalpindi / Islamabad region of Pakistan. It parses real OpenStreetMap road network data, builds a weighted navigation graph that accounts for road type, elevation changes, and restricted no-fly zones, then lets the user visually compare **A\*** and **Dijkstra** shortest-path algorithms through an interactive SFML graphical interface.

Key features:
- **Real map data** — parses `.osm` XML exports covering the bbox `33.55–33.80°N, 72.95–73.20°E`
- **Elevation-aware routing** — simulated terrain model for Margalla Hills (~1100 m) down to city center (~500 m)
- **No-fly zone enforcement** — three hardcoded restricted areas (Benazir Bhutto International Airport, Rawalpindi Cantonment, Islamabad Secure Zone) that the pathfinder avoids
- **Side-by-side algorithm comparison** — run A\* or Dijkstra on the same graph and compare nodes explored, cost, distance, and execution time
- **Interactive GUI** — click to set start/end points, run algorithms, and see explored nodes and the final path rendered on the map

---

## How to Get the Map Data

### rawalpindi.osm — Overpass Query

```
[out:xml][timeout:90][bbox:33.55,72.95,33.80,73.20];
(way["highway"];);out body;>;out skel qt;
```

**Steps:**
1. Go to [https://overpass-turbo.eu](https://overpass-turbo.eu)
2. Paste the query above into the editor and click **Run**
3. Click **Export → Download as raw OSM data**
4. Save the file as `data/rawalpindi.osm` in the project root

### map.png

1. Go to [https://www.openstreetmap.org](https://www.openstreetmap.org)
2. Navigate and zoom to the Rawalpindi / Islamabad area (approx bbox `33.55–33.80°N, 72.95–73.20°E`)
3. Take a screenshot of the map view
4. Save it as `data/map.png` in the project root

> **Note:** The map image is optional. If `data/map.png` is not found, the application will display a dark green grid as a fallback background.

---

## How to Build on Windows

**Prerequisites:**
- CMake 3.16 or later
- A C++17 compiler (MSVC / MinGW)
- SFML 2.5 or later (installed or via vcpkg)

**Build commands:**

```bash
cmake -B build -DSFML_DIR="C:/SFML/lib/cmake/SFML"
cmake --build build --config Release
.\build\Release\DroneNavReal.exe
```

> If you use **vcpkg**, pass the toolchain file instead:
> ```bash
> cmake -B build -DCMAKE_TOOLCHAIN_FILE="[vcpkg-root]/scripts/buildsystems/vcpkg.cmake"
> cmake --build build --config Release
> ```

> If SFML is installed at a non-default location, change the `SFML_DIR` path in `CMakeLists.txt` or pass it via `-DSFML_DIR=`.

---

## How to Use the Application

1. **Launch** — Run `DroneNavReal.exe`. The console prints a startup banner, parses the OSM file, builds the graph, and opens the GUI window.
2. **Set Start** — Click the `Set Start` button in the sidebar, then click any point on the map. The nearest graph node is selected and marked with a green "S" circle.
3. **Set End** — Click the `Set End` button, then click the map to place a red "E" marker.
4. **Run A\*** — Click `Run A*` to find the shortest path using the A\* algorithm. Yellow dots show explored nodes; a blue polyline shows the final path.
5. **Run Dijkstra** — Click `Run Dijkstra` to run Dijkstra's algorithm on the same endpoints and compare the results.
6. **View Stats** — The sidebar displays: algorithm name, path found status, distance (meters), cost, nodes explored, and execution time (ms).
7. **Clear Path** — Click `Clear Path` to reset the visualization and stats.
8. **No-fly zones** — Semi-transparent red circles on the map indicate restricted airspace. The pathfinder automatically routes around them.

---

## Algorithm Complexity Analysis

### A\* Search

| Metric | Complexity |
|---|---|
| **Time** | O((V+E) log V) |
| **Space** | O(V) |
| **Big-Theta** | Θ((V+E) log V) |
| **Big-Omega** | Ω(V) best case (goal is immediate neighbor) |
| **Recurrence** | T(n) = T(n-1) + O(1) → O(n) path reconstruction |

A\* uses the **Haversine distance** as an admissible heuristic, guaranteeing optimal paths while exploring fewer nodes than Dijkstra by directing the search toward the goal.

### Dijkstra

| Metric | Complexity |
|---|---|
| **Time** | O((V+E) log V) |
| **Space** | O(V) |
| **Big-Theta** | Θ((V+E) log V) |
| **Big-Omega** | Ω(V) best case |
| **Recurrence** | T(n) = T(n-1) + O(1) → O(n) path reconstruction |

Dijkstra expands uniformly in all directions with no heuristic guidance. It guarantees optimal paths and is better suited when multiple destinations need to be evaluated.

### A\* vs Dijkstra Comparison Table

| Factor | A\* | Dijkstra |
|---|---|---|
| Heuristic | Haversine (great-circle distance) | None |
| Nodes Explored | Fewer (goal-directed) | More (uniform expansion) |
| Optimality | Guaranteed (admissible heuristic) | Guaranteed |
| Best For | Single target pathfinding | All destinations / uniform search |
| Time Complexity | O((V+E) log V) | O((V+E) log V) |
| Space Complexity | O(V) | O(V) |

---

## Data Structures Used

| Data Structure | Where Used | Purpose |
|---|---|---|
| **Min-Heap** (priority queue) | A\*, Dijkstra | Efficiently extract the lowest-cost node for expansion |
| **Adjacency List** | Graph | Store edges per node — O(1) neighbor lookup |
| **Hash Map** (`unordered_map`) | Graph, A\*, Dijkstra | O(1) average-case node lookup, distance tracking, parent tracking |
| **Unordered Set** | A\*, Dijkstra | O(1) average-case visited-node membership checks |