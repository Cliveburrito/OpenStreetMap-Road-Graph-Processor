# OpenStreetMap-Road-Graph-Processor
A Java application that loads a road network from an OpenStreetMap (.osm) XML file, builds a directed weighted graph, and performs classical graph algorithms such as Dijkstra, BFS, DFS, A*(A star) search and graph compression. It also provides a command-line interface for interacting with the map and even opens shortest paths directly in Google Maps.

## Key Features
OSM Integration: Parse .osm XML files into a weighted graph of Vertices and Edges.

Dual-Pathfinding Engine:

Dijkstra's Algorithm: Guarantees the absolute shortest path by exploring all possibilities.

A (A-Star) Search*: Uses a Haversine heuristic (optimized with a highway factor) to focus the search and reduce computation time.

Graph Compression: Reduces the memory footprint by collapsing intermediate "one-way" and "two-way" nodes without losing connectivity or distance accuracy.

Geometric Precision: Calculations account for the Earth's curvature using the Haversine formula.

Visual Integration: Automatically generates and opens a Google Maps URL to visualize the resulting path.

Interactive CLI: A robust command-line interface for real-time graph manipulation
