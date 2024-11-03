## Optimized-Drone-Routing
This is my project on Optimized Drone Routing using Advanced Data Structures (Quad Trees , Segment Trees , Min heaps ) and Algorithms (astar) for finding the best path for the drone

## Drone Route Optimization

This project implements a drone route optimization algorithm using A* pathfinding with obstacle handling. The algorithm uses a QuadTree for spatial indexing of obstacles (usually static obstacles) and a SegmentTree for obstacles (usually dynamic obstacles) tracking across ranges.

## Classes

### Point
Represents a point in 2D space.
- `__init__(self, x, y)`: Initializes a point with coordinates (x, y).

### Rectangle
Represents a rectangle for QuadTree boundaries.
- `__init__(self, x, y, w, h)`: Initializes a rectangle with top-left corner (x, y) and width `w` and height `h`.
- `contains(self, point)`: Checks if the rectangle contains a given point.
- `intersects(self, range_rect)`: Checks if the rectangle intersects with another rectangle.

### QuadTree
A spatial index structure for efficient querying of points within a rectangular region.
- `__init__(self, boundary, capacity)`: Initializes a QuadTree with a given boundary and capacity.
- `subdivide(self)`: Subdivides the QuadTree into four quadrants.
- `insert(self, point)`: Inserts a point into the QuadTree.
- `query(self, range_rect, found)`: Queries the QuadTree for points within a given rectangular region.

### SegmentTree
A data structure for efficient range queries and updates.
- `__init__(self, size)`: Initializes a SegmentTree with a given size.
- `update(self, index, value)`: Updates the value at a given index.
- `query(self, left, right)`: Queries the minimum value in a given range [left, right).

### Node
Represents a node in the A* pathfinding algorithm.
- `__init__(self, position, g=0, h=0, f=0)`: Initializes a node with a position, cost `g`, heuristic `h`, and total cost `f`.
- `__lt__(self, other)`: Compares nodes based on their total cost `f`.

## Functions

### reconstruct_path(came_from, start, goal)
Reconstructs the path from start to goal using the `came_from` dictionary.

### heuristic(a, b)
Calculates the heuristic (Manhattan distance) between two points `a` and `b`.

### astar_with_obstacle_handling(start, goal, quad_tree, segment_tree, graph, heuristic)
Implements the A* pathfinding algorithm with obstacle handling using a QuadTree and SegmentTree.

### handle_dynamic_obstacles(segment_tree, quad_tree, new_obstacle_position)
Handles dynamic obstacles by updating the SegmentTree and inserting the new obstacle into the QuadTree.

