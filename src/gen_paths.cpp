/* Utility definitions (not critical) */

#include <unordered_set> 
#include <iostream>
#include <cmath>

#include "gen_paths.h"


std::vector<Coord> get_neighbors(int x, int y, int z) {
	std::vector<Coord> neighbors;

	if (x > 0)				neighbors.push_back({ x - 1, y, z });
	if (x < GRID_W - 1)		neighbors.push_back({ x + 1, y, z });
	if (y > 0)				neighbors.push_back({ x, y - 1, z });
	if (y < GRID_H - 1)		neighbors.push_back({ x, y + 1, z });
	if (z > 0)				neighbors.push_back({ x, y, z - 1 });
	if (z < GRID_Z - 1)		neighbors.push_back({ x, y, z + 1 });

	if (x > 0 && y > 0)                       neighbors.push_back({ x - 1, y - 1, z });
	if (x > 0 && y < GRID_H - 1)              neighbors.push_back({ x - 1, y + 1, z });
	if (x < GRID_W - 1 && y > 0)              neighbors.push_back({ x + 1, y - 1, z });
	if (x < GRID_W - 1 && y < GRID_H - 1)     neighbors.push_back({ x + 1, y + 1, z });

	return neighbors;
}

// Euclidean distance between two points
double SegLenEuclid(const Coord& a, const Coord& b) {
	int dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;

	double cost = sqrt(double(dx * dx + dy * dy + dz * dz));

	if (b.z != a.z) cost += VIA_COST; // if via, cost + 0.2

	return cost;
}


PathDataset all_results;