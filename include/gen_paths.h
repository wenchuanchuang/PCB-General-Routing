/* Utility definitions (not critical) */
#ifndef GEN_PATHS_H 
#define GEN_PATHS_H

#include <vector>
#include <algorithm>
#include <variant>
#include <queue>

#include "initial/common.h"
#include "initial/config.h"


// Candidate path results for different (source, target) pairs
extern PathDataset all_results;

// ------------------ Utility Functions ------------------

std::vector<Coord> get_neighbors(int x, int y, int z);

// Euclidean distance between two coordinates
double SegLenEuclid(const Coord& a, const Coord& b);


#endif
