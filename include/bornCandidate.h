#ifndef BORNCANDIDATE_H 
#define BORNCANDIDATE_H 

#include "initial/common.h"
#include "initial/config.h"

// ==========================================================
// Generate a diverse candidate set by selecting paths
// from multiple path-length / segment-count categories.
// ==========================================================

// Generate candidate paths between S and T and store them in a PathGroup.
//
// target_segments:
//   Specifies the target number of segments for candidate generation.
//   A path with N segments has (N - 1) turning points.
//
// max_results:
//   Maximum number of candidate paths to generate for each segment count.
PathGroup bornCandidate(const Coord& S, const Coord& T, std::vector<int> target_segments, int max_results);



#endif