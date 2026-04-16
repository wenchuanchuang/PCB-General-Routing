#ifndef CONFIG_H
#define CONFIG_H

#include <cstddef>
#include <limits>
#include <array>
#include <vector>
#include <utility>
#include <string>
#include <unordered_set>
#include <map>

#include "common.h"

// ------------------ System Configuration ------------------

// Number of threads used for computation
extern int threads;

// ------------------ Grid / UI Configuration ------------------

// Grid dimensions
extern std::size_t GRID_W;
extern std::size_t GRID_H;
extern std::size_t GRID_Z ;
// Compile-time grid size (must be consistent with config.cpp)
inline constexpr std::size_t GRID_W_ = 200;
inline constexpr std::size_t GRID_H_ = 200;
inline constexpr std::size_t GRID_Z_ = 2; // number of layers


// ------------------ Board Configuration ------------------

extern double      VIA_COST;
extern double      EXTRA_LENGTH;
extern uint8_t obstacles_map[GRID_W_][GRID_H_][GRID_Z_]; // Obstacle map: 0 = free, 1 = blocked
extern std::vector<std::vector<std::vector<uint8_t>>> pin_map; // Pin location map on the board
using DistanceGrid = std::array<std::array<std::array<double, GRID_Z_>, GRID_H_>, GRID_W_>;

// Obstacle maps for each pin pair
// Key: pin pair index
// Value: obstacle map specific to that pair
extern std::map< int, std::array<std::array<std::array<uint8_t, GRID_Z_>, GRID_H_>, GRID_W_>> obs_maps; 


// ------------------ File Configuration ------------------

extern std::string components_yaml;
extern std::string boardcomponents_yaml;
extern std::string boardNets_yaml;
extern std::string candidate_paths_bin;


// ------------------ Component Configuration ------------------

extern std::unordered_set<std::string> references;
extern RefToPos ref_pos;

// ------------------ Net Configuration ------------------

extern std::vector<std::pair<Coord, Coord>> path_endpoints;
extern std::vector<int> electrical_net;


// ------------------ Net Functions ------------------

void InitObstacles();

// ------------------ Numeric Constants ------------------

namespace num_config {
	inline constexpr double		 INF = std::numeric_limits<double>::infinity();
	inline const double			 SQRT2 = 1.4142135623730950488;
}


extern int current_layer;

extern int g_grid_w;
extern int g_grid_h;
extern int g_grid_z;


#endif
