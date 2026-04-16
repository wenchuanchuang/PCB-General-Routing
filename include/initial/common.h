// Data structure definitions (not critical)
#ifndef COMMON_H 
#define COMMON_H


#include <tuple>        
#include <vector>
#include <utility>		
#include <unordered_map>
#include <string>


struct Coord { 
	int x, y, z;

	constexpr Coord(int _x = -1, int _y = -1, int _z = -1) : x(_x), y(_y), z(_z) {}

	bool operator==(const Coord& other) const {
		return x == other.x && y == other.y && z == other.z;
	}
	bool operator!=(const Coord& other) const {
		return !(*this == other);
	}

	bool operator<(const Coord& other) const {
		return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
	}
};
using Path = std::vector<Coord>;							
using PathInfo = std::pair<double, Path>;
using PathGroup = std::vector<PathInfo>;					
using PathDataset = std::vector<PathGroup>;

inline constexpr Coord INVALID_COORD(-1, -1, -1);


// ------------------ Component Section -------------------------

// Stores placement information of a component on the board
struct PositionData {
	std::pair<int,int> position; // (x, y) coordinates
	int angle;                   // rotation angle:
	                             // 0: no rotation
	                             // 1: 90° CCW
	                             // 2: 180°
	                             // 3: 270° CCW
};

// Mapping from component reference to its placement data
// Key:   component reference (string, primary identifier)
// Value: PositionData (position + rotation)
using RefToPos = std::unordered_map<std::string, PositionData>;


#endif
