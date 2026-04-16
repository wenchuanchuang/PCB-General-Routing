// Board configuration
#include "initial/config.h"
#include "initial/common.h"


// ---------------------- System Configuration ----------------------

// Number of threads used for computation
int threads = 1;

// ------------------ Grid / UI Configuration ------------------

// If these values are changed, make sure GRID_W_, GRID_H_, and GRID_Z_
// in config.h are updated accordingly.
std::size_t GRID_W = 200; 
std::size_t GRID_H = 200;
std::size_t GRID_Z = 2; // number of layers, must be >= 1

// ---------------------- Board Configuration ----------------------

double VIA_COST = 0.2; 
double EXTRA_LENGTH = 80.0;

int g_grid_w = GRID_W;
int g_grid_h = GRID_H;
int g_grid_z = GRID_Z;

// ---------------------- File Configuration ----------------------

// The YAML files are converted from KiCad files.
std::string components_yaml = R"(data/yaml/components.yaml)";
std::string boardcomponents_yaml = R"(data/yaml/boardComponents.yaml)";
std::string boardNets_yaml = R"(data/yaml/boardNets.yaml)";
// Binary file used to store candidate paths
std::string candidate_paths_bin = R"(data/candidate_paths/candidate.bin)";


// ---------------------- Component Configuration ----------------------

// A subset of components from the Blixten gateway
// Add or remove components here.
std::unordered_set<std::string> references1 = { "U1","U7" ,"U8"
/*,"U6",
"C1","C2","C4","C5","C6","C7","C8","C11","C13","C16","C19","C20","C44","C49","C50","C52",
"C53","C54","D1","R6","C55","C56","C57","C58","C59","C60","C61","C62","C63","C64","C65",
"C66","C67","C68","C69","C70","C71",
"R21","R22","R2","R18","R19","R20",
"L2","L8","L9","L12",
"C42","C73","C74","C75","C76","C77","C78","C79","C80","C26",
"X2","R36","R8","D3","Q1"*/
};

std::unordered_set<std::string> references = references1;

// Component placement information:
// { reference, placement position of the component origin, rotation }
// Rotation is currently limited to multiples of 90 degrees.
// 0: no rotation
// 1: 90° counterclockwise
// 2: 180°
// 3: 270° counterclockwise (or 90° clockwise)
RefToPos ref_pos = {
	{"U1", {{38, 97}, 0}}
	,{"U7", {{60, 128}, 1}}
	,{"U8", {{154, 78}, 3}} // coordinates currently need to be assigned manually
};

// Pin location map on the board, size = W * H * Z
std::vector<std::vector<std::vector<uint8_t>>> pin_map(
    GRID_W, std::vector<std::vector<uint8_t>>(GRID_H, std::vector<uint8_t>(GRID_Z, 0)
    )
);

// ---------------------- Net Configuration ----------------------

// Routing start and end points for each connection
std::vector<std::pair<Coord, Coord>> path_endpoints = {};

// electrical_net[i] stores the electrical net ID corresponding to path_endpoints[i]
std::vector<int> electrical_net = {};

//obstacles map:
uint8_t obstacles_map[GRID_W_][GRID_H_][GRID_Z_] = {};
void InitObstacles()
{
	// Initialize all grid cells as non-obstacles
	for (int x = 0; x < GRID_W; ++x)
		for (int y = 0; y < GRID_H; ++y)
			for (int z = 0; z < GRID_Z; ++z)
				obstacles_map[x][y][z] = 0;

}

// Per-path obstacle maps
// Key: index of path_endpoints
// Value: obstacle map for that path pair
std::map< int,std::array<std::array<std::array<uint8_t, GRID_Z_>, GRID_H_>, GRID_W_>> obs_maps = {};

