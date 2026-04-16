/*
Temporary utility for converting KiCad data to YAML.
This is not a core part of the routing algorithm.
Currently, multi-pin nets are not supported.
*/

#include <unordered_map>
#include <string>
#include <stdexcept>
#include <iostream>
#include <unordered_set>
#include <filesystem> 
#include <cstdlib>  
#include <cstring>    
#include <algorithm> 
#include <queue> 
#include <cmath>

#include <loadYaml.h>
#include <yaml-cpp/yaml.h>


template <typename T>
struct Vec2T {
    T x{};
    T y{};
};

template <typename CoordT>
struct PinInfoT {
    std::string pad_num;             // Pin number (pad number)
    Vec2T<CoordT> pos;               // Pin position in the footprint (from YAML or rounded)
};
using PinInfo = PinInfoT<double>;    // Pin position uses double (raw coordinates from YAML)
using IntPinInfo = PinInfoT<int>;    // Pin position after rounding to integer grid coordinates


// Component definition
template <typename PinT>
struct ComponentT {
    std::string value;              // For example: "CC2520"
    std::string footprint;          // For example: "Housings_DFN_QFN:QFN-28-1EP_5x5mm_Pitch0.5mm"
    std::vector<PinT> pins;
};
using ComponentDef = ComponentT<PinInfo>;        // Component pin coordinates from YAML
using GridComponentDef = ComponentT<IntPinInfo>; // Component pin coordinates rounded to grid


// Component instance placed on the board (with reference designator)
struct ComponentOnBoard {
    std::string ref;        // For example: "U3"
    std::string value;      
    std::string footprint;  
    std::vector<IntPinInfo> pins;
};

// board net
struct Net {
    std::string net_name;        
    int net_code;
    int pin_count; // Number of pins
    std::vector<std::pair<std::string, std::string>> pins; // (component reference of pin, pad_name)
};



// ------------------------- Load yaml ------------------------- 

std::vector<ComponentDef> LoadComponentsFromYaml(const std::string& yaml_filename)
{
    std::vector<ComponentDef> components_yaml;

    YAML::Node root = YAML::LoadFile(yaml_filename);
    if (!root["components"]) {
        throw std::runtime_error("YAML file missing 'components' key");
    }

    YAML::Node comps = root["components"];
    if (!comps.IsSequence()) {
        throw std::runtime_error("'components' should be a sequence");
    }

    components_yaml.reserve(comps.size());

    for (const auto& cnode : comps) {
        ComponentDef comp;

        if (!cnode["value"] || !cnode["footprint"]) {
            std::cerr << "[WARN] one component missing 'value' or 'footprint'\n";
            continue; 
        }
        comp.value = cnode["value"].as<std::string>();
        comp.footprint = cnode["footprint"].as<std::string>();
        YAML::Node pads = cnode["pads"];
        if (!pads || !pads.IsSequence()) {
            std::cerr << "[WARN] skip component: "<< comp.value << " ("<< comp.footprint << ")\n";
            continue; 
        } 

        for (const auto& pnode : pads) {
            PinInfo pin;
            pin.pad_num = pnode["pin"].as<std::string>(); // pin: 0 may indicate an exceptional or special pin type
            pin.pos.x = pnode["x_mm"].as<double>();
            pin.pos.y = pnode["y_mm"].as<double>();

            comp.pins.push_back(pin);
        }
        components_yaml.push_back(std::move(comp));
    }

    return components_yaml;
}


std::vector<ComponentOnBoard> LoadBoardComponentsFromYaml(
    const std::string& yaml_filename,
    const std::unordered_set<std::string>& ref 
    )
{
    std::vector<ComponentOnBoard> board_components;

    YAML::Node root = YAML::LoadFile(yaml_filename);
    if (!root["components"]) {
        throw std::runtime_error("YAML file missing 'components' key");
    }
    YAML::Node comps = root["components"];
    if (!comps.IsSequence()) {
        throw std::runtime_error("'components' should be a sequence");
    }


    for (const auto& cnode : comps) {
        
        if (!cnode["ref"] || !cnode["value"] || !cnode["footprint"]) {
            std::cerr << "[WARN] one component missing 'ref' or 'value' or 'footprint'\n";
            continue;
        }

        std::string comp_ref = cnode["ref"].as<std::string>();
        if (!ref.count(comp_ref)) continue;

        ComponentOnBoard comp;
        comp.ref = std::move(comp_ref);
        comp.value = cnode["value"].as<std::string>();
        comp.footprint = cnode["footprint"].as<std::string>();

        board_components.push_back(std::move(comp));
    }

    return board_components;
}


std::vector<Net> LoadNetsFromYaml(const std::string& yaml_filename) {

    std::vector<Net> board_nets;

    YAML::Node root = YAML::LoadFile(yaml_filename);
    if (!root["nets"]) {
        throw std::runtime_error("YAML file missing 'nets' key");
    }
    YAML::Node nets = root["nets"];
    if (!nets.IsSequence()) {
        throw std::runtime_error("'nets' should be a sequence");
    }

    board_nets.reserve(nets.size());

    for (const auto& cnode : nets) {

        if (!cnode["name"] || !cnode["code"] ) {
            std::cerr << "[WARN] one net missing 'name' or 'code'\n";
            continue;
        }

        std::string net_name = cnode["name"].as<std::string>();
        if (net_name.empty()) continue;

        Net net;
        net.net_name = std::move(net_name);
        net.net_code = cnode["code"].as<int>();

        YAML::Node pins = cnode["pins"];
        if (!pins || !pins.IsSequence()) {
            std::cerr << "[WARN] skip pins: " << net.net_name << ", net_code : (" << net.net_code << ")\n";
            continue;
        }
        
        for (const auto& pnode : pins) {
            std::pair<std::string, std::string> pin_info;
            pin_info.first = pnode["ref"].as<std::string>(); 
            pin_info.second = pnode["pin"].as<std::string>();
            net.pins.push_back(pin_info);
        }

        net.pin_count = static_cast<int>(net.pins.size());
        if (cnode["pin_count"]) {
            int pin_cnt_yaml = cnode["pin_count"].as<int>();
            if (pin_cnt_yaml != net.pin_count) {
                std::cerr << "[WARN] net " << net.net_name << " pin_count mismatch: yaml=" << pin_cnt_yaml<< ", actual=" << net.pin_count << "\n";
            }
        }

        board_nets.push_back(std::move(net));
    }

    return board_nets;
}


// ------------------------- Map components onto the integer grid -------------------------

// Encode (x, y) into a 64-bit key for occupancy checks in an unordered_set
static inline long long EncodeGridKey(int x, int y)
{
    return (static_cast<long long>(x) << 32) ^
        (static_cast<std::uint32_t>(y));
}

// Map floating-point coordinates of a ComponentDef onto the integer grid
GridComponentDef SnapToGrid(const ComponentDef& src, double min_grid_dist = 2.0) // // default = 2
{
    GridComponentDef out;
    out.value = src.value;
    out.footprint = src.footprint;

    const auto& pins = src.pins;
    const std::size_t N = pins.size();
    if (N == 0) {
        return out;     
    }

    double sumx = 0.0, sumy = 0.0;
    for (const auto& p : pins) {
        sumx += p.pos.x;
        sumy += p.pos.y;
    }
    const double cx = sumx / static_cast<double>(N);
    const double cy = sumy / static_cast<double>(N);

    double d_min = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = i + 1; j < N; ++j) {
            double dx = pins[i].pos.x - pins[j].pos.x;
            double dy = pins[i].pos.y - pins[j].pos.y;
            double dist2 = dx * dx + dy * dy;
            if (dist2 <= 0.0) continue;
            double dist = std::sqrt(dist2);
            if (dist < d_min) {
                d_min = dist;
            }
        }
    }

    if (d_min == std::numeric_limits<double>::max()) {
        d_min = 1.0; 
    }

    double scale = min_grid_dist / d_min;

    if (scale < 1.0) scale = 1.0;

    std::vector<IntPinInfo> snapped_pins;
    snapped_pins.reserve(N);

    std::unordered_set<long long> occupied;
    occupied.reserve(N * 2);

    for (const auto& srcPin : pins) {
        double sx = cx + scale * (srcPin.pos.x - cx);
        double sy = cy + scale * (srcPin.pos.y - cy);

        int gx = static_cast<int>(std::lround(sx));
        int gy = static_cast<int>(std::lround(sy));

        int final_x = gx;
        int final_y = gy;

        long long key = EncodeGridKey(final_x, final_y);
        if (occupied.find(key) != occupied.end()) {

            bool placed = false;
            int max_radius = 3; 

            for (int radius = 1; radius <= max_radius && !placed; ++radius) {
                for (int dx = -radius; dx <= radius && !placed; ++dx) {
                    for (int dy = -radius; dy <= radius && !placed; ++dy) {
                        if (std::abs(dx) + std::abs(dy) != radius) continue;
                        int nx = gx + dx;
                        int ny = gy + dy;
                        long long k2 = EncodeGridKey(nx, ny);
                        if (occupied.find(k2) == occupied.end()) { 
                            final_x = nx;
                            final_y = ny;
                            key = k2;
                            placed = true;
                        }
                    }
                }
            }

            if (!placed) {
                std::cerr << "[WARN] no valid grid vertex found for pin position (" << gx << "," << gy << ")\n";
            }
        }

        occupied.insert(key);

        IntPinInfo ip;
        ip.pad_num = srcPin.pad_num;
        ip.pos.x = final_x;
        ip.pos.y = -final_y;

        snapped_pins.push_back(ip);
    }

    std::size_t anchor_index = 0;

    int ax = snapped_pins[anchor_index].pos.x;
    int ay = snapped_pins[anchor_index].pos.y;

    for (auto& p : snapped_pins) {
        p.pos.x -= ax;
        p.pos.y -= ay;
    }

    out.pins = std::move(snapped_pins);
    return out;
}



// ------------------------- construct on-board component ------------------------- 

std::vector<ComponentOnBoard>* FillIntPins(
    const std::vector<GridComponentDef>& grid_components,
    std::vector<ComponentOnBoard>& board_components
)
{
    for (ComponentOnBoard& comp : board_components) {

        const GridComponentDef* found = nullptr;
        for (const auto& gc : grid_components) {
            if (gc.value == comp.value && gc.footprint == comp.footprint)
            {
                found = &gc;
                break;  
            }
        }

        if (!found) {
            std::cerr << "[WARN] No grid component found for " << comp.value << " (" << comp.footprint << ")\n";
            continue;
        }

        comp.pins = found->pins;
    }

    return &board_components;
}


// ------------------------- Build index tables -------------------------

// key = (ref, pad_num)
using RefPinKey = std::pair<std::string, std::string>;
struct RefPinKeyHash {
    std::size_t operator()(RefPinKey const& k) const noexcept {
        std::hash<std::string> h;
        return h(k.first) ^ ( h(k.second) << 1);
    }
};
// Mapping: (ref, pad_num) -> on-board coordinates
using RefToBoardPos = std::unordered_map<RefPinKey, Vec2T<int>, RefPinKeyHash>;

// key { comp.ref, pin.pad_num } -->  pin.pos
RefToBoardPos MapRefToBoardPos(const std::vector<ComponentOnBoard>& board_comps)
{
    RefToBoardPos umap;
    std::size_t totalPins = 0;
    for (const auto& comp : board_comps) {
        totalPins += comp.pins.size();
    }
    umap.reserve(totalPins); 

    for (const auto& comp : board_comps) {
        for (const auto& pin : comp.pins) {
            RefPinKey key{ comp.ref, pin.pad_num };
            umap.emplace(std::move(key), pin.pos);
        }
    }
    return umap;
}


// ------------------------- Place components on the board -------------------------

void RotateIntComponent(ComponentOnBoard& comp, int quarter_turns)
{

    quarter_turns = ((quarter_turns % 4) + 4) % 4;

    for (auto& pin : comp.pins) {
        int x = pin.pos.x;
        int y = pin.pos.y;
        int nx = x;
        int ny = y;


        switch (quarter_turns) {
        case 0:
            nx = x;  ny = y;
            break;
        case 1: 
            nx = y;  ny = -x;
            break;
        case 2:
            nx = -x; ny = -y;
            break;
        case 3: 
            nx = -y; ny = x;
            break;
        }


        pin.pos.x = nx;
        pin.pos.y = ny;
    }
}



void RecordPinOnBoard(const std::vector<ComponentOnBoard>& board_components,
    std::vector<std::vector<std::vector<uint8_t>>>& pin_map,
    const RefToPos& ref_pos) {

    for (int x = 0; x < GRID_W; ++x)
        for (int y = 0; y < GRID_H; ++y)
            for (int z = 0; z < GRID_Z; ++z)
                pin_map[x][y][z] = 0;

    for (auto& comp_board : board_components) {

        auto it = ref_pos.find(comp_board.ref);
        if (it != ref_pos.end()) {

            for (auto& pin : comp_board.pins) {
                int x = pin.pos.x; int y = pin.pos.y; 
                int z = 0;
                if ( !( x < 0 || x >= GRID_W || y <0 || y >= GRID_H || z < 0 || z >= GRID_H )) {
                    pin_map[x][y][z] = 1;
                }
            }
        }
        else {
            continue;
        }
    }

}




// ------------------------- Obstacle configuration -------------------------

// Helper function: compute the bounding box of all pins of a component
// (currently approximated using a rectangular bounding box)
struct BBox {
    int min_x, max_x, min_y, max_y;
};
BBox GetCompBBox(const ComponentOnBoard& comp) {

    int min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (const auto& p : comp.pins) {
        if (p.pos.x < min_x) min_x = p.pos.x;
        if (p.pos.x > max_x) max_x = p.pos.x;
        if (p.pos.y < min_y) min_y = p.pos.y;
        if (p.pos.y > max_y) max_y = p.pos.y;
    }


    min_x = std::max(0, min_x);
    max_x = std::min((int)GRID_W_ - 1, max_x);
    min_y = std::max(0, min_y);
    max_y = std::min((int)GRID_H_ - 1, max_y);


    if (min_x > max_x || min_y > max_y) return { 0, 0, 0, 0 };
    
    return { min_x, max_x, min_y, max_y };
}


bool isPointInTri(int px, int py, int x1, int y1, int x2, int y2, int x3, int y3) {

    auto cross = [](int px, int py, int ax, int ay, int bx, int by) {
        return (long long)(bx - ax) * (py - ay) - (long long)(by - ay) * (px - ax);
        };

    long long d1 = cross(px, py, x1, y1, x2, y2);
    long long d2 = cross(px, py, x2, y2, x3, y3);
    long long d3 = cross(px, py, x3, y3, x1, y1);

    bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}


std::vector<Coord> get_neigh(int x, int y, int z) {
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

double SegLenEuclid2(const Coord& a, const Coord& b) {
    int dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;

    double cost = sqrt(double(dx * dx + dy * dy + dz * dz));

    if (b.z != a.z) cost += VIA_COST; // if via, cost + 0.2

    return cost;
}

using Grid3D = std::array<std::array<std::array<uint8_t, GRID_Z_>, GRID_H_>, GRID_W_>;

//return shortest path
Path Dijkstra2(
    Coord start_point, Coord end_point,
    const Grid3D& obs_mask	/*obstacle map*/ )
{
    DistanceGrid dist{}; 
    Coord prev[GRID_W_][GRID_H_][GRID_Z_]; 
    bool visited[GRID_W_][GRID_H_][GRID_Z_] = { false }; 

    //初始化
    for (int i = 0; i < GRID_W; ++i)
        for (int j = 0; j < GRID_H; ++j)
            for (int k = 0; k < GRID_Z; ++k) {
                dist[i][j][k] = num_config::INF; 
                prev[i][j][k] = INVALID_COORD;
            }


    dist[start_point.x][start_point.y][start_point.z] = 0;

 
    std::priority_queue<
        std::tuple<double, int, int, int>,     
        std::vector<std::tuple<double, int, int, int>>, 
        std::greater<>                 
    > pq;
    pq.push({ 0.0f, start_point.x, start_point.y, start_point.z }); 


    while (!pq.empty()) {
        auto [d, x, y, z] = pq.top(); pq.pop();
        if (visited[x][y][z]) continue;
        visited[x][y][z] = true;

        for (auto [nx, ny, nz] : get_neigh(x, y, z)) {

            Coord next = { nx, ny,nz };

            if (next != end_point) {
                if (obs_mask[nx][ny][nz]) continue; 			
            }

            if (prev[x][y][z] != INVALID_COORD) {
                int prev_x = prev[x][y][z].x; int prev_y = prev[x][y][z].y; int prev_z = prev[x][y][z].z;


                if (prev_z == z && z == nz) {

                    float dot_xy = (prev_x - x) * (nx - x) + (prev_y - y) * (ny - y);
                    if (dot_xy >= 0) continue;
                }
            }


            Coord now = { x, y,z };
            double edge_penalty = SegLenEuclid2(now, next);

            double new_dist = dist[x][y][z] + edge_penalty; 
            if (new_dist < dist[nx][ny][nz]) {
                dist[nx][ny][nz] = new_dist;
                prev[nx][ny][nz] = { x, y,z };
                pq.push({ new_dist, nx, ny ,nz });
            }
        }
    }

    Path path;

    if (dist[end_point.x][end_point.y][end_point.z] == num_config::INF) {
        return path;  
    }
    for (Coord at = end_point; at != start_point; at = prev[at.x][at.y][at.z]) {
        path.push_back(at);
    }
    path.push_back(start_point);
    std::reverse(path.begin(), path.end());

    return  path;
}

std::map<int, Grid3D> SetObs2(
    const std::vector<std::pair<Coord, Coord>>& path_endpoints,
    const std::vector<ComponentOnBoard>& board_components,
    const std::vector<int>& electrical_net,
    const RefToBoardPos& r2bp
) {

    std::map< int,  Grid3D> out;

    std::unordered_map<std::string, const ComponentOnBoard*> ref_to_comp;
    for (const auto& comp : board_components) {
        ref_to_comp[comp.ref] = &comp;
    }

    std::map<std::pair<int, int>, std::string> pos_to_ref;
    for (const auto& kv : r2bp) {
        pos_to_ref[{kv.second.x, kv.second.y}] = kv.first.first; 
    }
    

    for (int i = 0; i < (int)path_endpoints.size(); ++i) {

        int net_code = electrical_net[i];

        Grid3D obs_mask1;
        for (auto& plane : obs_mask1) { for (auto& row : plane) { row.fill(2); } }

        Coord pts[2] = { path_endpoints[i].first, path_endpoints[i].second };

        for (const Coord& pt : pts) {
            
            std::pair<int, int> pt_key = { pt.x, pt.y };

            if (pos_to_ref.find(pt_key) != pos_to_ref.end()) { 

                std::string ref = pos_to_ref[pt_key];
                const ComponentOnBoard* comp = ref_to_comp[ref]; 
                BBox comp_box = GetCompBBox(*comp);

                Vec2T<int> pin_pos = { pt.x, pt.y };

                bool near_min_x = std::abs(pin_pos.x - comp_box.min_x) < std::abs(pin_pos.x - comp_box.max_x);  
                int a1_x = near_min_x ? comp_box.min_x : comp_box.max_x;
                int b1_x = near_min_x ? comp_box.max_x : comp_box.min_x; 

                bool near_min_y = std::abs(pin_pos.y - comp_box.min_y) < std::abs(pin_pos.y - comp_box.max_y);
                int b1_y = near_min_y ? comp_box.min_y : comp_box.max_y; 
                int a1_y = near_min_y ? comp_box.max_y : comp_box.min_y; 


                auto [min_l, max_l] = std::minmax({ ((comp_box.max_x - comp_box.min_x)/2)/2 , ((comp_box.max_y - comp_box.min_y)/2)/2 }); 
                int p_min_x = pin_pos.x - max_l, p_max_x = pin_pos.x + max_l; int p_min_y = pin_pos.y - max_l, p_max_y = pin_pos.y + max_l;


                for (int x = comp_box.min_x; x <= comp_box.max_x; ++x) {
                    for (int y = comp_box.min_y; y <= comp_box.max_y; ++y) {

                        bool in_open_zone = false;

                        if (x >= p_min_x && x <= p_max_x && y >= p_min_y && y <= p_max_y) in_open_zone = true;

                        auto [min_x, max_x] = std::minmax({ pin_pos.x, a1_x }); 
                        auto [min_y, max_y] = std::minmax({ pin_pos.y, b1_y });
                        if (x >= min_x && x <= max_x && y >= min_y && y <= max_y) in_open_zone = true;

                        

                        if (in_open_zone) {
                            if (x >= 0 && x < GRID_W_ && y >= 0 && y < GRID_H_) {
                                    obs_mask1[x][y][0] = 0; 
                            }
                            
                        }
                    }
                }
            }

        }

        for (const auto& comp : board_components) { 

            BBox comp_box = GetCompBBox(comp); 
            for (int x = comp_box.min_x; x <= comp_box.max_x; x++) { 
                for (int y = comp_box.min_y; y <= comp_box.max_y; y++) { 
                    if (obs_mask1[x][y][0] == 2) {  
                        obs_mask1[x][y][0] = 1; 
                    }
                } 
            } 
        } 

        for (int x = 0; x < GRID_W; ++x)
            for (int y = 0; y < GRID_H; ++y)
                for (int z = 0; z < GRID_Z; ++z) {
                    if (obs_mask1[x][y][z] == 2) {
                        obs_mask1[x][y][z] = 0;
                    }
                }
        
        
        


        Grid3D obs_mask5;
        for (auto& plane : obs_mask5) { for (auto& row : plane) { row.fill(0); } } 
        for (int j = 0; j < (int)path_endpoints.size(); ++j) {

            if (i == j) continue;

            int x1 = path_endpoints[j].first.x; int y1 = path_endpoints[j].first.y; int z1 = path_endpoints[j].first.z;
            int x2 = path_endpoints[j].second.x; int y2 = path_endpoints[j].second.y; int z2 = path_endpoints[j].second.z;
            obs_mask5[x1][y1][z1] = 1;
            obs_mask5[x2][y2][z2] = 1;

        }

        // =========================================================================
        // Sliding Window
        // =========================================================================
        Grid3D obs_mask6;
        for (auto& plane : obs_mask6) { for (auto& row : plane) { row.fill(1); } } 

        int win_size = 6; 

        for (const Coord& pt : pts) { 
            
            std::pair<int, int> pt_key = { pt.x, pt.y };

            if (pos_to_ref.find(pt_key) != pos_to_ref.end()) { 

                    std::string ref = pos_to_ref[pt_key];
                    const ComponentOnBoard* comp = ref_to_comp[ref];
                    BBox comp_box = GetCompBBox(*comp);

                    if (comp_box.max_x - comp_box.min_x + 1 < win_size || 
                        comp_box.max_y - comp_box.min_y + 1 < win_size) {
                        continue;
                    }

                    for (int start_x = comp_box.min_x; start_x <= comp_box.max_x - win_size + 1; ++start_x) {
                        for (int start_y = comp_box.min_y; start_y <= comp_box.max_y - win_size + 1; ++start_y) {
                            
                            bool is_clean = true;

                            for (const auto& pin : comp->pins) {
                                if (pin.pos.x >= start_x && pin.pos.x < start_x + win_size &&
                                    pin.pos.y >= start_y && pin.pos.y < start_y + win_size) {
                                    is_clean = false;
                                    break;
                                }
                            }

                            if (is_clean) {
                                for (int x = start_x; x < start_x + win_size && is_clean; ++x) {
                                    for (int y = start_y; y < start_y + win_size && is_clean; ++y) {
                                        if (x >= 0 && x < GRID_W_ && y >= 0 && y < GRID_H_) {
                                            if (obstacles_map[x][y][0] == 1 ||  obs_mask5[x][y][0] == 1) { 
                                                is_clean = false;
                                            }
                                        }
                                    }
                                }
                            }

                            if (is_clean) {
                                for (int x = start_x; x < start_x + win_size; ++x) {
                                    for (int y = start_y; y < start_y + win_size; ++y) {
                                        if (x >= 0 && x < GRID_W_ && y >= 0 && y < GRID_H_) {
                                            obs_mask6[x][y][0] = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
            }

        }




        for (int x = 0; x < GRID_W; ++x)
            for (int y = 0; y < GRID_H; ++y)
                for (int z = 0; z < GRID_Z; ++z) 
                    if (obs_mask6[x][y][z] == 0 ) obs_mask1[x][y][z] = 0;
            

        Grid3D obs_mask3;
        for (auto& plane : obs_mask3) { for (auto& row : plane) { row.fill(0); } } 

        for (int x = 0; x < GRID_W; ++x)
            for (int y = 0; y < GRID_H; ++y)
                for (int z = 0; z < GRID_Z; ++z) 
                    if (obstacles_map[x][y][z] == 1 || obs_mask1[x][y][z] == 1 ) obs_mask3[x][y][z] = 1;
                

        

        Grid3D obs_mask4;
        for (auto& plane : obs_mask4) { for (auto& row : plane) { row.fill(1); } }

        Path path = Dijkstra2(path_endpoints[i].first, path_endpoints[i].second, obs_mask3);
        if (path.empty()) {
            std::cout << "  [WARN] No shortest path while using obs_mask3\n";

            for (int x = 0; x < GRID_W; ++x)
                for (int y = 0; y < GRID_H; ++y)
                    for (int z = 0; z < GRID_Z; ++z)
                        obs_mask3[x][y][z] = obstacles_map[x][y][z];
            out[i] = obs_mask3;
            continue;
        }
        else {
            int r = (EXTRA_LENGTH/2); 
            for (int j = 0; j + 1 < (int)path.size(); ++j) {

                const Coord& u = path[j];
                
                int r_sq = r * r; 

                int min_x = std::max(0, u.x - r);
                int max_x = std::min((int)GRID_W - 1, u.x + r);
                int min_y = std::max(0, u.y - r);
                int max_y = std::min((int)GRID_H - 1, u.y + r);
                int min_z = std::max(0, u.z - r);
                int max_z = std::min((int)GRID_Z_ - 1, u.z + r);

                for (int x = min_x; x <= max_x; ++x) {
                    int dx = x - u.x;
                    int dx_sq = dx * dx;

                    for (int y = min_y; y <= max_y; ++y) {
                        int dy = y - u.y;
                        int dxy_sq = dx_sq + (dy * dy);

                        if (dxy_sq > r_sq) continue;

                        for (int z = min_z; z <= max_z; ++z) {
                            int dz = z - u.z;

                            if (dxy_sq + (dz * dz) <= r_sq) obs_mask4[x][y][z] = 0;
                        }
                    }
                }
            }



            for (auto& plane : obs_mask3) { for (auto& row : plane) { row.fill(0); } } 

            for (int x = 0; x < GRID_W; ++x) {
                for (int y = 0; y < GRID_H; ++y) {
                    for (int z = 0; z < GRID_Z; ++z) {

                        if (obstacles_map[x][y][z] == 1 || obs_mask1[x][y][z] == 1 || obs_mask4[x][y][z] == 1 || obs_mask5[x][y][z] == 1) obs_mask3[x][y][z] = 1; // obs_mask4太小會擋住obs_mask6開的區域,不過目前先算了
                        
                    }
                }
            }

        }

        out[i] = obs_mask3;

    }
    return out;
}

// ------------------------- Place components on the board (integrated) -------------------------


void PutComponenetOnBoard(
    const std::string& comps_yaml, const std::string& boardcomps_yaml, const std::string& boardNets_yaml,
    const std::unordered_set<std::string>& ref /*references to be placed on the board*/,
    const RefToPos& ref_pos /*component placement information*/, 
    std::vector<std::vector<std::vector<uint8_t>>>& pin_map/*records pin locations on the board*/,
    std::vector<std::pair<Coord, Coord>>& path_endpoints /*routing start and end points*/,
    std::vector<int>& electrical_net
    ) 
{   

    if (!std::filesystem::exists(comps_yaml)) {
        std::cerr << "Error: File not found. The specified path '" << comps_yaml << "' does not exist." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::vector<ComponentDef> components_yaml = LoadComponentsFromYaml(comps_yaml);

    std::vector<ComponentOnBoard> board_components = LoadBoardComponentsFromYaml(boardcomps_yaml, ref);
    std::vector<Net> board_nets = LoadNetsFromYaml(boardNets_yaml);

    std::vector<GridComponentDef> grid_components;
    grid_components.reserve(components_yaml.size());
    for (auto& comp_yaml : components_yaml) {
        grid_components.push_back(SnapToGrid(comp_yaml, 3.0)); 
    }

    FillIntPins(grid_components, board_components);
    // At this stage, board_components already has integer coordinates,
    // but rotation and translation to board positions have not been applied yet

   // Place board_components onto the board 
    for (auto& comp_board : board_components) {

        auto it = ref_pos.find(comp_board.ref);
        if (it != ref_pos.end()) {
            const PositionData& pos_data = it->second;
            RotateIntComponent(comp_board, pos_data.angle);


            IntPinInfo comp_anchor = comp_board.pins.front();
            int dx = pos_data.position.first - comp_anchor.pos.x;
            int dy = pos_data.position.second - comp_anchor.pos.y;

            for (auto& pin : comp_board.pins) {
                pin.pos.x += dx;
                pin.pos.y += dy;
            }
        }
        else {
            std::cerr << "[WARN] board component " << comp_board.ref << " has no position data\n";
        }
    }


    // Build an index table for board_components
    RefToBoardPos r2bp = MapRefToBoardPos(board_components);

    std::vector<std::vector<std::vector<uint8_t>>> pin_map_used( // records used pins
        GRID_W, std::vector<std::vector<uint8_t>>(GRID_H, std::vector<uint8_t>(GRID_Z, 0)
        )
    );

    // According to the nets, push the corresponding coordinates into path_endpoints
    // Currently, only nets with exactly two pins are added to path_endpoints
    for (auto& net : board_nets) {
        if (net.pin_count != 2) continue;
        if (net.pins.empty()) continue;
        
        std::string pin0_ref = net.pins[0].first;
        std::string pin0_pad_name = net.pins[0].second;
        std::string pin1_ref = net.pins[1].first;
        std::string pin1_pad_name = net.pins[1].second;

        // Look up board_components
        RefPinKey key0{ pin0_ref, pin0_pad_name };
        auto it0 = r2bp.find(key0);
        if (it0 == r2bp.end()) {
            // One of the pins in this net cannot be found in board_components
            //std::cerr << "[WARN] one pad of net can't be found\n";
            continue;
        }

        RefPinKey key1{ pin1_ref, pin1_pad_name };
        auto it1 = r2bp.find(key1);
        if (it1 == r2bp.end()) {
            //std::cerr << "[WARN] one pad of net can't be found\n";
            continue;
        }

        Vec2T<int>& outPos0 = it0->second;
        Vec2T<int>& outPos1 = it1->second;

        int x0 = outPos0.x; int y0 = outPos0.y; int z0 = 0;
        int x1 = outPos1.x; int y1 = outPos1.y; int z1 = 0;
        
        if ( (x0 >= 0 && x0 < GRID_W) && (y0 >= 0 && y0 < GRID_H) && (z0 >= 0 && z0 < GRID_Z) &&
             (x1 >= 0 && x1 < GRID_W) && (y1 >= 0 && y1 < GRID_H) && (z1 >= 0 && z1 < GRID_Z)) {

            path_endpoints.push_back({ { x0 , y0 , z0 },
                                       { x1 , y1 , z1 } });
            electrical_net.push_back(net.net_code);

            pin_map_used[x0][y0][z0] = 1;
            pin_map_used[x1][y1][z1] = 1;
        }
        else {
            std::cerr << "[WARN] at least one pad of net is out of the range of board\n";
        }

        

        std::cout << "[OK] a set of pins has load successfully: "
            << "(" << outPos0.x << "," << outPos0.y << "," << 0 << ")"
            << "-->"
            << "(" << outPos1.x << "," << outPos1.y << "," << 0 << ")"
            << "\n";
    }


    // Record all pin locations on the board
    RecordPinOnBoard(board_components, pin_map, ref_pos);

    // Mark unused pins as obstacles
    for (int x = 0; x < GRID_W; ++x) {
        for (int y = 0; y < GRID_H; ++y) {
            for (int z = 0; z < GRID_Z; ++z) {

                if (pin_map[x][y][z] == 1 && pin_map_used[x][y][z] == 0) {
                    obstacles_map[x][y][z] = 1;
                }

            }
        }
    }

    std::map<int, Grid3D> obs_maps_tmp = SetObs2(path_endpoints, board_components, electrical_net, r2bp);  // Set obstacle regions here

    obs_maps = std::move(obs_maps_tmp);
}