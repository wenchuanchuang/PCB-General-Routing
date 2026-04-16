#include <iostream>
#include <vector>
#include <cmath>
#include <set>
#include <numeric>
#include <algorithm>
#include <atomic>
#include <future>
#include <chrono>
#include <random>

#include <queue> //priority queue

#include "bornCandidate.h"

// -------------------------- Parameters (not absolute, adjust as needed) --------------------------
const double VIA_PENALTY = 50; // Larger values make the routing stage less likely to use vias
double min_discount = 0.2;
int step_limit = 1000;  // Base exploration budget assigned to each thread
unsigned int random_seed = 88;
unsigned int random_seed_shuffle = 777;


// ----------------------- Helper Functions -------------------------------

// Check whether the angle between two consecutive segments is <= 45 degrees
bool IsAngleValid(int prev_dx, int prev_dy, int prev_dz, int curr_dx, int curr_dy, int curr_dz) {
    // If this is the first ray from the start point (no previous segment),
    // then all directions are allowed
    if (prev_dx == 0 && prev_dy == 0 && prev_dz == 0) return true;
    
     // If the transition involves a via (vertical segment),
    // the physical angle is necessarily 90 degrees, so it is always allowed
    if (prev_dz != 0/* previous segment is a via */ || curr_dz != 0/* target_z - current z */) {
        // Safety check: disallow two consecutive via segments
        // (e.g., drilling up and immediately drilling down)
        if (prev_dz != 0 && curr_dz != 0) return false; 
        return true;
    }
    
    // Dot product formula: dot = |a| * |b| * cos(theta)
    double dot = prev_dx * curr_dx + prev_dy * curr_dy;
    double mag_prev = std::sqrt(prev_dx * prev_dx + prev_dy * prev_dy);
    double mag_curr = std::sqrt(curr_dx * curr_dx + curr_dy * curr_dy);
    
    double cos_theta = dot / (mag_prev * mag_curr);
    
    // ==========================================
    // Only allow straight movement (0°) or 45° turns
    // ==========================================
    // cos(0°) = 1.0
    bool is_straight = std::abs(cos_theta - 1.0) < 1e-5;
    
    // cos(45°) = sqrt(2)/2 ≈ 0.707106781
    bool is_45_deg = std::abs(cos_theta - 0.707106781) < 1e-5;
    
    return is_straight || is_45_deg;
}

// Segment intersection test using cross products
// Helper function: check whether point r lies within the bounding box of segment pq
// (used when the three points are collinear)
bool onSegment(Coord p, Coord q, Coord r) {
    if (r.x <= std::max(p.x, q.x) && r.x >= std::min(p.x, q.x) &&
        r.y <= std::max(p.y, q.y) && r.y >= std::min(p.y, q.y))
       return true;
    return false;
}

// Helper function: compute the orientation of points p, q, r using cross products
// Return value:
//   0: collinear
//   1: clockwise
//   2: counterclockwise
int orientation(Coord p, Coord q, Coord r) {
    // Use long long to avoid overflow when coordinates are large
    long long val = 1LL * (q.y - p.y) * (r.x - q.x) - 1LL * (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;   // collinear
    return (val > 0) ? 1 : 2; // clockwise or counterclockwise
}

// Check whether segments p1q1 and p2q2 intersect
bool doSegmentsIntersect(Coord p1, Coord q1, Coord p2, Coord q2) {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // The two segments cross each other
    if (o1 != o2 && o3 != o4) return true;

    // The two segments are collinear and overlap
    if (o1 == 0 && onSegment(p1, q1, p2)) return true;
    if (o2 == 0 && onSegment(p1, q1, q2)) return true;
    if (o3 == 0 && onSegment(p2, q2, p1)) return true;
    if (o4 == 0 && onSegment(p2, q2, q1)) return true;

    return false;
}

// ------------------------------- Generate all valid rays from the start point o in 360 degrees -------------------------------

// collision check for vertical vias
bool isViaClear(int ox, int oy, int z1, int z2, const uint8_t is_obstacle[GRID_W_][GRID_H_][GRID_Z_]) {
    // Check whether xy is out of bounds
    if (ox < 0 || ox >= GRID_W_ || oy < 0 || oy >= GRID_H_) return false;
    
    int step = (z2 > z1) ? 1 : -1;
    // Traverse along the Z axis from z1 to z2 and check whether each layer is clear
    for (int z = z1; z != z2 + step; z += step) {
        if (z < 0 || z >= GRID_Z_) return false;
        if (is_obstacle[ox][oy][z]) return false;
    }
    return true;
}

// If the path from (ox, oy) to (tx, ty) is unobstructed, return true;
// otherwise return false when an obstacle is encountered.
bool isLineOfSightClear(int ox, int oy, int oz, int tx, int ty, const uint8_t is_obstacle[GRID_W_][GRID_H_][GRID_Z_]/* obstacle map specific to each net */) {
    int dx = tx - ox;
    int dy = ty - oy;
    
    int dirX = (dx > 0) ? 1 : ((dx < 0) ? -1 : 0);
    int dirY = (dy > 0) ? 1 : ((dy < 0) ? -1 : 0);

    auto is_obs = [&](int vx, int vy) {
        if (vx < 0 || vx >= GRID_W || vy < 0 || vy >= GRID_H) return true;
        return (bool)is_obstacle[vx][vy][oz];
    };

    // If the target itself is blocked, there is no need to proceed
    if (is_obs(tx, ty)) { return false; }

    if (dirY == 0) {
        int x = ox + dirX;
        while (x != tx) {
            if (is_obs(x, oy)) return false; 
            x += dirX;
        }
        return true;
    }

    if (dirX == 0) {
        int y = oy + dirY;
        while (y != ty) {
            if (is_obs(ox, y)) return false; 
            y += dirY;
        }
        return true;
    }

    // --- Diagonal traversal ---
    // The first grid cell entered by the ray (lower-left corner coordinates)
    int cx = (dirX >= 0) ? ox : ox - 1;
    int cy = (dirY >= 0) ? oy : oy - 1;
    
    double tDeltaX = std::abs(1.0 / dx); // how much total progress is consumed when crossing one grid width in X
    double tDeltaY = std::abs(1.0 / dy); 
    
    double tMaxX = tDeltaX;
    double tMaxY = tDeltaY;

    double slope_yx = (double)dy / dx; 
    double slope_xy = (double)dx / dy;

    while (true) {
        double t = std::min(tMaxX, tMaxY);  // overall progress ratio
        // Since t reaches the first primitive-grid crossing point,
        // if we reach 1.0 without returning false, the target is reached successfully
        if (std::abs(t - 1.0) < 1e-7 || t > 1.0) { // progress completed, target vertex reached
            return true; 
        }

        // 1. The ray hits a grid vertex exactly
        if (std::abs(tMaxX - tMaxY) < 1e-7) {
            cx += dirX;
            cy += dirY;
            tMaxX += tDeltaX;
            tMaxY += tDeltaY;
            
            // Compute the exact vertex being hit
            int vx = (dirX > 0) ? cx : cx + 1;
            int vy = (dirY > 0) ? cy : cy + 1;
            
            // The ray cannot pass through an obstacle vertex
            if (is_obs(vx, vy)) return false;
            
        } 
        // 2. The ray crosses a vertical grid line
        else if (tMaxX < tMaxY) {
            cx += dirX;
            tMaxX += tDeltaX;
            
            int vx = (dirX > 0) ? cx : cx + 1;
            int vy_bottom = cy;      // lower endpoint of the grid line
            int vy_top = cy + 1;     // upper endpoint of the grid line
            
             // Both adjacent vertices are obstacles (treated as a wall)
            if (is_obs(vx, vy_bottom) && is_obs(vx, vy_top)) {
                return false; 
            }

            // Compute the exact y-coordinate at the crossing
            double intersect_y = oy + (vx - ox) * slope_yx;

            // Check whether the ray passes too close to the lower obstacle (< 0.5)
            if (is_obs(vx, vy_bottom) && (intersect_y - vy_bottom) < 0.5 - 1e-6) {
                return false;
            }
            
            // Check whether the ray passes too close to the upper obstacle (< 0.5)
            if (is_obs(vx, vy_top) && (vy_top - intersect_y) < 0.5 - 1e-6) {
                return false;
            }
        } 
        // 3. The ray crosses a horizontal grid line
        else {
            cy += dirY;
            tMaxY += tDeltaY;
            
            int vy = (dirY > 0) ? cy : cy + 1;
            int vx_left = cx;        // left endpoint of the grid line
            int vx_right = cx + 1;   // right endpoint of the grid line
            
            if (is_obs(vx_left, vy) && is_obs(vx_right, vy)) {
                return false; 
            }

            double intersect_x = ox + (vy - oy) * slope_xy;

            if (is_obs(vx_left, vy) && (intersect_x - vx_left) < 0.5 - 1e-6) {
                return false;
            }
            
            if (is_obs(vx_right, vy) && (vx_right - intersect_x) < 0.5 - 1e-6) {
                return false;
            }
        }
    }
}



// Return all valid ray endpoints (alpha), which also serve as the starting points of the next rays
std::vector<Coord> CastRays360(
    int ox, int oy, int oz, 
    const uint8_t is_obstacle[GRID_W_][GRID_H_][GRID_Z_], 
    int prev_dx = 0, int prev_dy = 0, int prev_dz = 0 // direction vector of the previous ray; default 0 means this is the start point
) { 
    // Alpha queue: only stores target vertices with clear line of sight
    std::vector<Coord> alpha; 

    // Only scan the 8 strictly valid directions (dx, dy)
    int dirs[8][2] = {
        {1, 0}, {1, 1}, {0, 1}, {-1, 1},
        {-1, 0}, {-1, -1}, {0, -1}, {1, -1}
    };

    for (int i = 0; i < 8; ++i) {
        int dx = dirs[i][0];
        int dy = dirs[i][1];
        
        // feed dx, dy into IsAngleValid; only continue if valid
        if (!IsAngleValid(prev_dx, prev_dy, prev_dz, dx, dy, 0)) continue;

        // Extend the ray until blocked
        int step_x = dx; // gcd is already 1, so dx itself is the primitive step
        int step_y = dy; 

        int prev_x = ox;
        int prev_y = oy;
        int curr_x = ox + step_x;
        int curr_y = oy + step_y;

        while (true) {
                    
                    if (curr_x < 0 || curr_x >= GRID_W || curr_y < 0 || curr_y >= GRID_H) break;// Stop if the ray goes out of bounds
                    
                    if (!isLineOfSightClear(prev_x, prev_y, oz, curr_x, curr_y, is_obstacle)) {
                        break;
                    }
                    
                    // Clear line of sight: valid turning-point candidate
                    alpha.push_back({curr_x, curr_y, oz});
                    
                    prev_x = curr_x;
                    prev_y = curr_y;
                    
                    curr_x += step_x;
                    curr_y += step_y;
        }
    }

    // Vertical 3D via expansion (scan all layers at the same x,y)
    for (int target_z = 0; target_z < GRID_Z_; ++target_z) {
        if (target_z == oz) continue; // skip the current layer
        
        int dz = target_z - oz;

        if (!IsAngleValid(prev_dx, prev_dy, prev_dz, 0, 0, dz)) continue;

        if (isViaClear(ox, oy, oz, target_z, is_obstacle)) {  // check whether the vertical direction is clear
            alpha.push_back({ox, oy, target_z});
        }
    }

    return alpha;
}


// ------------------------------- Dijkstra Utilities -------------------------------

// Euclidean distance between two points
double SegLenEuclid_born(const Coord& a, const Coord& b) {
    int dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    double cost = sqrt(double(dx * dx + dy * dy + dz * dz));
    return cost;
}

std::vector<Coord> get_neigh_born(int x, int y, int z) {
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

// Return the shortest path length, used to determine DFS search depth
double Dijkstra_shortestDist(
    Coord start_point, Coord end_point,
    const uint8_t obs_mask[GRID_W_][GRID_H_][GRID_Z_] )
{
    DistanceGrid dist{};  
    Coord prev[GRID_W_][GRID_H_][GRID_Z_]; 
    bool visited[GRID_W_][GRID_H_][GRID_Z_] = { false }; 

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

        for (auto [nx, ny, nz] : get_neigh_born(x, y, z)) {

            Coord next = { nx, ny,nz };

            if (next != end_point) {
                if (obs_mask[nx][ny][nz]) continue; 			
            }

            if (prev[x][y][z] != INVALID_COORD) {
                int prev_x = prev[x][y][z].x; int prev_y = prev[x][y][z].y; int prev_z = prev[x][y][z].z;

                if (prev_z == z && z == nz) {
                    float dot_xy = (prev_x - x) * (nx - x) +
                        (prev_y - y) * (ny - y);
                    if (dot_xy >= 0) continue;
                }
            }


            Coord now = { x, y,z };
            double edge_penalty = SegLenEuclid_born(now, next);

            double new_dist = dist[x][y][z] + edge_penalty; 
            if (new_dist < dist[nx][ny][nz]) {
                dist[nx][ny][nz] = new_dist;
                prev[nx][ny][nz] = { x, y,z };
                pq.push({ new_dist, nx, ny ,nz });
            }
        }
    }

    // Return -1.0 if the target is unreachable
    if (dist[end_point.x][end_point.y][end_point.z] == num_config::INF) {
        return -1.0;  
    }
    
    // Return the actual distance cost directly
    return dist[end_point.x][end_point.y][end_point.z];
}

// ------------------------------- Collect all straight-line paths from start to end with (target_segments - 1) turning points -------------------------------


//  Hashing
inline double GetSpatialNoise(int x, int y, int z, unsigned int seed) {
    unsigned int h = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791) ^ seed;
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return ((double)(h % 10000) / 10000.0 - 0.5) * 0.1; 
}

// Collect all paths
void DFS_AllExactSegments(
    Coord initial_curr, Coord goal, 
    int initial_depth, int target_segments, 
    const uint8_t is_obstacle[GRID_W_][GRID_H_][GRID_Z_], 
    std::vector<Coord>& initial_path,
    std::vector<std::vector<Coord>>& all_paths, 
    size_t max_results,
    int max_steps,                   // step limit (replaces time-based timeout)
    bool is_uniform_heuristic = true,
    unsigned int child_seed = random_seed
) {
    const int FULL_HP = max_steps;

    struct DFS_State {
        Coord curr;
        int depth;
        std::vector<Coord> path; 
    };

    std::vector<DFS_State> stack;
    stack.reserve(1000); 
    stack.push_back({initial_curr, initial_depth, initial_path});


    while (!stack.empty()) {
        
        // 1. Early termination: stop if the global or local quota is reached
        if (all_paths.size() >= max_results) break;

        //  (Step Limit)
        if (--max_steps <= 0) {
            break;
        }

        DFS_State s = stack.back(); stack.pop_back();

        Coord curr = s.curr;
        int current_depth = s.depth;
        std::vector<Coord>& path = s.path;

        int prev_dx = 0, prev_dy = 0, prev_dz = 0;
        if (path.size() >= 2) {
            prev_dx = curr.x - path[path.size() - 2].x;
            prev_dy = curr.y - path[path.size() - 2].y;
            prev_dz = curr.z - path[path.size() - 2].z;

            if (prev_dz == 0) { // planar segment: reduce to unit direction vector
                int g = std::gcd(std::abs(prev_dx), std::abs(prev_dy));
                if (g > 0) { prev_dx /= g; prev_dy /= g; }
            } else { // vertical segment
                prev_dx = 0; prev_dy = 0; 
            }
        }

        if (current_depth == target_segments - 1) {
            int final_dx = goal.x - curr.x;
            int final_dy = goal.y - curr.y;
            int final_dz = goal.z - curr.z;

            // The final step must also be 8-connected
            if (final_dz != 0) {
                if (final_dx != 0 || final_dy != 0) continue;
            } else {
                int abs_dx = std::abs(final_dx);
                int abs_dy = std::abs(final_dy);
                if (abs_dx != 0 && abs_dy != 0 && abs_dx != abs_dy) {
                    continue;
                }
            }

            if (IsAngleValid(prev_dx, prev_dy, prev_dz, final_dx, final_dy, final_dz)) {

                bool is_clear = false;
                if (final_dz == 0) {
                    is_clear = isLineOfSightClear(curr.x, curr.y, curr.z, goal.x, goal.y, is_obstacle);
                } else if (final_dx == 0 && final_dy == 0) {
                    is_clear = isViaClear(curr.x, curr.y, curr.z, goal.z, is_obstacle);
                } // if dx, dy, dz are all nonzero, diagonal motion in 3D space is not allowed
                

                if (is_clear) {
                    // Self-intersection check
                    bool is_intersecting = false;
                    // Only needed if there is more than one previous segment
                    if (path.size() >= 2) { 
                        for (size_t i = 0; i < path.size() - 2; ++i) {
                            if (doSegmentsIntersect(path[i], path[i+1], curr, goal)) {
                                is_intersecting = true;
                                break;
                            }
                        }
                    }

                    if (!is_intersecting) {
                        path.push_back(goal);        
                        all_paths.push_back(path);   
                        max_steps = FULL_HP;
                    }
                }
                
            }
            continue; 
        }

        std::vector<Coord> next_points = CastRays360(curr.x, curr.y, curr.z, is_obstacle, prev_dx, prev_dy, prev_dz);

        // ==========================================
        // The logic below is for diversity only, not a core part
        // ==========================================
        // Compute current progress (0.0 ~ 1.0)
        // Use (target_segments) as the denominator to reflect how many segments have been traversed
        double progress = (double)current_depth / (target_segments - 1);
        double curve_factor = 4.0 * (progress - 0.5) * (progress - 0.5);
        // Here vias become very cheap in the middle of the path, but expensive near the ends
        double dynamic_via_penalty = VIA_PENALTY * (curve_factor * (1.0 - min_discount) + min_discount);

        auto get_score = [&](const Coord& p) {
            double tx, ty, tz;

            if (is_uniform_heuristic) {// find an ideal intermediate point
                int remaining_segments = target_segments - current_depth;
                tx = curr.x + (goal.x - curr.x) / (double)remaining_segments;
                ty = curr.y + (goal.y - curr.y) / (double)remaining_segments;
                // Estimate the ideal layer at the current progress
                // by linearly interpolating from the start layer to the goal layer
                double start_z = path[0].z;
                tz/*理想z位置*/ = start_z + (goal.z - start_z) * progress;
            } else {
                tx = goal.x;ty = goal.y;tz = goal.z;
            }

            double dx = p.x - tx;
            double dy = p.y - ty;
            double dz = p.z - tz;

            // Base score
            // Apply dynamic penalty: the dz^2 term is scaled by the U-shaped via penalty
            double base_score = (dx * dx + dy * dy) + (dz * dz * dynamic_via_penalty);
            
            // Add a small perturbation; +1.0 avoids zero-score cases with no perturbation
            double noise_factor = GetSpatialNoise(p.x, p.y, p.z, child_seed);
            double noise = (base_score + 1.0) * noise_factor;

            return base_score + noise;
        };

        // Sort candidate points in descending score order (worse first)
        std::sort(next_points.begin(), next_points.end(), [&](const Coord& a, const Coord& b) {
            double scoreA = get_score(a);
            double scoreB = get_score(b);
            

            long long sA = std::round(scoreA * 1000000.0);
            long long sB = std::round(scoreB * 1000000.0);

            if (sA != sB) return sA > sB;
            
            if (a.x != b.x) return a.x > b.x;
            if (a.y != b.y) return a.y > b.y;
            return a.z > b.z;
        });

        for (const auto& next_p : next_points) {
            if (next_p == goal) continue; 
            if (std::find(path.begin(), path.end(), next_p) != path.end()) continue;  


            // ==========================================
            // 3D same-direction filtering:
            // do not consume a new segment if there is no turn
            // ==========================================
            int next_dx = next_p.x - curr.x;
            int next_dy = next_p.y - curr.y;
            int next_dz = next_p.z - curr.z;

            if (next_dz == 0) { // If the next step is a planar segment
                int g = std::gcd(std::abs(next_dx), std::abs(next_dy));
                int base_dx = next_dx / g;
                int base_dy = next_dy / g;

                // If the previous segment is also planar and has exactly the same direction,
                // then this turning point is meaningless and should be skipped
                if (prev_dz == 0 && prev_dx == base_dx && prev_dy == base_dy) { continue; }

            } else { // If the next step is a via
            }


            // Added self-intersection check for the new segment (curr -> next_p)
            bool is_intersecting = false;
            if (path.size() >= 2) {
                for (size_t i = 0; i < path.size() - 2; ++i) {
                    Coord p1 = path[i], p2 = path[i+1], p3 = curr, p4 = next_p;
                    if (p1.z == p2.z && p3.z == p4.z && p1.z == p3.z) {
                        if (doSegmentsIntersect(p1, p2, p3, p4)) {
                            is_intersecting = true; break;
                        }
                    }
                }
            }
            
            // If the new segment would cut through the existing path, discard it
            if (is_intersecting) continue; 


            DFS_State next_state;
            next_state.curr = next_p;
            next_state.depth = current_depth + 1;
            next_state.path = path; 
            next_state.path.push_back(next_p); 

            stack.push_back(std::move(next_state)); 
        }
    }
}



// Return all straight-line paths from start to end with (target_segments - 1) turning points
std::vector<std::vector<Coord>> FindAllExactSegmentPaths(
    int start_x, int start_y, 
    int end_x, int end_y, 
    int target_segments, 
    const uint8_t is_obstacle[GRID_W_][GRID_H_][GRID_Z_],
    int dynamic_step_limit = step_limit,
    size_t max_results = 100, 
    int start_z=0, int end_z=0,
    bool is_uniform_heuristic = true 
) {
    if (target_segments <= 0) return {};

    Coord start = {start_x, start_y, start_z};
    Coord goal = {end_x, end_y, end_z};



    std::vector<std::vector<Coord>> final_all_paths; 

    if (target_segments == 1) { 
        // A 1-segment path must still be purely 8-connected
        int abs_dx = std::abs(goal.x - start.x);
        int abs_dy = std::abs(goal.y - start.y);
        
        bool is_8_connected = (abs_dx == 0 || abs_dy == 0 || abs_dx == abs_dy);

        if (start.z == goal.z && !is_8_connected) {
            return {}; 
        }

        bool is_clear = false;
        if (start.z == goal.z) {
            is_clear = isLineOfSightClear(start.x, start.y, start.z, goal.x, goal.y, is_obstacle);
        } else if (start.x == goal.x && start.y == goal.y) {
            is_clear = isViaClear(start.x, start.y, start.z, goal.z, is_obstacle);
        }
        
        if (is_clear) { final_all_paths.push_back({start, goal}); }
        return final_all_paths;
    }

    std::vector<Coord> first_layer_points = CastRays360(start.x, start.y, start.z, is_obstacle, 0, 0, 0);

    // ==========================================
    // First-layer sorting:
    // flexible linear guidance + (uniform / greedy) switch
    // This is mainly for diversity, not a core part
    // ==========================================
    
    std::mt19937 first_layer_rng(random_seed);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    // Since this is the first step of the path, progress is fixed at 0
    double progress = 0.0; 
    // Compute the dynamic penalty for the first step
    double curve_factor = 4.0 * (progress - 0.5) * (progress - 0.5);
    double dynamic_via_penalty = VIA_PENALTY * (curve_factor * (1.0 - min_discount) + min_discount);

    auto get_first_score = [&](const Coord& p) {
        double tx, ty, tz;

        if (is_uniform_heuristic) {
            tx = start.x + (goal.x - start.x) / (double)target_segments;
            ty = start.y + (goal.y - start.y) / (double)target_segments;
            tz = start.z + (goal.z - start.z) * progress; 
        } else {
            tx = goal.x; ty = goal.y; tz = goal.z;
        }

        double dx = p.x - tx;
        double dy = p.y - ty;
        double dz = p.z - tz;

        // Score = squared planar distance + (squared vertical deviation * current penalty)
        double base_score = (dx * dx + dy * dy) + (dz * dz * dynamic_via_penalty);
        double noise_factor = GetSpatialNoise(p.x, p.y, p.z, random_seed);
        double noise = (base_score + 1.0) * noise_factor; 
        
        return base_score + noise;
    };

    // Standard ascending sort here (smaller score first),
    // because the later loop takes elements starting from index 0
    std::sort(first_layer_points.begin(), first_layer_points.end(), [&](const Coord& a, const Coord& b) {
        double scoreA = get_first_score(a);
        double scoreB = get_first_score(b);
        
        long long sA = std::round(scoreA * 1000000.0);
        long long sB = std::round(scoreB * 1000000.0);

        if (sA != sB) return sA < sB;
        
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    });




    size_t local_max_results = std::max<size_t>(10, (max_results / first_layer_points.size()) * 10); 
    local_max_results = std::min(local_max_results, max_results);

    std::vector<std::future<std::vector<std::vector<Coord>>>> futures;

    unsigned int thread_seed_offset = 0;

    for (const auto& next_p : first_layer_points) {
        if (next_p == goal) continue; 

        unsigned int child_seed = random_seed ^ (++thread_seed_offset * 1234567);

        futures.push_back(std::async(std::launch::async, [=]() {
            std::vector<std::vector<Coord>> local_paths;     
            std::vector<Coord> local_path = {start, next_p}; 
            
            // Independent path search with no interference across branches
            // Pass child_seed to ensure different randomized walks in each branch
            // Pass max_results and the initial dynamic_step_limit
            DFS_AllExactSegments(next_p, goal, 1, target_segments, is_obstacle, local_path, local_paths, local_max_results, dynamic_step_limit, is_uniform_heuristic, child_seed);
            return local_paths;
        }));
        
    }


    std::vector<std::vector<Coord>> all_collected_paths;

    for (auto& f : futures) {
        std::vector<std::vector<Coord>> paths = f.get();
        for (const auto& p : paths) {
            all_collected_paths.push_back(p);
        }
    }

    std::mt19937 selection_rng(random_seed_shuffle); 
    std::shuffle(all_collected_paths.begin(), all_collected_paths.end(), selection_rng);

    for (size_t i = 0; i < all_collected_paths.size(); ++i) {
        if (final_all_paths.size() < max_results) {
            final_all_paths.push_back(all_collected_paths[i]);
        } else {
            break;
        }
    }

    return final_all_paths;
}


PathGroup bornCandidate(const Coord& S, const Coord& T, 
                        std::vector<int> target_segments = {1,2}/* number of turning points = target_segments - 1 */, 
                        int max_results = 10 /* maximum number of paths for each segment count */ ){


    std::vector<std::vector<Coord>> total_paths;

    // Use Dijkstra to obtain the actual shortest distance first,
    // then use it to determine a dynamic step limit
    double real_dist = Dijkstra_shortestDist(S, T, obstacles_map);
    int dynamic_step_limit = 0;
    if (real_dist < 0) {
        int manhattan_dist = std::abs(T.x - S.x) + std::abs(T.y - S.y) + std::abs(T.z - S.z);
        dynamic_step_limit = step_limit + manhattan_dist * 60;
    } else {
        dynamic_step_limit = 15000 + (int)real_dist * 60;
    }
    std::cout << "  dynamic_step_limit " << dynamic_step_limit <<std::endl;


    // Generate up to max_results paths for each segment count
    for(int target : target_segments){ 
        std::cout << "  Searching for " << target << "-segment paths..." << std::endl;


        // Quota allocation:
        // split the total budget into forward/backward search,
        // then split each of them into uniform/greedy modes
        // This is mainly to avoid all paths clustering on the same side
        int forward_quota = max_results / 2;
        int backward_quota = max_results - forward_quota; 

        int f_uni_quota = forward_quota / 2;         
        int f_gre_quota = forward_quota - f_uni_quota;

        int b_uni_quota = backward_quota / 2;        
        int b_gre_quota = backward_quota - b_uni_quota;

        // ==========================================
        // Forward search (S -> T)
        // ==========================================
        std::vector<std::vector<Coord>> paths_f_uni = FindAllExactSegmentPaths(
            S.x, S.y, T.x, T.y, target, obstacles_map, dynamic_step_limit, f_uni_quota, S.z, T.z, true
        );
        std::vector<std::vector<Coord>> paths_f_gre = FindAllExactSegmentPaths(
            S.x, S.y, T.x, T.y, target, obstacles_map, dynamic_step_limit, f_gre_quota, S.z, T.z, false
        );
        
        // ==========================================
        // Backward search (T -> S)
        // ==========================================
        std::vector<std::vector<Coord>> paths_b_uni = FindAllExactSegmentPaths(
            T.x, T.y, S.x, S.y, target, obstacles_map, dynamic_step_limit, b_uni_quota, T.z, S.z, true
        );
        std::vector<std::vector<Coord>> paths_b_gre = FindAllExactSegmentPaths(
            T.x, T.y, S.x, S.y, target, obstacles_map, dynamic_step_limit, b_gre_quota, T.z, S.z, false
        );

        // ==========================================
        // Collect and reverse
        // ==========================================
        for (const auto& path : paths_f_uni) { total_paths.push_back(path); }
        for (const auto& path : paths_f_gre) { total_paths.push_back(path); }
        
        for (auto& path : paths_b_uni) { 
            std::reverse(path.begin(), path.end()); 
            total_paths.push_back(path); 
        }
        for (auto& path : paths_b_gre) { 
            std::reverse(path.begin(), path.end()); 
            total_paths.push_back(path); 
        }
        
        size_t total_f = paths_f_uni.size() + paths_f_gre.size();
        size_t total_b = paths_b_uni.size() + paths_b_gre.size();
        
        std::cout << "      Found " << (total_f + total_b) << " paths in total\n"
                  << "      (forward-uniform:" << paths_f_uni.size() << ", forward-greedy:" << paths_f_gre.size() 
                  << " | backward-uniform:" << paths_b_uni.size() << ", backward-greedy:" << paths_b_gre.size() << ")" << std::endl;
    }


    // Remove duplicate paths
    std::sort(total_paths.begin(), total_paths.end());
    auto last = std::unique(total_paths.begin(), total_paths.end()); 
    total_paths.erase(last, total_paths.end());
    std::cout << "      " << total_paths.size() << " unique paths remain after deduplication.\n";


    PathGroup result;

    // Compute the total length of each generated path
    for (const auto& path : total_paths) {
        double total_length = 0.0;
        
        // Traverse each segment in the path
        for (size_t i = 0; i < path.size() - 1; ++i) {
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            double dz = path[i+1].z - path[i].z;
            
            total_length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        
        double rounded_length = std::round(total_length);
        result.push_back({rounded_length, path});
    }

    return result;

}