/* Path selection from multiple candidates */

#include <vector>
#include <unordered_set> 
#include <unordered_map>
#include <cmath>
#include <numeric>
#include <set>

#include "gurobi_c++.h"
#include "gen_paths.h"
#include "IPsolver.h"

//------------------variable-------------------

bool finished_solve = false;
std::vector<std::vector<bool>> choosen_path_record = {};
std::vector<std::vector<double>> path_len;



void CalculateEveryPathLength() {

	path_len.clear();
	path_len.resize(all_results.size());

	for (size_t group_id = 0; group_id < all_results.size(); group_id++) {

		path_len[group_id].resize(all_results[group_id].size(), 0.0);

		for (size_t path_id = 0; path_id < all_results[group_id].size(); path_id++) {

			const Path& path = all_results[group_id][path_id].second; 
			double L = 0.0;
			for (size_t t = 0; t + 1 < path.size(); t++) {

				L += SegLenEuclid(path[t], path[t + 1]); 
			}
			path_len[group_id][path_id] = L;


		}
	}
}


bool SolveNetwork()
{

	const int Ng = (int)all_results.size();


	try {
		choosen_path_record.clear();
		finished_solve = false;

		GRBEnv env = GRBEnv();
		GRBModel model = GRBModel(env);

		model.set(GRB_IntParam_Threads, 20); 
		model.set(GRB_DoubleParam_MIPGap, 0.1);


		//========================
		// Variables:
		// pathChoice[pin-pair index][path index] is a binary variable
		// indicating whether a path is selected
		//========================
		std::vector<std::vector<GRBVar>> pathChoice;
		pathChoice.reserve(all_results.size());
		for (int g = 0; g < (int)all_results.size(); ++g) {
			std::vector<GRBVar> pathChoice_g;
			pathChoice_g.reserve(all_results[g].size());
			for (int p = 0; p < (int)all_results[g].size(); ++p) {
				// One decision variable for each path
				pathChoice_g.push_back(model.addVar(0.0, 1.0, 0.0, GRB_BINARY));
			}
			pathChoice.push_back(move(pathChoice_g));
		}


		
		// =========================================================================
        // Extract all integer grid vertices and 1x1 cell interiors visited by paths,
        // then build mutual-exclusion constraints
        // =========================================================================

        // 1. Record all terminal coordinates (start and end points of each net)
        //    to enforce == 1 constraints on them
        std::set<std::tuple<int, int, int>> terminal_coords;
        for (int g = 0; g < Ng; ++g) {
            if (all_results[g].empty()) continue;
            const auto& first_path = all_results[g][0].second;
            terminal_coords.insert(std::make_tuple(first_path.front().x, first_path.front().y, first_path.front().z));
            terminal_coords.insert(std::make_tuple(first_path.back().x, first_path.back().y, first_path.back().z));
        }

        std::map<std::tuple<int, int, int>, GRBLinExpr> v_exprs;
        std::map<std::tuple<int, int, int>, std::set<int>> v_groups;
        
        std::map<std::tuple<int, int, int>, GRBLinExpr> cell_exprs;
        // Record which type of diagonal passes through the cell
        // 1 means '/', 2 means '\', 3 means both
        std::map<std::tuple<int, int, int>, int> cell_diag_mask; 

        
		// 2. Traverse all candidate paths
        for (int g = 0; g < Ng; ++g) {
            for (int p = 0; p < (int)all_results[g].size(); ++p) {
                const auto& path_vec = all_results[g][p].second;
                
                std::set<std::tuple<int, int, int>> visited_vertices;
                std::set<std::tuple<int, int, int, int>> visited_cells_with_dir;// store tuple<cx, cy, cz, diag_type>

                for (size_t t = 0; t + 1 < path_vec.size(); ++t) {
                    const auto& u = path_vec[t];
                    const auto& v = path_vec[t + 1];

                    int dx = v.x - u.x;
                    int dy = v.y - u.y;
                    int dz = v.z - u.z;
                    
                    int gcd_val = std::gcd(std::abs(dx), std::gcd(std::abs(dy), std::abs(dz)));
                    
                    if (gcd_val > 0) {
                        int step_x = dx / gcd_val;
                        int step_y = dy / gcd_val;
                        int step_z = dz / gcd_val;

                        // Add every integer point along this segment into visited_vertices
                        for (int k = 0; k <= gcd_val; ++k) {
                            visited_vertices.insert(std::make_tuple(u.x + k * step_x, u.y + k * step_y, u.z + k * step_z));
                        }


                        // Only handle X-shaped crossings caused by 45-degree diagonals
                        if (step_z == 0 && std::abs(step_x) == 1 && std::abs(step_y) == 1) {
                            
                            int diag_type = (step_x * step_y > 0) ? 1 : 2; 

                            // Split a long diagonal into unit steps and record
                            // every 1x1 cell it passes through
                            for (int k = 0; k < gcd_val; ++k) {
                                int px = u.x + k * step_x;
                                int py = u.y + k * step_y;
                                // Represent the cell by its lower-left corner
                                int cx = std::min(px, px + step_x);
                                int cy = std::min(py, py + step_y);
                                
                                visited_cells_with_dir.insert(std::make_tuple(cx, cy, u.z, diag_type));
                            }
                        }
                    }
                }

                // 3. Inject the path decision variable into the corresponding vertex and cell expressions
                for (const auto& v_pos : visited_vertices) {
                    v_exprs[v_pos] += pathChoice[g][p];
                    v_groups[v_pos].insert(g);
                }
                for (const auto& cell_info : visited_cells_with_dir) {
                    auto cell_pos = std::make_tuple(std::get<0>(cell_info), std::get<1>(cell_info), std::get<2>(cell_info));
                    int diag_type = std::get<3>(cell_info);

                    cell_exprs[cell_pos] += pathChoice[g][p];
                    // Use bitwise OR to accumulate the mask. If both 1 and 2 appear, the result becomes 3.
                    cell_diag_mask[cell_pos] |= diag_type; 
                }
            }
        }

        // =========================================================================
        // Build Gurobi constraints
        // =========================================================================
        
        // 1. Add integer-vertex constraints
        int v_conflict_count = 0;
        for (auto const& [pos, expr] : v_exprs) {
            bool is_terminal = terminal_coords.count(pos);
            
            // If this is a terminal of any net, enforce flow == 1
            if (is_terminal) {
                std::string cName = "Terminal_V_x" + std::to_string(std::get<0>(pos)) + "_y" + std::to_string(std::get<1>(pos)) + "_z" + std::to_string(std::get<2>(pos));
                model.addConstr(expr == 1.0, cName);
                v_conflict_count++;
            } 
            // If this is not a terminal but is used by >= 2 different groups,
            // add a capacity constraint to prevent collisions
            else if (v_groups[pos].size() > 1) {
                std::string cName = "Capacity_V_x" + std::to_string(std::get<0>(pos)) + "_y" + std::to_string(std::get<1>(pos)) + "_z" + std::to_string(std::get<2>(pos));
                model.addConstr(expr <= 1.0, cName);
                v_conflict_count++;
            }
        }
        std::cout << "[DEBUG] Added " << v_conflict_count << " vertex constraints (Terminals + Collisions)." << std::endl;

        // 2. Add 1x1 cell interior constraints
        int cell_conflict_count = 0;
        for (auto const& [cell, expr] : cell_exprs) {
            
            // Only add a mutual-exclusion constraint when both '/' and '\' diagonals appear in the same cell (mask == 3)
            if (cell_diag_mask[cell] == 3) {
                std::string cName = "Cell_Excl_x" + std::to_string(std::get<0>(cell)) + "_y" + std::to_string(std::get<1>(cell)) + "_z" + std::to_string(std::get<2>(cell));
                model.addConstr(expr <= 1.0, cName);
                cell_conflict_count++;
            }
        }
        std::cout << "[DEBUG] Added " << cell_conflict_count << " cell interior crossing constraints." << std::endl;



		//=====================
		// Objective: minimize the total path length
		//=====================
		model.update();

		GRBLinExpr obj = 0.0;
		for (int g = 0; g < (int)all_results.size(); ++g) {
			for (int p = 0; p < (int)all_results[g].size(); ++p) {
				obj += path_len[g][p] * pathChoice[g][p];
			}
		}
		model.setObjective(obj, GRB_MINIMIZE);

		//=====================
		// solve the problem!
		//=====================
		model.optimize();
		int status = model.get(GRB_IntAttr_Status);
		if (status == GRB_INFEASIBLE)
		{
			//infeasible
			std::cout << "solve failed: the problem is infeasible!" << std::endl;
			return false;
		}
		else if (status != 9/*time-out*/ && status != 2 && status != 11 && status != 13)
		{
			//some other failure
			std::cout << "solve failed: status=" << status << std::endl;
			return false;
		}


		//=====================
		// get result
		//=====================
		choosen_path_record.assign(all_results.size(), {});
		for (int g = 0; g < (int)all_results.size(); ++g) {
			choosen_path_record[g].assign(all_results[g].size(), false);
		}

		for (int g = 0; g < (int)all_results.size(); ++g) {
			for (int p = 0; p < (int)all_results[g].size(); ++p) {
				if (pathChoice[g][p].get(GRB_DoubleAttr_X) > 0.5) {

					choosen_path_record[g][p] = true;

					std::cout << "Group " << g << ", selected path " << p << ", length =" << path_len[g][p] << "\n";
					for (const Coord& c : all_results[g][p].second) {
						std::cout << "(" << c.x << "," << c.y << "," << c.z << ") ";
					}
					std::cout << "\n";
				}
			}
		}


		std::cout << "Final objective value: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
		finished_solve = true;
		return true;
	}
	catch (GRBException& e) {
		std::cout << "Gurobi exception: " << e.getMessage()
			<< " (Error code: " << e.getErrorCode() << ")" << std::endl;
		return false;
	}
	catch (std::exception& e) {
		std::cout << "Standard exception: " << e.what() << std::endl;
		return false;
	}
	catch (...) {
		std::cout << "Unknown exception occurred in SolveNetwork()" << std::endl;
		return false;
	}
}