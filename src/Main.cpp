#define NOMINMAX	
#include <stdio.h>
#ifdef _WIN32
    #include <windows.h>
#else
    // linux version
#endif
#include <fstream>
#include <vector>
#include <array>
#include <map>
#include <iostream>
#include <filesystem>
#include <queue>
#include <tuple>
#include <functional>
#include <variant>
# include <cstdlib>
#include <thread>
#include <chrono>
#include <ctime>
#include <iomanip>

#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h" 
#include <GLFW/glfw3.h>
#include <GL/glut.h>
#include "ILMBase.h"


#include "IPsolver.h"
#include "gen_paths.h"
#include "bornCandidate.h"
#include "ui/ui.h"
#include "initial/config.h"
#include "loadYaml.h"
#include "loadData.h"

using namespace std;



// Simple time-printing utility (not critical)
void printTime() {
	auto now = chrono::system_clock::now();
	time_t now_time = chrono::system_clock::to_time_t(now);
	tm* local_time = localtime(&now_time);

	char buffer[80];
	strftime(buffer, sizeof(buffer), "%Y/%m/%d %H:%M:%S", local_time);
	std::cout << "Current time: " << buffer << std::endl;
	
}


// Simple timing utility (not critical)
class Timer {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    
public:
    Timer() : start_time(std::chrono::high_resolution_clock::now()) {}
    
    void printTime(const std::string& label = "") {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        long long total_microseconds = duration.count();
        
        long long minutes = total_microseconds / 60000000LL;
        long long seconds = (total_microseconds % 60000000LL) / 1000000LL;
        long long milliseconds = (total_microseconds % 1000000LL) / 1000LL;
        
        if (!label.empty()) {
            std::cout << label << ": ";
        }
        
		std::cout << minutes << " min "
                  << std::setfill('0') << std::setw(2) << seconds << " s "
                  << std::setfill('0') << std::setw(3) << milliseconds << " ms"
                  << std::endl;
    }
    
    void reset() {
        start_time = std::chrono::high_resolution_clock::now();
    }
};



// Visualization routine (not critical)
void DrawNetwork()
{
	int win_w = g_viewport[2] - 20;
	int win_h = g_viewport[3] - 20;

	float grid_aspect = (float)g_grid_w / (float)g_grid_h;

	int vp_w = win_w;
	int vp_h = (int)(vp_w / grid_aspect);

	if (vp_h > win_h) {
		vp_h = win_h;
		vp_w = (int)(vp_h * grid_aspect);
	}

	glViewport(20, 20, vp_w, vp_h); 
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-0.01, 1.01, -0.01, 1.01, -1, 1);


	int layer = current_layer;

	glBegin(GL_QUADS);
	for (int x = 0; x < g_grid_w; x++)
	{
		for (int y = 0; y < g_grid_h; y++)
		{
			// Check whether all four corners of this cell are obstacle vertices.
			// If so, render the cell in dark gray to indicate an obstacle region.
			bool is_obstacle_cell = false;
			bool range_check = true;
			if ((x + 1) >= GRID_W || (y + 1) >= GRID_H || layer < 0 || layer >= GRID_Z) range_check = false;

			if (range_check) {
				is_obstacle_cell = (
					obstacles_map[x][y][layer] &&
					obstacles_map[x + 1][y][layer] &&
					obstacles_map[x][y + 1][layer] &&
					obstacles_map[x + 1][y + 1][layer]
					);
			}

			Vec3f color;
			if (is_obstacle_cell)
				color = Vec3f(0.3f, 0.3f, 0.3f);  // dark gray obstacle region
			else
				color = Vec3f(0.9f, 0.9f, 0.9f);  // regular light gray cell

			glColor3f(color.x, color.y, color.z);

			glVertex2f((float)x / (float)g_grid_w, (float)y / (float)g_grid_h);
			glVertex2f((float)(x + 1) / (float)g_grid_w, (float)y / (float)g_grid_h);
			glVertex2f((float)(x + 1) / (float)g_grid_w, (float)(y + 1) / (float)g_grid_h);
			glVertex2f((float)x / (float)g_grid_w, (float)(y + 1) / (float)g_grid_h);
		}
	}
	glEnd();
	
	glDisable(GL_DEPTH_TEST);

	// Draw obstacle vertices
	glPointSize(10.0f);                     
	glColor3f(0.3f, 0.3f, 0.3f);           // dark gray
	//glColor3f(0.8f, 0.2f, 1.0f);
	glBegin(GL_POINTS);
	for (int x = 0; x < g_grid_w; ++x) {
		for (int y = 0; y < g_grid_h; ++y) {
			bool range_ok = (layer >= 0 && layer < GRID_Z);
			if (!range_ok) continue;

			if (obstacles_map[x][y][layer]) {
				float nx = static_cast<float>(x) / static_cast<float>(g_grid_w);
				float ny = static_cast<float>(y) / static_cast<float>(g_grid_h);
				glVertex2f(nx, ny);
			}
		}
	}
	glEnd();

	// Draw pins
	glPointSize(3.0f);                     
	glColor3f(0.5f, 0.5f, 0.5f);           // dark gray
	glBegin(GL_POINTS);
	for (int x = 0; x < g_grid_w; ++x) {
		for (int y = 0; y < g_grid_h; ++y) {
			bool range_ok = (layer >= 0 && layer < GRID_Z);
			if (!range_ok) continue;

			if (pin_map[x][y][layer]) {
				float nx = static_cast<float>(x) / static_cast<float>(g_grid_w);
				float ny = static_cast<float>(y) / static_cast<float>(g_grid_h);
				glVertex2f(nx, ny);
			}
		}
	}
	glEnd();

	//draw edges:
	glColor3f(0, 0, 0);
	for (int x = 0; x < g_grid_w; x++)
	{
		for (int y = 0; y <= g_grid_h; y++)
		{
			glLineWidth(2);

			glBegin(GL_LINES);
			glVertex2f((float)x / (float)g_grid_w, (float)y / (float)g_grid_h);
			glVertex2f((float)(x + 1) / (float)g_grid_w, (float)y / (float)g_grid_h);
			glEnd();
		}
	}
	for (int x = 0; x <= g_grid_w; x++)
	{
		for (int y = 0; y < g_grid_h; y++)
		{
			glLineWidth(2);

			glBegin(GL_LINES);
			glVertex2f((float)x / (float)g_grid_w, (float)y / (float)g_grid_h);
			glVertex2f((float)x / (float)g_grid_w, (float)(y + 1) / (float)g_grid_h);
			glEnd();
		}
	}

	// draw candidates paths
	glLineWidth(2.0f);
	for (int group_id = 0; group_id < all_results.size(); ++group_id) {
		for (int path_id = 0; path_id < all_results[group_id].size(); ++path_id) {

			if (finished_solve) { // If optimization is complete, only show results
				if (choosen_path_record[group_id][path_id] == false) continue;
			}

			const auto& [cost, path] = all_results[group_id][path_id];

			float colors[][3] = {
				{1.0f, 0.0f, 0.0f},    // Red
				{0.0f, 1.0f, 0.0f},    // Green
				{0.0f, 0.0f, 1.0f},    // Blue
				{1.0f, 1.0f, 0.0f},    // Yellow
				{1.0f, 0.0f, 1.0f},    // Magenta
				{0.0f, 1.0f, 1.0f},    // Cyan
				{1.0f, 0.5f, 0.0f},    // Orange
				{0.5f, 0.0f, 0.5f},    // Purple
				{0.6f, 0.3f, 0.0f},    // Brown
				{0.0f, 0.6f, 0.3f},    // Teal
				{0.3f, 0.3f, 0.3f},    // Dark Gray
				{0.7f, 0.7f, 0.7f},    // Light Gray
				{0.8f, 0.2f, 0.2f},    // Salmon
				{0.2f, 0.8f, 0.2f},    // Light Green
				{0.2f, 0.2f, 0.8f},    // Light Blue
				{0.9f, 0.6f, 0.0f},    // Gold
				{0.4f, 0.0f, 0.4f},    // Dark Purple
				{0.0f, 0.4f, 0.4f},    // Dark Cyan
				{0.5f, 0.5f, 0.0f},    // Olive
				{0.8f, 0.4f, 0.4f}     // Pinkish
			};
			int color_id;
			//color_id = path_id % 20;
			if (finished_solve) {  
				color_id = electrical_net[group_id] % 20;
			}
			else { 
				color_id = group_id % 20;
			}

			glColor3f(colors[color_id][0], colors[color_id][1], colors[color_id][2]);

			glBegin(GL_LINES);
			for (size_t i = 1; i < path.size(); ++i) {
				const auto& p1 = path[i - 1];
				const auto& p2 = path[i];

				// Only draw segments on the current layer
				if (p1.z == current_layer && p2.z == current_layer) {
					float fx1 = (float)p1.x / (float)GRID_W;
					float fy1 = (float)p1.y / (float)GRID_H;
					float fx2 = (float)p2.x / (float)GRID_W;
					float fy2 = (float)p2.y / (float)GRID_H;

					glVertex2f(fx1, fy1);
					glVertex2f(fx2, fy2);
				}
			}
			glEnd();
		}
	}



	// draw grey circles at pin
	glPointSize(6.0f);
	glColor3f(0.8f, 0.2f, 1.0f);   // dark gray
	glBegin(GL_POINTS);
	for (const auto& [start, goal] : path_endpoints) {
		if (start.z == current_layer) {
			glVertex2f((float)start.x / g_grid_w, (float)start.y / g_grid_h);
		}
		if (goal.z == current_layer) {
			glVertex2f((float)goal.x / g_grid_w, (float)goal.y / g_grid_h);
		}
	}
	glEnd();

	glEnable(GL_DEPTH_TEST);

}


// Visualization routine (not critical)
void Display(GLFWwindow* window)
{
	glClearColor(1, 1, 1, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	DrawNetwork();
}


int main() {

	threads = std::thread::hardware_concurrency() ? std::thread::hardware_concurrency() : 8;
	std::cout << "threads amount : " << threads << endl;

	InitObstacles();

	PutComponenetOnBoard(components_yaml, boardcomponents_yaml, boardNets_yaml,
						 references,ref_pos,
						 pin_map,
						 path_endpoints, electrical_net
	);

	std::cout << "EXTRA_LENGTH : " << EXTRA_LENGTH << endl;
	

	{
		if (!glfwInit())
			return 1;

		const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		g_window_w = mode->width - 15;
		g_window_h = mode->height - 15;

		g_window = glfwCreateWindow(g_window_w, g_window_h, "IP", NULL, NULL);
		glfwMakeContextCurrent(g_window);
		
		
    	IMGUI_CHECKVERSION();
    	ImGui::CreateContext(); 
    	ImGuiIO& io = ImGui::GetIO(); (void)io;

		ImGui_ImplGlfw_InitForOpenGL(g_window, false);
		ImGui_ImplOpenGL3_Init("#version 130");
	
		ImGui::GetIO().FontGlobalScale = (float)g_window_w / (float)1920;

		glfwSetErrorCallback(ErrorCallback);
		glfwSetFramebufferSizeCallback(g_window, FramebufferSizeCallback);
		glfwSetMouseButtonCallback(g_window, MouseButtonCallback);
		glfwSetCursorPosCallback(g_window, CursorPosCallback);
		glfwSetKeyCallback(g_window, KeyCallback);
		glfwSetCharCallback(g_window, ImGui_ImplGlfw_CharCallback);


		g_viewport[0] = 0;
		g_viewport[1] = 0;
		glfwGetWindowSize(g_window, &g_viewport[2], &g_viewport[3]);

		const int W = g_window_w * 0.2;
		const int WItem = (W / 2) * 0.4;
		const int Padding = 10;
		const int MainWindowH = g_window_h * 0.1;
		const int MainWindowX = g_viewport[2] - W - Padding;
		const int MainWindowY = Padding;

		int counter2 = 0;

		bool FirstLoop = true;
		bool run_yen_finish = false;

		uint8_t tmp[GRID_W_][GRID_H_][GRID_Z_] = {};
		for (int x = 0; x < GRID_W; ++x) {
			for (int y = 0; y < GRID_H; ++y) {
				for (int z = 0; z < GRID_Z; ++z) {
					tmp[x][y][z] = obstacles_map[x][y][z];
				}
			}
		}

		while (!glfwWindowShouldClose(g_window))
		{
			glfwPollEvents();

   	 		ImGui_ImplOpenGL3_NewFrame();
    		ImGui_ImplGlfw_NewFrame();    
    		ImGui::NewFrame();            
			

			ImGui::SetNextWindowPos(ImVec2(MainWindowX, MainWindowY));
			ImGui::SetNextWindowSize(ImVec2(W, MainWindowH));
			ImGui::Begin("Main");
			{
				if (ImGui::Button("born solid line candidate")) { // Generate Paths with High Diversity
					
					printTime();
					Timer timer;
					std::cout << "Run bornCandidate\n";
					std::cout << "If candidate generation takes too long, you can directly click 'load candidate paths' instead.\n";


					finished_solve = false;

					// Generate the sequence of target segment counts
					std::vector<int> target_segments;
					int target_fold = 10;
					for (int i = 1; i <= target_fold; i++){target_segments.push_back(i);} // {1,2,3,4,...,target_fold}

					int counter = 0;
					for (const auto& [start, goal] : path_endpoints) {

						for (int x = 0; x < GRID_W; ++x) {
							for (int y = 0; y < GRID_H; ++y) {
								for (int z = 0; z < GRID_Z; ++z) {
										obstacles_map[x][y][z] = obs_maps[counter][x][y][z];
								}
							}
						}

						std::cout << "path group " << counter << "\n";
						std::cout << "(" << start.x << "," << start.y << "," << start.z << ") --> (" << goal.x << "," << goal.y << "," << goal.z << ")" << endl;

						PathGroup result = bornCandidate(start, goal, target_segments /* number of turns = value - 1 */, 600/* maximum number of paths per segment count */);

						all_results.push_back(result);
						if(result.size() > 0){std::cout << "    Number of paths found: " << result.size() << "\n";}
						counter++;
						//break;
					}
					std::cout << "\n";

					CalculateEveryPathLength();

					run_yen_finish = true;
					printTime();
					timer.printTime("Run bornCandidate cost"); 


					// Restore obstacle map
					for (int x = 0; x < GRID_W; ++x) {
						for (int y = 0; y < GRID_H; ++y) {
							for (int z = 0; z < GRID_Z; ++z) {
								 obstacles_map[x][y][z] = tmp[x][y][z];
							}
						}
					}

				}

				if (ImGui::Button("load candidate paths")) { 

					// Restore obstacle map
					for (int x = 0; x < GRID_W; ++x) {
						for (int y = 0; y < GRID_H; ++y) {
							for (int z = 0; z < GRID_Z; ++z) {
								 obstacles_map[x][y][z] = tmp[x][y][z];
							}
						}
					}

					std::cout << "load candidate paths\n";
					if (std::filesystem::exists(candidate_paths_bin)) {
						std::cout << "Found cached paths! Loading from disk..." << std::endl;
						LoadPathDataset(all_results, candidate_paths_bin);
					}
					else{
						std::cout << "[DEBUG] Failed\n" << std::endl;
					}
				}
				
				if (ImGui::Button("Show obs")) { // Show obstacle regions

					for (const auto& [start, goal] : path_endpoints) {
						std::cout << "" << counter2 << "\n";

						for (int x = 0; x < GRID_W; ++x) {
							for (int y = 0; y < GRID_H; ++y) {
								for (int z = 0; z < GRID_Z; ++z) {
									obstacles_map[x][y][z] = obs_maps[counter2][x][y][z];
								}
							}
						}
					}
					counter2++;
				}
				if (ImGui::Button("Run Optimize Solution")) {

					// Restore obstacle map
					for (int x = 0; x < GRID_W; ++x) {
						for (int y = 0; y < GRID_H; ++y) {
							for (int z = 0; z < GRID_Z; ++z) {
								 obstacles_map[x][y][z] = tmp[x][y][z];
							}
						}
					}
					
					CalculateEveryPathLength(); // IPSolver.cpp
					printTime();
					SolveNetwork();
					printTime();
				}
				if (ImGui::Button("save candidate paths")) { 
					std::cout << "save candidate paths...\n";
					SavePathDataset(all_results, candidate_paths_bin);
				}

			}
			ImGui::End();


			g_WantCaptureKeyboard = ImGui::GetIO().WantCaptureKeyboard;
			g_WantCaptureMouse = ImGui::GetIO().WantCaptureMouse;
			g_WantCaptureTextInput = ImGui::GetIO().WantTextInput;


			Display(g_window);


			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			ImGui::Render();
			

			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

			glfwSwapBuffers(g_window);

			FirstLoop = false;
		}

		ImGui_ImplOpenGL3_Shutdown(); 
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();     
		glfwTerminate();
	}

	return 0;
}



