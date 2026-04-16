#include "initial/config.h"
#include "ui/ui.h"
#include <imgui.h>
#include <imgui_impl_glfw.h>

#include <iostream>
#include <algorithm>

int current_layer = 0;


//io state from ImGui
bool g_WantCaptureKeyboard = false;
bool g_WantCaptureMouse = false;
bool g_WantCaptureTextInput = false;
//global stuff
Vec4i g_viewport(0, 0, 800, 600);
GLFWwindow* g_window = NULL; 
int g_window_w = 1900; 
int g_window_h = 1060; 

//mouse/keyboard parameters
int g_mouse_button = -1;  //GLFW_MOUSE_BUTTON_1, GLFW_MOUSE_BUTTON_2, ...
int g_mouse_state = -1;  //GLFW_PRESS, GLFW_RELEASE
int g_mouse_x = 0;
int g_mouse_y = 0;
int g_mouse_modifier = 0;  //GLFW_MOD_SHIFT, GLFW_MOD_CONTROL, GLFW_MOD_ALT, GLFW_MOD_SUPER, ...
int g_down_x = 0;
int g_down_y = 0;


//imgui callbacks
void ErrorCallback(int Error, const char* Description)
{
	fprintf(stderr, "[ImGui_ErrorCallback] Error %d: %s\n", Error, Description);
}

void FramebufferSizeCallback(GLFWwindow* window, int w, int h)
{
	g_window_w = w;
	g_window_h = h;
	g_viewport[2] = w;
	g_viewport[3] = h;
}

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mod)
{

    	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mod);

    	g_WantCaptureMouse = ImGui::GetIO().WantCaptureMouse;
    
	if (g_WantCaptureMouse)
		return;

	g_mouse_button = button;
	g_mouse_state = action;
	g_mouse_modifier = mod;

	//remember x/y at mouse down
	if (g_mouse_state == GLFW_PRESS)
	{
		g_down_x = g_mouse_x;
		g_down_y = g_mouse_y;
	}
}

void CursorPosCallback(GLFWwindow* window, double x, double y)
{
	if (g_WantCaptureMouse)
		return;

	//camera control (only in non-panorama mode)
	if (g_mouse_state == GLFW_PRESS)
	{
		if (g_mouse_button == GLFW_MOUSE_BUTTON_1)
		{

		}
	}

	g_mouse_x = (int)x;
	g_mouse_y = (int)y;
}

void KeyCallback(GLFWwindow*, int key, int scancode, int action, int mods)
{
	if (g_WantCaptureKeyboard)
		return;

	g_mouse_modifier = mods;

	if (action == GLFW_PRESS)
	{
		if (key == GLFW_KEY_Z)
		{
			current_layer = std::max(0, current_layer - 1);
			std::cout << "Switched to layer " << current_layer << std::endl;
		}
		else if (key == GLFW_KEY_X)
		{
			current_layer = std::min(g_grid_z - 1, current_layer + 1);
			std::cout << "Switched to layer " << current_layer << std::endl;
		}
	}
}

