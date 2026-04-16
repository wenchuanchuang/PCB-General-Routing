#ifndef UI_H  
#define UI_H

#include <GLFW/glfw3.h>
#include <imgui.h>
#include "ILMBase.h"

//io state from ImGui
extern bool g_WantCaptureKeyboard;
extern bool g_WantCaptureMouse;
extern bool g_WantCaptureTextInput;
//global stuff
extern Vec4i g_viewport;
extern GLFWwindow* g_window;
extern int g_window_w;
extern int g_window_h;

//imgui callbacks
void ErrorCallback(int Error, const char* Description);

void FramebufferSizeCallback(GLFWwindow* window, int w, int h);

void MouseButtonCallback(GLFWwindow* window, int button, int action, int mod);

void CursorPosCallback(GLFWwindow* window, double x, double y);

void KeyCallback(GLFWwindow*, int key, int scancode, int action, int mods);



#endif
