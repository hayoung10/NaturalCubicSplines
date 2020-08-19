#include "glSetup.h"

#include <Windows.h>
#include "include/GLFW/glfw3.h"
#include "include/GL/gl.h"
#include "include/GL/glut.h"
#include "include/GL/glew.h"

#pragma comment(lib, "lib/glfw3.lib")
#pragma comment(lib, "lib/opengl32.lib")
#pragma comment(lib, "lib/glut32.lib")
#pragma comment(lib, "lib/glew32.lib")

#include "Eigen/Dense"
using namespace Eigen;

#include <iostream>
using namespace std;

#define _USE_MATH_DEFINES
#include <math.h>

void init();
void render(GLFWwindow* window);
void reshape(GLFWwindow* window, int w, int h);
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse(GLFWwindow* window, int button, int action, int mods);
void cursor(GLFWwindow* window, double xpos, double ypos);

// Colors
GLfloat bgColor[4] = { 1,1,1,1 };

int datapoints = 0;
Vector3f ee_goal;
bool keya = false;
bool keyr = false;
bool keyd = false;
bool keyi = false;

int main(int argc, char* argv[])
{
	// Orthographics viewing
	perspectiveView = false;

	// Initialize the OpenGL system
	GLFWwindow* window = initializeOpenGL(argc, argv, bgColor);
	if (window == NULL) return -1;

	// Callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetMouseButtonCallback(window, mouse);
	glfwSetCursorPosCallback(window, cursor);

	// Depth test
	glDisable(GL_DEPTH_TEST);

	// Viewport and perspective setting
	reshape(window, windowW, windowH);

	// Initialization - Main loop
	init();

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		render(window); // Draw one frame
		glfwSwapBuffers(window); // Swap buffers
		glfwPollEvents(); // Events
	}

	// Terminate the glfw system
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

// (x, y, z) of data points
const int N = 9;
float p[10][3];

// Linear system: Ac = b
//		from n equations of the form:
//		p_i[t] = c_0^i + (c_1^i * t^1) + (c_2^i * t^2) + (c_3^i * t^3)
//
MatrixXf A;
MatrixXf b; // 4n x 3 to solve the 3 linear systems at once
MatrixXf c; // 4n x 3 to solve the 3 linear systems at once

void buildLinearSystem()
{
	// Build A and b for N segments. A is independent of the locations of the points.
	A.resize(4 * N, 4 * N);
	A.setZero();

	b.resize(4 * N, 3);

	// Equation number
	int row = 0;

	// 2n equations for end point interpolation
	for (int i = 0; i < datapoints-1; i++, row += 2)
	{
		// p_i(0) = c_0^i
		A(row, 4 * i + 0) = 1;

		b(row, 0) = p[i][0];
		b(row, 1) = p[i][1];
		b(row, 2) = p[i][2];

		// p_i(1) = c_0^i + c_1^i + c_2^i + c_3^i
		A(row + 1, 4 * i + 0) = 1;
		A(row + 1, 4 * i + 1) = 1;
		A(row + 1, 4 * i + 2) = 1;
		A(row + 1, 4 * i + 3) = 1;

		b(row + 1, 0) = p[i + 1][0];
		b(row + 1, 1) = p[i + 1][1];
		b(row + 1, 2) = p[i + 1][2];
	}

	// (n-1) equations for the tangential continuity
	for (int i = 0; i < datapoints - 2; i++, row++)
	{
		// p'_i(1) = 1*c_1^i + 2*c_2^i + 3*c_3^i = c_1^(i+1) = p'_(i+1)(0)
		A(row, 4 * i + 1) = 1;
		A(row, 4 * i + 2) = 2;
		A(row, 4 * i + 3) = 3;
		A(row, 4 * i + 5) = -1;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;
	}

	// (n-1) equations for the second-derivative continuity
	for (int i = 0; i < datapoints - 2; i++, row++)
	{
		// p''_i(1) = 2*c_2^i + 6*c_3^i = 2*c_2^(i+1) = p''_(i+1)(0)
		A(row, 4 * i + 2) = 2;
		A(row, 4 * i + 3) = 6;
		A(row, 4 * i + 6) = -2;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;
	}

	// 2 equations for the natural boundary condition
	{
		// p''_0(0) = 2*c_2^0 = 0
		A(row, 2) = 2;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;

		row++;

		// p''_(n-1)(1) = 2*c_2^(n-1) + 6*c_3^(n-1) = 0
		A(row, 4 * (N - 1) + 2) = 2;
		A(row, 4 * (N - 1) + 3) = 6;

		b(row, 0) = 0;
		b(row, 1) = 0;
		b(row, 2) = 0;

		row++;
	}
}

void solveLinearSystem()
{
	// Solve the 3 linear systems at once
	c = A.colPivHouseholderQr().solve(b);
}

void init()
{
	// Keyboard
	cout << "Add 10 data points (a key)" << endl;
	cout << "Select/remove 3 dta points (r key)" << endl;
	cout << "Select/drag 2 data points (d key)" << endl;
	cout << "Select edges and insert 2 data points (i key)" << endl;
}

// Draw the natural cubic spline
void drawNaturalCubicSpline()
{
	int N_SUB_SEGMENTS = 40;

	// Curve
	glLineWidth(1.5*dpiScaling);
	glColor3f(0, 0, 0);
	for (int i = 0; i < datapoints - 1; i++)
	{
		// N_SUB_SEGMENTS for each curve segment
		glBegin(GL_LINE_STRIP);
		for (int j = 0; j < N_SUB_SEGMENTS; j++)
		{
			float t = (float)j / (N_SUB_SEGMENTS - 1);

			float x = c(4 * i + 0, 0) + (c(4 * i + 1, 0) + (c(4 * i + 2, 0) + c(4 * i + 3, 0)*t)*t)*t;
			float y = c(4 * i + 0, 1) + (c(4 * i + 1, 1) + (c(4 * i + 2, 1) + c(4 * i + 3, 1)*t)*t)*t;
			float z = c(4 * i + 0, 2) + (c(4 * i + 1, 2) + (c(4 * i + 2, 2) + c(4 * i + 3, 2)*t)*t)*t;

			glVertex3f(x, y, z);
		}
		glEnd();
	}

	// Data points
	glPointSize(10);
	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	for (int i = 0; i < datapoints; i++)
		glVertex3f(p[i][0], p[i][1], p[i][2]);
	glEnd();
}

void removePoint(float x, float y, float z)
{
	float length = 0.25, dis;
	int index;
	for (int i = 0; i < datapoints; i++)
	{
		dis = sqrt(pow((p[i][0] - x), 2) + pow((p[i][1] - y), 2) + pow((p[i][2] - z), 2));
		if (length > dis) {
			length = dis;
			index = i;
		}
	}

	// Remove data point
	if (length < 0.25)
	{
		for (int j = index; j < datapoints - 1; j++)
		{
			p[j][0] = p[j + 1][0];
			p[j][1] = p[j + 1][1];
			p[j][2] = p[j + 1][2];
		}
		p[datapoints - 1][0] = 0;
		p[datapoints - 1][1] = 0;
		p[datapoints - 1][2] = 0;
		datapoints--;
	}
	if (datapoints == 0)
		keyr = false;
}

int findIndex(float x, float y, float z)
{
	float length = 0.25, dis;
	int index;
	for (int i = 0; i < datapoints; i++)
	{
		dis = sqrt(pow((p[i][0] - x), 2) + pow((p[i][1] - y), 2) + pow((p[i][2] - z), 2));
		if (length > dis) {
			length = dis;
			index = i;
		}
	}

	if (length < 0.25)
		return index;
	else
		return 15;
}

void movePoint(float x, float y, float z)
{
	int index = findIndex(x, y, z);
	
	if (index != 15)
	{
		p[index][0] = x;
		p[index][1] = y;
		p[index][2] = z;
	}
}

void selectEdge(float x, float y, float z)
{
	float length = 0.25, dis;
	for (int i = 0; i < datapoints; i++)
	{
		dis = sqrt(pow((p[i][0] - x), 2) + pow((p[i][1] - y), 2) + pow((p[i][2] - z), 2));
		if (length > dis)
			length = dis;
	}

	float length2 = 0.25, dis2;
	float x3, y3, z3;
	int index2;
	int N_SUB_SEGMENTS = 40;

	for (int i = 0; i < datapoints - 1; i++)
	{
		for (int j = 0; j < N_SUB_SEGMENTS; j++)
		{
			float t = (float)j / (N_SUB_SEGMENTS - 1);

			float x2 = c(4 * i + 0, 0) + (c(4 * i + 1, 0) + (c(4 * i + 2, 0) + c(4 * i + 3, 0)*t)*t)*t;
			float y2 = c(4 * i + 0, 1) + (c(4 * i + 1, 1) + (c(4 * i + 2, 1) + c(4 * i + 3, 1)*t)*t)*t;
			float z2 = c(4 * i + 0, 2) + (c(4 * i + 1, 2) + (c(4 * i + 2, 2) + c(4 * i + 3, 2)*t)*t)*t;

			dis2 = sqrt(pow((x - x2), 2) + pow((y - y2), 2) + pow((z - z2), 2));
			if (dis2 < length2)
			{
				length2 = dis2;
				x3 = x2; y3 = y2; z3 = z2;
				index2 = i;
			}	
		}
	}

	// Add data point
	if (length2 < length && length2 < 0.25)
	{
		for (int i = datapoints; i > index2+1; i--)
		{
			p[i][0] = p[i - 1][0];
			p[i][1] = p[i - 1][1];
			p[i][2] = p[i - 1][2];
		}
		p[index2 + 1][0] = x3;
		p[index2 + 1][1] = y3;
		p[index2 + 1][2] = z3;
		datapoints++;
	}
}

void render(GLFWwindow* window)
{
	// Background color
	glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Data point
	if (datapoints > 0) {
		buildLinearSystem();
		solveLinearSystem();
		drawNaturalCubicSpline();
	}
}

void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
			// Quit
		case GLFW_KEY_Q:
		case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;

		case GLFW_KEY_A: keya = true; keyr = false; keyd = false; keyi = false; break;
		case GLFW_KEY_R: keya = false; keyr = true; keyd = false; keyi = false; break;
		case GLFW_KEY_D: keya = false; keyr = false; keyd = true; keyi = false; break;
		case GLFW_KEY_I: keya = false; keyr = false; keyd = false; keyi = true; break;
		}
	}
}

bool drag = false;

void mouse(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		// In the screen coordinate
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		ee_goal = Vector3f(xpos, ypos, 0);

		// In the workspace. See reshape() in glSetup.cpp
		float aspect = (float)screenW / screenH;
		ee_goal[0] = 2.0 * (ee_goal[0] / screenW - 0.5)*aspect;
		ee_goal[1] = -2.0 * (ee_goal[1] / screenH - 0.5);

		if (datapoints == 10)
		{
			keya = false; keyi = false;
		}
		else if (keya) {
			p[datapoints][0] = ee_goal[0];
			p[datapoints][1] = ee_goal[1];
			p[datapoints][2] = 0;
			datapoints++;
		}
		else if (keyi)
		{
			if (datapoints > 1)
				selectEdge(ee_goal[0], ee_goal[1], 0);
		}
		
		if (datapoints == 0)
			keyr = false;
		else if (keyr)
			removePoint(ee_goal[0], ee_goal[1], 0);

		if (keyd)
			drag = true;
	}
	else if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (keyd)
			if (drag)
				drag = false;
	}
}

void cursor(GLFWwindow* window, double xpos, double ypos)
{
	if (drag)
	{
		ee_goal = Vector3f(xpos, ypos, 0);

		float aspect = (float)screenW / screenH;
		ee_goal[0] = 2.0 * (ee_goal[0] / screenW - 0.5)*aspect;
		ee_goal[1] = -2.0 * (ee_goal[1] / screenH - 0.5);

		movePoint(ee_goal[0], ee_goal[1], 0);
	}
		
}