#pragma once
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <windows.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <math.h>
using namespace cv;
using namespace std;

/* Using the GLUT library for the base windowing setup */


#define FORWARD_STEP_SCALE 50
#define BACKWARD_STEP_SCALE 50
#define RIGHT_STEP_SCALE 10
#define LEFT_STEP_SCALE 10
#define UP_STEP_SCALE 50
#define DOWN_STEP_SCALE 50
#define WIDTH 480
#define HEIGHT 240

class Visualizer
{
public:
	Visualizer();
	virtual ~Visualizer(void);
	static void init(float initialXpos, float initialYpos,
			   float initialZpos, float initialXrot,
	           float initialYrot, float initialAngle);
	static void start(int argc, char **argv);
	static void addPoints(GLfloat *newPoints, GLfloat *pointsColors, int noNewPixels);
	static void addImageFrame(Mat *color, Mat *depth);

private:
	static GLfloat *vertices;
	static GLfloat *colors;
	static float xpos, ypos, zpos;
	static float xrot, yrot;
	static float angle;
	static float lastx, lasty;
	static int noPixels;

	static void enable (void);
	static void camera (void);
	static void display (void);
	static void drawScene (void);
	static void drawCoordinates(void);
	static void reshape (int w, int h);
	static void keyboard (unsigned char key, int x, int y);
	static void mouseMovement(int x, int y);
	
};

