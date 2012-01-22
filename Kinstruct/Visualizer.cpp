#include "Visualizer.h"
float Visualizer::xpos = 0;
float Visualizer::ypos = 0;
float Visualizer::zpos = 0;
float Visualizer::xrot = 0;
float Visualizer::yrot = 0;
float Visualizer::angle = 0;
float Visualizer::lastx = 0;
float Visualizer::lasty = 0;
int  Visualizer::noPixels = 0;
GLfloat *Visualizer::vertices = 0;
GLfloat *Visualizer::colors = 0;

Visualizer::Visualizer()
{	
}



Visualizer::~Visualizer(void)
{
	
}

void Visualizer::init(float initialXpos, float initialYpos, float initialZpos, float initialXrot,
	                   float initialYrot, float initialAngle )
{
	Visualizer::xpos = initialXpos;
	Visualizer::ypos = initialYpos;
	Visualizer::zpos = initialZpos;
	Visualizer::xrot = initialXrot;
	Visualizer::yrot = initialYrot;
	Visualizer::angle = initialAngle;
	Visualizer::lastx = 0;
	Visualizer::lasty = 0;
	Visualizer::noPixels = 0;
	Visualizer::vertices = 0;
	Visualizer::colors = 0;
}

void Visualizer::addPoints(GLfloat *newPoints, GLfloat *pointsColors, int noNewPixels)
{
	GLfloat *newVertices = new GLfloat[(noPixels + noNewPixels) * 3];
	if(vertices != NULL )
		memcpy(newVertices, vertices, noPixels * 3 * sizeof FLOAT);
	
	memcpy(newVertices + noPixels * 3, newPoints, noNewPixels * 3 * sizeof FLOAT);
	delete(vertices);
	vertices = newVertices;

	GLfloat *newColors = new GLfloat[(noPixels + noNewPixels) * 3];
	if(colors != NULL )
		memcpy(newColors, colors, noPixels * 3 * sizeof FLOAT);
	memcpy(newColors + noPixels * 3, pointsColors, noNewPixels * 3 * sizeof FLOAT);
	delete(colors);
	colors = newColors;

	noPixels += noNewPixels;
}

void Visualizer::start(int argc, char **argv)
{
	   glutInit (&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_DEPTH);
	//set the display to Double buffer, with depth
    glutInitWindowSize (500, 500); //set the window size
    glutInitWindowPosition (100, 100);
	//set the position of the window
    glutCreateWindow ("A basic OpenGL Window"); 
	//the caption of the window
    glutDisplayFunc (Visualizer::display); 
	//use the display function to draw everything
    //glutIdleFunc (display); 
	//update any variables in display, display can be changed to anyhing, as long as you move the variables to be updated, in this case, angle++;
    glutReshapeFunc (Visualizer::reshape); //reshape the window accordingly
	glutPassiveMotionFunc(Visualizer::mouseMovement); //check for mouse
    glutKeyboardFunc (Visualizer::keyboard); //check the keyboard
    glutMainLoop (); //call the main loop
}


void Visualizer::drawCoordinates()
{
    glPointSize(10.0f);
	glBegin(GL_POINTS);
		glBegin(GL_LINE_LOOP);
		glColor3f (1.0, 0, 0);		
		glVertex3f(0, 0, 0);
		glVertex3f(500, 0, 0);

		glColor3f (0, 1.0, 0);		
		glVertex3f(0, 0, 0);
		glVertex3f(0, 500, 0);

		glColor3f (0, 0, 1.0);		
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 500);
	glEnd();
}

//drawing the scene
void Visualizer::drawScene (void) {
	glPointSize(3);
	glEnableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
  
	glColorPointer(3, GL_FLOAT, 0, colors);
	glVertexPointer(3, GL_FLOAT, 0, vertices);

	glDrawArrays(GL_POINTS, 0, noPixels);

	glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
	glDisableClientState(GL_COLOR_ARRAY);

	    
}


void Visualizer::enable (void) {
    glEnable (GL_DEPTH_TEST); //enable the depth testing
	glEnable(GL_COLOR_MATERIAL);
    glEnable (GL_LIGHTING); //enable the lighting
    glEnable (GL_LIGHT0); //enable LIGHT0, our Diffuse Light
    glShadeModel (GL_SMOOTH); //set the shader to smooth shader
}

void Visualizer::camera (void) {
    glRotatef(xrot,1.0,0.0,0.0); 
	//rotate our camera on teh x-axis (left and right)
    glRotatef(yrot,0.0,1.0,0.0); 
	//rotate our camera on the y-axis (up and down)
    glTranslated(-xpos,-ypos,-zpos);
	//translate the screen to the position of our camera
}

void Visualizer::display (void) {
    glClearColor (0.0,0.0,0.0,1.0);
	//clear the screen to black
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//clear the color buffer and the depth buffer
    glLoadIdentity();  
    camera();
    enable();
	drawCoordinates();
	drawScene(); //Draw the scene
    glutSwapBuffers(); //swap the buffers
    angle++; //increase the angle
}

void Visualizer::reshape (int w, int h) {
    glViewport (0, 0, (GLsizei)w, (GLsizei)h); 
	//set the viewport to the current window specifications
    glMatrixMode (GL_PROJECTION); //set the matrix to projection

    glLoadIdentity ();
    gluPerspective (60, (GLfloat)w / (GLfloat)h, 1.0, 1000.0
); //set the perspective (angle of sight, width, height, , depth)
    glMatrixMode (GL_MODELVIEW); //set the matrix back to model

}

void Visualizer::keyboard (unsigned char key, int x, int y) {
    if (key=='q')
    {
		xrot += 1 * DOWN_STEP_SCALE;
		if (xrot >360) xrot -= 360;
    }

    if (key=='z')
    {
		xrot -= 1 * UP_STEP_SCALE;
		if (xrot < -360) xrot += 360;
    }

    if (key=='w')
    {
		float xrotrad, yrotrad;
		yrotrad = (yrot / 180 * 3.141592654f);
		xrotrad = (xrot / 180 * 3.141592654f); 
		xpos += float(sin(yrotrad)) * FORWARD_STEP_SCALE;
		zpos -= float(cos(yrotrad)) * FORWARD_STEP_SCALE;
		ypos -= float(sin(xrotrad)) * FORWARD_STEP_SCALE;
    }

    if (key=='s')
    {
		float xrotrad, yrotrad;
		yrotrad = (yrot / 180 * 3.141592654f);
		xrotrad = (xrot / 180 * 3.141592654f); 
		xpos -= float(sin(yrotrad)) * BACKWARD_STEP_SCALE;
		zpos += float(cos(yrotrad)) * BACKWARD_STEP_SCALE;
		ypos += float(sin(xrotrad)) * BACKWARD_STEP_SCALE;
    }

    if (key=='d')
    {
		yrot += 1 * RIGHT_STEP_SCALE;
		if (yrot >360) yrot -= 360;
    }

    if (key=='a')
    {
		yrot -= 1 * LEFT_STEP_SCALE;
		if (yrot < -360) yrot += 360;
    }
    if (key==27)
    {
		exit(0);
    }
	display();
}

void Visualizer::mouseMovement(int x, int y) {
    int diffx = x - lastx; //check the difference between the current x and the last x position
    int diffy = y - lasty; //check the difference between the current y and the last y position
    lastx = x; //set lastx to the current x position
    lasty = y; //set lasty to the current y position
    xrot += (float) diffy / 10; //set the xrot to xrot with the addition of the difference in the y position
    yrot += (float) diffx / 10;    //set the xrot to yrot with the addition of the difference in the x position
	display();
}

void Visualizer::addImageFrame(Mat *color, Mat *depth)
{
	int noPixels = color->rows * color->cols;
	GLfloat *vertices = new GLfloat[noPixels * 3];
	GLfloat *colors = new GLfloat[noPixels * 3];
		
	
	int rowsRGB = color->rows;
	int colsRGB = color->cols;
	int pixelIndex = 0;
	for (int k = 0; k < rowsRGB ; k++) {
		for (int m = 0; m < colsRGB; m++) {
			int bit0 = depth->at<cv::Vec3b>(k,m)[0];
			int bit1 = depth->at<cv::Vec3b>(k,m)[1];
			int depth = (bit0 | bit1 << 8 );
				
			double blue = color->at<cv::Vec3b>(k,m)[0];
			double green = color->at<cv::Vec3b>(k,m)[1];
			double red = color->at<cv::Vec3b>(k,m)[2];

			vertices[pixelIndex] = m;
			vertices[pixelIndex + 1] = -k;
			vertices[pixelIndex + 2] = depth / 100;

			colors[pixelIndex] = red / 255;
			colors[pixelIndex + 1] = green / 255;
			colors[pixelIndex + 2] = blue / 255;
				
			
			pixelIndex += 3;
			
		}
	}
	Visualizer::addPoints(vertices, colors, noPixels);
	delete vertices;
	delete colors;
}


