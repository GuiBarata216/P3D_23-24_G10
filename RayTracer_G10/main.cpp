 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2024 by Jo�o Madeiras Pereira
// Ray Tracing P3F scenes and drawing points with Modern OpenGL
// 
// G10:
// Done by David Martins ist199197, Guilherme Barata ist193718, Jo�o Ramos ist199253
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"

//Enable OpenGL drawing.  
bool drawModeEnabled = false;

bool P3F_scene = false; //choose between P3F scene or a built-in random scene

#define MAX_DEPTH 6  //number of bounces

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1
#define SHADOW_BIAS 0.001

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;

Grid* grid_ptr = NULL;
BVH* bvh_ptr = NULL;
accelerator Accel_Struct = NONE; //NONE or GRID_ACC or BVH_ACC

int RES_X, RES_Y;

int WindowHandle = 0;

//NEW VARIABLES

bool ANTIALIASING = true;
bool DEPTH_OF_FIELD = true;
bool FUZZY_REFLECTIONS = false;
bool SOFT_SHADOWS = true;

int SPP = 4; // Sample Per Pixel - Number of rays called for each pixel
int NUM_LIGHTS = 4; // Should be the same as SPP
int off_x, off_y; // Used for more even distribution using SOFT_SHADOWS + ANTIALIASING
float ROUGHNESS = 0.3f;

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}


// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////

Object* getClosestObject(Ray ray, float& t) {
	float min_dist = INFINITY;
	Object* closest = NULL;

	for (int i = 0; i < scene->getNumObjects(); i++) {
		Object* obj = scene->getObject(i);
		float dist = 0.0f;
		if (obj->intercepts(ray, dist)) {
			if (dist < min_dist) {
				closest = obj;
				min_dist = dist;
			}
		}
	}
	t = min_dist;
	return closest;
}

bool getIntersection(Ray ray, float min_dist, bool in_shadow, float sf_length) {
	Object* obj = NULL;

	for (int i = 0; i < scene->getNumObjects(); i++) {
		obj = scene->getObject(i);

		if (obj->intercepts(ray, min_dist) && min_dist < sf_length) {
			in_shadow = true;
			return in_shadow;
		}
	}

	return in_shadow;
}


Color calculateColor(Vector hit_pnt, Vector hit_norm, Light* light, Vector L, Vector ray_dir, Material* mat) {

	Color color = Color(0, 0, 0);
	Color light_color = light->color;
	float intensity, sf_length;
	float min_dist = INFINITY;
	bool in_shadow = false;
	Vector s_ray_dir, halfway_dir;

	Ray shadow_feeler = Ray(hit_pnt, L);

	if (bvh_ptr != NULL) {
		if (bvh_ptr->Traverse(shadow_feeler)) in_shadow = true;
	}
	else if (grid_ptr != NULL) {
		if (grid_ptr->Traverse(shadow_feeler)) in_shadow = true;
	}
	else {
		sf_length = shadow_feeler.direction.length(); //distance between light and intersection point
		shadow_feeler.direction.normalize();
		in_shadow = getIntersection(shadow_feeler, min_dist, in_shadow, sf_length);
	}

	if (!in_shadow) {
		s_ray_dir = ray_dir * -1;
		L = L.normalize();

		halfway_dir = L + s_ray_dir;
		halfway_dir = halfway_dir.normalize();

		float diff_int = max(hit_norm * L, 0.0);
		Color diffuse = light_color * mat->GetDiffColor() * diff_int * mat->GetDiffuse();

		float spec_int = pow(max((hit_norm * halfway_dir), 0.0), mat->GetShine());
		Color specular = light_color * mat->GetSpecColor() * spec_int * mat->GetSpecular();

		int num_lights = scene->getNumLights();
		color = (diffuse + specular) / (num_lights * 0.5f);
	}

	return color;
	
}

Color calculateLightReflection(Vector light_pos, Vector hit_pnt, Vector hit_norm, Vector ray_dir, Material* mat, Light* light) {
	Color color = Color(0, 0, 0);
	Vector L = light_pos - hit_pnt;
	L = L.normalize();

	float intensity = L * hit_norm;

	Vector reflect = ray_dir - hit_norm * (ray_dir * hit_norm) * 2;
	reflect = reflect.normalize();

	if (intensity > 0) {
		Vector exact_hit_pnt = hit_pnt + hit_norm * SHADOW_BIAS;
		color = calculateColor(exact_hit_pnt, hit_norm, light, L, ray_dir, mat);
		return color;
	}
	
	return color;
}

Color calculateLightContribution(Ray ray, Vector hit_pnt, Vector hit_norm, Material* mat) {
	Light* light = NULL;
	Vector light_pos, L, reflect;
	Color light_color = Color(0, 0, 0);

	for (int i = 0; i < scene->getNumLights(); i++) {
		light = scene->getLight(i);
		light_pos = light->position;

		if (SOFT_SHADOWS) {
			float l_jitt_size = 0.5f;
			Color aux_color = Color(0, 0, 0);

			if (!ANTIALIASING) {
				float dist_btw_l = l_jitt_size / NUM_LIGHTS;

				for (int k = 0; k < SPP; k++) {
					for (int j = 0; j < SPP; j++) {
						light_pos.x = light_pos.x - dist_btw_l * NUM_LIGHTS * l_jitt_size;
						light_pos.y = light_pos.y - dist_btw_l * NUM_LIGHTS * l_jitt_size;

						aux_color += calculateLightReflection(light_pos, hit_pnt, hit_norm, ray.direction, mat, light);
					}
				}

				aux_color = aux_color / pow(SPP, 2);
				light_color += aux_color;
			}

			else {
				light_pos.x = light_pos.x - l_jitt_size + (off_x + rand_float()) / SPP;
				light_pos.y = light_pos.y - l_jitt_size + (off_y + rand_float()) / SPP;
				light_color += calculateLightReflection(light_pos, hit_pnt, hit_norm, ray.direction, mat, light);
			}
		}

		else {
			light_color += calculateLightReflection(light_pos, hit_pnt, hit_norm, ray.direction, mat, light);
		}
	}
	return light_color;
}

float schlickApproximation(float cos_d, float ior_1, float ior_2) {

	float R_0 = pow(((ior_1 - ior_2) / (ior_1 + ior_2)), 2);
	float Kr = R_0 + (1.0f - R_0) * pow((1 - cos_d), 5);

	return Kr;
}

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	Color color = Color(0,0,0);
	float min_dist = INFINITY;
	float ior_2;
	Object* hit_obj = NULL;
	Material* mat = NULL;
	Vector hit_pnt, exact_hit_pnt, hit_norm, light_Pos, L, v_t, refraction, reflection;
	Light* light = NULL;
	bool is_hit = false, r_inside = false;

	//Grid is active
	if (grid_ptr != NULL) {
		is_hit = grid_ptr->Traverse(ray, &hit_obj, hit_pnt);
	}
	//BVH is active
	else if (bvh_ptr != NULL) {
		is_hit = bvh_ptr->Traverse(ray, &hit_obj, hit_pnt);
	}
	else {
		hit_obj = getClosestObject(ray, min_dist);

		if (hit_obj != NULL) {
			hit_pnt = ray.origin + ray.direction * min_dist;
		}
	}

	//If ray intercepts no object return Background or Skybox color
	if (hit_obj == NULL && !is_hit) {
		if (scene->GetSkyBoxFlg()) return scene->GetSkyboxColor(ray);
		else return scene->GetBackgroundColor();
	}
	
	hit_norm = hit_obj->getNormal(hit_pnt);

	// If ray is inside the object
	if (hit_norm * ray.direction > 0) {
		hit_norm *= -1.0f;
		r_inside = true;
	}

	exact_hit_pnt = hit_pnt + hit_norm * SHADOW_BIAS;
	//hit_norm = hit_norm.normalize();
	mat = hit_obj->GetMaterial();

	color += calculateLightContribution(ray, exact_hit_pnt, hit_norm, mat);
	
	if (depth == MAX_DEPTH) {
		return color;
	}

	float Kr = mat->GetReflection();
	float transmittance = mat->GetTransmittance();

	if (transmittance != 0) {
		float cos_t_i = -(hit_norm * ray.direction);
		//If r_inside == true, then the outside material is Air, thus ior = 1.0f 
		if (r_inside) ior_2 = 1.0f;
		else ior_2 = mat->GetRefrIndex();

		Kr = schlickApproximation(cos_t_i, ior_1, ior_2);
		v_t = hit_norm * cos_t_i + ray.direction;
		float sin_t_t = (ior_1 / ior_2) * v_t.length();
		v_t = v_t.normalize();
		
		if (sin_t_t < 1) {
			float cos_t_t = sqrt( 1 - pow(sin_t_t, 2) );
			refraction = v_t * sin_t_t - hit_norm * cos_t_t;
			refraction = refraction.normalize();
			Ray refr_ray = Ray((hit_pnt - hit_norm * SHADOW_BIAS), refraction);
			color += rayTracing(refr_ray, depth + 1, ior_2) * (1 - Kr);
		}
		else {
			Kr = 1.0f;
		}
	}

	if (mat->GetReflection() > 0) {
		reflection = ray.direction - hit_norm * (ray.direction * hit_norm) * 2;

		if (FUZZY_REFLECTIONS) {
			Ray refl_ray = Ray(exact_hit_pnt, (reflection + (rnd_unit_sphere() * ROUGHNESS)).normalize());
			color += rayTracing(refl_ray, depth + 1, ior_1) * Kr;
		}

		else {
			Ray refl_ray = Ray(exact_hit_pnt, reflection);
			color += rayTracing(refl_ray, depth + 1, ior_1) * Kr;
		}
	}

	return color;
}


// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ)); //Camera motion
	}

	// Set random seed for this iteration
	set_rand_seed(time(NULL)); 

	Ray* ray = nullptr;

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;
			Vector pixel; //viewport coordinates

			if (!ANTIALIASING) {
				pixel.x = x + 0.5f;
				pixel.y = y + 0.5f;

				if (DEPTH_OF_FIELD) {
					float aperture = scene->GetCamera()->GetAperture();
					Vector lens_sample = rnd_unit_disk() * aperture;
					ray = &scene->GetCamera()->PrimaryRay(lens_sample, pixel);

				}
				else {
					ray = &scene->GetCamera()->PrimaryRay(pixel);
				}

				color = rayTracing(*ray, 1, 1.0).clamp();

			}
			else {
				for (int k = 0; k < SPP; k++) {
					for (int j = 0; j < SPP; j++) {
						off_x = k;
						off_y = j;
						pixel.x = x + (k + rand_float()) / SPP;
						pixel.y = y + (j + rand_float()) / SPP;

						if (DEPTH_OF_FIELD) {
							float aperture = scene->GetCamera()->GetAperture();
							Vector lens_sample = rnd_unit_disk() * aperture;

							ray = &scene->GetCamera()->PrimaryRay(lens_sample, pixel);
						}
						else {
							ray = &scene->GetCamera()->PrimaryRay(pixel);
						}

						color += rayTracing(*ray, 1, 1.0).clamp();
					}
				}
				color = color / pow(SPP, 2);

			}

			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}
		}

	}
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}
void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}
	else
		printf("No acceleration data structure.\n\n");

	unsigned int spp = scene->GetSamplesPerPixel();
	if (spp == 0)
		printf("Whitted Ray-Tracing\n");
	else
		printf("Distribution Ray-Tracing\n");

}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int 
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////