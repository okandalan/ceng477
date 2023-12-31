#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "tinyxml2.h"
#include "Triangle.h"
#include "Helpers.h"
#include "Scene.h"

using namespace tinyxml2;
using namespace std;

//Transformation helper functions
Matrix4 translationMatrix(double tx, double ty, double tz)
{
	Matrix4 translationMatrix = getIdentityMatrix();
	translationMatrix.values[0][3] = tx;
	translationMatrix.values[1][3] = ty;
	translationMatrix.values[2][3] = tz;

	return translationMatrix;
}

Matrix4 transposeMatrix4(Matrix4 m)
{
	Matrix4 m_transpose;

	for (int i=0; i < 4; i++){
		for (int j=0;  j < 4; j++){
			m_transpose.values[j][i] = m.values[i][j];
		}
	}
	return m_transpose;
}

Matrix4 rotationMatrix(double angle, double ux, double uy, double uz)
{
	Vec3 v;
	if (abs(ux) <= abs(uy) && abs(ux) <= abs(uz)) {
		v = Vec3(0.0, -uz, uy);
	}
	else if (abs(uy) <= abs(ux) && abs(uy) <= abs(uz)) {
		v = Vec3(-uz, 0.0, ux);
	}
	else if (abs(uz) <= abs(ux) && abs(uz) <= abs(uy)) {
		v = Vec3(-uy, ux, 0.0);
	}

	Vec3 w = crossProductVec3(Vec3(ux, uy, uz), v);
	v = normalizeVec3(v);
	w = normalizeVec3(w);

	Matrix4 m = getIdentityMatrix();

	m.values[0][0] = ux;
	m.values[0][1] = uy;
	m.values[0][2] = uz;
	m.values[1][0] = v.x;
	m.values[1][1] = v.y;
	m.values[1][2] = v.z;
	m.values[2][0] = w.x;
	m.values[2][1] = w.y;
	m.values[2][2] = w.z;

	Matrix4 rotation = getIdentityMatrix();
	rotation.values[1][1] = cos(angle * M_PI / 180.0);
	rotation.values[1][2] = -sin(angle * M_PI / 180.0);
	rotation.values[2][1] = sin(angle * M_PI / 180.0);
	rotation.values[2][2] = cos(angle * M_PI / 180.0);
	
	return multiplyMatrixWithMatrix(transposeMatrix4(m), multiplyMatrixWithMatrix(rotation, m));
}

Matrix4 createScaleMatrix(double x, double y, double z) 
{
	Matrix4 scaleMatrix = getIdentityMatrix();
	scaleMatrix.values[0][0] = x;
	scaleMatrix.values[1][1] = y;
	scaleMatrix.values[2][2] = z;

	return scaleMatrix;
}

Matrix4 translateToOriginAndBack(Matrix4 m, double x, double y, double z)
{
	Matrix4 toOrigin = translationMatrix(-x, -y, -z);
	Matrix4 backFromOrigin = translationMatrix(x, y, z);

	return multiplyMatrixWithMatrix(backFromOrigin, multiplyMatrixWithMatrix(m, toOrigin));
}

Matrix4 cameraMatrix(double cx, double cy, double cz, Vec3 u, Vec3 v, Vec3 w)
{
	// Translate the camera to the world origin
	Matrix4 toOrigin = translationMatrix(-cx, -cy, -cz);

	// Rotate uvw to align it with xyz
	Matrix4 m;
	m.values[0][0] = u.x;
	m.values[0][1] = u.y;
	m.values[0][2] = u.z;
	m.values[0][3] = 0.0;
	
	m.values[1][0] = v.x;
	m.values[1][1] = v.y;
	m.values[1][2] = v.z;
	m.values[1][3] = 0.0;
	
	m.values[2][0] = w.x;
	m.values[2][1] = w.y;
	m.values[2][2] = w.z;
	m.values[2][3] = 0.0;

	m.values[3][0] = 0.0;
	m.values[3][1] = 0.0;
	m.values[3][2] = 0.0;
	m.values[3][3] = 1.0;

	return multiplyMatrixWithMatrix(m, toOrigin);
}

Matrix4 perspectiveToOrtographicMatrix(double near, double far)
{
	Matrix4 m = getIdentityMatrix();

	m.values[0][0] = near;
	m.values[1][1] = near;
	m.values[2][2] = far + near;
	m.values[2][3] = far * near;
	m.values[3][2] = -1;
	// Remove the extra 1 that comes from the identity matrix
	m.values[3][3] = 0;
	
	return m;
}

Matrix4 orthographicMatrix(double l, double r, double b, double t, double n, double f)
{
	Matrix4 m = getIdentityMatrix();
	m.values[0][0] = 2. / (r - l);
	m.values[1][1] = 2. / (t - b);
	m.values[2][2] = -2. / (f - n);
	m.values[0][3] = -(r + l) / (r - l);
	m.values[1][3] = -(t + b) / (t - b);
	m.values[2][3] = -(f + n) / (f - n);

	return m;
}

Vec4 perspectiveDivide(Vec4 v)
{
	v.x /= v.t;
	v.y /= v.t;
	v.z /= v.t;
	v.t = 1;
	
	return v;
}

// Check for bugs
//TODO normalde ders slaytlarında bu matrix 3x4 çünkü Homogeneous coordinatelara artık ihtiyacımız yok
// ama ben matrix3 yok diye böyle yaptım bu matrixi kullandıktan sonraki sonuçlarda HC'yi atarız vertexlerden
//TODO ödevde sanırım bu xmin ymin kısmı yok ama dursun şimdilik
Matrix4 viewportMatrix(int nx, int ny, double xmin = 0, double ymin = 0)
{
	Matrix4 m = getIdentityMatrix();
	m.values[0][0] = nx / 2.0;
	m.values[1][1] = ny / 2.0;
	m.values[2][2] = 1 / 2.0;
	m.values[0][3] = ((nx - 1) / 2.0) + xmin;
	m.values[1][3] = ((ny - 1) / 2.0) + ymin;
	m.values[2][3] = 1 / 2.0;

	return m;
}

//Rasterization functions
Color roundColor(Color c)
{
	c.r = round(c.r);
	c.g = round(c.g);
	c.b = round(c.b);
	
	return c; 
}

void Scene::lineRasterization(int x0, int y0, double z0, Color c0, int x1, int y1, double z1, Color c1, Camera* c)
{
	int x_inc = 0;
	int y_inc = 0;
	int x_index = x0;
	int y_index = y0;
	double slope = double(y1 - y0) / (x1 - x0);
	double depth = z0;
	Color tmp_c = c0;
	double d;
	double alpha;
	// There are four cases for line direction thus I init these coefficients.
	if (x0 < x1)
		x_inc = 1;
	else
		x_inc = -1;
	if (y0 < y1)
		y_inc = 1;
	else
		y_inc = -1;

	if ((y0 == y1) || (abs(slope) <= 1)) {
		d = x_inc * (y0 - y1) + y_inc * 0.5 * (x1 - x0);
		for (; x_index != x1 + x_inc; x_index += x_inc) {
			alpha = double(x_index - x0) / (x1 - x0);
			depth = (1 - alpha) * z0 + alpha * z1;
			if (depth < this->depth[x_index][y_index] && x_index < c->horRes && y_index < c->verRes) {
				tmp_c.r = (double) makeBetweenZeroAnd255((1 - alpha) * c0.r + alpha * c1.r); 
				tmp_c.g = (double) makeBetweenZeroAnd255((1 - alpha) * c0.g + alpha * c1.g); 
				tmp_c.b = (double) makeBetweenZeroAnd255((1 - alpha) * c0.b + alpha * c1.b);
				this->assignColorToPixel(x_index, y_index, roundColor(tmp_c));
			}

			if ((slope > 0 && d < 0) || (slope < 0 && d > 0)) {
				y_index += y_inc;
				d += x_inc * (y0 - y1) + y_inc * (x1 - x0);
			}
			else {
				d += x_inc * (y0 - y1);
			}
		}
	}

	if ((x0 == x1) || (abs(slope) > 1)) {
		d = x_inc * 0.5 * (y0 - y1) + y_inc * (x1 - x0);
		for (; y_index != y1 + y_inc; y_index += y_inc) {
			alpha = double(y_index - y0) / (y1 - y0);
			depth = (1 - alpha) * z0 + alpha * z1;
			if (depth < this->depth[x_index][y_index] && x_index < c->horRes && y_index < c->verRes) {
				tmp_c.r = (1 - alpha) * c0.r + alpha * c1.r; 
				tmp_c.g = (1 - alpha) * c0.g + alpha * c1.g; 
				tmp_c.b = (1 - alpha) * c0.b + alpha * c1.b;
				this->assignColorToPixel(x_index, y_index, roundColor(tmp_c));
			}

			if ((slope > 0 && d > 0) || (slope < 0 && d < 0)) {
				x_index += x_inc;
				d += x_inc * (y0 - y1) + y_inc * (x1 - x0);
			}
			else {
				d += y_inc * (x1 - x0);
			}
		}
	}

}

void Scene::triangleRasterization(int x0, int y0, double z0, Color c0, int x1, int y1, double z1, Color c1, int x2, int y2, double z2, Color c2, Camera *c)
{
	int x_min, x_max, y_min, y_max;
	
	x_min = min(x0, min(x1, x2));
	x_max = max(x0, max(x1, x2));
	y_min = min(y0, min(y1, y2));
	y_max = max(y0, max(y1, y2));

	int f_0_1 = lineEquationF(x2, y2, x0, y0, x1, y1); 
	int f_1_2 = lineEquationF(x0, y0, x1, y1, x2, y2);
	int f_2_0 = lineEquationF(x1, y1, x2, y2, x0, y0);
	double alpha, beta, gama, depthCalc;

	for (int y = y_min; y <= y_max; y++) {
		for (int x = x_min; x <= x_max; x++) {
			
			alpha = double(lineEquationF(x, y, x1, y1, x2, y2)) / f_1_2;
			beta = double(lineEquationF(x, y, x2, y2, x0, y0)) / f_2_0;
			gama = double(lineEquationF(x, y, x0, y0, x1, y1)) / f_0_1;
			depthCalc = (z0 * alpha) + (z1 * beta) + (z2 * gama);

			if (alpha >= 0 && beta >= 0 && gama >= 0) {
				// If the fragment is at the front, color it
				if (depthCalc <= this->depth[x][y]) {
					
					// Update the depth buffer
					this->depth[x][y] = depthCalc;
					
					// Calculate the interpolated color
					Color interpolatedColor;

					interpolatedColor.r = (double) makeBetweenZeroAnd255(alpha * c0.r + beta * c1.r + gama * c2.r);
					interpolatedColor.g = (double) makeBetweenZeroAnd255(alpha * c0.g + beta * c1.g + gama * c2.g);
					interpolatedColor.b = (double) makeBetweenZeroAnd255(alpha * c0.b + beta * c1.b + gama * c2.b);

					assignColorToPixel(x, y, interpolatedColor);
				}
			}
		}
	}

}

bool isBack(Vec3 v1, Vec3 v2, Vec3 v3, Vec3 camera)
{
	Vec3 v1_to_v2 = v2 - v1;
	Vec3 v1_to_v3 = v3 - v1;
	Vec3 normal = crossProductVec3(v1_to_v2, v1_to_v3);
	normal = normalizeVec3(normal);

	Vec3 ray = normalizeVec3(v1 - camera);

	//TODO belki >= olması gerekebilir
	return dotProductVec3(normal, ray) > 0;
}

bool visible(double den, double num, double& te, double& tl)
{
	double t;
	if (den > 0) {
		t = num / den;
		if (t > tl) {
			return false;
		}
		if (t > te) {
			te = t;
		}
	}
	else if (den < 0) {
		t = num / den;
		if (t < te) {
			return false;
		}
		if (t < tl) {
			tl = t;
		}
	}
	else if (num > 0) {
		return false;
	} 
	return true;
}

void Scene::lineClipping(Line& l)
{
	Vec4 initialV0 = l.v0;
	Vec4 initialV1 = l.v1;
	double xMin = -1;
	double xMax = 1;
	double yMin = -1;
	double yMax = 1;
	double zMin = -1;
	double zMax = 1;
	double te = 0.;
	double tl = 1.;
	double dx = l.v1.x - l.v0.x;
	double dy = l.v1.y - l.v0.y;
	double dz = l.v1.z - l.v0.z;
	if (visible(dx, xMin - l.v0.x, te, tl) &&
	    visible(-dx, l.v0.x - xMax, te, tl) &&
		visible(dy, yMin - l.v0.y, te, tl) &&
		visible(-dy, l.v0.y - yMax, te, tl) &&
		visible(dz, zMin - l.v0.z, te, tl) &&
		visible(-dz, l.v0.z - zMax, te, tl)) {
			l.visible = true;
			if (tl < 1) {
				l.v1.x = l.v0.x + (dx * tl);
				l.v1.y = l.v0.y + (dy * tl);
				l.v1.z = l.v0.z + (dz * tl);
			}
			if (te > 0) {
				l.v0.x = l.v0.x + (dx * te);
				l.v0.y = l.v0.y + (dy * te);
				l.v0.z = l.v0.z + (dz * te);
			}
	}

	double distanceV0ToV1 = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
	double distanceV0ToNewV1 = sqrt(pow(l.v1.x - initialV0.x, 2) + pow(l.v1.y - initialV0.y, 2) + pow(l.v1.z - initialV0.z, 2));
	double distanceV0ToNewV0 = sqrt(pow(l.v0.x - initialV0.x, 2) + pow(l.v0.y - initialV0.y, 2) + pow(l.v0.z - initialV0.z, 2));;
	double alpha0 = distanceV0ToNewV0 / distanceV0ToV1;
	double alpha1 = distanceV0ToNewV1 / distanceV0ToV1;
	Color *c0 = this->colorsOfVertices[l.v0.colorId - 1];
	Color *c1 = this->colorsOfVertices[l.v1.colorId - 1];

	l.colorV0.r = (1 - alpha0) * c0->r + alpha0 * c1->r;
	l.colorV0.g = (1 - alpha0) * c0->g + alpha0 * c1->g;
	l.colorV0.b = (1 - alpha0) * c0->b + alpha0 * c1->b;

	l.colorV1.r = (1 - alpha1) * c0->r + alpha1 * c1->r;
	l.colorV1.g = (1 - alpha1) * c0->g + alpha1 * c1->g;
	l.colorV1.b = (1 - alpha1) * c0->b + alpha1 * c1->b;
}

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *xmlElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *rootNode = xmlDoc.FirstChild();

	// read background color
	xmlElement = rootNode->FirstChildElement("BackgroundColor");
	str = xmlElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	xmlElement = rootNode->FirstChildElement("Culling");
	if (xmlElement != NULL)
	{
		str = xmlElement->GetText();

		if (strcmp(str, "enabled") == 0)
		{
			this->cullingEnabled = true;
		}
		else
		{
			this->cullingEnabled = false;
		}
	}

	// read cameras
	xmlElement = rootNode->FirstChildElement("Cameras");
	XMLElement *camElement = xmlElement->FirstChildElement("Camera");
	XMLElement *camFieldElement;
	while (camElement != NULL)
	{
		Camera *camera = new Camera();

		camElement->QueryIntAttribute("id", &camera->cameraId);

		// read projection type
		str = camElement->Attribute("type");

		if (strcmp(str, "orthographic") == 0)
		{
			camera->projectionType = ORTOGRAPHIC_PROJECTION;
		}
		else
		{
			camera->projectionType = PERSPECTIVE_PROJECTION;
		}

		camFieldElement = camElement->FirstChildElement("Position");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->position.x, &camera->position.y, &camera->position.z);

		camFieldElement = camElement->FirstChildElement("Gaze");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->gaze.x, &camera->gaze.y, &camera->gaze.z);

		camFieldElement = camElement->FirstChildElement("Up");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf", &camera->v.x, &camera->v.y, &camera->v.z);

		camera->gaze = normalizeVec3(camera->gaze);
		camera->u = crossProductVec3(camera->gaze, camera->v);
		camera->u = normalizeVec3(camera->u);

		camera->w = inverseVec3(camera->gaze);
		camera->v = crossProductVec3(camera->u, camera->gaze);
		camera->v = normalizeVec3(camera->v);

		camFieldElement = camElement->FirstChildElement("ImagePlane");
		str = camFieldElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &camera->left, &camera->right, &camera->bottom, &camera->top,
			   &camera->near, &camera->far, &camera->horRes, &camera->verRes);

		camFieldElement = camElement->FirstChildElement("OutputName");
		str = camFieldElement->GetText();
		camera->outputFilename = string(str);

		this->cameras.push_back(camera);

		camElement = camElement->NextSiblingElement("Camera");
	}

	// read vertices
	xmlElement = rootNode->FirstChildElement("Vertices");
	XMLElement *vertexElement = xmlElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (vertexElement != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = vertexElement->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = vertexElement->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		this->vertices.push_back(vertex);
		this->colorsOfVertices.push_back(color);

		vertexElement = vertexElement->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	xmlElement = rootNode->FirstChildElement("Translations");
	XMLElement *translationElement = xmlElement->FirstChildElement("Translation");
	while (translationElement != NULL)
	{
		Translation *translation = new Translation();

		translationElement->QueryIntAttribute("id", &translation->translationId);

		str = translationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		this->translations.push_back(translation);

		translationElement = translationElement->NextSiblingElement("Translation");
	}

	// read scalings
	xmlElement = rootNode->FirstChildElement("Scalings");
	XMLElement *scalingElement = xmlElement->FirstChildElement("Scaling");
	while (scalingElement != NULL)
	{
		Scaling *scaling = new Scaling();

		scalingElement->QueryIntAttribute("id", &scaling->scalingId);
		str = scalingElement->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		this->scalings.push_back(scaling);

		scalingElement = scalingElement->NextSiblingElement("Scaling");
	}

	// read rotations
	xmlElement = rootNode->FirstChildElement("Rotations");
	XMLElement *rotationElement = xmlElement->FirstChildElement("Rotation");
	while (rotationElement != NULL)
	{
		Rotation *rotation = new Rotation();

		rotationElement->QueryIntAttribute("id", &rotation->rotationId);
		str = rotationElement->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		this->rotations.push_back(rotation);

		rotationElement = rotationElement->NextSiblingElement("Rotation");
	}

	// read meshes
	xmlElement = rootNode->FirstChildElement("Meshes");

	XMLElement *meshElement = xmlElement->FirstChildElement("Mesh");
	while (meshElement != NULL)
	{
		Mesh *mesh = new Mesh();

		meshElement->QueryIntAttribute("id", &mesh->meshId);

		// read projection type
		str = meshElement->Attribute("type");

		if (strcmp(str, "wireframe") == 0)
		{
			mesh->type = WIREFRAME_MESH;
		}
		else
		{
			mesh->type = SOLID_MESH;
		}

		// read mesh transformations
		XMLElement *meshTransformationsElement = meshElement->FirstChildElement("Transformations");
		XMLElement *meshTransformationElement = meshTransformationsElement->FirstChildElement("Transformation");

		while (meshTransformationElement != NULL)
		{
			char transformationType;
			int transformationId;

			str = meshTransformationElement->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			mesh->transformationTypes.push_back(transformationType);
			mesh->transformationIds.push_back(transformationId);

			meshTransformationElement = meshTransformationElement->NextSiblingElement("Transformation");
		}

		mesh->numberOfTransformations = mesh->transformationIds.size();

		// read mesh faces
		char *row;
		char *cloneStr;
		int v1, v2, v3;
		XMLElement *meshFacesElement = meshElement->FirstChildElement("Faces");
		str = meshFacesElement->GetText();
		cloneStr = strdup(str);

		row = strtok(cloneStr, "\n");
		while (row != NULL)
		{
			int result = sscanf(row, "%d %d %d", &v1, &v2, &v3);

			if (result != EOF)
			{
				mesh->triangles.push_back(Triangle(v1, v2, v3));
			}
			row = strtok(NULL, "\n");
		}
		mesh->numberOfTriangles = mesh->triangles.size();
		this->meshes.push_back(mesh);

		meshElement = meshElement->NextSiblingElement("Mesh");
	}
}

void Scene::assignColorToPixel(int i, int j, Color c)
{
	this->image[i][j].r = c.r;
	this->image[i][j].g = c.g;
	this->image[i][j].b = c.b;
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;
			vector<double> rowOfDepths;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
				rowOfDepths.push_back(1.01);
			}

			this->image.push_back(rowOfColors);
			this->depth.push_back(rowOfDepths);
		}
	}
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				assignColorToPixel(i, j, this->backgroundColor);
				this->depth[i][j] = 1.01;
				this->depth[i][j] = 1.01;
				this->depth[i][j] = 1.01;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFilename.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFilename << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
*/
void Scene::convertPPMToPNG(string ppmFileName)
{
	string command;

	// TODO: Change implementation if necessary.
	command = "./magick convert " + ppmFileName + " " + ppmFileName + ".png";
	system(command.c_str());
}

void processViewportVertex(Vec4& viewportVertex, Camera * camera)
{
	if (viewportVertex.x < 0) {
		viewportVertex.x = 0.0;
	}

	else if (viewportVertex.x >= camera->horRes - 0.5) {
		viewportVertex.x = camera->horRes - 1;
	}

	else {
		// Round x coordinate to obtain 2D integer viewport pixel coordinates
		viewportVertex.x = (double) round(viewportVertex.x);
	}

	if (viewportVertex.y < 0) {
		viewportVertex.y = 0.0;
	}

	else if (viewportVertex.y >= camera->verRes - 0.5) {
		viewportVertex.y = camera->verRes - 1;
	}

	else {
		// Round y coordinate to obtain 2D integer viewport pixel coordinates
		viewportVertex.y = (double) round(viewportVertex.y);
	}

}

/*
	Transformations, clipping, culling, rasterization are done here.
*/
void Scene::forwardRenderingPipeline(Camera *camera)
{
	// TODO: Implement this function

	// The same viewing transformation matrix will be used for all meshes, so compute it once
	Matrix4 viewingTransformMatrix;

	// projectionType == 1, apply perspective projection
	if (camera->projectionType) {

		viewingTransformMatrix = multiplyMatrixWithMatrix(orthographicMatrix(camera->left, camera->right, camera->bottom, camera->top, camera->near, camera->far), 
														multiplyMatrixWithMatrix(perspectiveToOrtographicMatrix(camera->near, camera->far), cameraMatrix(camera->position.x, camera->position.y, camera->position.z, camera->u, camera->v, camera->w)));
	}

	// projectionType == 0, apply orthographic projection
	else {
		
		viewingTransformMatrix = multiplyMatrixWithMatrix(orthographicMatrix(camera->left, camera->right, camera->bottom, camera->top, camera->near, camera->far), 
															  cameraMatrix(camera->position.x, camera->position.y, camera->position.z, camera->u, camera->v, camera->w));
	}

	// Traverse over all meshes to compute their own versions of vertices after applying modeling and viewing transformations
	for (int i = 0; i < this->meshes.size(); i++) {

		Mesh mesh = *(this->meshes[i]);

		// Calculate the composite modeling transformation matrix for this specific mesh
		if (mesh.numberOfTransformations > 0) {

			Matrix4 compositeModeling = getIdentityMatrix();

			// For each modeling transformation, multiply the previously obtained matrix from the right with the corresponding matrix of the transformation
			for (int j = mesh.numberOfTransformations - 1; j >= 0; j--) {

				int transformationId = mesh.transformationIds[j];
				char transformationType = mesh.transformationTypes[j];

				switch (transformationType) {
					case 't': {
						Translation *translation = this->translations[transformationId - 1];

						// Multiply the previous composite matrix with the translation matrix in order to find the new composite matrix
						compositeModeling = multiplyMatrixWithMatrix(compositeModeling, translationMatrix(translation->tx, translation->ty, translation->tz));
						break;
					}
						
					case 's': {
						Scaling *scaling = this->scalings[transformationId - 1];

						// Multiply the previous composite matrix with the scaling matrix in order to find the new composite matrix
						compositeModeling = multiplyMatrixWithMatrix(compositeModeling, createScaleMatrix(scaling->sx, scaling->sy, scaling->sz));
						break;
					}

					case 'r': {
						Rotation *rotation = this->rotations[transformationId - 1];

						// Multiply the previous composite matrix with the rotation matrix in order to find the new composite matrix
						compositeModeling = multiplyMatrixWithMatrix(compositeModeling, rotationMatrix(rotation->angle, rotation->ux, rotation->uy, rotation->uz));
						break;
					}
				}
			}

			// Now, to every vertex in the scene, apply the composite modeling transformation and store these transformed vertices inside the mesh
			for (int j = 0; j < this->vertices.size(); j++) {

				Vec3 *originalVertex = this->vertices[j];
				Vec4 transformedVertex;

				transformedVertex.x = originalVertex->x;
				transformedVertex.y = originalVertex->y;
				transformedVertex.z = originalVertex->z;
				transformedVertex.t = 1.0;
				transformedVertex.colorId = originalVertex->colorId;

				// Multiply the composite modeling matrix with the vector with homogeneous coordinates
				transformedVertex = multiplyMatrixWithVec4(compositeModeling, transformedVertex);

				// Add the transformed vertex to the std::vector of the mesh
				mesh.transformedVertices.push_back(transformedVertex);
			}
		}

		// There are no modeling transformations specified for this mesh, simply deep-copy the original vertices in the scene
		else {

			// Deep-copy every vertex in the scene into the std::vector of the mesh
			for (int j = 0; j < this->vertices.size(); j++) {

				Vec3 *originalVertex = this->vertices[j];
				Vec4 homogeneousVertex;

				homogeneousVertex.x = originalVertex->x;
				homogeneousVertex.y = originalVertex->y;
				homogeneousVertex.z = originalVertex->z;
				homogeneousVertex.t = 1.0;
				homogeneousVertex.colorId = originalVertex->colorId;

				// Add the vertex with homogeneous coordinates into the std::vector of the mesh
				mesh.transformedVertices.push_back(homogeneousVertex);
			}
		}

		// MODELING TRANSFORMATIONS ARE DONE, MOVING ONTO THE VIEWING TRANSFORMATIONS

		// Apply camera transformation and projection transformation to each transformed vertex of the mesh
		for (int j = 0; j < mesh.transformedVertices.size(); j++) {

			Vec4 viewTransformedVertex = multiplyMatrixWithVec4(viewingTransformMatrix, mesh.transformedVertices[j]);

			mesh.transformedVertices[j].x = viewTransformedVertex.x;
			mesh.transformedVertices[j].y = viewTransformedVertex.y;
			mesh.transformedVertices[j].z = viewTransformedVertex.z;
			mesh.transformedVertices[j].t = viewTransformedVertex.t;
		}

		// VIEWING TRANSFORMATIONS ARE DONE, MOVING ONTO PRIMITIVE (LINE) ASSEMBLY

		// type == 0 (wireframe mode), compute the edges from the given triangles and store them in a vector inside the mesh, for clipping purposes
		if (!mesh.type) {

			// For each triangle of the mesh, add 3 constituent edges to the lines vector of the mesh
			for (int j = 0; j < mesh.triangles.size(); j++) {

				Triangle triangle = mesh.triangles[j];
				Vec4 a = mesh.transformedVertices[triangle.vertexIds[0] - 1];
				Vec4 b = mesh.transformedVertices[triangle.vertexIds[1] - 1];
				Vec4 c = mesh.transformedVertices[triangle.vertexIds[2] - 1];


				if (isBack(Vec3(a.x, a.y, a.z), Vec3(b.x, b.y, b.z), Vec3(c.x, c.y, c.z), camera->position)) {
					// Set the willBeCulled field of the triangle to true
					continue;
				}

				Line line1(a, b);
				Line line2(b, c);
				Line line3(c, a);


				line1.v0 = perspectiveDivide(line1.v0);
				line1.v1 = perspectiveDivide(line1.v1);
				line2.v0 = perspectiveDivide(line2.v0);
				line2.v1 = perspectiveDivide(line2.v1);
				line3.v0 = perspectiveDivide(line3.v0);
				line3.v1 = perspectiveDivide(line3.v1);

				lineClipping(line1);
				lineClipping(line2);
				lineClipping(line3);

				Matrix4 viewportTransform = viewportMatrix(camera->horRes, camera->verRes);

				// Transform the coordinates of the vertices that belong to the mesh to viewport coordinates

				Vec4 viewportLine1Vertex0 = multiplyMatrixWithVec4(viewportTransform, line1.v0);
				Vec4 viewportLine1Vertex1 = multiplyMatrixWithVec4(viewportTransform, line1.v1);
				Vec4 viewportLine2Vertex0 = multiplyMatrixWithVec4(viewportTransform, line2.v0);
				Vec4 viewportLine2Vertex1 = multiplyMatrixWithVec4(viewportTransform, line2.v1);
				Vec4 viewportLine3Vertex0 = multiplyMatrixWithVec4(viewportTransform, line3.v0);
				Vec4 viewportLine3Vertex1 = multiplyMatrixWithVec4(viewportTransform, line3.v1);
				processViewportVertex(viewportLine1Vertex0, camera);
				processViewportVertex(viewportLine1Vertex1, camera);
				processViewportVertex(viewportLine2Vertex0, camera);
				processViewportVertex(viewportLine2Vertex1, camera);
				processViewportVertex(viewportLine3Vertex0, camera);
				processViewportVertex(viewportLine3Vertex1, camera);

				if (line1.visible){
					lineRasterization(viewportLine1Vertex0.x, viewportLine1Vertex0.y, viewportLine1Vertex0.z, line1.colorV0,
									  viewportLine1Vertex1.x, viewportLine1Vertex1.y, viewportLine1Vertex1.z, line1.colorV1,
									  camera);
				}
				if (line2.visible){
					lineRasterization(viewportLine2Vertex0.x, viewportLine2Vertex0.y, viewportLine2Vertex0.z, line2.colorV0,
									  viewportLine2Vertex1.x, viewportLine2Vertex1.y, viewportLine2Vertex1.z, line2.colorV1,
									  camera);
				}
				if (line3.visible){
					lineRasterization(viewportLine3Vertex0.x, viewportLine3Vertex0.y, viewportLine3Vertex0.z, line3.colorV0,
									  viewportLine3Vertex1.x, viewportLine3Vertex1.y, viewportLine3Vertex1.z, line3.colorV1,
									  camera);
				}

					// This vector consists of floating point x,y,z, and w coordinates, where w = 1
			}


		}

		// type == 1 (solid mode), DO NOT PERFORM CLIPPING
		else {

			// Perform perspective divide, as the last component of vertices may have values other than 1, due to perspective projection
			for (int j = 0; j < mesh.transformedVertices.size(); j++) {
				mesh.transformedVertices[j] = perspectiveDivide(mesh.transformedVertices[j]);
			}

			// Culling is enabled, perform back-face culling
			if (this->cullingEnabled) {

				// Traverse all triangles of the mesh and mark the back-facing ones as to-be-culled
				for (int j = 0; j < mesh.triangles.size(); j++) {

					Triangle triangle = mesh.triangles[j];
					Vec4 a = mesh.transformedVertices[triangle.vertexIds[0] - 1];
					Vec4 b = mesh.transformedVertices[triangle.vertexIds[1] - 1];
					Vec4 c = mesh.transformedVertices[triangle.vertexIds[2] - 1];

					if (isBack(Vec3(a.x, a.y, a.z), Vec3(b.x, b.y, b.z), Vec3(c.x, c.y, c.z), camera->position)) {
						// Set the willBeCulled field of the triangle to true
						mesh.triangles[j].willBeCulled = true;
					}
				}
			}

			// CULLING IS DONE (IF IT WAS ENABLED), CONTINUE WITH THE VIEWPORT TRANSFORMATION

			// Note that in this implementation, viewport transformation matrix is 4x4, with the last row being [0 0 0 1]
			Matrix4 viewportTransform = viewportMatrix(camera->horRes, camera->verRes);

			// Transform the coordinates of the vertices that belong to the mesh to viewport coordinates
			for (int j = 0; j < mesh.transformedVertices.size(); j++) {

				// This vector consists of floating point x,y,z, and w coordinates, where w = 1
				Vec4 viewportVertex = multiplyMatrixWithVec4(viewportTransform, mesh.transformedVertices[j]);

				if (viewportVertex.x < 0) {
					viewportVertex.x = 0.0;
				}

				else if (viewportVertex.x >= camera->horRes - 0.5) {
					viewportVertex.x = camera->horRes - 1;
				}

				else {
					// Round x coordinate to obtain 2D integer viewport pixel coordinates
					viewportVertex.x = (double) round(viewportVertex.x);
				}

				if (viewportVertex.y < 0) {
					viewportVertex.y = 0.0;
				}

				else if (viewportVertex.y >= camera->verRes - 0.5) {
					viewportVertex.y = camera->verRes - 1;
				}

				else {
					// Round y coordinate to obtain 2D integer viewport pixel coordinates
					viewportVertex.y = (double) round(viewportVertex.y);
				}

				mesh.transformedVertices[j].x = viewportVertex.x;
				mesh.transformedVertices[j].y = viewportVertex.y;
				mesh.transformedVertices[j].z = viewportVertex.z;
				mesh.transformedVertices[j].t = viewportVertex.t;
			}

			// VIEWPORT TRANSFORMATION IS DONE, CONTINUE WITH TRIANGLE RASTERIZATION

			// Traverse all triangles of the mesh and rasterize fragments (if they are not culled)
			for (int j = 0; j < mesh.triangles.size(); j++) {

				Triangle triangle = mesh.triangles[j];

				// If the triangle is not back-facing
				if (!triangle.willBeCulled) {

					Vec4 a = mesh.transformedVertices[triangle.vertexIds[0] - 1];
					Vec4 b = mesh.transformedVertices[triangle.vertexIds[1] - 1];
					Vec4 c = mesh.transformedVertices[triangle.vertexIds[2] - 1];

					Color colorA = *(this->colorsOfVertices[a.colorId - 1]);
					Color colorB = *(this->colorsOfVertices[b.colorId - 1]);
					Color colorC = *(this->colorsOfVertices[c.colorId - 1]);

					triangleRasterization((int) a.x, (int) a.y, a.z, colorA, (int) b.x, (int) b.y, b.z, colorB, (int) c.x, (int) c.y, c.z, colorC, camera);
				}
			}

		}

	}
}
