#include "Line.h"

Line::Line(Vec4 v0, Vec4 v1) {
	this->v0 = v0;
	this->v1 = v1;
}

Line::Line(const Line &other) {
	this->v0 = other.v0;
	this->v1 = other.v1;
}