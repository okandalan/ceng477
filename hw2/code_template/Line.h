#ifndef _LINE_H_
#define _LINE_H_
#include "Vec4.h"
#include "Color.h"

class Line {
public:
	Vec4 v0, v1;
    bool visible = false;
    Color colorV0;
    Color colorV1;

	Line(Vec4 v0, Vec4 v1);
	Line(const Line &other);
};

#endif