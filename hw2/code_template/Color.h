#ifndef __COLOR_H__
#define __COLOR_H__

class Color
{
public:
    double r, g, b;

    Color();
    Color(double r, double g, double b);
    Color(const Color &other);
    Color operator*(double scalar) const;
    Color operator+(const Color& other) const;
    friend std::ostream &operator<<(std::ostream &os, const Color &c);
};

#endif