#ifndef Point_h
#define Point_h

#include "point.h"
#include <stdio.h>
#include <math.h>

class Point{
	public:
		double x;
		double y;

		Point(double x=0, double y=0);
		Point(const Point& point);
		double change_to(double x, double y);
		double dist(double x, double y);
		double dist(Point point);
		double print();
};

#endif
