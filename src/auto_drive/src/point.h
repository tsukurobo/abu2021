#ifndef Point_h
#define Point_h

class Point {
	public:
		double x;
		double y;

		Point(double x=0, double y=0);
		Point(const Point& point);
		double dist(double x, double y);
		double print();
};

#endif
