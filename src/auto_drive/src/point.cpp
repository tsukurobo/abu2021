#include "point.h"
#include <stdio.h>
#include <math.h>

//コンストラクタ
Point::Point(double x, double y){
	this->x = x;
	this->y = y;
}

//コピーコンストラクタ（vector用）
Point::Point(const Point& point){
	this->x = point.x;
	this->y = point.y;
}

//2点間距離
double Point::dist(double x, double y){
	return sqrt(pow(x - this->x, 2) + pow(y - this->y, 2));
}

//表示
double Point::print(){
	printf("x:%f\ty:%f\n", x, y);
}
