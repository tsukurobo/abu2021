#include "point.h"

//コンストラクタ
Point::Point(double x, double y){
	this->x = x;
	this->y = y;
}

//コピーコンストラクタ（vector<Point>用）
Point::Point(const Point& point){
	this->x = point.x;
	this->y = point.y;
}

//座標変化
double Point::change_to(double x, double y){
	this->x = x;
	this->y = y;
}

//2点間距離
double Point::dist(double x, double y){
	return sqrt(pow(x - this->x, 2) + pow(y - this->y, 2));
}
double Point::dist(Point point){
	return sqrt(pow(point.x - this->x, 2) + pow(point.y - this->y, 2));
}

//表示
double Point::print(){
	printf("x:%f\ty:%f\n", x, y);
}
