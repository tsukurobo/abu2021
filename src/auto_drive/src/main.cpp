#include <vector>
#include "point.h"
#include "pure_pursuit.h"

//global variable
const double MAX_SPEED = 2.5;
const int AHEAD_NUM = 10;

int main(int argc, char const* argv[]){
	Pure_pursuit pp("../pathes/hoge4.csv", AHEAD_NUM);

	Point state(3,5);
	pp.print_path();

	printf("\n");
	pp.target_point(state).print();
	return 0;
}
