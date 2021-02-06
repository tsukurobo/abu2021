#include <fstream>
#include <vector>
#include <string>
#include "point.h"

//global variable
const double MAX_SPEED = 2.5;

//function prototype
int load_csv(std::vector<Point> *vct, std::string file_name);

//main
int main(int argc, char const* argv[]){
	std::vector<Point> path;
	load_csv(&path, "../pathes/hoge4.csv");


	for(int i=0; i<path.size(); i++){
		path.at(i).print();
	}
	return 0;
}

int load_csv(std::vector<Point> *vct, std::string file_name){
	std::ifstream file(file_name);

	// check file open
	if(file.fail()){
		printf("Failed to open file\n");
		return -1;
	}

	// get file
	while(1){
		std::string line_x;
		std::string line_y;

		//取得
		getline(file, line_x, ',');
		getline(file, line_y);

		//ファイル終端判定
		if(file.eof()) break;

		//pathに追加
		Point tmp(stod(line_x), stod(line_y));
		vct->push_back(tmp);
	}

	file.close();

	return 0;
}

