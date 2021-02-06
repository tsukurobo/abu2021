#ifndef Pure_pursuit_h
#define Pure_pursuit_h

#include "pure_pursuit.h"
#include <fstream> //file read
#include <vector>
#include <string>
#include <algorithm> //min_element
#include "point.h"

class Pure_pursuit{
	public:
		Pure_pursuit(std::string file_name, int ahead_num); //コンストラクタ
		/* double target_dir(Point state); //目標方向(グローバル角度) */
		Point target_point(Point state); //目標点
		int print_path(); //経路点列表示
	
	private:
		std::string file_name; //読み込むCSVファイル名
		int ahead_num; //最近経路点からahead_num個先の点を目標点とする
		std::vector<Point> path; //経路点列

		int load_csv(); //CSVフィアル読み込み

};

#endif
