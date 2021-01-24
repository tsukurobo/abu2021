#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from math import *
import csv
from plot_fig import Plot_fig as PF


class Path:
	def __init__(self):
		self.POINT_GAP = POINT_GAP # 直線上の経路点列の幅[m]

		self.path_x = []
		self.path_y = []
	
	# 経路点列に点を追加
	def add_point(self, x, y, col='r'):
		# 点描画
		pf.make_point(x, y, col)
		# 点追加
		self.path_x.append(x)
		self.path_y.append(y)

	# 経路点列に直線上の点列を追加
	def add_line(self, x1, y1, x2, y2, col='r'):
		# 2点間の線分表示
		pf.make_line(x1, y1, x2, y2, col)

		# 距離，傾きの計算
		dist = sqrt((x2-x1)**2 + (y2-y1)**2)
		slope = atan2(y2-y1, x2-x1)
		
		# 等間隔点列生成＆表示
		x = []
		y = []
		for i in range(int(dist/self.POINT_GAP) + 1):
			# 点列生成
			x.append(x1 + self.POINT_GAP*cos(slope)*i)
			y.append(y1 + self.POINT_GAP*sin(slope)*i)
			# 点列を経路に追加
			self.path_x.append(x[i])
			self.path_y.append(y[i])
			# 点列表示
			pf.make_point(x[i],y[i],col) 

	# CSVファイル出力
	def make_csv(self, filename):
		with open(filename,'w') as csvfile:
			writer = csv.writer(csvfile, lineterminator='\n')

			for i in range(len(self.path_x) - 1):
				writer.writerow([self.path_x[i], self.path_y[i]])

# 直線上の経路点列の幅[m]
POINT_GAP = 0.1

# 座標
P_TR_START = (11.4, 0.5)
P_DR_START = (0.5, 5.425)
P_DR_RETRY = (5.425, 2.45)

pf = PF()

def main():
	pf.make_point(*P_TR_START,'r')

	path1 = Path()
	path1.add_line(*P_DR_START, *(2, 2), 'b')
	path1.add_line(*(2, 2), *(5.4, 1), 'b')
	path1.add_line(*(5.4, 1), *P_DR_RETRY, 'b')
	path1.add_line(*P_DR_RETRY,*(5.425, 5.4), 'b')
	path1.add_point(*(5.425, 5.4), 'b')

	path1.make_csv('pathes/hoge4.csv')

	pf.show()



if __name__ == '__main__':
	main()
