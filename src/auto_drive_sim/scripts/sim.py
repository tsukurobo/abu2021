#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from math import *
import csv
from matplotlib.animation import FuncAnimation
from plot_fig import Plot_fig as PF


# アニメーション（継承クラス）
class Animation(PF):
	def __init__(self, frames, interval):
		super().__init__()
		self.FRAMES = frames # 最大フレーム数
		self.INTERVAL = interval # インターバル [ms]

		self.col = None
		self.trace_x = []
		self.trace_y = []

	def set_robo(self, x, y, col, mkr):
		self.col = col
		self.robo, = self.ax.plot(x, y, col, marker=mkr, zorder=4)

	def set_trace(self, x, y):
		self.trace_x = x
		self.trace_y = y

	def set_frames(self, frames):
		self.FRAMES = frames

	def update(self, num):
		# 現在地点表示
		self.robo.set_data(self.trace_x[num], self.trace_y[num])
		# 軌跡表示
		if num > 0:
			self.make_line(self.trace_x[num], self.trace_y[num], 
					self.trace_x[num-1], self.trace_y[num-1], self.col)

	def run_animation(self):
		return FuncAnimation(self.fig, self.update, frames=self.FRAMES, interval=self.INTERVAL)

# 経路点列
class Target_path:
	def __init__(self, filename,col):
		self.x = []
		self.y = []

		self._read_csv(filename)
		self._plot_path(col)

	# 経路点列表示
	def _plot_path(self, col):
		for x, y in zip(self.x, self.y):
			pf.make_point(x, y, col, zorder=2)

	# CSV読み込み
	def _read_csv(self, filename):
		with open(filename,'r') as csvfile:
			reader = csv.reader(csvfile)
			for lin in reader:
				self.x.append(float(lin[0]))
				self.y.append(float(lin[1]))

# ロボット状態
class State:
	def __init__(self, x=0, y=0, yaw=0, col='r', speed=2, frame_param=1/2, fin = 0.01):
		# 状態
		self.x = x
		self.y = y
		self.yaw = yaw
		self.col = col
		self.speed = speed
		self.frame_param = frame_param
		self.fin = fin

		# 追従データ
		self.path = None
		self.ahead_n = 0
		self.target_x = 0
		self.target_y = 0

		# 軌跡
		self.trace_x = [x]
		self.trace_y = [y]

		pf.set_robo(x, y, col, 'o')

	# 経路設定
	def load_pursuit_data(self, path, ahead_n):
		self.path = path
		self.ahead_n = ahead_n

	# 更新
	def update(self):
		self.get_target_point() # 目標点設定
		# 更新
		self.yaw = atan2(self.target_y - self.y, self.target_x - self.x)
		self.x += self.speed * cos(self.yaw) * DT
		self.y += self.speed * sin(self.yaw) * DT
		# 軌跡追加
		self.trace_x.append(self.x)
		self.trace_y.append(self.y)

	# 位置・フレーム描画
	def plot(self):
		pf.make_point(self.x, self.y, self.col, zorder=3)
		self._plot_body()

	# 終了判定
	def finish_judge(self):
		path = self.path
		dist = sqrt((self.x - path.x[len(path.x)-1])**2 + (self.y - path.y[len(path.y)-1])**2)
		if dist < self.fin:
			return False
		else:
			return True

	# 目標点（経路点列の内，最小距離の点からahead_n個先の点）
	def get_target_point(self):
		# 距離^2
		dist_x = [(self.x - i)**2 for i in self.path.x]
		dist_y = [(self.y - i)**2 for i in self.path.y]
		dist2  = [x+y for (x,y) in zip(dist_x,dist_y)]
		ind = dist2.index(min(dist2)) # 最小距離点のインデックス
		# 目標点
		if len(self.path.x) > ind + self.ahead_n: 
			tx = self.path.x[ind + self.ahead_n]
			ty = self.path.y[ind + self.ahead_n]
		else:
			tx = self.path.x[len(self.path.x)-1]
			ty = self.path.y[len(self.path.y)-1]

		# 描画
		pf.make_point(tx, ty, 'g', 'x', 3)

		self.target_x = tx
		self.target_y = ty

	# 機体フレーム描画
	def _plot_body(self):
		L = self.frame_param
		x = self.x
		y = self.y
		yaw = self.yaw
		odr = 3 # zorder（プロットの前後）
		# 方向線
		pf.make_line(x, y, x+L/sqrt(2)*cos(yaw), y+L/sqrt(2)*sin(yaw), self.col, zorder=odr)
		# フレーム
		pf.make_line(x+L*cos(yaw+pi/4), y+L*sin(yaw+pi/4),
				x+L*cos(yaw-pi/4), y+L*sin(yaw-pi/4), self.col, zorder=odr)
		pf.make_line(x+L*cos(yaw+pi/4), y+L*sin(yaw+pi/4),
				x+L*cos(yaw+pi*3/4), y+L*sin(yaw+pi*3/4), self.col, zorder=odr)
		pf.make_line(x+L*cos(yaw+pi*3/4), y+L*sin(yaw+pi*3/4),
				x+L*cos(yaw+pi*5/4), y+L*sin(yaw+pi*5/4), self.col, zorder=odr)
		pf.make_line(x+L*cos(yaw+pi*5/4), y+L*sin(yaw+pi*5/4),
				x+L*cos(yaw+pi*7/4), y+L*sin(yaw+pi*7/4), self.col, zorder=odr)

# 座標
P_TR_START = (11.4, 0.5)
P_DR_START = (0.5, 5.425)
P_DR_RETRY = (5.425, 2.45)

# パラメータ
FRAMES = 100 # 最大フレーム数
FRAME_PARAM = 1/2 # 機体フレームパラメータ
AHEAD_NUM = 3 # 最近経路点から何個先の点を目標点にするか
SPEED = 3 # 最大移動速度[m/s] ############機体の向きにかかわらず出せる速度は一定（今後直す予定）
FINISH_RANGE = 0.3 # 終了判定範囲[m]
DT = 0.1 #周期[s]

pf = Animation(FRAMES, DT*1000)

def main():
	path1 = Target_path('pathes/hoge2.csv', 'r')

	robo_D = State(*P_DR_START, -pi/2, 'b', SPEED, FRAME_PARAM, FINISH_RANGE)
	robo_D.load_pursuit_data(path1, AHEAD_NUM)
	
	while robo_D.finish_judge():
		# robo_D.plot()
		robo_D.update()

	

	# アニメーション開始
	pf.set_trace(robo_D.trace_x, robo_D.trace_y)
	pf.set_frames(len(robo_D.trace_x)-1)
	ani = pf.run_animation()
	# ani.save('gif/hoge.gif', writer='pillow')
	pf.show()


if __name__ == '__main__':
	main()
