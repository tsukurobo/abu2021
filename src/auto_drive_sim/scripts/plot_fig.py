#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

class Plot_fig:
	def __init__(self):
		self.X_LIM_BOTTOM = -0.5
		self.X_LIM_TOP = 12.5
		self.Y_LIM_BOTTOM = -0.5
		self.Y_LIM_TOP = 10

		self._init_figure()
		self._init_field()

		# 十字線表示
		self.ln_v = plt.axvline(0)
		self.ln_h = plt.axhline(0)
		# クリックプロット
		self.ln, = plt.plot([],[],'o')

		# self.DR, = self.ax.plot(0.5, 5.425, c='b', marker='o')

	# 描画設定
	def _init_figure(self):
		self.fig = plt.figure(figsize=(10,10))
		self.ax = self.fig.add_subplot(111)

		# 設定
		self.ax.set_xlim(self.X_LIM_BOTTOM, self.X_LIM_TOP)
		self.ax.set_ylim(self.Y_LIM_BOTTOM, self.Y_LIM_TOP)
		self.ax.grid()
		self.ax.set_aspect('equal')

		# マウスカーソル
		plt.connect('motion_notify_event', self._motion)

	# マウスカーソル
	def _motion(self, event):  
		x = event.xdata
		y = event.ydata

		# 十字線表示
		self.ln_v.set_xdata(x)
		self.ln_h.set_ydata(y)

		# クリックプロット
		if event.inaxes == self.ax:
			self.ln.set_data(x,y)

		plt.draw()

	# 点描画
	def make_point(self, x=0, y=0, col='r', mrk='o', zorder=1):
		return self.ax.plot(x,y,c=col, marker=mrk, zorder=zorder)

	def make_line(self, x1=0, y1=0, x2=1, y2=1, col='r', zorder=1):
		return self.ax.plot([x1,x2],[y1,y2], col, zorder=zorder)

	# # アニメーション
	# def update(self, num):
	# 	self.dr.set_data(0.5+num,5.425)

	# def make_animation(self):
	# 	return funcanimation(self.fig, self.update, frames=10, interval=100)

	# 図表示
	def show(self):
		plt.show()

	# フィールドデータ
	def _init_field(self):
		# frame
		self.ax.plot([0, 0],[0, 5.925],'k')
		self.ax.plot([0, 1.95],[5.925, 5.925],'k')
		self.ax.plot([1.95, 1.95],[5.925, 3.95],'k')
		self.ax.plot([1.95, 3.95],[3.95, 1.95],'k')
		self.ax.plot([3.95, 4.925],[1.95, 1.95],'k')
		self.ax.plot([4.925, 4.925],[1.95, 2],'k')
		self.ax.plot([4.925, 3.9854],[2, 2],'k')
		self.ax.plot([3.9854, 2],[2, 3.9854],'k')
		self.ax.plot([2, 2],[3.9854, 7.9146],'k')
		self.ax.plot([2, 3.9854],[7.9146, 9.9],'k')
		self.ax.plot([3.9854, 5.925],[9.9, 9.9],'k')
		self.ax.plot([5.925, 5.925],[9.9, 1.95],'k')
		self.ax.plot([5.925, 7.95],[1.95, 1.95],'k')
		self.ax.plot([7.95, 9.95],[1.95, 3.95],'k')
		self.ax.plot([9.95, 9.95],[3.95, 5.925],'k')
		self.ax.plot([9.95, 11.9],[5.925, 5.925],'k')
		self.ax.plot([11.9,11.9],[5.925, 0],'k')
		self.ax.plot([11.9, 0],[0, 0],'k')

		# DR Start Zone
		self.ax.plot([0, 1],[4.925, 4.925],'k')
		self.ax.plot([0, 1],[5.425, 5.425],'k')
		self.ax.plot([1, 1],[4.925, 5.925],'k')
		self.ax.plot([0.5, 0.5],[4.925, 5.925],'k')

		# TR Start Zone
		self.ax.plot([10.9,11.9],[1, 1],'k')
		self.ax.plot([10.9,11.9],[0.5, 0.5],'k')
		self.ax.plot([10.9,10.9],[0, 1],'k')
		self.ax.plot([11.4,11.4],[0, 1],'k')

		# Arrow Rack
		self.ax.plot([0, 1],[0.5, 0.5],'k')
		self.ax.plot([1, 1],[0, 0.5],'k')

		# DR Retry Zone
		self.ax.plot([4.925, 5.925],[1.95, 1.95],'k')
		self.ax.plot([4.925, 5.925],[2.95, 2.95],'k')
		self.ax.plot([4.925, 5.925],[2.45, 2.45],'k')
		self.ax.plot([4.925, 4.925],[1.95, 2.95],'k')
		self.ax.plot([5.425, 5.425],[1.95, 2.95],'k')

		# Outer Area Guid Lines
		self.ax.plot([0, 11.9],[1.5, 1.5],'k')
		self.ax.plot([0.5, 0.5],[0.5, 1.5],'k')
		self.ax.plot([1, 1.95],[5.425, 5.425],'k')
		self.ax.plot([10.4, 10.9],[0.5, 0.5],'k')
		self.ax.plot([11.4, 11.4],[1, 1.5],'k')
		self.ax.plot([9.95, 11.9],[5.425, 5.425],'k')
		self.ax.plot([1.5, 1.5],[0, 5.925],'k')
		self.ax.plot([10.4, 10.4],[0, 5.925],'k')
		self.ax.plot([5.425, 5.425],[1.5, 1.95],'k')

		# Inner Area Guid Lines
		self.ax.plot([4.75, 4.925],[2.45, 2.45],'k')
		self.ax.plot([4.75, 5.925],[3.45, 3.45],'k')
		self.ax.plot([4.75, 5.925],[5.95, 5.95],'k')
		self.ax.plot([4.75, 5.925],[8.45, 8.45],'k')
		self.ax.plot([4.75, 4.75], [2.45, 8.45],'k')
		self.ax.plot([4.75, 3.15], [3.45, 5.05],'k')
		self.ax.plot([4.75, 3.15], [8.45, 6.85],'k')
		self.ax.plot([3.15, 3.15], [5.05, 6.85],'k')
		self.ax.plot([4.75, 3.15], [5.95, 5.95],'k')

		# Table
		self.ax.plot(5.95, 3.45,'k', marker='o')
		self.ax.plot(5.95, 5.95,'k', marker='o')
		self.ax.plot(5.95, 8.45,'k', marker='o')
		self.ax.plot(3.95, 5.95,'k', marker='o')



def main():
	# hoge = Plot_fig()
	# ani = hoge.make_animation()
	# hoge.plot()
	pass

if __name__ == '__main__':
	main()

# マウスカーソル
# https://qiita.com/ceptree/items/c9239ce4e442482769b3
