#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(111)

DR, = ax.plot(0.5,5.425,'o')

def main():
	init_figure()
	init_field()
	anim = FuncAnimation(fig, update, frames=10, interval=100)
	plt.show()

def update(num):
	DR.set_data(0.5+num,5.425)

def init_figure():
	ax.set_xlim(-0.5,12.5)
	ax.set_ylim(-0.5,10)
	ax.grid()
	ax.set_aspect('equal')

def init_field():
	# frame
	ax.plot([0, 0],[0, 5.925],'k')
	ax.plot([0, 1.95],[5.925, 5.925],'k')
	ax.plot([1.95, 1.95],[5.925, 3.95],'k')
	ax.plot([1.95, 3.95],[3.95, 1.95],'k')
	ax.plot([3.95, 4.925],[1.95, 1.95],'k')
	ax.plot([4.925, 4.925],[1.95, 2],'k')
	ax.plot([4.925, 3.9854],[2, 2],'k')
	ax.plot([3.9854, 2],[2, 3.9854],'k')
	ax.plot([2, 2],[3.9854, 7.9146],'k')
	ax.plot([2, 3.9854],[7.9146, 9.9],'k')
	ax.plot([3.9854, 5.925],[9.9, 9.9],'k')
	ax.plot([5.925, 5.925],[9.9, 1.95],'k')
	ax.plot([5.925, 7.95],[1.95, 1.95],'k')
	ax.plot([7.95, 9.95],[1.95, 3.95],'k')
	ax.plot([9.95, 9.95],[3.95, 5.925],'k')
	ax.plot([9.95, 11.9],[5.925, 5.925],'k')
	ax.plot([11.9,11.9],[5.925, 0],'k')
	ax.plot([11.9, 0],[0, 0],'k')

	# DR Start Zone
	ax.plot([0, 1],[4.925, 4.925],'k')
	ax.plot([0, 1],[5.425, 5.425],'k')
	ax.plot([1, 1],[4.925, 5.925],'k')
	ax.plot([0.5, 0.5],[4.925, 5.925],'k')

	# TR Start Zone
	ax.plot([10.9,11.9],[1, 1],'k')
	ax.plot([10.9,11.9],[0.5, 0.5],'k')
	ax.plot([10.9,10.9],[0, 1],'k')
	ax.plot([11.4,11.4],[0, 1],'k')

	# Arrow Rack
	ax.plot([0, 1],[0.5, 0.5],'k')
	ax.plot([1, 1],[0, 0.5],'k')

	# DR Retry Zone
	ax.plot([4.925, 5.925],[1.95, 1.95],'k')
	ax.plot([4.925, 5.925],[2.95, 2.95],'k')
	ax.plot([4.925, 5.925],[2.45, 2.45],'k')
	ax.plot([4.925, 4.925],[1.95, 2.95],'k')
	ax.plot([5.425, 5.425],[1.95, 2.95],'k')
	
	# Others Guid Lines
	ax.plot([0, 11.9],[1.5, 1.5],'k')
	ax.plot([0.5, 0.5],[0.5, 1.5],'k')
	ax.plot([1, 1.95],[5.425, 5.425],'k')
	ax.plot([10.4, 10.9],[0.5, 0.5],'k')
	ax.plot([11.4, 11.4],[1, 1.5],'k')
	ax.plot([9.95, 11.9],[5.425, 5.425],'k')
	ax.plot([1.5, 1.5],[0, 5.925],'k')
	ax.plot([10.4, 10.4],[0, 5.925],'k')
	ax.plot([5.425, 5.425],[1.5, 1.95],'k')
	

if __name__ == '__main__':
	main()
