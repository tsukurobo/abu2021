#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from auto_drive_sim.msg import *
from math import sqrt
import numpy as np

D = 0.5*sqrt(2) # 機体中心からタイヤまでの距離[m]
C = 1/sqrt(2) # ただの定数
MAT_MODEL = np.array([[-C,  C, D],
					  [-C, -C, D],
					  [ C, -C, D],
					  [ C,  C, D]])


def main():
	rospy.init_node('omni_inv_kinem', anonymous=True)
	sub = rospy.Subscriber('target_pose', vw_cmd, callback)

	rospy.spin()


def callback(pose):
	pub = rospy.Publisher('sim_in', wheel_cmd, queue_size=1)
	wheels = wheel_cmd()

	x = np.array([[pose.vx],[pose.vy],[pose.w]])
	v = MAT_MODEL @ x
	wheels.w1 = v[0]
	wheels.w2 = v[1]
	wheels.w3 = v[2]
	wheels.w4 = v[3]
	pub.publish(wheels)


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass

