#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import *
import numpy as np


class FixedTFBroadcaster:

    def __init__(self, model_name, frame_id='world',
                 child_frame_id='camera_link'):
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id

        rospy.init_node('fixed_tf2_broadcaster')
        self.pub_tf = rospy.Publisher(
            '/tf', tf2_msgs.msg.TFMessage, queue_size=1)
        pose = self.get_model_state(model_name).pose

        rospy_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.main(pose)
            rospy_rate.sleep()

    def get_model_state(self, model_name):
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            model_coordinates = rospy.ServiceProxy(
                'gazebo/get_model_state', GetModelState)
            model_state = model_coordinates(model_name, self.frame_id)
            return model_state
        except rospy.ServiceException as e:
            print('Service call failed: %s' % e)

    def main(self, pose):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = self.frame_id
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        quaternion = pose.orientation
        euler = euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        quaternion = quaternion_from_euler(
            euler[0] + np.pi, euler[2], euler[1] + np.pi)

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)


if __name__ == '__main__':
    tfb = FixedTFBroadcaster('kinect')
