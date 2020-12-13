#!/usr/bin/env python3
from math import frexp
import rospy
from gazebo_msgs.srv import GetModelState
from rospy.core import rospyinfo
import tf2_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import open3d as o3d
import roslib.packages
from tf.transformations import *
from cv_bridge import CvBridge
import cv2


class DatasetMaker:

    def __init__(self):
        rospy.init_node('arrow_detector')

        self.count = 0
        self.pub_tf = rospy.Publisher(
            '/tf', tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_pc2 = rospy.Publisher('/output', PointCloud2, queue_size=1)
        self.sub_image_color = rospy.Subscriber(
            '/camera/color/image_raw', Image, self.sub_image_color_callback)
        self.sub_image_depth = rospy.Subscriber(
            '/camera/depth/image_raw', Image, self.sub_image_depth_callback)

        pose = self.get_model_state('arrow_clone_0').pose
        abu2021_gazebo_path = roslib.packages.get_pkg_dir('abu2021_gazebo')
        mesh_arrow = o3d.io.read_triangle_mesh(
            abu2021_gazebo_path + '/meshes/arrow/arrow.obj')
        self.pcd_arrow = mesh_arrow.sample_points_uniformly(
            number_of_points=5000)

        rospy_rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.main(pose)
            rospy_rate.sleep()

    def get_model_state(self, model_name):
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            model_coordinates = rospy.ServiceProxy(
                'gazebo/get_model_state', GetModelState)
            model_state = model_coordinates(model_name, 'world')
            return model_state
        except rospy.ServiceException as e:
            print('Service call failed: {}'.format(e))

    def publish_tf(self, frame_id, child_frame_id,
                   position=[0, 0, 0], quaternion=[0, 0, 0, 1]):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = frame_id
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child_frame_id

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        # quaternion = quaternion_from_euler(
        #     orientation[0], orientation[1], orientation[2])

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    def sub_image_color_callback(self, msg):
        self.image_color = CvBridge().imgmsg_to_cv2(msg, 'bgr8')

    def sub_image_depth_callback(self, msg):
        self.image_depth = CvBridge().imgmsg_to_cv2(msg, 'passthrough')
        cv2.normalize(self.image_depth, self.image_depth, 0, 255, cv2.NORM_MINMAX)

    def main(self, pose):
        for i in range(5):
            model_name = 'arrow_clone_{}'.format(i)
            pose = self.get_model_state(model_name).pose
            self.publish_tf(frame_id='world', child_frame_id=model_name,
                            position=[
                                pose.position.x,
                                pose.position.y,
                                pose.position.z],
                            quaternion=[
                                pose.orientation.x,
                                pose.orientation.y,
                                pose.orientation.z,
                                pose.orientation.w])

        abu2021_gazebo_path = roslib.packages.get_pkg_dir('abu2021_gazebo')
        path_w = abu2021_gazebo_path + '/data/answer/file{}.txt'.format(self.count)
        with open(path_w, mode='w') as f:
            f.write('{}'.format(pose.position.x))
        
        # save image
        cv2.imwrite(abu2021_gazebo_path + '/data/color/color{}.jpg'.format(self.count),
                    self.image_color)
        cv2.imwrite(abu2021_gazebo_path + '/data/depth/color{}.jpg'.format(self.count),
                    self.image_depth)

        rospy.loginfo('step {} is finish!'.format(self.count))
        self.count += 1


if __name__ == '__main__':
    arrowDetector = DatasetMaker()
