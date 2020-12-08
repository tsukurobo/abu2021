#!/usr/bin/env python3
from math import frexp
import rospy
from gazebo_msgs.srv import GetModelState
import tf2_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import roslib.packages
from tf.transformations import *
import open3d_ros
import model_base_matching


class ArrowDetector:

    def __init__(self):
        rospy.init_node('arrow_detector')

        self.pub_tf = rospy.Publisher(
            '/tf', tf2_msgs.msg.TFMessage, queue_size=1)
        self.sub_pc2 = rospy.Subscriber(
            '/camera/depth/points',
            PointCloud2,
            self.sub_pc2_callback)
        self.pub_pc2 = rospy.Publisher('/output', PointCloud2, queue_size=1)

        pose = self.get_model_state('arrow_clone_0').pose
        abu2021_gazebo_path = roslib.packages.get_pkg_dir('abu2021_gazebo')
        mesh_arrow = o3d.io.read_triangle_mesh(
            abu2021_gazebo_path + '/meshes/arrow/arrow.obj')
        self.pcd_arrow = mesh_arrow.sample_points_uniformly(
            number_of_points=5000)

        rospy_rate = rospy.Rate(10)
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
                   position=[0, 0, 0], orientation=[0, 0, 0]):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = frame_id
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child_frame_id

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        quaternion = quaternion_from_euler(
            orientation[0], orientation[1], orientation[2])

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    def sub_pc2_callback(self, msg):
        self.pcd_camera = open3d_ros.ros2open3d(msg)
        mbm = model_base_matching.ModelBaseMatching()
        result = mbm.calc(self.pcd_camera, self.pcd_arrow, voxel_size=0.01)
        print(result.transformation)

        self.publish_tf(frame_id='camera_link', child_frame_id='arrow_detected',
                position=[
                    result.transformation[3, 0],
                    result.transformation[3, 1],
                    result.transformation[3, 2]])

    def main(self, pose):
        self.publish_tf(frame_id='world', child_frame_id='arrow_clone_0',
                        position=[
                            pose.position.x,
                            pose.position.y,
                            pose.position.z],
                        orientation=[
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w])


if __name__ == '__main__':
    arrowDetector = ArrowDetector()
