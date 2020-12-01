#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetModelState
import tf2_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import roslib.packages
import open3d_ros


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
            number_of_points=500)
        # o3d.visualization.draw_geometries([mesh])

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
            print('Service call failed: %s' % e)

    def sub_pc2_callback(self, msg):
        self.pointcloud = open3d_ros.ros2open3d(msg)
        downpcd = self.pointcloud.voxel_down_sample(voxel_size=0.05)
        # output = open3d_ros.open3d2ros(downpcd, msg.header.frame_id)
        # self.pub_pc2.publish(output)
        rospy.loginfo(type(downpcd))
        dists = downpcd.compute_point_cloud_distance(self.pcd_arrow)
        dists = np.asarray(dists)
        rospy.loginfo(dists)

    def main(self, pose):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = 'world'
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = 'arrow_clone_0'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.z
        t.transform.rotation.z = pose.orientation.y
        t.transform.rotation.w = pose.orientation.w

        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)


if __name__ == '__main__':
    arrowDetector = ArrowDetector()
