#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState
import tf2_msgs.msg
import geometry_msgs.msg

class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)
        pose = self.get_model_state('kinect').pose

        while not rospy.is_shutdown():
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'world'
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = 'camera_link'
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z

            t.transform.rotation.x = pose.orientation.x
            t.transform.rotation.y = pose.orientation.z
            t.transform.rotation.z = pose.orientation.y
            t.transform.rotation.w = pose.orientation.w

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

    def get_model_state(self, model_name):
        rospy.wait_for_service('gazebo/get_model_state')
        try:
            model_coordinates = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            model_state = model_coordinates(model_name, 'world')
            return model_state
        except rospy.ServiceException as e:
            print('Service call failed: %s'%e)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()
