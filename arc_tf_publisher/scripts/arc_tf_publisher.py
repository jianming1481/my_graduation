#!/usr/bin/env python  
import roslib
roslib.load_manifest('arc_tf_publisher')
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from manipulator_h_base_module_msgs.msg import IK_Cmd
from sensor_msgs.msg import JointState

PI = 3.1415926

def Joint_feedback_CB(eef_pose_msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((0, 0, 0.125),
                     tf.transformations.quaternion_from_euler(0, 0, eef_pose_msg.position[0]),
                     rospy.Time.now(),
                     "Joint_1",
		     "robot_arm_base")

    br.sendTransform((0,0,0.03),
                     tf.transformations.quaternion_from_euler(0,eef_pose_msg.position[1],0),
                     rospy.Time.now(),
                     "Joint_2",
		     "Joint_1")

    br.sendTransform((0,0,0.23),
                     tf.transformations.quaternion_from_euler(0, 0, eef_pose_msg.position[2]),
                     rospy.Time.now(),
                     "Joint_3",
		     "Joint_2")

    br.sendTransform((0.03,0,0.06),
                     tf.transformations.quaternion_from_euler(0, eef_pose_msg.position[3], 0),
                     rospy.Time.now(),
                     "Joint_4",
		     "Joint_3")

    br.sendTransform((-0.03,0,0.23),
                     tf.transformations.quaternion_from_euler(0, 0, eef_pose_msg.position[4]),
                     rospy.Time.now(),
                     "Joint_5",
		     "Joint_4")

    br.sendTransform((0,0,0.03),
                     tf.transformations.quaternion_from_euler(0,eef_pose_msg.position[5], 0),
                     rospy.Time.now(),
                     "Joint_6",
		     "Joint_5")

    br.sendTransform((0,0,0.135),
                     tf.transformations.quaternion_from_euler(0,0,eef_pose_msg.position[6]),
                     rospy.Time.now(),
                     "Joint_7",
		     "Joint_6")

    br.sendTransform((0.03,0,0.09),
                     tf.transformations.quaternion_from_euler(0,-PI/2,0),
                     rospy.Time.now(),
                     "camera_link",
		     "Joint_7")

def arm_feedback_CB(eef_pose_msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((eef_pose_msg.data[0], eef_pose_msg.data[1]*-1, eef_pose_msg.data[2]),
                     tf.transformations.quaternion_from_euler(eef_pose_msg.data[3], eef_pose_msg.data[4], eef_pose_msg.data[5]),
                     rospy.Time.now(),
                     "tcp",
		     "robot_arm_base")


if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/robotis/present_joint_states',
                     JointState,
                     Joint_feedback_CB
                     )
    rospy.Subscriber('/robotis/fk_fb',
                     IK_Cmd,
                     arm_feedback_CB
                     )
    rospy.spin()
