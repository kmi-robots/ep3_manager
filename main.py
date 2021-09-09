#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist, PoseStamped, Pose, Pose2D
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import planar as pl

class CartManager():

    def __init__(self):

        self.gripper_pub = rospy.Publisher("/parallel_gripper_controller/command trajectory_msgs", JointTrajectory)
        self.base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist)
        self.pose_pub = rospy.Publisher("/cart_pose", Pose2D)
        self.int_sub = rospy.Subscriber("/controller_mode", Int8, self.callback)
        self.pose_sub = rospy.Subscriber('/move_base_simple/pose', Pose, self.conv_callback)
        self.pose_conv_pub = rospy.Publisher('/move_base_simple/goal', Pose)
        self.tag_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tag_callback)
        self.mode = 0
        self.mode_saved = False
        self.rate = rospy.Rate(1)

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Mode is %s" % str(self.mode))
            if self.mode == 0: # do nothing
                continue
            elif self.mode ==1: # straight done, turn
                #Open gripper
                joint_msg = JointTrajectory()
                joint_msg.joint_names = ['parallel_gripper_joint']
                joint_msg.points.positions = [0.03] #open by 3 cm
                self.gripper_pub.publish(joint_msg)
                rospy.loginfo("Problem with opening the gripper")

                # Go back
                self.go_back()

                # TODO change arm position

                rospy.loginfo("Move arm position")
                rospy.loginfo("")

            elif self.mode ==2:
                continue

            self.rate.sleep()

    def callback(self, int_msg):
        if self.mode_saved: return
        self.mode = int_msg
        self.mode_saved = True

    def conv_callback(self, pose_msg):
        pose_msg_st = PoseStamped()
        pose_msg_st.header.frame_id = 'base_footprint'
        pose_msg_st.pose = pose_msg.pose
        self.pose_conv_pub.publish(pose_msg_st)

    def tag_callback(self, tag_msg):
        #From captured tag to cart pose
        pose_msg_st = PoseStamped()
        pose_msg_st.header.frame_id = 'base_footprint'

        all_markers = tag_msg.markers
        for m_msg in all_markers:
            tag_id = m_msg.id
            pose_msg_st.pose = m_msg.pose.pose
            if tag_id == 4 or tag_id==5: #manico or front of cart
                vec1 = np.array(0., 0.,1., 0.)
                vec2 = np.array(0.,0., m_msg.position.x, m_msg.position.y)
            elif tag_id == 2: #cart side right
                vec1 = []
                vec2 = []
            elif tag_id == 0: #cart side left
                vec1= []
                vec2 = []
            th = self.calc_angle(vec1,vec2)

            pose_msg_st.pose.x = m_msg.position.x
            pose_msg_st.pose.y = m_msg.position.y
            pose_msg_st.pose.theta = th

            self.pose_pub.publish(pose_msg_st)

    def calc_angle(self, vA,vB):
        num = np.dot(vA,vB)
        denom = np.linalg.norm(vA)*np.linalg.norm(vB)
        return np.arccos(num / denom) #angle in radiants

    def go_back(self):
        end_time = rospy.Time.now() + rospy.Duration(2)
        twist_msg = Twist()
        twist_msg.linear.x = -0.5
        while rospy.Time.now() < end_time: #keep doing for 2 sec
            self.base_pub.publish(twist_msg)
            rospy.sleep(0.1)

def main():
    rospy.init_node('tiago_server')
    rospy.loginfo("Initialize node and server")

    cm = CartManager()
    rospy.loginfo("Node and server initialized")
    cm.run()


if __name__ == "__main__":
    main()
