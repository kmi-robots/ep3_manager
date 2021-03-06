#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, PoseStamped, Pose, Pose2D
from ar_track_alvar_msgs.msg import AlvarMarkers
from control_msgs.msg import PointHeadActionGoal
import numpy as np


waypoints = [(1.58, 0.094), (2.36, 0.094), (3.13, 0.094), (3.4, 0.117),
    (3.6, 0.1), (3.89, 0.063), (4.08, 0.1), (4.26, -0.07), (4.5, -0.233),
    (4.5, -0.5), (4.5, -0.7), (4.3, -1), (3.36, -1.08), (3.11, -0.8),
    (2.25, -0.4), (1.43, -0.3), (0.77, -0.125), (0.36, -0.34), (0.178, -0.34),
    (0.163, -1.3), (0.163, -1.8)]

class CartManager():

    def __init__(self):

        self.cnt = 0
        self.gripper_pub = rospy.Publisher("/parallel_gripper_controller/command", JointTrajectory, queue_size=5, latch=True)
        self.head_pub = rospy.Publisher('/head_controller/point_head_action/goal', PointHeadActionGoal, queue_size=1, latch=True)
        self.base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.pose_pub = rospy.Publisher("/cart_pose", Pose2D, queue_size=5)
        self.way_pub = rospy.Publisher("/cart_goal_position", Pose2D, queue_size=5, latch=True)
        self.int_sub = rospy.Subscriber("/controller_mode", Int16, self.callback)
        self.pose_sub = rospy.Subscriber('/move_base_simple/pose', Pose, self.conv_callback)
        self.pose_conv_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.tag_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tag_callback)
        self.mode = 0
        self.mode_saved = False
        self.rate = rospy.Rate(1)

    def run(self):
        while not rospy.is_shutdown():
            if self.mode == 0: # do nothing
                pass
            elif self.mode == 1: # straight done, turn
                rospy.loginfo("Mode is %s" % str(self.mode))
                #Open gripper
                joint_msg = JointTrajectory()
                joint_msg.joint_names = ["parallel_gripper_joint"]
                p = JointTrajectoryPoint()
                p.positions = [0.09]
                p.time_from_start = rospy.rostime.Duration(1)
                joint_msg.points.append(p)
                self.gripper_pub.publish(joint_msg)
                rospy.loginfo("Published grip open command")

                # Go back
                # self.go_back()

                # TODO change arm position

                rospy.loginfo("Move arm position")

                self.mode = 0
                self.mode_saved = False

            self.rate.sleep()

    def callback(self, int_msg):
        if int_msg.data ==0:
            # rospy.loginfo("Sending current/last waypoint")
            goal_pose = Pose2D()
            goal_x, goal_y = waypoints[self.cnt]
            goal_pose.x = goal_x
            goal_pose.y = goal_y
            self.way_pub.publish(goal_pose)

        elif int_msg.data == 1:
            # rospy.loginfo("I am ready for a new waypoint")
            goal_pose = Pose2D()
            goal_x, goal_y = waypoints[self.cnt]
            goal_pose.x = goal_x
            goal_pose.y = goal_y
            self.way_pub.publish(goal_pose)
            self.cnt += 1
            # rospy.loginfo("Waypoint sent")

    def conv_callback(self, pose_msg):
        pose_msg_st = PoseStamped()
        pose_msg_st.header.frame_id = 'base_footprint'
        pose_msg_st.pose = pose_msg
        self.pose_conv_pub.publish(pose_msg_st)

    def tag_callback(self, tag_msg):
        #From captured tag to cart pose
        pose_msg_st = Pose2D()
        # pose_msg_st.header.frame_id = 'base_footprint'

        all_markers = tag_msg.markers
        for m_msg in all_markers:
            tag_id = m_msg.id
            if tag_id==0: #only look at the tag on front of cart
                # rospy.loginfo("I see the cart front")
                vec1 = [0., 0.,1., 0.]
                vec2 = [0.,0., m_msg.pose.pose.position.x, m_msg.pose.pose.position.y]
                th = self.calc_angle(vec1, vec2)

                pose_msg_st.x = m_msg.pose.pose.position.x
                pose_msg_st.y = m_msg.pose.pose.position.y
                pose_msg_st.theta = th
                # rospy.loginfo(pose_msg_st)
                self.pose_pub.publish(pose_msg_st)

                #Keep looking at pub
                pg = PointHeadActionGoal()
                pg.header.frame_id = "/base_footprint"
                pg.goal.max_velocity = 1.0
                pg.goal.min_duration = rospy.Duration(0.2)
                pg.goal.target.header.frame_id = "/base_footprint"
                pg.goal.pointing_axis.x = 1.0
                pg.goal.pointing_frame = "/head_2_link"
                pg.goal.target.point.x = m_msg.pose.pose.position.x
                pg.goal.target.point.y = m_msg.pose.pose.position.y
                pg.goal.target.point.z = m_msg.pose.pose.position.z

                # rospy.loginfo("Follow with head")
                self.head_pub.publish(pg)

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
