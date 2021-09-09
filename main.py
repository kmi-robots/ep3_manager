#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Twist, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers

class CartManager():

    def __init__(self):
        self.int_sub = rospy.Subscriber("/controller_mode", Int8, self.callback)
        self.gripper_pub = rospy.Publisher("/parallel_gripper_controller/command trajectory_msgs", \
                                           JointTrajectory)
        self.base_pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist)
        self.tag_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tag_callback)
        self.pose_pub = rospy.Publisher('/cart_pose', Pose)

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

                # TODO reset arm position

                # TODO catch AR tag

                # TODO publish robot pose
                continue
            elif self.mode ==2:
                continue

            self.rate.sleep()

    def callback(self, int_msg):
        if self.mode_saved: return
        self.mode = int_msg
        self.mode_saved = True

    def tag_callback(self, tag_msg):
        

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
