import rospy
from geometry_msgs.msg import Pose, PoseStamped

pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)

def main():
    rospy.init_node('pose_converter')
    pose_sub = rospy.Subscriber('/move_base_simple/pose', Pose, callback)
    rospy.spin()

def callback(pose_msg):
    pose_msg_st = PoseStamped()
    pose_msg_st.header.frame_id = 'base_footprint'
    pose_msg_st.pose = pose_msg
    pose_pub.publish(pose_msg_st)

if __name__ == "__main__":
    main()