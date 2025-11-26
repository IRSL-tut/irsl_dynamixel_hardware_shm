#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rospy.init_node('trajectory_publisher_py')
    pub = rospy.Publisher(
        '/dynamixel/trajectory_controller/command',
        JointTrajectory,
        queue_size=10
    )
    rospy.sleep(1)  # パブリッシャーが接続されるまで待つ

    joint_names = ['LINK_0', 'LINK_1', 'LINK_2', 'LINK_3', 'LINK_4']

    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # 適当な角度(rad)
    point.velocities = []
    point.time_from_start = rospy.Duration(1.0)

    traj_msg.points.append(point)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(traj_msg)
        rospy.loginfo("Published trajectory point")
        rate.sleep()

if __name__ == '__main__':
    main()
