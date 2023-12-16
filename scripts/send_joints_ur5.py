#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import rospy

# Joint names for UR5
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                'wrist_3_joint']
TOPIC_joint_control = '/pos_joint_traj_controller/command'
TOPIC_joint_state = '/joint_states'

target_joints = [0, 0, 0.754, -0.754, 0, -0.754]

#target_joints = [0, 0, 0, 0, 0, 0]

current_joints = {}


def joint_state_cb(data):
    global current_joints
    for n,p in zip(data.name,data.position):
        current_joints[n] = p

def diff(current,target):
    d = [0,0,0,0,0,0]
    for i,n in enumerate(joint_names):
        d[i] = abs(current[n]-target[i])
    return max(d)

def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher(TOPIC_joint_control, JointTrajectory, queue_size=1)
    sub = rospy.Subscriber(TOPIC_joint_state, JointState, joint_state_cb)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    traj.joint_names = joint_names

    rate = rospy.Rate(10)

    t0 = rospy.Time.now()
    while t0.secs==0 and t0.nsecs==0:
        rate.sleep()
        t0 = rospy.Time.now()

    t1 = rospy.Time.now()
    d = 1.0
    
    while not rospy.is_shutdown() and t1.secs-t0.secs<10 and d>0.001:

        t1 = rospy.Time.now()
        traj.header.stamp = t1
        pts = JointTrajectoryPoint()
        pts.positions = target_joints
        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)

        #print(traj)

        # Publish the message
        pub.publish(traj)

        rate.sleep()

        d = diff(current_joints, target_joints)
        print(d)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")

