#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

# Declare Variables/Parameters to be used

# Setup Variables to be used
def callback(data):
    global tau1, tau2
    tau1, tau2=data.data

# Wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi

# Main Function
if _name=='main_':
    # Initialise and Setup node
    rospy.init_node("DLM_Sim")

    # Get DLM Parameters
    dt = 0.02

    m1 = 3.0
    m2 = 3.0
    M1 = 1.5
    M2 = 1.5

    a = 0.2
    d = 0.2
    l1 = 0.4
    l2 = 0.4

    q1 = 0.10
    q2 = 0.1
    q1_dot = 0.0
    q2_dot = 0.0

    tau1 = 0.0
    tau2 = 0.0

    g = 9.8

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    pos_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.Subscriber("/tau", Float32, callback)

    print("The DLM sim is Running")
    try:
        # Run the node
        while not rospy.is_shutdown():
            # Dynamics Calculation
            mM = np.array([
                [m1 * (a**2) + M1 * (l1**2) + m2 * ((l1**2) + (d**2) + 2 * l1 * d * np.cos(q2)) + M2 * ((l1**2) + (l2**2) + 2 * l1 * l2 * np.cos(q2)), m2 * d * (l1 * np.cos(q2) + d) + M2 * l2 * (l1 * np.cos(q2) + l2)],
                [m2 * d * (l1 * np.cos(q2) + d) + M2 * l2 * (l1 * np.cos(q2) + l2), m2 * (d**2) + M2 * (l2**2)]
            ])

            mC = np.array([
                [-2 * l1 * np.sin(q2) * (m2 * d + M2 * l2) * q2_dot, -l1 * np.sin(q2) * (m2 * d + M2 * l2) * q2_dot],
                [l1 * np.sin(q2) * (m2 * d + M2 * l2) * q1_dot, 0]
            ])

            mG = np.array([
                [m1 * a * np.cos(q1) + M1 * l1 * np.cos(q1) + m2 * (l1 * np.cos(q1) + d * np.cos(q1 + q2)) + M2 * (l1 * np.cos(q1) + l2 * np.cos(q1 + q2))],
                [np.cos(q1 + q2) * (m2 * d + M2 * l2)]
            ])

            qpunto = np.array([[q1_dot], [q2_dot]])
            tau = np.array([[tau1], [tau2]])
            mfor = np.dot(np.linalg.inv(mM), (tau - np.dot(mC, qpunto) - mG))

            q1_dot = q1_dot + (mfor[0][0] * dt)
            q2_dot = q2_dot + (mfor[1][0] * dt)

            q1 = q1 + (q1_dot * dt)
            q2 = q2 + (q2_dot * dt)

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ["joint2", "joint3"]  # Names of the joints
            joint_state_msg.position = [q1, q2]  # Joint angles
            joint_state_msg.velocity = [q1_dot, q2_dot]  # Joint velocities

            pos_pub.publish(joint_state_msg)
            # Wait and repeat
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass  # Initialize and Setup node