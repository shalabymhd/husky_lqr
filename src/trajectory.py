# %%
# # This script reads the trajectory data from a rosbag file, fits a spline to the data,
# and publishes pose and velocity data at a specified rate.
# The trajectory data is published to the following topics:
# /trajectory/pose
# /trajectory/cmd_vel

from bagpy import bagreader
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.interpolate as interp
import scipy.spatial.transform as transform
import scipy as sp

from geometry_msgs.msg import PoseStamped, TwistStamped
# from gazebo_msgs.msg import ModelStates

rospy.init_node('trajectory_publisher', anonymous=True)

# Generate publishers
pose_pub = rospy.Publisher('/husky/traj_pose', PoseStamped, queue_size=10)
vel_pub = rospy.Publisher('/husky/traj_feedforward_cmd_vel', TwistStamped, queue_size=10)

# Read the trajectory data from the rosbag file
bag = bagreader('/home/shalaby/catkin_tar/src/tar_husky/bags/simulation_reference_bag/result_stamped.bag')
bag_data = bag.message_by_topic('/husky/traj_pose')

# Convert bag_data to a pandas dataframe
df_data = pd.read_csv(bag_data)

# Drop rows with repeating timestamps
df_data.drop_duplicates(subset=['Time'], inplace=True)

# Get the x, y, and yaw data from the dataframe
x = df_data['pose.position.x'].to_numpy()
y = df_data['pose.position.y'].to_numpy()

qx = df_data['pose.orientation.x'].to_numpy()
qy = df_data['pose.orientation.y'].to_numpy()
qz = df_data['pose.orientation.z'].to_numpy()
qw = df_data['pose.orientation.w'].to_numpy()

# Fit a spline to the data
t = df_data['Time'].to_numpy()
# t = t - t[0]
spl_x = interp.CubicSpline(t, x)
spl_y = interp.CubicSpline(t, y)
spl_qx = interp.CubicSpline(t, qx)
spl_qy = interp.CubicSpline(t, qy)
spl_qz = interp.CubicSpline(t, qz)
spl_qw = interp.CubicSpline(t, qw)

# Get the derivative of the position spline
spl_x_dot = spl_x.derivative()
spl_y_dot = spl_y.derivative()

# Publish the trajectory data at a specified rate
freq = 100 # Hz
rate = rospy.Rate(freq)
# t0 = rospy.Time.now().to_sec()
dt = 0.0
while dt < t[-1]:
    # Publish pose data
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = spl_x(dt)
    pose.pose.position.y = spl_y(dt)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = spl_qx(dt)
    pose.pose.orientation.y = spl_qy(dt)
    pose.pose.orientation.z = spl_qz(dt)
    pose.pose.orientation.w = spl_qw(dt)
    pose_pub.publish(pose)

    # TODO: what if velocity is negative?
    # Publish velocity data
    speed = np.sqrt(spl_x_dot(dt)**2 + spl_y_dot(dt)**2)
    q = np.array([
        spl_qx(dt), 
        spl_qy(dt), 
        spl_qz(dt), 
        spl_qw(dt)]
    )
    q_next = np.array([
        spl_qx(dt + 1.0/freq), 
        spl_qy(dt + 1.0/freq), 
        spl_qz(dt + 1.0/freq), 
        spl_qw(dt + 1.0/freq)]
    )
    C = transform.Rotation.from_quat(q).as_matrix()[0:2, 0:2]
    C_next = transform.Rotation.from_quat(q_next).as_matrix()[0:2, 0:2]
    omega = sp.linalg.logm(C.transpose() @ C_next)[1, 0] / (1.0/freq)
    vel = TwistStamped()
    vel.header.stamp = rospy.Time.now()
    vel.twist.linear.x = speed
    vel.twist.linear.y = 0.0
    vel.twist.linear.z = 0.0
    vel.twist.angular.x = 0.0
    vel.twist.angular.y = 0.0
    vel.twist.angular.z = omega
    vel_pub.publish(vel)
    
    dt += 1.0/freq

    rate.sleep()


# %%
