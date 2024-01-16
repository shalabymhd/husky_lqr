#include "Lqr.h"

/* ------------------------ Constructor ------------------------ */
Lqr::Lqr() {

    /* ------------------------ Subscribers ------------------------ */
    m_traj_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/husky/traj_pose",
        1,
        boost::bind(
            &Lqr::_traj_pose_cb,
            boost::placeholders::_1,
            boost::ref(m_traj_pose)
        )
    );

    // TODO: make customizable topic name
    m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/gazebo/model_states",
        1,
        boost::bind(
            &Lqr::_pose_cb,
            boost::placeholders::_1,
            boost::ref(m_pose)
        )
    );

    m_ff_cmd_sub = nh.subscribe<geometry_msgs::TwistStamped>(
        "/husky/traj_feedforward_cmd_vel",
        1,
        boost::bind(
            &Lqr::_ff_cmd_cb,
            boost::placeholders::_1,
            boost::ref(m_ff_cmd)
        )
    );

    /* ------------------------ Publishers ------------------------ */
    m_cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(
        "/husky_velocity_controller/cmd_vel", 
        1
    );

    while (ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.5;
        cmd_vel.angular.z = 0.5;
        m_cmd_vel_pub.publish(cmd_vel);
    }
}

/* ------------------------ Callbacks ------------------------ */
void Lqr::_traj_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg,
    geometry_msgs::PoseStamped& traj_pose
) {
    traj_pose = *msg;
}

void Lqr::_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg,
    geometry_msgs::PoseStamped& pose
) {
    pose = *msg;
}

void Lqr::_ff_cmd_cb(
    const geometry_msgs::TwistStamped::ConstPtr& msg,
    geometry_msgs::TwistStamped& ff_cmd
) {
    ff_cmd = *msg;
}