#include "Lqr.h"

/* ------------------------ Constructor ------------------------ */
Lqr::Lqr() {

    /* ------------------------ Parameters ------------------------ */
    // TODO: make these parameters customizable
    Q << 25, 0, 0,
         0, 30, 0,
         0, 0, 30;
    R << 5, 0,
         0, 5;
    S << 30, 0, 0,   
         0, 30, 0,
         0, 0, 30;
    N = 50;

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

    // while (ros::ok()) {
    //     ros::spinOnce();
    //     geometry_msgs::Twist cmd_vel;
    //     cmd_vel.linear.x = 0.5;
    //     cmd_vel.angular.z = 0.5;
    //     m_cmd_vel_pub.publish(cmd_vel);
    // }

    /* ------------------------ Main Loop ------------------------ */
    Eigen::Matrix<double, 2, 3> K;
    while (!ros::isShuttingDown()) {
        ros::spinOnce();

        manif::SE2d X_current(
            m_pose.pose.position.x, 
            m_pose.pose.position.y, 
            m_pose.pose.orientation.z
        );
        manif::SE2d X_desired(
            m_traj_pose.pose.position.x, 
            m_traj_pose.pose.position.y, 
            m_traj_pose.pose.orientation.z
        );
        Eigen::Vector2d u_ff(
            m_ff_cmd.twist.angular.z,
            m_ff_cmd.twist.linear.x
        );

        K = _compute_gain(X_current, u_ff);
        manif::SE2d X_error = X_current.between(X_desired);

        Eigen::Vector2d u_fb = -K * X_error.log().coeffs().tail<3>(); 
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = u_ff(1) - u_fb(1);
        cmd_vel.angular.z = u_ff(0) - u_fb(0);
        m_cmd_vel_pub.publish(cmd_vel);
    }
}

/* ------------------------ Private Methods ------------------------ */
Eigen::Matrix<double, 2, 3> Lqr::_compute_gain(manif::SE2d X, Eigen::Vector2d u_ff) {
    // TODO: for now, we linearize about the current trajectory pose,
    // but we should linearize about current and future trajectory poses
    Eigen::Matrix3d P = S;
    Eigen::Matrix2d R;
    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;
    // linearization state to be input into _process_jacobians()
    
    for (int i = 0; i < N; i++) {
        _process_jacobians(X, u_ff, &A, &B);
        R = R + B.transpose() * P * B;
        P = A.transpose() * (P - P * B * R.inverse() * B.transpose() * P) * A + Q;
    }
    return R.inverse() * B.transpose() * P * A;
}

void Lqr::_process_jacobians(
    manif::SE2d X, 
    Eigen::Vector2d u_ff, 
    Eigen::Matrix3d* A, 
    Eigen::Matrix<double, 3, 2>* B
) {
    manif::SE2Tangentd u_tilde(u_ff(0), u_ff(1), 0);
    *A = u_tilde.exp().inverse().adj();
    *B = u_tilde.rjac() * Eigen::Matrix<double, 3, 2>::Identity();
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