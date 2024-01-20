#include "Lqr.h"

/* ------------------------ Constructor ------------------------ */
Lqr::Lqr() {

    /* ------------------------ Parameters ------------------------ */
    // TODO: make these parameters customizable
    Q << 0.05, 0, 0,
         0, 0.03, 0,
         0, 0, 0.04;
    R << 0.01, 0,
         0, 0.01;
    S << 0.05, 0, 0,
         0, 0.03, 0,
         0, 0, 0.04;
    N = 25;

    dt = 0.02;

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
    m_pose_sub = nh.subscribe<gazebo_msgs::ModelStates>(
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

    ros::topic::waitForMessage<gazebo_msgs::ModelStates>(
        "/gazebo/model_states"
    );
    
    // /* ------------------------ Main Loop ------------------------ */
    Eigen::Matrix<double, 2, 3> K;
    geometry_msgs::Twist cmd_vel;
    ros::Rate rate(1/dt);
    Eigen::Vector2d u_fb = Eigen::Vector2d::Zero();

    Eigen::Matrix2d M;
    M << 0, 1,
         1, 0;

    double offset = 999;
    while (!ros::isShuttingDown()) {
        ros::spinOnce();

        if (offset == 999 & m_ff_cmd.header.stamp.toSec() > 1) {
            offset = ros::Time::now().toSec() - m_ff_cmd.header.stamp.toSec();
        }

        double time_diff = ros::Time::now().toSec() - m_ff_cmd.header.stamp.toSec() - offset;
        ROS_INFO("ROS TIME: %f", ros::Time::now().toSec());
        ROS_INFO("FF CMD TIME: %f", m_ff_cmd.header.stamp.toSec());
        ROS_INFO("TIME DIFF: %f", time_diff);

        Eigen::Vector2d u_ff;
        if (abs(time_diff) < dt) {
            u_ff << m_ff_cmd.twist.angular.z, m_ff_cmd.twist.linear.x;
        }
        else {
            u_ff << 0, 0;
        }
        
        manif::SE2d X_current(
            m_pose.pose[1].position.x, 
            m_pose.pose[1].position.y, 
            m_pose.pose[1].orientation.z
        );
        manif::SE2d X_desired(
            m_traj_pose.pose.position.x, 
            m_traj_pose.pose.position.y, 
            m_traj_pose.pose.orientation.z
        );

        K = _compute_gain(X_current, u_ff - M*u_fb);
        // manif::SE2Tangentd X_error = X_desired - X_current;
        manif::SE2d X_error = X_current.between(X_desired);

        u_fb = - K * X_error.log().coeffs();

        ROS_INFO("X_current: %f, %f, %f", X_current.translation().x(), X_current.translation().y(), X_current.angle());
        ROS_INFO("X_desired: %f, %f, %f", X_desired.translation().x(), X_desired.translation().y(), X_desired.angle());
        ROS_INFO("X_error: %f, %f, %f", X_error.log().coeffs()[0], X_error.log().coeffs()[1], X_error.log().coeffs()[2]);
        
        // ROS_INFO("K_row_1: %f, %f, %f", K(0,0), K(0,1), K(0,2));
        // ROS_INFO("K_row_2: %f, %f, %f", K(1,0), K(1,1), K(1,2));
        
        ROS_INFO("u_fb: %f, %f", u_fb[0], u_fb[1]);
        ROS_INFO("u_ff: %f, %f", u_ff[1], u_ff[0]);

        cmd_vel.linear.x = u_ff[1];// - u_fb[0];
        cmd_vel.angular.z = u_ff[0];// - u_fb[1];
        
        m_cmd_vel_pub.publish(cmd_vel);
        rate.sleep();
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
    
    for (int i = N-1; i > 0; i--) {
        _process_jacobians(X, u_ff, &A, &B);
        R = R + B.transpose() * P * B; // R at time step i+1 
        P = A.transpose() * (P - P * B * R.inverse() * B.transpose() * P) * A + Q; // P at time step i
    }
    R = R + B.transpose() * P * B; // R at time step 0
    return R.inverse() * B.transpose() * P * A;
}

void Lqr::_process_jacobians(
    manif::SE2d X, 
    Eigen::Vector2d u_ff, 
    Eigen::Matrix3d* A, 
    Eigen::Matrix<double, 3, 2>* B
) {
    manif::SE2Tangentd u_tilde(u_ff(1), 0, u_ff(0));
    u_tilde = u_tilde * dt;
    *A = (-u_tilde).exp().adj();
    // *A = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 3, 2> M;
    M << 1, 0,
         0, 0,
         0, 1;
    *B = u_tilde.rjac() * M * dt;
}

/* ------------------------ Callbacks ------------------------ */
void Lqr::_traj_pose_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg,
    geometry_msgs::PoseStamped& traj_pose
) {
    traj_pose = *msg;
}

void Lqr::_pose_cb(
    const gazebo_msgs::ModelStates::ConstPtr& msg,
    gazebo_msgs::ModelStates& pose
) {
    pose = *msg;
}

void Lqr::_ff_cmd_cb(
    const geometry_msgs::TwistStamped::ConstPtr& msg,
    geometry_msgs::TwistStamped& ff_cmd
) {
    ff_cmd = *msg;
}