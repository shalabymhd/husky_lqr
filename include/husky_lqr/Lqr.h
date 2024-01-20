#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include "manif/SE2.h"
#include <gazebo_msgs/ModelStates.h>
// #include <vector>

class Lqr{
public:
    Lqr();

    geometry_msgs::TwistStamped m_ff_cmd;
    gazebo_msgs::ModelStates m_pose;
    geometry_msgs::PoseStamped m_traj_pose;

protected:
    ros::NodeHandle nh; // ROS node handle

private:
    ros::Publisher m_cmd_vel_pub;
    ros::Subscriber m_traj_pose_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_ff_cmd_sub;

    Eigen::Matrix3d Q; // state cost
    Eigen::Matrix2d R; // control cost
    Eigen::Matrix3d S; // terminal cost

    int N; // number of timesteps
    float dt; // timestep size

    /* ------------------------ Methods ------------------------ */
    Eigen::Matrix<double, 2, 3> _compute_gain(
        manif::SE2d, 
        Eigen::Vector2d
    );
    void _process_jacobians(
        manif::SE2d, 
        Eigen::Vector2d, 
        Eigen::Matrix3d*, 
        Eigen::Matrix<double, 3, 2>*
    );

    /* ------------------------ Callbacks ------------------------ */
    static void _traj_pose_cb(
        const geometry_msgs::PoseStamped::ConstPtr& msg,
        geometry_msgs::PoseStamped& traj_pose
    );

    static void _pose_cb(
        const gazebo_msgs::ModelStates::ConstPtr& msg,
        gazebo_msgs::ModelStates& pose
    );

    static void _ff_cmd_cb(
        const geometry_msgs::TwistStamped::ConstPtr& msg,
        geometry_msgs::TwistStamped& ff_cmd
    );
        
};