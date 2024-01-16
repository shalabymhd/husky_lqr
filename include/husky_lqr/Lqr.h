#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class Lqr{
public:
    Lqr();

    geometry_msgs::TwistStamped m_ff_cmd;
    geometry_msgs::PoseStamped m_pose;
    geometry_msgs::PoseStamped m_traj_pose;

protected:
    ros::NodeHandle nh; // ROS node handle

private:
    ros::Publisher m_cmd_vel_pub;
    ros::Subscriber m_traj_pose_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_ff_cmd_sub;

    /* ------------------------ Callbacks ------------------------ */
    static void _traj_pose_cb(
        const geometry_msgs::PoseStamped::ConstPtr& msg,
        geometry_msgs::PoseStamped& traj_pose
    );

    static void _pose_cb(
        const geometry_msgs::PoseStamped::ConstPtr& msg,
        geometry_msgs::PoseStamped& pose
    );

    static void _ff_cmd_cb(
        const geometry_msgs::TwistStamped::ConstPtr& msg,
        geometry_msgs::TwistStamped& ff_cmd
    );
        
};