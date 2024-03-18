#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include "tf/transform_datatypes.h"

nav_msgs::Path path;
geometry_msgs::PoseStamped current_pose;
ros::Publisher goal_pub;
ros::Publisher path_pub;
geometry_msgs::PoseStamped latest_goal;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    path.poses.clear();
    double eps = 1e-6;
    for(float x = 1.0; x < 8.0 +eps; x += 1.0)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = 3.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    for(float y = 4.0 ; y < 7 + eps; y += 1.0)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 8;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0.7;
        pose.pose.orientation.w = 0.7;
        path.poses.push_back(pose);
    }
    // 获取当前定位信息
    current_pose.pose = odom_msg->pose.pose;
    current_pose.header = odom_msg->header;

    //获取当前方位角
    tf::Quaternion q;
    tf::quaternionMsgToTF(current_pose.pose.orientation, q);
    double rool, pitch, yaw;
    tf::Matrix3x3(q).getRPY(rool, pitch, yaw);
    
    // 初始化最小距离和最近点的索引
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = -1;

    // 遍历轨迹中的每个点，计算与当前位置的距离并找到最小距离的点
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
        double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);

        // 更新最小距离和最近点的索引
        if (distance < min_distance)
        {
            
            // 计算车辆当前朝向与点的夹角
            double angle = std::atan2(dy, dx);
            double angle_diff = std::abs(angle - yaw);
            //输出 angel , yaw, angle_diff
            // ROS_INFO("angle: %.2f, yaw: %.2f, angle_diff: %.2f", angle, yaw, angle_diff);
            // 确保点在车辆前方（夹角小于阈值）
            if (angle_diff < (M_PI / 2))  // 以弧度为单位调整阈值
            {
                min_distance = distance;
                closest_index = i;
            }
        }
    }

    closest_index =  closest_index + 2 < path.poses.size() ? closest_index + 2 : path.poses.size() - 1;
    //输出当前位置
    ROS_INFO("Current position: (%.2f, %.2f)", current_pose.pose.position.x, current_pose.pose.position.y);
    // 输出最近点的索引和坐标
    auto closest_point = path.poses[closest_index].pose;
    // ROS_INFO("Closest point: (%.2f, %.2f)", closest_point.position.x, closest_point.position.y);
    // ROS_INFO("Closest index: %d", closest_index);

    // 把closest point 作为目标点发送出去 话题为 /move_base_simple/goal
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose = closest_point;
    if(latest_goal.pose != goal.pose)
    {
        latest_goal = goal;
        // ROS_INFO("Send goal: (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
        goal_pub.publish(goal);
    }
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    for (size_t i = closest_index; i < path.poses.size(); ++i)
    {
        path_msg.poses.push_back(path.poses[i]);
    }
    path_pub.publish(path_msg);
}

void pathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
    // 接收到新的轨迹消息时更新轨迹
    path = *path_msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_following_node");
    ros::NodeHandle nh;

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/pp_path", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber path_sub = nh.subscribe("/path", 10, pathCallback);

    ros::spin();

    return 0;
}
