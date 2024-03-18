#include "tf/transform_datatypes.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

ros::Publisher cmd_pub;
ros::Publisher path_pub;
ros::Publisher goal_pub;

nav_msgs::Path path;
geometry_msgs::PoseStamped current_pose;

namespace Constants {
double k = 0.1;                      // 前视距离系数
double Lfc = 0.35;                   // 前视距离
double L = 0.4;                      // 车辆轴距，单位：m
double radius = 0.6;                 // 车辆半径，单位：m
double max_steer = atan(L / radius); // 最大转角，单位：rad
double max_speed = 0.8;              // 最大速度，单位：m/s
double kv = 0.8;                     // 速度控制系数
} // namespace Constants

void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  // 多次发送目标点，防止move_base_goal丢失
  //  static int move_base_goal = 5;
  //  if(move_base_goal > 0){
  //      geometry_msgs::PoseStamped goal;
  //      goal.header.frame_id = "map";
  //      goal.header.stamp = ros::Time::now();
  //      goal.pose = path.poses.back().pose;
  //      goal_pub.publish(goal);
  //      move_base_goal--;
  //  }
  path_pub.publish(path);

  double eps = 1e-6;

  current_pose.pose = odom_msg->pose.pose;
  current_pose.header = odom_msg->header;
  double last_v = odom_msg->twist.twist.linear.x;
  // 获取当前方位角
  tf::Quaternion q;
  tf::quaternionMsgToTF(current_pose.pose.orientation, q);
  double rool, pitch, yaw;
  tf::Matrix3x3(q).getRPY(rool, pitch, yaw);

  // 初始化最小距离和最近点的索引
  double min_distance = std::numeric_limits<double>::max();
  int closest_index = -1;
  static int index = 0; // 从上一次的最近点开始搜索
  for (size_t i = index; i < path.poses.size(); ++i) {
    double dx = path.poses[i].pose.position.x - current_pose.pose.position.x;
    double dy = path.poses[i].pose.position.y - current_pose.pose.position.y;
    double distance = std::hypot(dx, dy);
    // 更新最小距离和最近点的索引
    if (distance < min_distance) {
      closest_index = i;
      min_distance = distance;
    }
  }
  if (closest_index == -1) {
    ROS_ERROR("No closest point found!");
    return;
  }
  index = closest_index;

  // 计算前视距离
  double Lf = Constants::k * last_v + Constants::Lfc;

  double target_index = closest_index + 1;
  for (int i = closest_index + 1; i < path.poses.size(); ++i) {
    double temp_lf = Lf;
    double dx = path.poses[i].pose.position.x -
                path.poses[closest_index].pose.position.x;
    double dy = path.poses[i].pose.position.y -
                path.poses[closest_index].pose.position.y;
    temp_lf -= std::hypot(dx, dy);
    if (temp_lf < 0) {
      target_index = i;
      break;
    }
  }

  // 计算车辆当前朝向与点的夹角
  std::array<double, 2> target_point = {
      path.poses[target_index].pose.position.x,
      path.poses[target_index].pose.position.y};
  std::array<double, 2> current_point = {current_pose.pose.position.x,
                                         current_pose.pose.position.y};
  double dx = target_point[0] - current_point[0];
  double dy = target_point[1] - current_point[1];
  double angle = std::atan2(dy, dx);
  double alpha = angle - yaw;
  alpha = (alpha > M_PI) ? (alpha - 2 * M_PI)
                         : (alpha < -M_PI) ? (alpha + 2 * M_PI) : alpha;
  double delta = std::atan2(2.0 * Constants::L * std::sin(alpha) / Lf, 1.0);
  ROS_INFO("alpha: %.2f, delta: %.2f , Lf: %.2f", alpha, delta, Lf);
  delta =
      std::max(std::min(delta, Constants::max_steer), -Constants::max_steer);
  geometry_msgs::Twist cmd;
  cmd.angular.z = delta;
  cmd.linear.x =
      Constants::kv * last_v + (1 - Constants::kv) * Constants::max_speed;

  double distance_to_goal = std::hypot(
      current_pose.pose.position.x - path.poses.back().pose.position.x,
      current_pose.pose.position.y - path.poses.back().pose.position.y);
  if (distance_to_goal < 0.5) {
    cmd.linear.x = 0;
    cmd.angular.z = 0;
  }
  ROS_INFO("cmd: %.2f, %.2f", cmd.linear.x, cmd.angular.z);
  cmd_pub.publish(cmd);

  // 发布teb目标点，closest_index之10m的点
  double delta_distance = 0;
  double max_distance = 3;
  int teb_target_index = path.poses.size() - 1;
  for (int i = closest_index; i + 1 < path.poses.size(); ++i) {
    double dx =
        path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    double dy =
        path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    delta_distance += std::hypot(dx, dy);
    ROS_INFO("delta_distance: %.2f", delta_distance);
    if (delta_distance > max_distance) {
      teb_target_index = i + 1;
      break;
    }
  }

  // 每秒发布一次目标点
  static ros::Time last_time = ros::Time::now();
  if (ros::Time::now() - last_time > ros::Duration(1)) {
    last_time = ros::Time::now();
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose = path.poses[teb_target_index].pose;
    goal_pub.publish(goal);
    goal_pub.publish(goal);
  }
  //输出cloest_index 与 teb_target_index
  ROS_INFO("closest_index: %d, teb_target_index: %d", closest_index,
           teb_target_index);
  ROS_INFO("delta_distance: %.2f", delta_distance);
}

void pathCallback(const nav_msgs::Path::ConstPtr &path_msg) {
  // 接收到新的轨迹消息时更新轨迹
  path = *path_msg;
}
void loadPath() {
  std::string input_bag_filename =
      "/home/wangyu/path_follow/src/path_following/src/odom_data.bag";
  ROS_WARN("Loading path from %s", input_bag_filename.c_str());
  rosbag::Bag bag;
  bag.open(input_bag_filename, rosbag::bagmode::Read);

  path.poses.clear();
  rosbag::View view(bag, rosbag::TopicQuery("/odom"));
  geometry_msgs::Point last_added_point;
  path.header.frame_id = "map";
  for (rosbag::MessageInstance const m : view) {
    nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
    if (odom_msg != nullptr) {
      geometry_msgs::PoseStamped pose_stamped;
      // pose_stamped.header = odom_msg->header;
      pose_stamped.header.frame_id = "map";
      static int seq = 0;
      pose_stamped.pose = odom_msg->pose.pose;
      double distance =
          std::hypot(pose_stamped.pose.position.x - last_added_point.x,
                     pose_stamped.pose.position.y - last_added_point.y);
      if (path.poses.empty() || distance > 0.1) {
        pose_stamped.header.seq = seq++;
        path.poses.push_back(pose_stamped);
        last_added_point = pose_stamped.pose.position;
      }
    } else {
      std::cout << "odom null " << std::endl;
    }
  }
  bag.close();
  path_pub.publish(path);
  ROS_INFO("Load path with %d points", path.poses.size());

  // 发布teb目标点，轨迹的最后一个点
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_following_node");
  ros::NodeHandle nh;
  path_pub = nh.advertise<nav_msgs::Path>("/odom_path", 2);
  goal_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  loadPath();
  ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
  // ros::Subscriber path_sub = nh.subscribe("/path", 10, pathCallback);
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_pp", 10);
  // cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ros::spin();

  return 0;
}
