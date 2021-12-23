#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/kino_replan_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");

  KinoReplanFSM kino_replan;//实例化一个KinoReplanFSM类
  kino_replan.init(nh);//对该类进行初始化

  ros::Duration(1.0).sleep();
  ros::spin();//跳入到回调函数,不再进行主函数.

  return 0;
}
