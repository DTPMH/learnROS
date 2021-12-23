#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;//只接受一次全局点云

  ROS_WARN("Global Pointcloud received..");

  pcl::PointCloud<pcl::PointXYZ> cloud_input;//定义pcl库的点云
  pcl::fromROSMsg(pointcloud_map, cloud_input);//将接受到的全局点云转变为pcl库的点云，以便利用pcl库对点云进行处理
//对点云进行体素滤波，体素滤波的含义是将三维世界划分为一个个格子，每个格子里只有一个点
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);//体素边长为0.1
  _voxel_sampler.setInputCloud(cloud_input.makeShared());//将点云输入体素滤波中
  _voxel_sampler.filter(_cloud_all_map);//将经过体素滤波后的点云赋予_cloud_all_map

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());//将_cloud_all_map放入_kdtreeLocalMap中，查找起来快速，节省时间

  has_global_map = true;//将得到全局地图标志位置1
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map || !has_odom) return;
//将位姿赋予四元数q,以方便计算当前偏航角yaw
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;//定义三维旋转矩阵rot
  rot = q;//将代表旋转的四元数转变为三维旋转矩阵
  Eigen::Vector3d yaw_vec = rot.col(0);//旋转矩阵的第一列,代表偏航角的旋转(ZYX)

  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  //以目前的位置为中心，搜索局部点                          
  _pointIdxRadiusSearch.clear();//搜索到点的索引值都会存储在该向量中
  _pointRadiusSquaredDistance.clear();//存放的是近邻的中心距，（具体意思还不太了解，以后补充）

  pcl::PointXYZ pt;
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];//注意，_pointIdxRadiusSearch里存储的是附近点的索引值

      if ((fabs(pt.z - _odom.pose.pose.position.z) / (sensing_horizon)) >//sensing_horizon=5.0
          tan(M_PI / 12.0))
        continue;//当计算出的邻域点超出局部半径(r=arctan(15)*sensing_horizon),（即delatZ大于5*tan(15)=1.33时，舍弃）舍弃(计算形式:(z轴坐标之差/更新半径)>tan(15))

      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      if (pt_vec.dot(yaw_vec) < 0) continue;//.dot函数返回的是两个向量的内积，一般来说都会大于0（此语句含义不太懂，以后补充）

      _local_map.points.push_back(pt);//将搜索到的附近点都存储在_local_map中
    }
  } else {
    return;
  }

  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);//将pcl形式的点云转换成ROS版的点云，便于ROS发送等操作
  _local_map_pcd.header.frame_id = "map";

  pub_cloud.publish(_local_map_pcd);//发布局部点云
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);//5.0
  nh.getParam("sensing_rate", sensing_rate);//30
  nh.getParam("estimation_rate", estimation_rate);//30

  nh.getParam("map/x_size", _x_size);//50
  nh.getParam("map/y_size", _y_size);//50
  nh.getParam("map/z_size", _z_size);//5

  // subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);//接收到全局点云,将其进行体素降采样,将数据传到_cloud_map
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);//啥也不干
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);//证明接收到odom

  // publisher depth image and color image
  pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 10);

  double sensing_duration = 1.0 / sensing_rate * 2.5;//=1.0/(30*2.5)=0.0133

  local_sensing_timer =
      nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);//每隔0.013s进行一次局部点更新,然后将局部点发送
//下面这些语句暂没用到
  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
