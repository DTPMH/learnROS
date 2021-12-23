#include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace fast_planner {

FastPlannerManager::FastPlannerManager() {
}

FastPlannerManager::~FastPlannerManager() {
  std::cout << "des manager" << std::endl;
}//~表示析构函数,在利用该类结束时,会调用该函数,清楚该类计算时所占用的内存.

void FastPlannerManager::initializePlanningModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);//0.5
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);//0.7
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);//4
  nh.param("manager/dynamic_environment", pp_.dynamic_environment_, -1);//0
  nh.param("manager/clearance_threshold", pp_.clearance_threshold_, -1.0);//0.2
  nh.param("manager/local_segment_length", pp_.local_segment_length_, -1.0);//6.0
  nh.param("manager/control_points_distance", pp_.control_points_distance_, -1.0);//0.5

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh.param("manager/use_geometric_path", use_geometric_path, false);//false
  nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);//true
  nh.param("manager/use_topo_path", use_topo_path, false);//false
  nh.param("manager/use_optimization", use_optimization, false);//true
  nh.param("manager/use_active_perception", use_active_perception, false);//false

  traj_info_.traj_id_ = 0;//traj_info_是类属于LocalTrajectoryInfo的结构体
  sdf_map_.reset(new SDFMap);//auto ptr 的赋值操作,于plan_manage.h中实例化
  sdf_map_->initMap(nh);//
  edt_environment_.reset(new EDTEnvironment);//在plan_manage.h中实例化
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geometric_path_finder_.reset(new Astar);
    geometric_path_finder_->setParam(nh);
    geometric_path_finder_->setEnvironment(edt_environment_);
    geometric_path_finder_->init();
  }

  if (use_kinodynamic_path) {//初始化一些动力学约束A*算法的参数
    kinodynamic_path_finder_.setParam(nh);
    kinodynamic_path_finder_.setEnvironment(edt_environment_);/*将环境设置为上一步初始化好的SDF地图*/
    kinodynamic_path_finder_.init();
  }

  if (use_optimization) {
    for (int i = 0; i < 10; ++i) {
      BsplineOptimizer optimizer;
      optimizer.setParam(nh);
      optimizer.setEnvironment(edt_environment_);
      bspline_optimizers_.push_back(optimizer);//一个类型为 BsplineOptimizer 类的向量
    }
  }
}

void FastPlannerManager::setGlobalWaypoints(vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {

  double t_now = (ros::Time::now() - traj_info_.time_traj_start_).toSec();

  double tm, tmp;
  traj_info_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = traj_info_.position_traj_.evaluateDeBoor(tm + t_now);

  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < traj_info_.traj_duration_) {
    fut_pt = traj_info_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.09) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

EDTEnvironment::Ptr FastPlannerManager::getEDTEnvironment() {
  return edt_environment_;
}

PlanParameters* FastPlannerManager::getPlanParameters() {
  return &pp_;
}

LocalTrajectoryInfo* FastPlannerManager::getLocalTrajectoryInfo() {
  return &traj_info_;
}

IntermediatePlanData* FastPlannerManager::getIntermediatePlanData() {
  return &plan_data_;
}

bool FastPlannerManager::kinodynamicReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                           Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
                                           Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
       << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
       << endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }

  ros::Time t1, t2;

  traj_info_.time_traj_start_ = ros::Time::now();//traj_info_是包含各种轨迹信息的一个LocalTrajectoryInfo结构体,定义在plan_manage.h
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = ros::Time::now();

  kinodynamic_path_finder_.reset();//auto ptr赋初值

  int status = kinodynamic_path_finder_.search(start_pt, start_vel, start_acc, end_pt, end_vel, true);//只有一个加速度,8个时间t(0.1-0.8)

  if (status == KinodynamicAstar::NO_PATH) {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // retry searching with discontinuous initial state
    kinodynamic_path_finder_.reset();
    status = kinodynamic_path_finder_.search(start_pt, start_vel, start_acc, end_pt, end_vel, false);//有125种加速度,10个时间t

    if (status == KinodynamicAstar::NO_PATH) {
      //if (status == Astar::NO_PATH) {
      cout << "[kino replan]: Can't find path." << endl;
      return false;
    } else {
      cout << "[kino replan]: retry search success." << endl;
    }

  } else {
    cout << "[kino replan]: kinodynamic search success." << endl;
  }

  plan_data_.kino_path_ = kinodynamic_path_finder_.getKinoTraj(0.01);//plan_data_.是一个实例化的类IntermediatePlanData,将delta t为0.01的路径点放入到kino_path_中
//plan_data_.kino_path_返回的数据类型是std::vector<Eigen::Vector3d>，内容是搜索的路径点的位置

  t_search = (ros::Time::now() - t1).toSec();//A*搜索的时间

  // parameterize the path to bspline

  double ts = pp_.control_points_distance_ / pp_.max_vel_;// 0.12/0.5=0.0.06,控制点之间的距离/最大的速度即表示两个控制点的时间间隔
  vector<Eigen::Vector3d> point_set, start_end_derivatives;//
  kinodynamic_path_finder_.getSamples(ts, point_set, start_end_derivatives);//
//point_set存储的是根据ts计算的路径点，而start_end_derivatives存储的是开始和最后的速度与加速度
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);//ctrl_pts存储的是反算的控制点的坐标
  NonUniformBspline init(ctrl_pts, 3, ts);//设置非均匀B样条的控制点，阶数，以及ts（每个控制点的时间间隔，即节点向量）

  // bspline trajectory optimization

  t1 = ros::Time::now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;//令cost_function包含光滑项，距离项以及可行性项

  if (status != KinodynamicAstar::REACH_END) {
//if (status != Astar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ctrl_pts = bspline_optimizers_[0].BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);//ctrl_pts是反算的控制点
//ctrl_pts得到的是优化好的控制点
  t_opt = (ros::Time::now() - t1).toSec();

  // iterative time adjustment

  t1 = ros::Time::now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  double to = pos.getTimeSum();//得到总时间
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) {

    feasible = pos.reallocateTime();//重新分配时间

    if (++iter_num >= 3) break;
  }

  // pos.checkFeasibility(true);
  // cout << "[Main]: iter num: " << iter_num << endl;

  double tn = pos.getTimeSum();

  cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  t_adjust = (ros::Time::now() - t1).toSec();

  // save planned results

  traj_info_.position_traj_ = pos;//将优化以及调整后的控制点等信息存储在traj_info_.position_traj_中

  double t_total = t_search + t_opt + t_adjust;
  cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
       << ", adjust time:" << t_adjust << endl;

  pp_.time_search_ = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_ = t_adjust;

  updateTrajInfo();

  return true;
}

void FastPlannerManager::updateTrajInfo() {
  traj_info_.velocity_traj_ = traj_info_.position_traj_.getDerivative();
  traj_info_.acceleration_traj_ = traj_info_.velocity_traj_.getDerivative();

  traj_info_.position_traj_.getTimeSpan(traj_info_.t_start_, traj_info_.t_end_);//得到轨迹开始的时间与结束的时间
  traj_info_.pos_traj_start_ = traj_info_.position_traj_.evaluateDeBoor(traj_info_.t_start_);
  //pos_traj_start_是一个三维向量，表示的是轨迹开始的坐标点
  traj_info_.traj_duration_ = traj_info_.t_end_ - traj_info_.t_start_;

  traj_info_.traj_id_ += 1;
}

void FastPlannerManager::reparameterizeBspline(NonUniformBspline& bspline, double ratio,
                                               Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  /* extend time and reparameterize */
  double time_origin = bspline.getTimeSum();
  int seg_num = bspline.getControlPoint().rows() - 3;

  bspline.lengthenTime(ratio);

  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  double tm, tmp;
  bspline.getTimeSpan(tm, tmp);
  vector<Eigen::Vector3d> point_set;

  for (double time = tm; time <= tmp + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoor(time));
  }

  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
}

void FastPlannerManager::planHeading(const Eigen::Vector3d& start_yaw) {

  auto t1 = ros::Time::now();

  // calculate waypoints of heading

  auto& pos = traj_info_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw = 0.3;
  int seg_num = ceil(duration / dt_yaw);
  dt_yaw = duration / seg_num;

  const double forward_t = 2.0;
  double last_yaw = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 1; i < seg_num; ++i) {
    double tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);

    double tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-3) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));

    } else {
      waypt = waypts.back();
    }

    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;

  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = traj_info_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));

  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve

  bspline_optimizers_[1].setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;

  yaw = bspline_optimizers_[1].BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 3, 3);

  // update traj info

  traj_info_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  traj_info_.yawdot_traj_ = traj_info_.yaw_traj_.getDerivative();
  traj_info_.yawdotdot_traj_ = traj_info_.yawdot_traj_.getDerivative();

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}


/*****************************************************************************************/

void FastPlannerManager::planYaw(
    const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd, const double& relax_time) {
  const int seg_num = 12;
  auto& pos = traj_info_.position_traj_;
  double duration = pos.getTimeSum();
  double dt_yaw = duration / seg_num;  // time of B-spline segment
  Eigen::Vector3d start_yaw3d = start_yaw;
  std::cout << "dt_yaw: " << dt_yaw << ", start yaw: " << start_yaw3d.transpose() << ", end: " << end_yaw
            << std::endl;

  while (start_yaw3d[0] < -M_PI)
    start_yaw3d[0] += 2 * M_PI;
  while (start_yaw3d[0] > M_PI)
    start_yaw3d[0] -= 2 * M_PI;
  double last_yaw = start_yaw3d[0];

  // Yaw traj control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  // Initial state
  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;

  // Add waypoint constraints if look forward is enabled
  vector<Eigen::Vector3d> waypts;
  vector<int> waypt_idx;
  
  // Final state
  Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
  calcNextYaw(last_yaw, end_yaw3d(0));
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  // Debug rapid change of yaw
  if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI) {
    ROS_ERROR("Yaw change rapidly!");
    std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
  }
  // // Interpolate start and end value for smoothness
  // for (int i = 1; i < seg_num; ++i)
  // {
  //   double tc = i * dt_yaw;
  //   Eigen::Vector3d waypt = (1 - double(i) / seg_num) * start_yaw3d + double(i) / seg_num * end_yaw3d;
  //   std::cout << "i: " << i << ", wp: " << waypt[0] << ", ";
  //   calcNextYaw(last_yaw, waypt(0));
  // }
  // std::cout << "" << std::endl;

  auto t1 = ros::Time::now();

  // Call B-spline optimization solver
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw = bspline_optimizers_[1].BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 3, 3);
  // std::cout << "2: " << (ros::Time::now() - t1).toSec() << std::endl;

  // Update traj info
  traj_info_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  traj_info_.yawdot_traj_ = traj_info_.yaw_traj_.getDerivative();
  traj_info_.yawdotdot_traj_ = traj_info_.yawdot_traj_.getDerivative();

  // plan_data_.path_yaw_ = path;
  // plan_data_.dt_yaw_path_ = dt_yaw * subsp;
}

void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

}  // namespace fast_planner
