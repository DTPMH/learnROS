
#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_ = 0;
  exec_state_ = FSM_EXEC_STATE::INIT;/*FSM的状态*/
  have_target_ = false;/*没有目标*/
  have_odom_ = false;/*没有里程记消息*/

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);//1
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);//1.5
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);//2.0

  nh.param("fsm/waypoint_num", waypoint_num_, -1);/*设立目标点数量的信息*/
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules *//*初始化main单元*/
  planner_manager_.reset(new FastPlannerManager);//在kino_replan_fsm.h中实例化的一个plan_manage类指针,reset()函数是auto ptr 的函数,是重新设置auto ptr的指向对象,类似于赋值操作.
                                                 //即将普通指针planner_manager_赋值给auto ptr.
  planner_manager_->initializePlanningModules(nh);//初始化一些要用到的cpp文件函数及类(SDF,A*,B样条)的参数,
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);/*查看程序状态*/
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);//如果规划的轨迹与障碍物碰撞则重新进行规划，程序处于checkCollisionCallback
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);//没有该消息
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  way_pub_=nh.advertise<geometry_msgs::PoseStamped>("/my_waypoint", 1);//改动部分
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;
  geometry_msgs::PoseStamped my_way;
  cout << "Triggered!" << endl;
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y,0.5;//

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;

  }
    //改动部分
    my_way.pose.position.x=end_pt_(0);
    my_way.pose.position.y=end_pt_(1);
    my_way.pose.position.z=end_pt_(2);
  //改动部分
   way_pub_.publish(my_way);//改动部分

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();

  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;
  //odom_pos_(2) = -0.3;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }/*初始化状态后，修改为等待触发状态*/

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }/*等待触发状态后，若有目标修改为产生轨迹状态*/

    case GEN_NEW_TRAJ: {
      start_pt_ = odom_pos_;/*开始点为当前点*/
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      //std::cout<<"四元数w:"<<odom_orient_.w()<<"四元数x:"<<odom_orient_.x()<<"四元数y:"<<odom_orient_.y()<<"四元数z:"<<odom_orient_.z()<<std::endl;
      //std::cout<<"旋转阵r21:"<<rot_x(1)<<"旋转阵r11:"<<rot_x(0)<<std::endl;
      start_yaw_(0) = atan2(rot_x(1), rot_x(0));
      start_yaw_(0) = start_yaw_(0);
      //std::cout<<"初始偏航角"<< start_yaw_(0)<<std::endl;
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();//进行A*以及B样条的计算，成功为1
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajectoryInfo* info = planner_manager_->getLocalTrajectoryInfo();
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->time_traj_start_).toSec();
      t_cur = min(info->traj_duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoor(info->t_start_ + t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->traj_duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {//2.0
        // cout << "near end" << endl;
        return;

      } else if ((info->pos_traj_start_ - pos).norm() < replan_thresh_) {//1.5
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajectoryInfo* info = planner_manager_->getLocalTrajectoryInfo();
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->time_traj_start_).toSec();

      start_pt_ = info->position_traj_.evaluateDeBoor(info->t_start_ + t_cur);
      //std::cout << "pos: " << start_pt_.transpose() << std::endl;
      //std::cout << "odom: " << odom_pos_.transpose() << std::endl;
      //start_pt_=odom_pos_;
      start_vel_ = info->velocity_traj_.evaluateDeBoor(info->t_start_ + t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoor(info->t_start_ + t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoor(info->t_start_ + t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoor(info->t_start_ + t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoor(info->t_start_ + t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajectoryInfo* info = planner_manager_->getLocalTrajectoryInfo();

  if (have_target_) {
    auto edt_env = planner_manager_->getEDTEnvironment();

    double dist = planner_manager_->getPlanParameters()->dynamic_environment_ ?
        edt_env->evaluateCoarseEDT(end_pt_, info->time_start_ + info->traj_duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;
      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->getPlanParameters()->dynamic_environment_ ?
                edt_env->evaluateCoarseEDT(new_pt, info->time_start_ + info->traj_duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_ = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);//进行A*算法以及B样条曲线优化

  if (plan_success) {

    planner_manager_->planHeading(start_yaw_);//应该是求解偏航角的函数
    double delta_x,delta_y,delta_z;
    delta_x = end_pt_(0) - start_pt_(0);
    delta_y = end_pt_(1) - start_pt_(1);
    end_yaw=atan2(delta_y,delta_x);

    //planner_manager_->planYaw(start_yaw_,end_yaw,true,2.0);//应该是求解偏航角的函数
    auto info = planner_manager_->getLocalTrajectoryInfo();//返回的值是traj_info,是一个结构体，里面包含轨迹信息

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->time_traj_start_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();//得到优化好的位置控制点

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);//将优化好的控制点给到bspline中
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();//得到节点向量
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));//将节点向量放入bspline中
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();//与上同理
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);//发布控制点以及节点向量

    /* visulization */
    auto plan_data = planner_manager_->getIntermediatePlanData();

    visualization_->drawGeometricPath(plan_data->kino_path_, 0.1, Eigen::Vector4d(1, 0, 0, 0.2));

    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0, 1.0, 0.0, 0.2), false, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
