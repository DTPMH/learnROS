#include <path_searching/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace fast_planner {
KinodynamicAstar::~KinodynamicAstar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic,
                             double time_start) {//dynamic=false，time_start=-1.0
  start_vel_ = start_v;
  start_acc_ = start_a;
/*  动力学可行A*算法*/
  /* ---------- initialize ---------- */
  PathNodePtr cur_node = path_node_pool_[0];//将路径池中的第一个点赋值给当前点cur_node，实际上cur_node是一个空节点
  cur_node->parent = NULL;//起始点的父节点是空的
  cur_node->state.head(3) = start_pt;//将起始点的位置赋值给cur_node->state的前三个变量
  cur_node->state.tail(3) = start_v;//将起始点的速度赋值给cur_node->state的后三个变量
  cur_node->index = posToIndex(start_pt);//index表示的是这个点在三维上的栅格数。即三维的
  cur_node->g_score = 0.0;//A*算法中的g值表示从父节点到当前点的代价值，由于起始点的父节点为空，因此起始点的g值为0

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;//将目标点的位置速度赋值给end_state
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);//同理，表示三维栅格数
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);//计算当前点的f值
  cur_node->node_state = IN_OPEN_SET;//将起始点放入OPENlist中

  open_set_.push(cur_node);
  use_node_num_ += 1;//use_node_num_表示在search过程在，访问的节点数量

  expanded_nodes_.insert(cur_node->index, cur_node);//将当前节点的索引以及节点状态放入到expanded_nodes_中,expanded_nodes_是一个NodeHashTable类
//排序方式是以index（三维的）的大小排序的，expanded_nodes_表示是所有加入到OPEN列表中的节点
  PathNodePtr neighbor = NULL;//创建一个pathnode的类指针，存储邻居节点
  PathNodePtr terminate_node = NULL;//创建一个pathnode的类指针，存储终端节点
  bool init_search = init;//init是一个bool变量
  const int tolerance = ceil(1 / resolution_);//1/0.1=10，ceil表示超过值的最小整数

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {//当open非空时
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();//排序由std::priority_queue实现,以F值从小到大排列，将OPEN列表中F值最小的值设置为当前点
    // cout << "pos: " << cur_node->state.head(3).transpose() << endl;
    // cout << "time: " << cur_node->time << endl;
    // cout << "dist: " <<
    // edt_environment_->evaluateCoarseEDT(cur_node->state.head(3),
    // cur_node->time) << endl;

    /* ---------- determine termination ---------- */

    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
        abs(cur_node->index(1) - end_index(1)) <= tolerance &&
        abs(cur_node->index(2) - end_index(2)) <= tolerance;//若是当前节点与目标节点的三个轴上的距离都小于1,则判定即将到达目标,令near_end=1;
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;//7.0,表示若是当前点的坐标与起始点坐标间的距离大于7,则说明到达一次路径规划的范畴.


/*************下面的语句块是处理是否要结束寻路算法************************************************************************/
/*想要结束有两个条件：第一个当前点离目标点十分接近;第二个当前点距离起始点已经到达一次路径的阈值************************************/

    if (reach_horizon || near_end) {
      cout << "[Kino Astar]:---------------------- " << use_node_num_ << endl;//use_node_num_表示搜索路径的次数
      cout << "use node num: " << use_node_num_ << endl;
      cout << "iter num: " << iter_num_ << endl;//iter_num_表示在一次路径搜索中用到的节点数量
      terminate_node = cur_node;//将当前节点赋值给结束点,一次来形成连续的路径
      retrievePath(terminate_node);//将现在搜索到的所有路径点放入到向量path_nodes中,通过搜索父节点的方式来寻找
      has_path_ = true;

      if (near_end) {//如果是判定快到达目标点了,
        cout << "[Kino Astar]: near end." << endl;

        /* one shot trajectory */
        estimateHeuristic(cur_node->state, end_state, time_to_goal);//计算出来的是最优的总时间，用在后端优化中的总时间
        computeShotTraj(cur_node->state, end_state, time_to_goal);//暂认为计算最后一段路程中是否正常,若是正常则is_shot_succ_==1,反之为0

        if (terminate_node->parent == NULL && !is_shot_succ_)//若是最后一个节点的没有父节点,或者最后一段路程不正常,则路径搜索失败
          return NO_PATH;
        else
          return REACH_END;
      } else if (reach_horizon) {
        cout << "[Kino Astar]: Reach horizon_" << endl;
        return REACH_HORIZON;
      }
    }//判断是否已经找到目标
/*****************************************************************************************************************************************/
    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();//.pop函数。删除栈顶的元素
    cur_node->node_state = IN_CLOSE_SET;//将当前节点放入到close列表中
    iter_num_ += 1;//iter_num_表示放入close中的节点，即确定使用的节点

    /* ---------- init state propagation ---------- */
    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 8.0;

    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;

    vector<Eigen::Vector3d> inputs;
    vector<double> durations;

    if (init_search) {
      inputs.push_back(start_acc_);//将初始加速度放入inputs，一般都为0
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_;
           tau += time_res_init * init_max_tau_)//for(tau=0.1;tau<=0.8;tau=tau+0.1)
        durations.push_back(tau);//durations.push_back(0.1);
    } else {
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)//for(ax=-3.0;ax<=3.001;ax+=1.5)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)//for(ay=-3.0;ay<=3.001;ay+=1.5)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)//for(az=-3.0;az<=3.001;az+=1.5)
          {//
            um << ax, ay, az;
            inputs.push_back(um);//一共125个加速度(每一个轴有5个可能的加速度,最大加速度是3.0,则为:-3.0,-1.5,0,1.5.3.0)
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)//for(tau=0.025;tau<=0.25;tau+=0.025)
        durations.push_back(tau);//一共10个t(0.025,0.05,0.075,0.1,0.125,0.15,0.175,0.2,0.225,0.25)
    }

    /* ---------- state propagation loop ---------- *///找邻居节点的程序块
    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    for (int i = 0; i < inputs.size(); ++i)//i表示加速度
      for (int j = 0; j < durations.size(); ++j) {//j表示时间
        init_search = false;
        um = inputs[i];//加速度赋值为um
        double tau = durations[j];//时间赋值为tau
        stateTransit(cur_state, pro_state, um, tau);//um相当于三轴加速度,tau相当于时间t,通过简单的运动学定律可以计算出临近点的坐标以及速度.
        //通过X(t+1)=X(t)+V(t)*t+0.5*a(t)*t^2;V(t+1)=V(t)+a(t)*t;
        pro_t = cur_node->time + tau;//pro_t表示从当前节点到该邻居节点所需的时间

        /* ---------- check if in free space ---------- */
/***********************************以下程序块表示的是检查该邻居节点是否超出地图边界*******************************/
        /* inside map range */   //超出地图边界,则开始考察下一个邻近点
        if (pro_state(0) <= origin_(0) || pro_state(0) >= map_size_3d_(0) ||
            pro_state(1) <= origin_(1) || pro_state(1) >= map_size_3d_(1) ||
            pro_state(2) <= origin_(2) || pro_state(2) >= map_size_3d_(2)) {
           //cout << "outside map" << endl;//原来注释的
          continue;
        }//map_size=40,40,5//通过launch文件设置的
/*************************************************************************************************************/

/********************************以下程序块表示检查该邻居节点是否已经处于closel列表中************************************/
        /* not in close set */
        Eigen::Vector3i pro_id = posToIndex(pro_state.head(3));//pro_id表示该邻居节点的三轴栅格数
        int pro_t_id = timeToIndex(pro_t);

        PathNodePtr pro_node = expanded_nodes_.find(pro_id);//在expanded_nodes_中寻找该邻居节点，expanded_nodes_中存储的是所有扩展过的节点
//该语句表示在所有扩展过的节点中（包含close以及open列表），寻找该节点。没找到为NULL
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
           //cout << "in closeset" << endl;//原来注释的
          continue;
        }//该相邻点已经处于close集合中,继续考察下一个邻近点
/******************************************************************************************************************/

/**********************************以下程序块表示检查该邻居节点的速度是否超过预先设定的速度*************************************/
        /* vel feasibe */
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_) {
           //cout << "vel infeasible" << endl;//原来注释的
          continue;
        }//如果该邻近点的速度大于设置的无人机的最大速度,则舍弃该邻近点
/*******************************************************************************************************************/

/********************************以下程序块表示检查该邻居点与当前点是否在同一个体素块中**************************************/
        /* not in the same voxel */
        Eigen::Vector3i diff = pro_id - cur_node->index;//diff表示该邻居节点与当前节点的三轴栅格数的差值
        int diff_time = pro_t_id - cur_node->time_idx;//时间差
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0)) {
          continue;
        }//如果两个状态id一样,则舍弃该邻近点
/**************************************************************************************************************/

/**********************************以下程序块表示检查该邻居点是否处于障碍物中，以及是否安全******************************/
        /* collision free */
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;

        for (int k = 1; k <= check_num_; ++k) {//check_num_=5，即检查当前点到该邻居点路线上的5个点是否碰上障碍物
          double dt = tau * double(k) / double(check_num_);//检查到达下一个时间点中间是否会撞上障碍物
          stateTransit(cur_state, xt, um, dt);//xt为下一个点
          pos = xt.head(3);

          double dist = edt_environment_->evaluateCoarseEDT(pos, -1.0);//在updateesdf中更新的距离值
          if (dist <= margin_) {//0.2
            is_occ = true;
            break;//break表示的打破检查碰撞循环
          }
        }

        if (is_occ) {
          // cout << "collision" << endl;//原来 注释的
          continue;
        }
/******************************************************************************************************************/

/************************************以下邻居块表示检查该邻居点是否已经处于open列表中，并进行相应的处理***********************/
        /* ---------- compute cost ---------- */
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;//L2范数 squareNorm()，等价于计算vector的自身点积，norm()返回squareNorm的开方根。
        //um表示加速度，w_time_=10,tau表示时间间隔
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);
        /* ---------- compare expanded node in this loop ---------- */

        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j) {
          PathNodePtr expand_node = tmp_expand_nodes[j];//tmp_expand_nodes存储的是包含OPEN以及CLOSE列表中的节点
          if ((pro_id - expand_node->index).norm() == 0 &&//pro_id表示当前邻居节点的三轴栅格数，
              ((!dynamic) || pro_t_id == expand_node->time_idx)) {//dynamic=false，即只要前一个条件符合就可以

            prune = true;//表示该节点已经在open列表中了
            //cout<<"IM IN"<<endl;

            if (tmp_f_score < expand_node->f_score) {//如果该邻居点点如今的f值小于以前的f值，则对其进行修改
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              expand_node->parent = cur_node;
            }          
            break;
          }
        }
        /* ---------- new neighbor in this loop ---------- */
//将新的邻居节点加入到open_list中
        if (!prune) {//如果该邻居节点是新节点
          if (pro_node == NULL) {//对该节点进行赋值，并将其加入到open列表中
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;

            open_set_.push(pro_node);

            expanded_nodes_.insert(pro_id, pro_node);//expanded_nodes_表示的是所有扩展过的节点

            tmp_expand_nodes.push_back(pro_node);//同上，多余的语句，加与不加没有区别

            use_node_num_ += 1;//节点数+1
            if (use_node_num_ == allocate_num_) {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }//如果你超出了预定的节点数,说明寻路失败
          } else if (pro_node->node_state == IN_OPEN_SET) {
            if (tmp_g_score < pro_node->g_score) {
              // pro_node->index = pro_idcost;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
            }
          } else {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }

        /* ----------  ---------- */
      }//遍历邻居节点结束
  }
  /* ---------- open set empty, no path ---------- */
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(ros::NodeHandle& nh) {//初始化第一步
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);//0.8
  nh.param("search/max_vel", max_vel_, -1.0);// 2.1
  nh.param("search/max_acc", max_acc_, -1.0);//1.0   
  nh.param("search/w_time", w_time_, -1.0);//10
  nh.param("search/horizon", horizon_, -1.0);//7.0
  nh.param("search/resolution_astar", resolution_, -1.0);//0.1
  nh.param("search/time_resolution", time_resolution_, -1.0);//0.8
  nh.param("search/lambda_heu", lambda_heu_, -1.0);//5
  nh.param("search/margin", margin_, -1.0);//0.2
  nh.param("search/allocate_num", allocate_num_, -1);//10000
  nh.param("search/check_num", check_num_, -1);//5

  cout << "margin:" << margin_ << endl;
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node) {
  PathNodePtr cur_node = end_node;//将末端节点赋值为当前节点
  path_nodes_.push_back(cur_node);//将当前节点放入path_nodes_中，path_nodes_表示路径点

  while (cur_node->parent != NULL) {//如果当前点父节点是空的，则路径点回朔完全
    cur_node = cur_node->parent;//将当前节点赋值为当前点的父节点
    path_nodes_.push_back(cur_node);//将当前节点放入path_nodes_中
  }

  reverse(path_nodes_.begin(), path_nodes_.end());//reverse函数是用来交换向量vetor里的数据的顺序,例如,s={0,1,2};reverse(s.begin,s.end);s={2,1,0}
}
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                                           double& optimal_time) {
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);//segment函数表示的是取向量的段，该语句含义是从x1的第三个向量开始，选择三个向量，组成新的向量v0
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;//10.0

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);//quartic是一个函数，计算的是到达终点的时间，（具体如何实现还需看论文）

  double v_max = max_vel_;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;//lpNorm<Infinity>()表示取向量各个元素中绝对值最大的那个元素的最大值
  //t_bar表示现在节点到目标节点中三轴最大的距离除以最大速度所得到的时间
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts) {
    if (t < t_bar) continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost) {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;//tie_breaker_=1.0+1.0/10000
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                                       double time_to_goal) {
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++) {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_) {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) ||
        coord(1) >= map_size_3d_(1) || coord(2) < origin_(2) || coord(2) >= map_size_3d_(2)) {
      return false;
    }

    if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d) {
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e) {
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar::init() {//初始化第三步
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;//1/0.1=10
  inv_time_resolution_ = 1.0 / time_resolution_;//1/0.8=1.25
  edt_environment_->getMapRegion(origin_, map_size_3d_);//origin_:-20,-20,-1;map_size_3d_:40,40,1//事先设定好的
                                                        //origin_=-x_size/2,-y_size/2,-1;map_size_3d_=x_size,y_size,z_size
  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);//10000
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new PathNode;//路径池每个点都包含着这个点的所有参数
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);//单位矩阵
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env) {//初始化第二步
  this->edt_environment_ = env;
}

void KinodynamicAstar::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
}
//getKinoTraj函数的作用是，将搜索好的轨迹以时间间隔为0.01分成一个个位置点，保存在state_list中。
std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t) {//0.01
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();//.back()返回末尾元素,即返回最后一个节点的信息,path_nodes_存储着路径点的信息。
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL) {
    Vector3d ut = node->input;//加速度
    double duration = node->duration;//duration=tau，为时间间隔，即寻找邻居节点是设置的时间间隔
    x0 = node->parent->state;//state包括位置与速度

    for (double t = duration; t >= -1e-5; t -= delta_t) {//即将两个路径点直接按时间间隔为0.01分为多个路径点
      stateTransit(x0, xt, ut, t);//转换状态，将时间间隔0.01的路径点转换至xt,
      state_list.push_back(xt.head(3));//将xt位置保存在state_list中,将分成的多个路径点的位置保存在state_list中
    }
    node = node->parent;//迭代
  }
  reverse(state_list.begin(), state_list.end());//将向量中的元素顺序反转

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_) {//快要到达目标点的时候才会判断is_shot_succ_
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t) {//t_shot_表示最后一段路程的时间
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);//将最后一个点到目标点之间的路径点加上
    }
  }

  return state_list;//返回分化的路径点
}

//getSamples函数的作用是，在搜索结束的路径中，每隔ts时间，采样一个控制点。并将控制点放入point_set列表中
void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives) {//ts是已知传入的形参
  /* ---------- final trajectory time ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_) T_sum += t_shot_;//如果路径可以直接到达终点，则is_shot_succ_=1,且计算所用时间t_shot_

  PathNodePtr node = path_nodes_.back();//node表示搜索的最后一个路径点
  while (node->parent != NULL) {
    T_sum += node->duration;
    node = node->parent;
  }//这个while循环的作用是得到从起始点到目标点一共会用到的时间，为T_sum
  // cout << "final time:" << T_sum << endl;

  /* ---------- init for sampling ---------- */
  int K = floor(T_sum / ts);//floor(x)返回的是小于或等于x的最大整数，
  //ts表示的是两个控制点的最小时间间隔，因此K表示一共可以有K个控制点
  ts = T_sum / double(K + 1);//在通过K反算平均时间间隔
  // cout << "K:" << K << ", ts:" << ts << endl;

  bool sample_shot_traj = is_shot_succ_;

  // Eigen::VectorXd sx(K + 2), sy(K + 2), sz(K + 2);
  // int sample_num = 0;
  node = path_nodes_.back();//node是最后一个路径点

  Eigen::Vector3d end_vel, end_acc;

  double t;
  if (sample_shot_traj) {//如果终点可直接到达
    t = t_shot_;//令时间t表示末端节点到终点所用的时间
    end_vel = end_vel_;//终点速度赋值

    for (int dim = 0; dim < 3; ++dim) {//dim表示三轴
      Vector4d coe = coef_shot_.row(dim);
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }

  } else {//如果终点不可达
    t = node->duration;//时间t表示到达最后一个节点所用的时间间隔
    end_vel = node->state.tail(3);
    end_acc = node->input;
  }

  for (double ti = T_sum; ti > -1e-5; ti -= ts) {//这个for循环用来表示一共几个控制点（完整轨迹中）
    /* ---------- sample shot traj---------- */
    if (sample_shot_traj) {

      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
//求解每隔ts时间的一个点，因为搜索的轨迹是曲线
      // sx(sample_num) = coord(0), sy(sample_num) = coord(1), sz(sample_num) = coord(2);
      // ++sample_num;
      point_set.push_back(coord);
      t -= ts;//计算上一个时间，用来计算控制点

      /* end of segment */
      if (t < -1e-5) {//如果t<0了，则表示该段轨迹结束，开始从上一段轨迹中提取采样点
        sample_shot_traj = false;//将到达终点标志为置负
        if (node->parent != NULL) t += node->duration;//t为末端节点的时间间隔
      }
    }
    /* ---------- sample search traj---------- */
    else {

      Eigen::Matrix<double, 6, 1> x0 = node->parent->state;//将起始点设置为末端节点的父节点
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d ut = node->input;

      stateTransit(x0, xt, ut, t);//得到末端状态
      // sx(sample_num) = xt(0), sy(sample_num) = xt(1), sz(sample_num) = xt(2);
      // ++sample_num;

      point_set.push_back(xt.head(3));//将末端点放入point_set中
      t -= ts;//t减去控制点的时间间隔ts

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL) {//如果该段轨迹结束，则开始下一段轨迹
        node = node->parent;
        t += node->duration;
      }
    }
  }

  /* ---------- return samples ---------- */
  // samples.col(K + 2) = start_vel_;
  // samples.col(K + 3) = end_vel_;
  // samples.col(K + 4) = node->input;

  reverse(point_set.begin(), point_set.end());//将采样的控制点反转

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(node->input);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes() {
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt) {
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);//*1.25
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0,
                                    Eigen::Matrix<double, 6, 1>& state1, Eigen::Vector3d um,
                                    double tau) {
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;//phi_是6*6的矩阵

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

}  // namespace fast_planner
