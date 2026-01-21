#include "carm_tool.h"

/*####################gripper###################*/
/**
 * 余弦公式
 * 已知三角形的三边 a, b, c , 计算边 c 对应的夹角 angle_c
 */
double carm_gripper::compute_angle(double a, double b, double c) {
  double cosine = (a * a + b * b - c * c) / (2.0 * a * b);
  // 防止数值误差导致余弦值超出[-1, 1]范围
  cosine = std::max(std::min(cosine, 1.0), -1.0);
  return std::acos(cosine);
}

void carm_gripper::gripper_inverse(const double &gap, const double &force,
                                   const double &actual_theta, double &angle,
                                   double &tau) {
  // 根据目标位置，计算目标角度
  double d = gap - bias_distance_;
  double hd = 0.5 * d;
  double a = compute_angle(gripper_radius_, hd, gripper_link_);
  angle = a - M_PI - bias_theta_;

  // 根据目标力矩和真实角度，计算力矩
  double theta = actual_theta + bias_theta_ + M_PI;
  tau = force / std::cos(M_PI_2 - theta);
  tau *= gripper_radius_;
}

void carm_gripper::gripper_forward(const double &angle, const double &tau,
                                   double &gap, double &force) {
  // 根据当前角度，计算当前位置
  double theta = angle + bias_theta_ + M_PI; // actual theta
  double sinR = std::sin(theta) * gripper_radius_;
  double hd = std::sqrt(gripper_link_ * gripper_link_ - sinR * sinR) +
              std::cos(theta) * gripper_radius_;
  double d = 2.0 * hd;
  gap = d + bias_distance_;

  // 根据当前角度和力矩，计算实际力矩
  force = tau / gripper_radius_;
  force *= std::cos(M_PI_2 - theta);
}

carm_gripper::carm_gripper(std::string path) {
  loadKinematics(path);
  M1_ = std::make_shared<damiao::Motor>(damiao::DM4310, 0x07, 0x00);
  // 初始化串口和电机控制
  serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  dm_ = std::make_shared<damiao::Motor_Control>(serial_);

  dm_->addMotor(M1_.get());
  dm_->disable(*M1_);
  sleep(1);

  dm_->save_motor_param(*M1_);
  dm_->enable(*M1_);
  sleep(1);
}

carm_gripper::~carm_gripper() { dm_->disable(*M1_); }

bool carm_gripper::loadKinematics(std::string path) {
  try {
    // 1. 检查文件是否存在
    std::ifstream file(path);
    if (!file.is_open()) {
      std::cout << "[INFO] 配置文件不存在，将使用默认参数" << std::endl;
      return false;
    }

    // 2. 读取并解析JSON
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errs;

    if (!Json::parseFromStream(reader, file, &root, &errs)) {
      std::cout << "path: " << path << std::endl;
      std::cout << "[WARN] JSON解析失败: " << errs << std::endl;
      return false;
    }

    Json::Value grp_kin;
    if (root.isMember("gripper_kinematics")) {
      grp_kin = root["gripper_kinematics"];
    } else {
      return false;
    }

    gripper_radius_ = grp_kin["gripper_radius"].asDouble();
    gripper_link_ = grp_kin["gripper_link"].asDouble();
    bias_theta_ = grp_kin["bias_theta"].asDouble();
    bias_distance_ = grp_kin["bias_distance"].asDouble();
    global_limit_ = {grp_kin["global_limit"][0].asDouble(),
                     grp_kin["global_limit"][1].asDouble()};
    printf("load tool kine: gripper_radius: %lf, gripper_link: %lf, "
           "bias_theta: %lf, "
           "bias_distance: %lf, global_limit: [%lf, %lf] \n",
           gripper_radius_, gripper_link_, bias_theta_, bias_distance_,
           global_limit_.first, global_limit_.second);
  } catch (const std::exception &e) {
    // 捕获所有标准异常
    std::cerr << "[ERROR] 加载运动学参数时发生异常: " << e.what() << std::endl;
    std::cerr << "异常类型: " << typeid(e).name() << std::endl;
    return false;
  } catch (...) {
    // 捕获所有其他异常
    std::cerr << "[ERROR] 加载运动学参数时发生未知异常" << std::endl;
    return false;
  }
  return true;
}

/*初始化*/
bool carm_gripper::set_ready() {
  dm_->enable(*M1_);
  drag_mode = false;
  return dm_->switchControlMode(*M1_, damiao::Control_Mode::POS_FORCE_MODE);
}

/*下使能*/
void carm_gripper::set_disable() { dm_->disable(*M1_); }

/*更新函数*/
void carm_gripper::update() {
  gripper_status = M1_->Get_status();
  if (gripper_status > 1) {
    std::string err_str = "";
    switch (gripper_status) {
    case 8:
      err_str = "over voltage";
      break;
    case 9:
      err_str = "under voltage";
      break;
    case 0xa:
      err_str = "over current";
      break;
    case 0xb:
      err_str = "mos over temperature";
      break;
    case 0xc:
      err_str = "motor over temperature";
      break;
    case 0xe:
      err_str = "overload";
      break;
    }
    std::cout << "motor error: " << err_str << std::endl;
    dm_->clearError(*M1_);
    return;
  }

  if (gripper_status == 1) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    cur_gripper_pos_ = M1_->Get_Position();
    cur_gripper_tau_ = M1_->Get_tau();
    if (drag_mode) {
      dm_->control_mit(*M1_, 0, 0, 0, 0, 0);
      gripper_pos_ = cur_gripper_pos_;
      gripper_tau_ = 0.0;
    } else {
      double gap, tau, pos, force;
      gripper_inverse(gripper_pos_, gripper_tau_, cur_gripper_pos_, pos, tau);
      uint16_t tau_int = fabs(tau) * tau_ratio_;
      tau_int = std::min(tau_int, (uint16_t)10000);
      dm_->control_pos_force(*M1_, pos, max_vel_, tau_int);
    }
  } else {
    dm_->refresh_motor_status(*M1_);
    cur_gripper_pos_ = M1_->Get_Position();
    cur_gripper_tau_ = M1_->Get_tau();
    gripper_pos_ = cur_gripper_pos_;
    gripper_tau_ = 0.0;
  }
}

/*拖动*/
bool carm_gripper::set_drag(bool drag) {
  drag_mode = drag;
  if (drag) {
    return dm_->switchControlMode(*M1_, damiao::Control_Mode::MIT_MODE);
  } else {
    return dm_->switchControlMode(*M1_, damiao::Control_Mode::POS_FORCE_MODE);
  }
}

/*驱动夹爪*/
void carm_gripper::set_gripper(double pos, double tau) {
  double cmd_pos, cmd_tau;
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    cmd_pos = pos;
    cmd_tau = tau;
  }

  gripper_pos_ =
      std::max(std::min(cmd_pos, global_limit_.second + bias_distance_),
               global_limit_.first);
  gripper_tau_ = cmd_tau;
}

/*获取当前位置*/
void carm_gripper::get_gripper(double &gap, double &force) {
  double pos, tau;
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    pos = cur_gripper_pos_;
    tau = cur_gripper_tau_;
  }
  gripper_forward(pos, tau, gap, force);
}

/*设置零点*/
bool carm_gripper::set_zero() {
  gripper_status = M1_->Get_status();
  if (gripper_status == 1) {
    std::cout << "setGripperZeroPosition failed, 零位设置需要电机下使能"
              << std::endl;
    return false;
  } else {
    dm_->set_zero_position(*M1_);
    return true;
  }
}

/*查看ID是否正确*/
bool carm_gripper::check_ID() {
  return !(gripper_status == 1 && cur_gripper_pos_ == 0);
}

/*####################screwdriver###################*/

carm_screwdriver::carm_screwdriver() {
  M1_ = std::make_shared<damiao::Motor>(damiao::DM4310, 0x07, 0x00);
  // 初始化串口和电机控制
  serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  dm_ = std::make_shared<damiao::Motor_Control>(serial_);

  dm_->addMotor(M1_.get());
  dm_->disable(*M1_);
  sleep(1);

  dm_->save_motor_param(*M1_);
  dm_->enable(*M1_);
  sleep(1);
}

carm_screwdriver::~carm_screwdriver() { dm_->disable(*M1_); }

/*初始化*/
bool carm_screwdriver::set_ready() {
  dm_->enable(*M1_);
  drag_mode = false;
  return dm_->switchControlMode(*M1_, damiao::Control_Mode::MIT_MODE);
}

/*下使能*/
void carm_screwdriver::set_disable() { dm_->disable(*M1_); }

/*更新函数*/
void carm_screwdriver::update() {
  screwdriver_status = M1_->Get_status();
  if (screwdriver_status > 1) {
    std::string err_str = "";
    switch (screwdriver_status) {
    case 8:
      err_str = "over voltage";
      break;
    case 9:
      err_str = "under voltage";
      break;
    case 0xa:
      err_str = "over current";
      break;
    case 0xb:
      err_str = "mos over temperature";
      break;
    case 0xc:
      err_str = "motor over temperature";
      break;
    case 0xe:
      err_str = "overload";
      break;
    }
    std::cout << "motor error: " << err_str << std::endl;
    dm_->clearError(*M1_);
    return;
  }

  if (screwdriver_status == 1) {
    std::lock_guard<std::mutex> lock(data_mtx_);
    cur_screwdriver_vel_ = M1_->Get_Velocity();
    cur_screwdriver_tau_ = M1_->Get_tau();
    if (drag_mode) {
      dm_->control_mit(*M1_, 0, 0, 0, 0, 0);
      screwdriver_vel_ = cur_screwdriver_vel_;
      screwdriver_tau_ = 0.0;
    } else {
      double vel = 0.0;
      double tau = 0.0;
      if (fabs(screwdriver_vel_) > 1e-3) {
        if ((fabs(cur_screwdriver_vel_) < 0.3) ||
            fabs(cur_screwdriver_tau_) > screwdriver_tau_ * 0.9) {
          ratio_ -= 0.05;
        } else {
          ratio_ += 0.05;
        }

        ratio_ = std::min(std::max(ratio_, 0.0), 1.0);
        // printf("ratio_: %lf\n", ratio_);
        // printf("ratio_: %lf : %lf\n", cur_screwdriver_vel_,
        //        cur_screwdriver_tau_);
        vel = screwdriver_vel_ * ratio_;
        tau = screwdriver_vel_ > 0 ? screwdriver_tau_ * (1 - ratio_)
                                   : -screwdriver_tau_ * (1 - ratio_);
      }
      dm_->control_mit(*M1_, 0, 1, 0, vel, tau);
    }
  } else {
    dm_->refresh_motor_status(*M1_);
    cur_screwdriver_vel_ = M1_->Get_Velocity();
    cur_screwdriver_tau_ = M1_->Get_tau();
    screwdriver_vel_ = cur_screwdriver_vel_;
    screwdriver_tau_ = 0.0;
  }
}

/*拖动*/
bool carm_screwdriver::set_drag(bool drag) {
  drag_mode = drag;
  return true;
}

/*驱动夹爪*/
void carm_screwdriver::set_screwdriver(double pos, double tau) {
  std::lock_guard<std::mutex> lock(data_mtx_);
  screwdriver_vel_ = pos;
  screwdriver_tau_ = tau;
}

/*获取当前*/
void carm_screwdriver::get_screwdriver(double &vel, double &tau) {
  std::lock_guard<std::mutex> lock(data_mtx_);
  vel = cur_screwdriver_vel_;
  tau = cur_screwdriver_tau_;
}

/*查看ID是否正确*/
bool carm_screwdriver::check_ID() {
  return !(screwdriver_status == 1 && cur_screwdriver_vel_ == 0);
}
