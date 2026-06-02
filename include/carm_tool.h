#include "damiao.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring> // 添加这个头文件用于strcmp
#include <fstream>
#include <iomanip>
#include <iostream>
#include <istream>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <signal.h>
#include <unistd.h>

class carm_gripper {
private:
  std::shared_ptr<damiao::Motor> M1_;
  std::shared_ptr<SerialPort> serial_;
  std::shared_ptr<damiao::Motor_Control> dm_;

  /*#########夹爪建模数据##########*/

  // 夹爪位置范围( 单位: 米 )
  double pos_lower_ = 0.0;
  double pos_upper_ = 0.077;

  // (0-Max_tau) -> (0-10000) I_des = tau_ratio_ * Nm
  double tau_ratio_ = 1000;

  // 常量: 连杆与电机的安装点到电机轴心距离( 单位: 米 )
  double gripper_radius_ = 0.022;

  // 常量: 连杆长度( 单位: 米 )
  double gripper_link_ = 0.032;

  // 待标定量: 电机角度参考点( radians)
  // 夹爪滑轨轴线与电机0点的夹角，右手坐标系
  double bias_theta_ = -0.63863;

  // 待标定量: 夹爪初始距离偏移量
  // 默认闭合为0点，闭合的时候连接点之间的距离
  double bias_distance_ = -0.023;

  // 常量: 夹爪滑动距离范围( 单位: 米 )
  std::pair<double, double> global_limit_ = {0.0, 0.1};

  // cmd
  bool drag_mode = false;
  std::mutex data_mtx_;
  double gripper_pos_;
  double gripper_tau_;
  uint16_t max_vel_ = 10000; // 固定最快

  // get data
  double cur_gripper_pos_;
  double cur_gripper_tau_;
  int8_t gripper_status;

  double compute_angle(double a, double b, double c);

  void gripper_inverse(const double &gap, const double &force,
                       const double &actual_theta, double &angle, double &tau);

  void gripper_forward(const double &angle, const double &tau, double &gap,
                       double &force);

public:
  carm_gripper(std::string path);
  ~carm_gripper();

  bool loadKinematics(std::string path);

  /*初始化*/
  bool set_ready();
  /*下使能*/
  void set_disable();
  /*更新函数*/
  void update();
  /*拖动*/
  bool set_drag(bool drag);
  /*驱动夹爪*/
  void set_gripper(double pos, double tau);
  /*获取当前位置*/
  void get_gripper(double &gap, double &force);
  /*设置零点*/
  bool set_zero();
  /*查看ID是否正确*/
  bool check_ID();
};

class carm_screwdriver {
private:
  std::shared_ptr<damiao::Motor> M1_;
  std::shared_ptr<SerialPort> serial_;
  std::shared_ptr<damiao::Motor_Control> dm_;

  // cmd
  bool drag_mode = false;
  std::mutex data_mtx_;
  double screwdriver_vel_ = 0.0;
  double screwdriver_tau_ = 0.0;

  // get data
  double cur_screwdriver_vel_ = 0.0;
  double cur_screwdriver_tau_ = 0.0;
  int8_t screwdriver_status;

  // 过渡
  double ratio_ = 1;

public:
  carm_screwdriver();
  ~carm_screwdriver();

  /*初始化*/
  bool set_ready();
  /*下使能*/
  void set_disable();
  /*更新函数*/
  void update();
  /*拖动*/
  bool set_drag(bool drag);
  /*驱动夹爪*/
  void set_screwdriver(double pos, double tau);
  /*获取当前*/
  void get_screwdriver(double &vel, double &tau);
  /*查看ID是否正确*/
  bool check_ID();
};
