#include "carm_tool.h"
#include <atomic>
#include <cmath>
#include <cstring> // 添加这个头文件用于strcmp
#include <fstream>
#include <iomanip>
#include <iostream>
#include <istream>
#include <limits.h>
#include <map>
#include <signal.h>
#include <thread>
#include <unistd.h>

std::string getCurrentWorkingDir() {
  char buffer[PATH_MAX];
  if (getcwd(buffer, sizeof(buffer)) != nullptr) {
    return std::string(buffer);
  }
  return ".";
}

Json::Value loadKinematics(const std::string &path, const std::string &name) {
  std::ifstream file(path);
  Json::Value root;
  Json::CharReaderBuilder reader;
  std::string errors;
  if (!file.is_open() || !Json::parseFromStream(reader, file, &root, &errors)) {
    std::cerr << "[ERROR] 无法读取配置文件 " << path << ": " << errors
              << std::endl;
    return {};
  }
  return root[name];
}

carm_gripper gripper(loadKinematics(getCurrentWorkingDir() +
                                        "/../../tool_model.json",
                                    "handle_kinematics"));
std::atomic<bool> running_{false};

void print_help() {
  std::cout << R"(
========== 手柄测试指令 ==========
  r                 上使能
  dis               下使能
  dg <0|1>          设置拖动模式：0 关闭，1 开启
  s <gap> <force>   设置手柄开口距离(m)和夹持力(N)
  p                 读取当前开口距离(m)和夹持力(N)
  sz                设置当前位置为零点
  c                 检查电机 ID 和通信状态
  h / help          显示本帮助
  q                 退出程序
==================================
)";
}

void set_ready() {
  printf("[CMD] 上使能\n");
  gripper.set_ready();
}
void set_disable() {
  printf("[CMD] 下使能\n");
  gripper.set_disable();
}
void set_drag() {
  int flag = 0;
  printf("请输入0或者1: ");
  std::cin >> flag;
  printf("[CMD] 拖动模式: %s\n", flag ? "开启" : "关闭");
  gripper.set_drag(flag);
}
void set_gripper() {
  std::array<double, 2> data;
  for (size_t i = 0; i < 2; i++) {
    if (i == 0)
      printf("位置： ");
    else
      printf("力矩： ");
    std::cin >> data[i];
  }
  printf("[CMD] 设置开口距离: %.6f m, 夹持力: %.6f N\n", data[0], data[1]);
  gripper.set_gripper(data[0], data[1]);
}
void get_gripper() {
  double pos;
  double tau;
  gripper.get_gripper(pos, tau);
  printf("[STATE] 开口距离: %.6f m, 夹持力: %.6f N\n", pos, tau);
}
void set_zero() {
  printf("[CMD] 设置零点\n");
  gripper.set_zero();
}

void check_ID() {
  std::cout << "[STATE] 电机 ID 和通信状态: "
            << (gripper.check_ID() ? "正常" : "异常") << std::endl;
}

int main(int argc, char const *argv[]) {
  running_.store(true);
  std::thread carm_([]() {
    while (running_) {
      gripper.update();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  carm_.detach();

  printf("手柄初始化完成。\n");
  print_help();

  std::string cmd;
  const std::map<std::string, void (*)()> cmd_handle = {
      {"r", &set_ready},   {"dis", &set_disable}, {"dg", &set_drag},
      {"s", &set_gripper}, {"p", &get_gripper},   {"sz", &set_zero},
      {"c", &check_ID},    {"h", &print_help},    {"help", &print_help}};
  while (std::cin >> cmd) {
    try {
      if (cmd == "q")
        break;
      else if (auto h = cmd_handle.find(cmd); h != cmd_handle.end())
        h->second();
      else
        printf("未知指令: %s，输入 h 查看帮助。\n", cmd.c_str());
    } catch (const std::exception &e) {
      printf("catch an exception, what = %s\n", e.what());
    } catch (...) {
      printf("catch an exception\n");
    }
  }

  running_.store(false);
  if (carm_.joinable())
    carm_.join();
  return 0;
}
