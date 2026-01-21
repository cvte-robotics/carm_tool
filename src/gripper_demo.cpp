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

carm_gripper gripper(getCurrentWorkingDir() + "/../../tool_model.json");
std::atomic<bool> running_{false};

void set_ready() {
  printf("上使能\n");
  gripper.set_ready();
}
void set_disable() { gripper.set_disable(); }
void set_drag() {
  int flag = 0;
  std::cin >> flag;
  gripper.set_drag(flag);
}
void set_gripper() {
  std::array<double, 2> data;
  for (size_t i = 0; i < 2; i++)
    std::cin >> data[i];
  printf("gap: %lf , %lf\n", data[0], data[1]);
  gripper.set_gripper(data[0], data[1]);
}
void get_gripper() {
  double pos;
  double tau;
  gripper.get_gripper(pos, tau);
  printf("gripper: pos(%lf), tau(%lf)\n", pos, tau);
}
void set_zero() { gripper.set_zero(); }

void check_ID() {
  std::cout << "check ID: " << gripper.check_ID() << std::endl;
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

  printf("初始化完成。\n");

  std::string cmd;
  const std::map<std::string, void (*)()> cmd_handle = {
      {"r", &set_ready},   {"dis", &set_disable}, {"dg", &set_drag},
      {"s", &set_gripper}, {"p", &get_gripper},   {"sz", &set_zero},
      {"c", &check_ID}};
  while (std::cin >> cmd) {
    try {
      if (cmd == "q")
        break;
      else if (auto h = cmd_handle.find(cmd); h != cmd_handle.end())
        h->second();
      else
        printf("unkown cmd: %s\n", cmd.c_str());
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
