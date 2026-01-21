#include "carm_tool.h"
#include <atomic>
#include <cmath>
#include <cstring> // 添加这个头文件用于strcmp
#include <fstream>
#include <iomanip>
#include <istream>
#include <map>
#include <signal.h>
#include <thread>
#include <unistd.h>

carm_screwdriver screwdriver;
std::atomic<bool> running_{false};

void set_ready() {
  printf("上使能\n");
  screwdriver.set_ready();
}
void set_disable() { screwdriver.set_disable(); }
void set_drag() {
  int flag = 0;
  std::cin >> flag;
  screwdriver.set_drag(flag);
}
void set_screwdriver() {
  std::array<double, 2> data;
  for (size_t i = 0; i < 2; i++)
    std::cin >> data[i];
  printf("set: %lf , %lf\n", data[0], data[1]);
  screwdriver.set_screwdriver(data[0], data[1]);
}
void get_screwdriver() {
  double pos;
  double tau;
  screwdriver.get_screwdriver(pos, tau);
  printf("screwdriver: vel(%lf), tau(%lf)\n", pos, tau);
}

void check_ID() {
  std::cout << "check ID: " << screwdriver.check_ID() << std::endl;
}

int main(int argc, char const *argv[]) {
  running_.store(true);
  std::thread carm_([]() {
    while (running_) {
      screwdriver.update();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
  carm_.detach();

  printf("初始化完成。\n");

  std::string cmd;
  const std::map<std::string, void (*)()> cmd_handle = {
      {"r", &set_ready},       {"dis", &set_disable},   {"dg", &set_drag},
      {"s", &set_screwdriver}, {"p", &get_screwdriver}, {"c", &check_ID}};
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
