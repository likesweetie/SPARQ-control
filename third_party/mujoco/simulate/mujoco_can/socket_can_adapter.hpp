#pragma once

#include <string>
#include "mujoco_can_bridge.hpp"

namespace mjcan {

class SocketCanAdapter {
public:
  SocketCanAdapter();
  ~SocketCanAdapter();

  SocketCanAdapter(const SocketCanAdapter&) = delete;
  SocketCanAdapter& operator=(const SocketCanAdapter&) = delete;

  bool open(const std::string& interface_name);
  void close();

  bool is_open() const {
    return socket_fd_ >= 0;
  }

  // vcan0/can0 -> bridge host queue
  void poll_rx(MujocoCanBridge* bridge, int max_frames = 1024);

  // bridge device queue -> vcan0/can0
  void flush_tx(MujocoCanBridge* bridge, int max_frames = 1024);

private:
  int socket_fd_ = -1;
  std::string interface_name_;
};

}  // namespace mjcan