#include "socket_can_adapter.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace mjcan {
namespace {

CanFrame from_linux_can_frame(const struct can_frame& src) {
  CanFrame dst;

  // 현재는 Standard ID 중심으로 사용합니다.
  // CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG는 제거합니다.
  dst.can_id = src.can_id & CAN_SFF_MASK;
  dst.dlc = src.can_dlc;
  dst.data.fill(0);

  const int n = std::min<int>(dst.dlc, 8);
  for (int i = 0; i < n; ++i) {
    dst.data[i] = src.data[i];
  }

  return dst;
}

struct can_frame to_linux_can_frame(const CanFrame& src) {
  struct can_frame dst {};
  dst.can_id = src.can_id & CAN_SFF_MASK;
  dst.can_dlc = std::min<uint8_t>(src.dlc, 8);

  const int n = std::min<int>(dst.can_dlc, 8);
  for (int i = 0; i < n; ++i) {
    dst.data[i] = src.data[i];
  }

  return dst;
}

}  // namespace

SocketCanAdapter::SocketCanAdapter() = default;

SocketCanAdapter::~SocketCanAdapter() {
  close();
}

bool SocketCanAdapter::open(const std::string& interface_name) {
  close();

  interface_name_ = interface_name;

  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "[SocketCanAdapter] socket() failed: "
              << std::strerror(errno) << "\n";
    socket_fd_ = -1;
    return false;
  }

  // Non-blocking read/write.
  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "[SocketCanAdapter] ioctl(SIOCGIFINDEX) failed for "
              << interface_name << ": " << std::strerror(errno) << "\n";
    close();
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_fd_,
             reinterpret_cast<struct sockaddr*>(&addr),
             sizeof(addr)) < 0) {
    std::cerr << "[SocketCanAdapter] bind() failed for "
              << interface_name << ": " << std::strerror(errno) << "\n";
    close();
    return false;
  }

  // 기본값:
  // - local loopback enabled
  // - own sent frame은 자기 socket으로 다시 받지 않음
  //
  // 이 기본값이 이 bridge 용도에는 적합합니다.
  // adapter가 쓴 feedback frame은 host HAL이 받지만,
  // adapter 자신의 socket으로는 다시 들어오지 않습니다.
  //
  // 필요하면 아래 옵션을 명시적으로 설정할 수 있습니다.
  int recv_own_msgs = 0;
  ::setsockopt(
      socket_fd_,
      SOL_CAN_RAW,
      CAN_RAW_RECV_OWN_MSGS,
      &recv_own_msgs,
      sizeof(recv_own_msgs));

  std::cout << "[SocketCanAdapter] opened " << interface_name << "\n";
  return true;
}

void SocketCanAdapter::close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

void SocketCanAdapter::poll_rx(
    MujocoCanBridge* bridge,
    int max_frames) {
  if (socket_fd_ < 0 || bridge == nullptr) {
    return;
  }

  for (int i = 0; i < max_frames; ++i) {
    struct can_frame linux_frame {};

    const ssize_t nbytes =
        ::read(socket_fd_, &linux_frame, sizeof(linux_frame));

    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }

      std::cerr << "[SocketCanAdapter] read() failed: "
                << std::strerror(errno) << "\n";
      return;
    }

    if (nbytes != static_cast<ssize_t>(sizeof(struct can_frame))) {
      std::cerr << "[SocketCanAdapter] incomplete CAN frame: "
                << nbytes << " bytes\n";
      continue;
    }

    const CanFrame frame = from_linux_can_frame(linux_frame);
    bridge->push_host_frame(frame);
  }

  std::cerr << "[SocketCanAdapter] RX frame limit reached\n";
}

void SocketCanAdapter::flush_tx(
    MujocoCanBridge* bridge,
    int max_frames) {
  if (socket_fd_ < 0 || bridge == nullptr) {
    return;
  }

  for (int i = 0; i < max_frames; ++i) {
    CanFrame frame;

    if (!bridge->pop_device_frame(&frame)) {
      return;
    }

    const struct can_frame linux_frame = to_linux_can_frame(frame);

    const ssize_t nbytes =
        ::write(socket_fd_, &linux_frame, sizeof(linux_frame));

    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // TX buffer가 잠시 찬 경우입니다.
        // 지금 skeleton에서는 frame을 재큐잉하지 않습니다.
        // 필요하면 bridge device queue에 다시 넣는 정책을 추가하십시오.
        return;
      }

      std::cerr << "[SocketCanAdapter] write() failed: "
                << std::strerror(errno) << "\n";
      return;
    }

    if (nbytes != static_cast<ssize_t>(sizeof(struct can_frame))) {
      std::cerr << "[SocketCanAdapter] incomplete write: "
                << nbytes << " bytes\n";
      return;
    }
  }

  std::cerr << "[SocketCanAdapter] TX frame limit reached\n";
}

}  // namespace mjcan