#include "RobstrideMotor.hpp"


int init_can(const char* interface) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return -1;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) return -1;
    return s;
}

bool readframe(int s, struct can_frame* frame){
    
    ssize_t nbytes = recv(s, frame, sizeof(struct can_frame), MSG_DONTWAIT);
    if (nbytes < 0) {
        const int err = errno;
        if ( err == EAGAIN || err == EWOULDBLOCK){
            return false; // 지금은 데이터 없음 (즉시 리턴)
        }else{
            perror("recv"); // 진짜 에러
            return false;
        }
    }
    return nbytes == sizeof(struct can_frame);
}