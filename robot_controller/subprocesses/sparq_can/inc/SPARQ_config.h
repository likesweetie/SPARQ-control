#ifndef HUMANOID_CONFIG_HPP
#define HUMANOID_CONFIG_HPP

#include <cmath>

// CAN IDs 
#define CAN_ID_LEFT_FRONT_HIP_ROLL         0x01
#define CAN_ID_LEFT_FRONT_HIP_PITCH        0x02
#define CAN_ID_LEFT_FRONT_KNEE_PITCH       0x03

#define CAN_ID_LEFT_REAR_HIP_ROLL          0x11
#define CAN_ID_LEFT_REAR_HIP_PITCH         0x12
#define CAN_ID_LEFT_REAR_KNEE_PITCH        0x13

#define CAN_ID_RIGHT_FRONT_HIP_ROLL        0x21
#define CAN_ID_RIGHT_FRONT_HIP_PITCH       0x22
#define CAN_ID_RIGHT_FRONT_KNEE_PITCH      0x23

#define CAN_ID_RIGHT_REAR_HIP_ROLL         0x31
#define CAN_ID_RIGHT_REAR_HIP_PITCH        0x32
#define CAN_ID_RIGHT_REAR_KNEE_PITCH       0x33

// SHM INDEX
#define SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL        0
#define SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH       1
#define SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH      2
#define SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL         3
#define SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH        4
#define SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH       5

#define SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL       6
#define SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH      7
#define SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH     8
#define SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL        9
#define SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH       10
#define SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH      11

// VECTOR INDEX
#define CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL     0
#define CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH    4
#define CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH   8

#define CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL      1
#define CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH     5
#define CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH    9

#define CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL    2
#define CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH   6
#define CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH  10

#define CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL     3
#define CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH    7
#define CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH   11

//LEFT LEG LIMITS - deg 2 rad 
constexpr double MAX_POS_LEFT_HIP_PITCH   = 130.0    *  M_PI/180;  
constexpr double MIN_POS_LEFT_HIP_PITCH   = -80.0    *  M_PI/180;
constexpr double MAX_POS_LEFT_HIP_ROLL    = 8        *  M_PI/180;
constexpr double MIN_POS_LEFT_HIP_ROLL    = -90      *  M_PI/180;
constexpr double MAX_POS_LEFT_HIP_YAW     = 90       *  M_PI/180;
constexpr double MIN_POS_LEFT_HIP_YAW     = -90      *  M_PI/180;
constexpr double MAX_POS_LEFT_KNEE_PITCH  = 0        *  M_PI/180;
constexpr double MIN_POS_LEFT_KNEE_PITCH  = -110     *  M_PI/180;
constexpr double MAX_POS_LEFT_ANKLE_A     = 65.5     *  M_PI/180;
constexpr double MIN_POS_LEFT_ANKLE_A     = -41      *  M_PI/180;
constexpr double MAX_POS_LEFT_ANKLE_B     = 41.5     *  M_PI/180;
constexpr double MIN_POS_LEFT_ANKLE_B     = -65.5    *  M_PI/180;  // 부호 확인 필요 

//RIGHT LEG LIMITS - deg 2 rad
constexpr double MAX_POS_RIGHT_HIP_PITCH   = 80.0     *  M_PI/180;  
constexpr double MIN_POS_RIGHT_HIP_PITCH   = -130.0   *  M_PI/180;
constexpr double MAX_POS_RIGHT_HIP_ROLL    = 90       *  M_PI/180;
constexpr double MIN_POS_RIGHT_HIP_ROLL    = -8       *  M_PI/180;
constexpr double MAX_POS_RIGHT_HIP_YAW     = 90       *  M_PI/180;
constexpr double MIN_POS_RIGHT_HIP_YAW     = -90      *  M_PI/180;
constexpr double MAX_POS_RIGHT_KNEE_PITCH  = 110      *  M_PI/180;
constexpr double MIN_POS_RIGHT_KNEE_PITCH  = 0        *  M_PI/180; 
constexpr double MAX_POS_RIGHT_ANKLE_A     = 0        *  M_PI/180;
constexpr double MIN_POS_RIGHT_ANKLE_A     = 65.5     *  M_PI/180;
constexpr double MAX_POS_RIGHT_ANKLE_B     = 41.0     *  M_PI/180;
constexpr double MIN_POS_RIGHT_ANKLE_B     = -65.5    *  M_PI/180;  // 부호 확인 필요 

#endif