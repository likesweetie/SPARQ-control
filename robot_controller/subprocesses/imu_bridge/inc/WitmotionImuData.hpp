#ifndef WITMOTION_IMU_DATA_HPP
#define WITMOTION_IMU_DATA_HPP

// Mirrors imu_serial_cpp/main.cpp's ImuData struct exactly, field-for-field.
// That struct is all `double`s with natural alignment (no padding, no
// pragma pack needed) -- if it changes, update this copy too.
// imu_serial_cpp publishes this into POSIX shm `/witmotion_imu` from a
// WitMotion USB-serial IMU (BLE/serial protocol, 0x55 header).
struct ImuData {
    double acc[3] = {0, 0, 0};    // g
    double gyro[3] = {0, 0, 0};   // deg/s
    double angle[3] = {0, 0, 0};  // deg (roll, pitch, yaw)
};

#endif  // WITMOTION_IMU_DATA_HPP
