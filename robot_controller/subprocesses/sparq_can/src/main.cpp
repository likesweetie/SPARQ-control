#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include "Sharemem.hpp"
#include "SPARQ_config.h"
#include <signal.h>
#include <vector>
#include <iostream>

#include <fcntl.h>
#include <pthread.h>


const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";
const char* CAN_INTERFACE_2 = "can2";
const char* CAN_INTERFACE_3 = "can3";
bool running = true;
constexpr long CONTROL_PERIOD = 2'000'000;
constexpr int MaxID = 52;

constexpr int MOTOR_NUM = 12;

Motor_con SparQ;


void signal_handler(int signum) { 
    running = false; 
    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) {
                motor.write_operation_frame(0, 0, 0);
                motor.control_param.pos = 0;
                motor.control_param.Kp = 0;
                motor.control_param.Kd = 0;
        }
        }(vecs));
    }, SparQ);
}

void* print_thread_func(void*) {
    while (running) {
        usleep(500000);
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (const auto& motor : vec) {
                    std::cout
                        << "[ID 0x" << std::hex << motor.can_id << std::dec << "] "
                        << "pos="   << motor.Feedback_param.pos.load(std::memory_order_relaxed)
                        << " corr=" << motor.Feedback_param.pos.load(std::memory_order_relaxed) + motor.pos_offset
                        << " vel="  << motor.Feedback_param.vel.load(std::memory_order_relaxed)
                        << " torq=" << motor.Feedback_param.torque.load(std::memory_order_relaxed)
                        << " temp=" << motor.Feedback_param.temp.load(std::memory_order_relaxed)
                        << " off="  << motor.pos_offset
                        << "\n";
                }
            }(vecs));
        }, SparQ);
        std::cout << "---\n";
    }
    return nullptr;
}

void* CAN_Comm_thread(void* arg) {  // TODO : 캘리브레이션을 위한 로직을 따로 뺄 것 그리고 뭐 캔 인터페이스 갯수 맞출 수 있게 탬플릿을 넣거나 
    struct sched_param param;
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    Rx_handler<MaxID> hRx(SparQ);
    can_frame cf;
    //int s1 = *(int*)arg;
    auto& can_interface_vec = *static_cast<std::vector<int>*>(arg);

    for(int s : can_interface_vec){
        while (readframe(s, &cf));
    }


    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.write_operation_frame(0, 0, 0);
        }(vecs));
    }, SparQ);

    // 첫 사이클: 피드백 수신 후 모든 모터 offset 캘리브레이션
    RealTimeClock RTC;
    RTC.wait_next(10'000'000);

    for(int s : can_interface_vec){
        while (readframe(s, &cf)) {
            auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
            if (!err) hRx.Write_Fb(id, p, v, t, tem);
        }
    }

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.calibrate();
        }(vecs));
    }, SparQ);

    RTC.reset();
    while (running) {
        RTC.wait_next(CONTROL_PERIOD);

        for(int s : can_interface_vec){    // 이 부분이 성능상 문제가 될 잠재적인 가능성이 있는데 그럴 경우 파리미터 업데이트 스레드로 옮기면 됨. 
            while (readframe(s, &cf)) {
                auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
                if (err) [[unlikely]] {
                    std::cerr << id << " errorcode: " << err;
                    return nullptr;
                }
                hRx.Write_Fb(id, p, v, t, tem);
            }
        }

        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                //for (auto& motor : vec) motor.write_updated_operation_frame();
                for (auto& motor : vec) motor.write_updated_operation_frame();
            }(vecs));
        }, SparQ);
    }

    return NULL;
}

void* update_Control_params(void* args){

    Control_Shm<MOTOR_NUM> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    Control_param ctrl_buf[MOTOR_NUM];
    Feedback_Param fb_buf[MOTOR_NUM];
    

 while(running){                                    // 이 루프 모듈화는 힘들기도 하고 했을때 오히려 의도가 불명확해보일 수 있음. 따라서 로봇이 바뀐다면 이거 정도는 뭐 합시다. 어려운거 아니자네 
                                                        // 실제 제어시에는 게인이 바뀔 일이 없으므로 게인을 쓰는 부분은 따로 빼거나. 



        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].torque = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);  //TORQUE
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].torque = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].torque = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].torque = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].torque = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].torque = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].torque = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].torque = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].torque = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].torque = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].torque = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].torque = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);


                                                                                                                // 온도 피드백은 실제 제어할땐 필요없을 것으로 예상됨. 
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].Feedback_param.temp.load(std::memory_order_relaxed);  //TEMP
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].temp = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].Feedback_param.temp.load(std::memory_order_relaxed);

        
                                                                                                                                    // END FEEDBACKS
                                                                                                                //피드백은 뭐 각 객체별 clamp도 없어서 반복문을 쓰려면 쓸순 있을듯함 다만 연결에 있어서 불편할듯. 

        // while (!shm_ptr->read_ctrl_relaxed(ctrl_buf));  // torn read면 재시도
        shm_ptr->read_ctrl_relaxed(ctrl_buf);                                                                                                                                // COMMANDS
                                                                                                                                // POS 

// POSITION COMMAND

        // double clamped_pos;

        // clamped_pos = -fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].pos < MIN_POS_HIP_ROLL ? MIN_POS_HIP_ROLL
        //             : -fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].pos > MAX_POS_HIP_ROLL ? MAX_POS_HIP_ROLL
        //             : -ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L HIP PITCH 

        // clamped_pos = -fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].pos < MIN_POS_HIP_PITCH ? MIN_POS_HIP_PITCH
        //             : -fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].pos > MAX_POS_HIP_PITCH ? MAX_POS_HIP_PITCH
        //             : -ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R HIP PITCH   

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].pos < MIN_POS_KNEE_PITCH ? MIN_POS_KNEE_PITCH
        //             : fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].pos > MAX_POS_KNEE_PITCH ? MAX_POS_KNEE_PITCH
        //             : -ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L KNEE PITCH

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].pos < MIN_POS_HIP_ROLL ? MIN_POS_HIP_ROLL
        //             : fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].pos > MAX_POS_HIP_ROLL ? MAX_POS_HIP_ROLL
        //             : ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R KNEE PITCH

        // clamped_pos = -fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].pos < MIN_POS_HIP_PITCH ? MIN_POS_HIP_PITCH
        //             : -fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].pos > MAX_POS_HIP_PITCH ? MAX_POS_HIP_PITCH
        //             : -ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L HIP ROLL

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].pos < MIN_POS_KNEE_PITCH ? MIN_POS_KNEE_PITCH
        //             : fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].pos > MAX_POS_KNEE_PITCH ? MAX_POS_KNEE_PITCH
        //             : -ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R HIP ROLL

        // clamped_pos = -fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].pos < MIN_POS_HIP_ROLL ? MIN_POS_HIP_ROLL
        //             : -fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].pos > MAX_POS_HIP_ROLL ? MAX_POS_HIP_ROLL
        //             : -ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L HIP YAW

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].pos < MIN_POS_HIP_PITCH ? MIN_POS_HIP_PITCH
        //             : fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].pos > MAX_POS_HIP_PITCH ? MAX_POS_HIP_PITCH
        //             : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R HIP YAW

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].pos < MIN_POS_KNEE_PITCH ? MIN_POS_KNEE_PITCH
        //             : fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].pos > MAX_POS_KNEE_PITCH ? MAX_POS_KNEE_PITCH
        //             : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L ANKLE A

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].pos < MIN_POS_HIP_ROLL ? MIN_POS_HIP_ROLL
        //             : fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].pos > MAX_POS_HIP_ROLL ? MAX_POS_HIP_ROLL
        //             : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R ANKLE A 

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].pos < MIN_POS_HIP_PITCH ? MIN_POS_HIP_PITCH
        //             : fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].pos > MAX_POS_HIP_PITCH ? MAX_POS_HIP_PITCH
        //             : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L ANKLE B 

        // clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].pos < MIN_POS_KNEE_PITCH ? MIN_POS_KNEE_PITCH
        //             : fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].pos > MAX_POS_KNEE_PITCH ? MAX_POS_KNEE_PITCH
        //             : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].pos;
        // std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R ANKLE B


        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL]
            .control_param.pos.store(
                -ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH]
            .control_param.pos.store(
                -ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH]
            .control_param.pos.store(
                -ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL]
            .control_param.pos.store(
                ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH]
            .control_param.pos.store(
                -ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH]
            .control_param.pos.store(
                -ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL]
            .control_param.pos.store(
                -ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH]
            .control_param.pos.store(
                ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH]
            .control_param.pos.store(
                ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL]
            .control_param.pos.store(
                ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH]
            .control_param.pos.store(
                ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].pos,
                std::memory_order_relaxed
            );

        std::get<RS02_Vec>(SparQ)
            [CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH]
            .control_param.pos.store(
                ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].pos,
                std::memory_order_relaxed
            );

                                                                                                            // Kp
                                                                                                            
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].Kp, std::memory_order_relaxed);       //
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].Kp, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].Kp, std::memory_order_relaxed); 

        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].Kp, std::memory_order_relaxed);
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].Kp, std::memory_order_relaxed);
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].Kp, std::memory_order_relaxed);

        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].Kp, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].Kp, std::memory_order_relaxed);
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].Kp, std::memory_order_relaxed);

        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].Kp, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].Kp, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].Kp, std::memory_order_relaxed);  

                                                                                                            //Kd
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].Kd, std::memory_order_relaxed);       //
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].Kd, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].Kd, std::memory_order_relaxed);  

        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].Kd, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].Kd, std::memory_order_relaxed);       
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].Kd, std::memory_order_relaxed);  

        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].Kd, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].Kd, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].Kd, std::memory_order_relaxed);       

        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].Kd, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].Kd, std::memory_order_relaxed);  
        std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].Kd, std::memory_order_relaxed);  
                                                                                                                                    // END COMMANDS

        
                                                                                                                                        // FEEDBACKS
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].pos = -(std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].pos_offset);
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].pos = -(std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].pos_offset);
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].pos = -(std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].pos_offset);
        
        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].pos = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].pos_offset;

        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].pos = -(std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].pos_offset);
        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].pos = -(std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].pos_offset);
        
        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].pos = -(std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].pos_offset);
        
        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].pos = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].pos_offset;

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].pos = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].pos = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].pos_offset;
        
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].pos = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].pos = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].pos_offset;



        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_ROLL].vel = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);      //VEL
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_HIP_PITCH].vel = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_HIP_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_FRONT_KNEE_PITCH].vel = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_FRONT_KNEE_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_ROLL].vel = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_HIP_PITCH].vel = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_HIP_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_REAR_KNEE_PITCH].vel = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_LEFT_REAR_KNEE_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_ROLL].vel = -std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_HIP_PITCH].vel = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_HIP_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].vel = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_FRONT_KNEE_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_ROLL].vel = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_HIP_PITCH].vel = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_HIP_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_REAR_KNEE_PITCH].vel = std::get<RS02_Vec>(SparQ)[CONTROL_VECTOR_INDEX_RIGHT_REAR_KNEE_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);


        shm_ptr->write_fb(fb_buf);
       usleep(100);  //이제 여기서도 RTC 쓰거나 해야하지 싶네 
    }
    return nullptr;
}


int main() {

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;
    int s2 = init_can(CAN_INTERFACE_1);
    if (s2 < 0) return -1;
    int s3 = init_can(CAN_INTERFACE_2);
    if (s3 < 0) return -1;
    int s4 = init_can(CAN_INTERFACE_3);
    if (s4 < 0) return -1;

    std::vector<int> can_interface = {s1, s2, s3, s4};


   std::get<RS02_Vec>(SparQ).emplace_back(s1, CAN_ID_LEFT_FRONT_HIP_ROLL   ,     40.0 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s2, CAN_ID_LEFT_REAR_HIP_ROLL    ,    -40.0 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s3, CAN_ID_RIGHT_FRONT_HIP_ROLL  ,    -40.0 *  M_PI/180);  
   std::get<RS02_Vec>(SparQ).emplace_back(s4, CAN_ID_RIGHT_REAR_HIP_ROLL   ,     40.0 *  M_PI/180);  

   std::get<RS02_Vec>(SparQ).emplace_back(s1, CAN_ID_LEFT_FRONT_HIP_PITCH  ,    100.0 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s2, CAN_ID_LEFT_REAR_HIP_PITCH   ,    -60.0 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s3, CAN_ID_RIGHT_FRONT_HIP_PITCH ,   -100.0 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s4, CAN_ID_RIGHT_REAR_HIP_PITCH  ,     20.0 *  M_PI/180);

   std::get<RS02_Vec>(SparQ).emplace_back(s1, CAN_ID_LEFT_FRONT_KNEE_PITCH ,  -147.947 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s2, CAN_ID_LEFT_REAR_KNEE_PITCH  ,  -147.947 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s3, CAN_ID_RIGHT_FRONT_KNEE_PITCH,   147.947 *  M_PI/180);
   std::get<RS02_Vec>(SparQ).emplace_back(s4, CAN_ID_RIGHT_REAR_KNEE_PITCH ,   147.947 *  M_PI/180);



    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.init_motor_MIT(100, 100);
        }(vecs));
    }, SparQ);

    //여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &can_interface);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 초반 확인용이라 나중에는 안쓰는 스레드임. 
    pthread_create(&shm_t, NULL, update_Control_params,  nullptr);



    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t , nullptr);

    return 0;
}
