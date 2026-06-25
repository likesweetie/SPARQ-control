#ifndef RX_HANDLER_HPP
#define RX_HANDLER_HPP

#include "RobstrideMotor.hpp"

static constexpr float CAN_DATA_ZERO_SCALE = 1.0f / 32767.0f;

using Fb_f = void (*) (void* ,float,float,float,float) ;

template<int MaxID>
class Rx_handler{
    private:
    Fb_f jump_table[MaxID] = {nullptr};
    void* Motor_object[MaxID] = {nullptr} ;

    template <typename Tuple>
    void init_helper(Tuple& t){
        auto& v_rs00 = std::get<RS00_Vec>(t);
        auto& v_rs01 = std::get<RS01_Vec>(t);
        auto& v_rs02 = std::get<RS02_Vec>(t);
        auto& v_rs03 = std::get<RS03_Vec>(t);
        auto& v_rs04 = std::get<RS04_Vec>(t);
        auto& v_rs05 = std::get<RS05_Vec>(t);
        auto& v_rs06 = std::get<RS06_Vec>(t);
        auto& v_EL05 = std::get<EL05_Vec>(t);
    
        register_vector(v_rs00);
        register_vector(v_rs01);
        register_vector(v_rs02);
        register_vector(v_rs03);
        register_vector(v_rs04);
        register_vector(v_rs05);
        register_vector(v_rs06);
        register_vector(v_EL05);
    }
    

    template <typename V>
    void register_vector(V& vec){
        for (auto& motor : vec) {
            int id = motor.can_id;
            if (id >= 0 && id < MaxID) {
                // 주소 저장 (const 에러 방지를 위해 void* 캐스팅)
                Motor_object[id] = (void*)&motor;
    
                // 람다도 훨씬 간결하게!
                jump_table[id] = [](void* obj, float p, float v, float t, float tmp) {
                    // obj를 다시 해당 모터 타입으로 변환
                    using M_Type = typename V::value_type; 
                    auto* m = (M_Type*)obj;
                    
                    // 데이터 업데이트
                    m->write_FB(p, v, t, tmp);
                };
            }
        }
    }

    public:
    //const Motor_con MC;


    Rx_handler(Motor_con& MC){
        init_helper(MC);
    }


    std::tuple<uint32_t, uint32_t, float,float,float,float> parse_Rx_frame(struct can_frame* frame){  // 피드백 프레임을 적절히 맞춰주는. 29th bit of can id is error flag. i think we can use it 
        //frame->can_id
        uint32_t motor_id = (frame->can_id>>8)&0xff;       // 데이터는 각각 1바이트, 2바이트이긴 함
        uint32_t err_st = (frame->can_id>>16)&0b00111111;     // 에러가 났다면 비트 내부에 에러가 들어 있을건데 밖에서 읽으라지. [[unlikely]]로 에러 처리 부분을 떼 둘 것
    
        uint16_t p_raw = (frame->data[0] << 8) | frame->data[1];
        uint16_t v_raw = (frame->data[2] << 8) | frame->data[3];
        uint16_t t_raw = (frame->data[4] << 8) | frame->data[5];
    
        // (Raw / 32767.0 - 1.0) * GAIN
        float pos = static_cast<float>(p_raw) * CAN_DATA_ZERO_SCALE - 1.0f;
        float vel = (static_cast<float>(v_raw) * CAN_DATA_ZERO_SCALE - 1.0f) ; // 게인 추가
        float torq = (static_cast<float>(t_raw) * CAN_DATA_ZERO_SCALE - 1.0f);  // 나눗셈 대신 곱셈을 사용할 것이 권장됨 
        float temp = static_cast<float>((frame->data[6] << 8) | frame->data[7]) * 0.1f;
        // 근데 여기서 다 처리하려면 밖에서 뭐 탬플릿 받아오고 해야하는데 귀찮으니까 로우 데이터 던지고 밖에서 처리? <<솔직히 킹쁘지 않음 
        return {motor_id , err_st,pos,vel,torq,temp};
    }

    void Write_Fb(int id , float pos, float vel , float torq , float temp){
        if (id >= 0 && id < MaxID && jump_table[id] && Motor_object[id]) {
            jump_table[id](Motor_object[id] , pos, vel, torq , temp);
        }
    }
};

#endif