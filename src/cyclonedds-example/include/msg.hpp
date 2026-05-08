#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

///////////////////////////
//         Header        //
///////////////////////////
namespace header {
    struct rt_header 
    {
        uint64_t seq;                   
        uint64_t timestamp_ns; 
    };
}

///////////////////////////
//      Joint State      //
///////////////////////////

namespace joint_state {
    struct rt_joint_state_msg
    {
        header::rt_header header;
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> efforts;

        explicit rt_joint_state_msg(size_t dofs = 0)
            : positions(dofs, 0.0)
            , velocities(dofs, 0.0)
            , efforts(dofs, 0.0)
        {}
    };
}


///////////////////////////
//          Imu          //
///////////////////////////

namespace imu {
    struct rt_orientation
    {
        double x = 0.0f;
        double y = 0.0f;
        double z = 0.0f;
        double w = 0.0f;
    };

    struct rt_vector3
    {
        double x = 0.0f;
        double y = 0.0f;
        double z = 0.0f;
    };

    struct rt_imu_msg
    {
        header::rt_header header;

        rt_vector3 linear_acceleration;
        rt_vector3 angular_velocity;
        rt_orientation orientation;

        uint32_t imu_ts      = 0; // internal IMU timestamp
        uint32_t temperature = 0;
        uint32_t digital_in  = 0;
        uint32_t fault       = 0;
        uint32_t rtt         = 0; // round-trip-time (us)
    };
}

///////////////////////////
//      Force Torque     //
///////////////////////////

namespace force_torque {
    struct rt_force {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct rt_torque {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct rt_wrench {
        rt_force force;
        rt_torque torque;
    };

    struct rt_force_torque_msg {
        header::rt_header header;
        rt_wrench wrench;

        uint32_t fault       = 0;
        uint32_t rtt         = 0;
        uint32_t op_idx_ack  = 0;
        float aux            = 0.0f;
    };
}

///////////////////////////
//      Power Board      //
///////////////////////////

namespace power_board {
    struct rt_power_board_msg
    {
        header::rt_header header;

        float v_batt            = 0.0f; 
        float v_load            = 0.0f;  
        float i_load            = 0.0f;  
        float temperature       = 0.0f;  
        float temp_batt         = 0.0f;  
        float temp_heatsink     = 0.0f;
        uint32_t status         = 0;
        uint32_t fault          = 0;
        uint32_t rtt            = 0;
        uint32_t op_idx_ack     = 0;
        float aux               = 0;
    };
}

///////////////////////////
//         Motor         //
///////////////////////////

namespace motor {
    struct rt_motor_msg
    {
        header::rt_header header;
        std::vector<std::string> name;

        std::vector<uint32_t>    statusword;
        std::vector<int32_t>     modes_of_op;
        std::vector<float>       motor_pos;
        std::vector<float>       motor_vel;
        std::vector<float>       link_pos;
        std::vector<float>       link_vel;
        std::vector<float>       current;
        std::vector<float>       torque;
        std::vector<float>       demanded_pos;
        std::vector<float>       demanded_vel;
        std::vector<float>       demanded_torque;
        std::vector<float>       demanded_current;
        std::vector<float>       control_effort;
        std::vector<float>       motor_temp;
        std::vector<int32_t>     drive_temp;
        std::vector<uint32_t>    error_code;
        std::vector<std::string> error_report;
        std::vector<uint32_t>    fault;
        std::vector<uint32_t>    rtt;

        explicit rt_motor_msg(size_t n = 0)
            : statusword        (n, 0)          
            , modes_of_op       (n, 0)
            , motor_pos         (n, 0.0f)        
            , motor_vel         (n, 0.0f)
            , link_pos          (n, 0.0f)         
            , link_vel          (n, 0.0f)
            , current           (n, 0.0f)          
            , torque            (n, 0.0f)
            , demanded_pos      (n, 0.0f)     
            , demanded_vel      (n, 0.0f)
            , demanded_torque   (n, 0.0f)  
            , demanded_current  (n, 0.0f)
            , control_effort    (n, 0.0f)   
            , motor_temp        (n, 0.0f)
            , drive_temp        (n, 0)          
            , error_code        (n, 0)
            , error_report      (n, "")
            , fault             (n, 0)               
            , rtt               (n, 0)
        {}
    };

    struct rt_motor
    {
        //header::rt_header header;
        uint32_t statusword       = 0;
        int32_t  modes_of_op      = 0;
        float    motor_pos        = 0.0f;
        float    motor_vel        = 0.0f;
        float    link_pos         = 0.0f;
        float    link_vel         = 0.0f;
        float    current          = 0.0f;
        float    torque           = 0.0f;
        float    demanded_pos     = 0.0f;
        float    demanded_vel     = 0.0f;
        float    demanded_torque  = 0.0f;
        float    demanded_current = 0.0f;
        float    control_effort   = 0.0f;
        float    motor_temp       = 0.0f;
        int32_t  drive_temp       = 0;
        uint32_t error_code       = 0;
        std::string error_report  = "";
        uint32_t fault            = 0;
        uint32_t rtt              = 0;
    };
}

///////////////////////////
//         Valve         //
///////////////////////////

namespace valve {
    struct rt_valve_msg
    {
        header::rt_header                header;
        std::vector<std::string> name;

        std::vector<float>    encoder_position;
        std::vector<float>    force;
        std::vector<float>    pressure1;
        std::vector<float>    pressure2;

        std::vector<float>    current;
        std::vector<float>    temperature;

        std::vector<float>    current_ref_fb;
        std::vector<float>    position_ref_fb;
        std::vector<float>    force_ref_fb;

        std::vector<uint32_t> fault;
        std::vector<uint32_t> rtt;
        std::vector<uint32_t> op_idx_ack;
        std::vector<float>    aux;

        explicit rt_valve_msg(size_t n = 0)
            : encoder_position(n, 0.f)
            , force           (n, 0.f)
            , pressure1       (n, 0.f)
            , pressure2       (n, 0.f)
            , current         (n, 0.f)
            , temperature     (n, 0.f)
            , current_ref_fb  (n, 0.f)
            , position_ref_fb (n, 0.f)
            , force_ref_fb    (n, 0.f)
            , fault           (n, 0)
            , rtt             (n, 0)
            , op_idx_ack      (n, 0)
            , aux             (n, 0.f)
        {}
    };

    struct rt_valve
    {
        header::rt_header header;
        float     encoder_position = 0.0f;
        float     force            = 0.0f;
        float     pressure1        = 0.0f;
        float     pressure2        = 0.0f;
        float     current          = 0.0f;
        float     temperature      = 0.0f;
        float     current_ref_fb   = 0.0f;
        float     position_ref_fb  = 0.0f;
        float     force_ref_fb     = 0.0f;
        uint32_t  fault            = 0;
        uint32_t  rtt              = 0;
        uint32_t  op_idx_ack       = 0;
        float     aux              = 0.0f;
    };
}

///////////////////////////
//          Pump         //
///////////////////////////

namespace pump {
    struct rt_pump_msg
    {
        header::rt_header header;
        float motor_current         = 0.0f;
        float motor_speed           = 0.0f;
        float pressure1             = 0.0f;
        float pressure2             = 0.0f;
        uint32_t temperature        = 0;
        uint32_t mosfet_temperature = 0;
        int32_t motor_temperature   = 0;
        uint32_t fault              = 0;
        uint32_t rtt                = 0;
        uint32_t op_idx_ack         = 0;
        float aux                   = 0.0f;
    };  
}