#ifndef __READPARAM_HPP
#define __READPARAM_HPP

#include <ros/ros.h>
#include <Eigen/Dense>

namespace ctrl_node
{

    class Parameter_t
    {
    private:
        template <typename TName, typename TVal>
        void read_param(const ros::NodeHandle &nh, const TName &name, TVal &val);

        // read_param 失败标志, 末尾汇总统一拒绝启动
        bool param_read_ok_ = true;

    public:
        struct Vel_Control_Pid
        {
            double Kvp0, Kvp1, Kvp2;
            double Kvd0, Kvd1, Kvd2;
        };

        struct Att_Control_Pid
        {
            double Kp0, Kp1, Kp2;
            double Ki0, Ki1, Ki2;
            double Kd0, Kd1, Kd2;
            double KAngp0, KAngp1, KAngp2;
            double KAngi0, KAngi1, KAngi2;
            double KAngd0, KAngd1, KAngd2;
        };

        struct Att_Control_Ude
        {
            double Trou0, Trou1, Trou2;
            double Tatt0, Tatt1, Tatt2;
        };

        struct Att_Control_Tvude
        {
            double Trou_min0, Trou_min1, Trou_min2;
            double Trou_max0, Trou_max1, Trou_max2;
            double t_z_min, t_z_max;
            double t_xy_min, t_xy_max;
            double t_min, t_max;
            double Tatt_min0, Tatt_min1, Tatt_min2;
            double Tatt_max0, Tatt_max1, Tatt_max2;
            double t_att_min, t_att_max;
        };

        struct Gain
        {
            Vel_Control_Pid vel;
            Att_Control_Pid att_pid;
            Att_Control_Ude att_ude;
            Att_Control_Tvude att_tvude;
        };

        struct AutoTakeoff
        {
            double height;
            double speed;
        };

        struct AutoLanding
        {
            // 旧版平滑下降兼容参数；当前 LAND 直接交接 PX4 AUTO.LAND。
            double speed;
            double target_height;
            double dis_arm_height;
        };

        struct kinematicsConstains
        {
            double vel_ver_max;
            double vel_hor_max;

            double acc_ver_max;
            double acc_hor_max;

            double tilt_max_deg;

            // MAVROS 输出前的体轴角速度安全限幅 [rad/s]
            double omega_roll_max;
            double omega_pitch_max;
            double omega_yaw_max;
        };

        struct FsmParam
        {
            int frequency;
        };

        struct MsgTimeOut
        {
            double odom;
            double imu;
            double state;
            double extended_state;
            double cmd;
        };

        struct ThrustMapping
        {
            // 在线 RLS 推力估计开关; false = 固定映射 gra/hover_percentage(验证阶段推荐)
            bool accurate_thrust_model;
            double hover_percentage;
            double thr2acc_min;
            double thr2acc_max;
        };

        int controller_type = 0; // 0:位置 1:速度 2:姿态 3:角速度控制
        int estimator_type = 0;  // 0:UDE 1:TVUDE 2:PD 3:TTV-UDE 4:PID
        int odom_source = 0;     // 0:线速度已在世界系; 1:机体系线速度, feed 时转世界系
        int yaw_mode = 1;        // 0:固定航向(0 rad); 1:轨迹切向航向
        double mass;
        double gra;

        Gain gain;
        kinematicsConstains kine_cons;
        AutoTakeoff takeoff_state;
        AutoLanding landing_state;
        FsmParam fsmparam;
        MsgTimeOut msg_timeout;
        ThrustMapping thr_map;

        // false = 参数缺失/非法, 调用方必须放弃启动
        bool config_from_ros_handle(const ros::NodeHandle &nh);
    };
}
#endif
