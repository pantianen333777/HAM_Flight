#include "fsm_nodelet.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <trajectory_math/rest_to_rest.hpp>

namespace ctrl_node
{

    namespace
    {
        // 降落最短时长 [s], 防极小高度差产生阶跃
        constexpr double kMinLandDuration = 2.0;
        // 手工 LAND 先退出轨迹并减速；RTK 悬停速度存在厘米级噪声，0.1m/s
        // 的单帧门限过严。若 3s 内仍未达到门限，也必须把降落请求交给 PX4。
        constexpr double kLandHorizontalSpeedMax = 0.20;
        constexpr double kLandBrakingTimeout = 3.0;

        // 平滑核(smoothP/V/A/clamp01)改用公共头 trajectory_math/rest_to_rest.hpp,
        // 与 trajectory_utils 共用同一实现, 避免两份副本分叉。
        using trajectory_math::clamp01;
        using trajectory_math::smoothA;
        using trajectory_math::smoothP;
        using trajectory_math::smoothV;
    }

    void FSM::update_navigation_failsafe(const ros::Time &now)
    {
        if (!navigation_monitor_enabled_ || !state_data.current_state.armed ||
            current_state == MANUAL || current_state == LAND)
        {
            navigation_unhealthy_since_ = ros::Time(0);
            return;
        }

        if (navigation_status_is_healthy(now))
        {
            navigation_unhealthy_since_ = ros::Time(0);
            return;
        }

        if (navigation_unhealthy_since_.isZero())
        {
            navigation_unhealthy_since_ = now;
            ROS_ERROR("[FSM]: RTK/EKF unhealthy (fix=%d, horiz=%d, vert=%d, glitch=%d); grace timer started.",
                      current_rtk_fix_type_, ekf_pos_horiz_abs_ok_,
                      ekf_pos_vert_abs_ok_, ekf_gps_glitch_);
            return;
        }

        const double unhealthy_duration =
            (now - navigation_unhealthy_since_).toSec();
        if (!std::isfinite(unhealthy_duration))
        {
            return;
        }
        const NavigationFailsafeStage failsafe_stage =
            classify_navigation_failsafe(
                unhealthy_duration,
                navigation_failure_grace_,
                navigation_auto_land_timeout_);
        if (failsafe_stage == NavigationFailsafeStage::Grace)
        {
            return;
        }

        if (current_state == TAKEOFF || current_state == MISSION)
        {
            const CurrentState failed_state = current_state;
            set_hov_with_odom();
            mission_trigger_data.deactivate();
            mission_trigger_data.cancelPending();
            cmd_data.invalidate();
            cancel_planner_handshake();
            current_state = HOVER;
            ROS_ERROR("[FSM]: RTK/EKF unhealthy for %.2f s; state(%d) --> HOVER while waiting for recovery.",
                      unhealthy_duration, static_cast<int>(failed_state));
        }

        if (current_state == HOVER &&
            failsafe_stage == NavigationFailsafeStage::AutoLand)
        {
            set_hov_with_odom();
            mission_trigger_data.deactivate();
            mission_trigger_data.cancelPending();
            cmd_data.invalidate();
            cancel_planner_handshake();
            land_gate_.reset();
            current_state = LAND;
            ROS_ERROR("[FSM]: RTK/EKF unhealthy for %.2f s; HOVER --> LAND and requesting PX4 AUTO.LAND.",
                      unhealthy_duration);
        }
        else if (current_state == HOVER)
        {
            ROS_ERROR_THROTTLE(1.0,
                               "[FSM]: RTK/EKF remains unhealthy in HOVER; AUTO.LAND in %.2f s unless recovered (fix=%d, horiz=%d, vert=%d, glitch=%d).",
                               std::max(0.0, navigation_auto_land_timeout_ - unhealthy_duration),
                               current_rtk_fix_type_, ekf_pos_horiz_abs_ok_,
                               ekf_pos_vert_abs_ok_, ekf_gps_glitch_);
        }
    }

    void FSM::handle_critical_input_loss(const ros::Time &now,
                                         const char *input_name,
                                         bool state_feedback_fresh,
                                         bool arm_result_accepted)
    {
        const bool controller_flight_active =
            current_state == TAKEOFF || current_state == HOVER ||
            current_state == MISSION || current_state == LAND;
        // 输入变陈旧时 ARM 服务可能已在工作线程执行, 无法安全取消;
        // 在其后排队 AUTO.LAND, 迟到的 ARM 成功会被立即压制。
        const bool arm_transition_pending =
            arm_result_accepted || service_request_pending(ServiceKind::Arm);
        const bool offboard = state_data.current_state.mode == "OFFBOARD";
        const bool auto_land_required =
            critical_input_loss_requires_auto_land(
                state_feedback_fresh,
                state_data.current_state.armed,
                offboard,
                controller_flight_active,
                arm_transition_pending);

        takeoff_trigger_received = false;
        land_trigger_received = false;
        land_trigger_received_at_ = ros::Time(0);
        mission_trigger_data.deactivate();
        mission_trigger_data.cancelPending();
        cmd_data.invalidate();
        if (expected_trajectory_id_ != 0U)
        {
            cancel_planner_handshake();
        }

        if (state_feedback_fresh &&
            state_data.current_state.mode == "AUTO.LAND")
        {
            critical_input_failsafe_active_ = false;
            current_state = MANUAL;
            ROS_INFO_THROTTLE(
                1.0,
                "[FSM]: %s unavailable, but PX4 already reports AUTO.LAND; "
                "controller output remains stopped.",
                input_name);
            return;
        }

        if (!auto_land_required)
        {
            critical_input_failsafe_active_ = false;
            current_state = MANUAL;
            ROS_WARN_THROTTLE(
                1.0,
                "[FSM]: %s unavailable outside armed OFFBOARD flight; "
                "control output stopped without forcing a mode change.",
                input_name);
            return;
        }

        if (current_state != LAND)
        {
            current_state = LAND;
            land_gate_.reset();
            ROS_ERROR(
                "[FSM]: critical %s loss during flight; entering LAND and "
                "requesting PX4 AUTO.LAND without using stale control data.",
                input_name);
        }
        else
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: critical %s loss persists; controller output stopped "
                "and AUTO.LAND request remains active.",
                input_name);
        }

        critical_input_failsafe_active_ = true;

        // 复用既有异步 service 工作线程, 绝不阻塞 FSM 队列;
        // 节流门耗尽后仍继续重试 —— 本路径降落必须完成。
        if (land_gate_.exhausted())
        {
            ROS_ERROR("[FSM]: emergency AUTO.LAND retries exhausted; resetting retry gate.");
            land_gate_.reset();
        }
        if (!service_request_pending(ServiceKind::AutoLand) &&
            land_gate_.should_attempt(now) &&
            !enqueue_service_request(ServiceKind::AutoLand))
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: failed to queue emergency AUTO.LAND service request.");
        }
    }

    void FSM::fsm_timer(const ros::TimerEvent &event)
    {

        ros::Time now_time = ros::Time::now();
        SrvResult arm_result = SrvResult::TransportError;
        SrvResult land_result = SrvResult::TransportError;
        const bool arm_result_ready =
            take_service_result(ServiceKind::Arm, arm_result);
        const bool land_result_ready =
            take_service_result(ServiceKind::AutoLand, land_result);
        if (arm_result_ready)
        {
            arm_gate_.update(now_time, arm_result);
        }
        if (land_result_ready)
        {
            land_gate_.update(now_time, land_result);
        }

        if (!state_is_received(now_time))
        {
            handle_critical_input_loss(
                now_time, "MAVROS state", false,
                arm_result_ready && arm_result == SrvResult::Accepted);
            return;
        }

        // PX4 连接检查: fsm_timer 与 state feed 同在 fsm_queue_ 单线程串行, 读无竞态
        if (!state_data.current_state.connected)
        {
            handle_critical_input_loss(
                now_time, "PX4 connection", true,
                arm_result_ready && arm_result == SrvResult::Accepted);
            return;
        }

        if (!odom_is_received(now_time))
        {
            handle_critical_input_loss(
                now_time, "odometry", true,
                arm_result_ready && arm_result == SrvResult::Accepted);
            return;
        }

        if (!imu_is_received(now_time))
        {
            handle_critical_input_loss(
                now_time, "IMU", true,
                arm_result_ready && arm_result == SrvResult::Accepted);
            return;
        }

        // 关键输入丢失一旦发生, 不会因数据流恢复一拍就恢复 setpoint 发布:
        // 旧悬停参考可能已不安全。持续请求 AUTO.LAND, 直到 PX4 确认/上锁/飞手退出 OFFBOARD。
        if (critical_input_failsafe_active_)
        {
            handle_critical_input_loss(
                now_time, "latched critical input", true,
                arm_result_ready && arm_result == SrvResult::Accepted);
            return;
        }

        // OFFBOARD 模式切换检测
        if (state_data.previous_state.mode != state_data.current_state.mode)
        {
            if (state_data.current_state.mode == "OFFBOARD")
            {
                ROS_INFO("\033[32m[FSM]:Switch to OFFBOARD mode!\033[32m");
            }
            else if (state_data.previous_state.mode == "OFFBOARD")
            {
                ROS_INFO("\033[32m[FSM]:Exit OFFBOARD mode! Switch to %s state.\033[32m", state_data.current_state.mode.c_str());
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cancel_planner_handshake();
                // LAND 对 AUTO.LAND 使用专门的确认分支；其他模式退出仍立即回 MANUAL。
                if (!(current_state == LAND &&
                      state_data.current_state.mode == "AUTO.LAND"))
                {
                    current_state = MANUAL;
                }
            }
            state_data.previous_state = state_data.current_state;
        }

        Controller::Desired_State_t des(odom_data);
        Controller::Controller_Output_t u;

        update_navigation_failsafe(now_time);

        // LAND 触发是保持型请求：MISSION 先退出轨迹并在当前点制动，达到速度门限
        // 后交接 PX4；请求不会再因某一帧 RTK 速度噪声而被永久丢弃。
        if (land_trigger_received)
        {
            const bool valid_flight_state =
                current_state == HOVER || current_state == MISSION;
            const bool offboard_and_armed =
                state_data.current_state.mode == "OFFBOARD" &&
                state_data.current_state.armed;

            if (!valid_flight_state || !offboard_and_armed)
            {
                ROS_ERROR("[FSM]:Land trigger rejected "
                          "(need HOVER/MISSION + OFFBOARD + armed), discarded.");
                land_trigger_received = false;
                land_trigger_received_at_ = ros::Time(0);
            }
            else
            {
                if (current_state == MISSION)
                {
                    set_hov_with_odom();
                    mission_trigger_data.deactivate();
                    mission_trigger_data.cancelPending();
                    cmd_data.invalidate();
                    cancel_planner_handshake();
                    current_state = HOVER;
                    ROS_WARN("[FSM]: LAND requested during MISSION; "
                             "MISSION --> HOVER for braking.");
                }

                const double horizontal_speed =
                    odom_data.v.head<2>().norm();
                const double waiting_time =
                    land_trigger_received_at_.isZero()
                        ? 0.0
                        : (now_time - land_trigger_received_at_).toSec();
                const bool braking_timeout =
                    std::isfinite(waiting_time) &&
                    waiting_time >= kLandBrakingTimeout;

                if (horizontal_speed <= kLandHorizontalSpeedMax ||
                    braking_timeout)
                {
                    if (braking_timeout &&
                        horizontal_speed > kLandHorizontalSpeedMax)
                    {
                        ROS_WARN("[FSM]: LAND braking timeout after %.2fs "
                                 "(horizontal speed=%.3fm/s); requesting "
                                 "PX4 AUTO.LAND.",
                                 waiting_time, horizontal_speed);
                    }

                    set_hov_with_odom();
                    land_gate_.reset();
                    mission_trigger_data.deactivate();
                    mission_trigger_data.cancelPending();
                    cmd_data.invalidate();
                    cancel_planner_handshake();
                    current_state = LAND;
                    land_trigger_received = false;
                    land_trigger_received_at_ = ros::Time(0);
                    ROS_INFO("\033[32m[FSM] HOVER --> LAND(L5); requesting "
                             "PX4 AUTO.LAND.\033[32m");
                }
                else
                {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[FSM]: LAND pending while braking "
                        "(horizontal speed=%.3fm/s, threshold=%.3fm/s).",
                        horizontal_speed, kLandHorizontalSpeedMax);
                }
            }
        }

        switch (current_state)
        {
        case MANUAL:
            if (state_data.current_state.mode == "OFFBOARD")
            {
                if (takeoff_trigger_received)
                {
                    if (arm_result_ready)
                    {
                        if (arm_result == SrvResult::Accepted)
                        {
                            // 禁止上一任务缓存的 PositionCommand 参与本次起飞。
                            cmd_data.rcv_stamp = ros::Time(0);
                            mission_trigger_data.deactivate();
                            current_state = TAKEOFF;
                            att_ang_controller.resetThrustMapping();
                            set_start_pose_for_takeoff(odom_data);
                            takeoff_trigger_received = false;
                            ROS_INFO("\033[32m[FSM] MANUAL --> TAKEOFF(L1)\033[32m");
                        }
                        else if (arm_result == SrvResult::Rejected || arm_gate_.exhausted())
                        {
                            ROS_ERROR("[FSM]:takeoff arming permanently rejected / retries exhausted; abort takeoff.");
                            takeoff_trigger_received = false;
                            arm_gate_.reset();
                        }
                    }
                    else if (service_request_pending(ServiceKind::Arm))
                    {
                        // 服务工作线程正在处理；控制循环继续发布 setpoint。
                    }
                    else if (!navigation_status_is_healthy(now_time))
                    {
                        // 实飞 RTK 监控开启时，TAKEOFF 必须在请求 ARM 之前就通过
                        // GPS Fixed + EKF 绝对位置新鲜度门禁，禁止“先解锁再触发降落”。
                        ROS_ERROR("[FSM]: Reject Auto_Takeoff: RTK/EKF is not healthy "
                                  "(fix=%d, horiz=%d, vert=%d, glitch=%d).",
                                  current_rtk_fix_type_, ekf_pos_horiz_abs_ok_,
                                  ekf_pos_vert_abs_ok_, ekf_gps_glitch_);
                        takeoff_trigger_received = false;
                        arm_gate_.reset();
                    }
                    else if (state_data.current_state.system_status != 3U)
                    {
                        // MAV_STATE_STANDBY(3) is the normal pre-arm state.
                        // UNINIT(0)/FLIGHT_TERMINATION(8) cannot be recovered
                        // by repeatedly sending ARM requests.
                        ROS_ERROR("[FSM]: Reject Auto_Takeoff: PX4 system_status=%u, expected STANDBY(3). Reboot PX4 and clear termination/pre-arm failures.",
                                  static_cast<unsigned int>(state_data.current_state.system_status));
                        takeoff_trigger_received = false;
                        arm_gate_.reset();
                    }
                    else if (!extended_state_is_received(now_time))
                    {
                        ROS_WARN_THROTTLE(1.0, "[FSM]: waiting for fresh MAVROS extended state before arming.");
                    }
                    else if (extended_state_data.current_extended_state.landed_state !=
                             mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
                    {
                        ROS_ERROR("[FSM]:Reject Auto_Takeoff, vehicle is not confirmed on ground.");
                        takeoff_trigger_received = false;
                    }
                    else if (state_data.current_state.armed)
                    {
                        ROS_ERROR("[FSM]:Reject Auto_Takeoff, vehicle is already armed!");
                        takeoff_trigger_received = false; // 已解锁,停止重试
                    }
                    else if (arm_gate_.should_attempt(now_time))
                    {
                        if (!enqueue_service_request(ServiceKind::Arm))
                        {
                            ROS_ERROR_THROTTLE(1.0, "[FSM]:failed to queue ARM service request.");
                        }
                    }
                }
                else if (state_data.current_state.armed)
                { // already in flight
                    mission_trigger_data.deactivate();
                    current_state = HOVER;
                    att_ang_controller.resetThrustMapping();
                    set_hov_with_odom();
                    ROS_INFO("\033[32m[FSM] MANUAL(L1) --> HOVER(L2)\033[32m");
                }
            }
            break;

        case TAKEOFF:
            if (state_data.current_state.mode != "OFFBOARD")
            {
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cancel_planner_handshake();
                current_state = MANUAL;
                ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, TAKEOFF --> MANUAL(L2)\033[32m");
            }
            else if (!state_data.current_state.armed &&
                     (now_time - takeoff_state.toggle_takeoff_time).toSec() >=
                         AutoTakeoff_t::MOTORS_SPEEDUP_TIME)
            {
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cancel_planner_handshake();
                current_state = MANUAL;
                ROS_ERROR("[FSM]: vehicle did not report armed after ARM acceptance; takeoff aborted.");
            }
            else if ((now_time - takeoff_state.toggle_takeoff_time).toSec() >
                     AutoTakeoff_t::MOTORS_SPEEDUP_TIME +
                         param.takeoff_state.height / param.takeoff_state.speed +
                         takeoff_timeout_margin_)
            {
                // 没有到达指定高度时不得把当前低空位置当成新的
                // 悬停点。旧逻辑会让飞机贴地停住，同时永远不发 READY，
                // 表面上看就是“起飞不到 1m + 轨迹没反应”。起飞超时
                // 本身已证明高度控制/估计不可信，安全动作是锁存 LAND
                // 并限频请求 PX4 AUTO.LAND。
                set_hov_with_odom();
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cmd_data.invalidate();
                cancel_planner_handshake();
                land_gate_.reset();
                current_state = LAND;
                ROS_ERROR("[FSM]: takeoff timeout before stable target height; TAKEOFF --> LAND and requesting PX4 AUTO.LAND.");
            }
            else if ((now_time - takeoff_state.toggle_takeoff_time).toSec() < AutoTakeoff_t::MOTORS_SPEEDUP_TIME)
            {
                des = get_pv_speed_up_des(now_time);
            }
            else
            {
                des = get_takeoff_des(odom_data);
                const double target_z =
                    takeoff_state.start_pose(2) + param.takeoff_state.height;
                const bool height_stable =
                    std::fabs(odom_data.p(2) - target_z) <=
                    AutoTakeoff_t::HEIGHT_TOLERANCE;
                const bool velocity_stable =
                    std::fabs(odom_data.v(2)) <=
                    AutoTakeoff_t::VERTICAL_SPEED_TOLERANCE;

                if (height_stable && velocity_stable)
                {
                    if (takeoff_state.settled_since.isZero())
                        takeoff_state.settled_since = now_time;

                    if ((now_time - takeoff_state.settled_since).toSec() >=
                        AutoTakeoff_t::SETTLE_TIME)
                    {
                        // Keep the exact requested target. Capturing odometry
                        // at target-0.10 m made the following hover too low.
                        hover_pose = takeoff_state.start_pose;
                        hover_pose(2) = target_z;
                        takeoff_state.delay_trigger.first = true;
                        takeoff_state.delay_trigger.second =
                            now_time + ros::Duration(AutoTakeoff_t::DELAY_TRIGGER_TIME);
                        current_state = HOVER;
                        ROS_INFO("\033[32m[FSM] TAKEOFF --> HOVER(L2), exact target z=%.3f\033[32m",
                                 target_z);
                    }
                }
                else
                {
                    takeoff_state.settled_since = ros::Time(0);
                }
            }
            break;

        case HOVER:
            if (state_data.current_state.mode != "OFFBOARD")
            {
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cancel_planner_handshake();
                current_state = MANUAL;
                ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, HOVER(L2) --> MANUAL(L2)\033[32m");
            }
            else if (cmd_is_received(now_time) && state_data.current_state.armed)
            {
                if (!navigation_status_is_healthy(now_time))
                {
                    mission_trigger_data.cancelPending();
                    cmd_data.invalidate();
                    cancel_planner_handshake();
                    des = get_hover_des();
                    ROS_ERROR_THROTTLE(1.0,
                                       "[FSM]: mission command rejected because RTK/EKF is unhealthy.");
                }
                else if (mission_trigger_data.activatePending(
                             now_time, mission_trigger_timeout_))
                {
                    current_state = MISSION;
                    des = get_cmd_des();
                    ROS_INFO("\033[32m[FSM] HOVER --> MISSION(L3)\033[32m");
                }
                else
                {
                    des = get_hover_des();
                    ROS_WARN_THROTTLE(1.0,
                                      "[FSM]: matching trajectory command arrived before the mission trigger callback; waiting in HOVER.");
                }
            }
            else if (!state_data.current_state.armed)
            {
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cancel_planner_handshake();
                current_state = MANUAL;
                ROS_INFO("\033[32m[FSM] HOVER --> MANUAL(L2)\033[32m");
            }
            else
            {
                des = get_hover_des();

                if (takeoff_state.delay_trigger.first && now_time > takeoff_state.delay_trigger.second)
                {
                    takeoff_state.delay_trigger.first = false;
                    publish_trigger(odom_data);
                    ROS_INFO("\033[32m[FSM]:TRIGGER sent, allow user command.\033[32m");
                }
            }
            break;

        case MISSION:
            if (state_data.current_state.mode != "OFFBOARD")
            {
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cancel_planner_handshake();
                current_state = MANUAL;
                ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, MISSION --> MANUAL(L2)\033[32m");
            }
            else if (!state_data.current_state.armed)
            {
                mission_trigger_data.deactivate();
                mission_trigger_data.cancelPending();
                cmd_data.invalidate();
                cancel_planner_handshake();
                current_state = MANUAL;
                ROS_ERROR("[FSM]: vehicle disarmed during MISSION; MISSION --> MANUAL.");
            }
            else if (!cmd_is_received(now_time))
            {
                mission_trigger_data.deactivate();
                current_state = HOVER;
                set_hov_with_odom();
                des = get_hover_des();
                ROS_INFO("\033[32m[FSM] MISSION --> HOVER(L4) \033[32m");
            }
            else
            {
                des = get_cmd_des();
            }
            break;

        case LAND:
            // 在 PX4 接受 AUTO.LAND 前保持进入 LAND 时捕获的位置。当前工程
            // 没有可靠的表面高度/AGL 输入，不能继续以世界系 z=0 作为交接门限。
            des = get_hover_des();
            if (!state_data.current_state.armed)
            {
                current_state = MANUAL;
                land_gate_.reset();
                ROS_INFO("\033[32m[FSM] LAND --> MANUAL (PX4 disarm confirmed)\033[32m");
                // 已上锁时本周期不再发布姿态/推力指令。
                return;
            }
            else if (state_data.current_state.mode == "AUTO.LAND")
            {
                // mode confirmation 不等于降落完成。必须保持 LAND 锁存到
                // PX4 真正上锁；同时停止 OFFBOARD setpoint 流，防止降落
                // 途中外部模式源把 PX4 重新切回 OFFBOARD。
                ROS_INFO_THROTTLE(
                    5.0,
                    "[FSM]: PX4 AUTO.LAND confirmed; LAND remains latched "
                    "until disarm and OFFBOARD output is inhibited.");
                return;
            }
            else if (state_data.current_state.mode != "OFFBOARD")
            {
                current_state = MANUAL;
                ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, LAND --> MANUAL\033[32m");
                // 飞手/PX4 已切入其他模式，不应再发 OFFBOARD setpoint。
                return;
            }
            else
            {
                if (land_result_ready)
                {
                    if (land_result == SrvResult::Accepted)
                    {
                        // mode_sent 只表示请求已发送；保持 LAND/悬停并继续限频请求，
                        // 直到 /mavros/state 明确观察到 AUTO.LAND。
                        ROS_INFO_THROTTLE(1.0,
                                          "[FSM]: AUTO.LAND request sent; waiting for mode confirmation.");
                    }
                    else if (land_gate_.exhausted())
                    {
                        // 重试耗尽: 降落必须完成, 不能放弃 → reset gate 继续尝试, 避免永久卡死。
                        ROS_ERROR("[FSM]:AUTO.LAND retries exhausted; reset gate and keep retrying (landing is mandatory).");
                        land_gate_.reset();
                    }
                }
                else if (!service_request_pending(ServiceKind::AutoLand) &&
                         land_gate_.should_attempt(now_time))
                {
                    if (!enqueue_service_request(ServiceKind::AutoLand))
                    {
                        ROS_ERROR_THROTTLE(1.0, "[FSM]:failed to queue AUTO.LAND service request.");
                    }
                }
            }
            break;
        }

        if ((current_state == HOVER || current_state == MISSION) &&
            param.thr_map.accurate_thrust_model)
        {
            // 在线 RLS 推力辨识仅 accurate_thrust_model=true 时执行;
            // 验证飞行阶段保持 false(固定映射)。
            att_ang_controller.estimateThrustModel(imu_data.a, param);
        }

        switch (param.controller_type)
        {
        case 0:
            // TODO: 位置控制器直发模式
            debug_msg = pos_controller.calculateControl(des, odom_data, u);
            publish_position_ctrl(u, now_time);
            break;

        case 1:
            debug_msg = vel_controller.calculateControl(des, odom_data, u, now_time);
            publish_velocity_ctrl(u, now_time);
            break;

        case 2:
            debug_msg = att_ang_controller.calculateControl(des, mission_trigger_data, odom_data, imu_data, u, now_time);
            publish_attitude_ctrl(u, now_time);
            break;

        case 3:
            debug_msg = att_ang_controller.calculateControlCMD(des, mission_trigger_data, odom_data, imu_data, u, now_time);
            publish_bodyrate_ctrl(u, now_time);
            break;
        default:
            // validate() 已在启动时拦截非法 controller_type; 此处为防御性第二道防线。
            ROS_ERROR_THROTTLE(1.0, "[FSM]: unknown controller_type=%d, no control published this cycle;",
                               param.controller_type);
            break;
        }

        debug_msg.header.stamp = now_time;
        debug_pub.publish(debug_msg);

        // takeoff_trigger_received 仅在起飞成功/已解锁时清除(见 MANUAL 分支), 支持 ARM 被拒后重试
    }

    void FSM::publish_trigger(const Odom_Data_t &odom)
    {
        // 轨迹代次经 header.frame_id("world:<id>") 传给规划器，规划器原样写入
        // PositionCommand::trajectory_id。不可用 header.seq 携带：roscpp 在
        // publish 时会用连接级自增计数覆盖 seq，订阅端读不到此处写入的值。
        // 0 保留为“无有效握手”；回绕时主动跳过。
        ++trajectory_generation_;
        if (trajectory_generation_ == 0U)
        {
            ++trajectory_generation_;
        }
        expected_trajectory_id_ = trajectory_generation_;

        // 位置使用起飞完成时的实时 odom；航向必须使用接收
        // TAKEOFF 时锁定的起飞初始航向。这样轨迹的“前方”不会
        // 因悬停期间的短暂偏航而改变。
        const double locked_takeoff_yaw = takeoff_state.start_pose(3);
        const Eigen::Quaterniond locked_q(
            Eigen::AngleAxisd(locked_takeoff_yaw, Eigen::Vector3d::UnitZ()));
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world:" + std::to_string(expected_trajectory_id_);
        msg.pose.position.x = odom.p.x();
        msg.pose.position.y = odom.p.y();
        msg.pose.position.z = odom.p.z();
        msg.pose.orientation.x = locked_q.x();
        msg.pose.orientation.y = locked_q.y();
        msg.pose.orientation.z = locked_q.z();
        msg.pose.orientation.w = locked_q.w();

        traj_start_trigger_pub.publish(msg);
        ROS_INFO("[FSM]: planner handshake published (trajectory_id=%u, locked_takeoff_yaw=%.1f deg).",
                 static_cast<unsigned int>(expected_trajectory_id_),
                 locked_takeoff_yaw * 180.0 / M_PI);
    }

    void FSM::cancel_planner_handshake()
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "cancel";
        traj_start_trigger_pub.publish(msg);
        expected_trajectory_id_ = 0U;
    }

    void FSM::publish_position_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp)
    {
        if (!u.position.allFinite() || !std::isfinite(u.yaw))
        {
            ROS_ERROR_THROTTLE(1.0, "[FSM]: invalid position command blocked.");
            return;
        }
        mavros_msgs::PositionTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.position.x = u.position.x();
        msg.position.y = u.position.y();
        msg.position.z = u.position.z();
        msg.yaw = u.yaw;

        ctrl_pv_pub.publish(msg);
    }

    void FSM::publish_velocity_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp)
    {
        if (!u.velocity.allFinite() || !std::isfinite(u.yaw))
        {
            ROS_ERROR_THROTTLE(1.0, "[FSM]: invalid velocity command blocked.");
            return;
        }
        mavros_msgs::PositionTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.velocity.x = u.velocity.x();
        msg.velocity.y = u.velocity.y();
        msg.velocity.z = u.velocity.z();
        msg.yaw = u.yaw;

        ctrl_pv_pub.publish(msg);
    }

    double FSM::sanitize_thrust(double thrust) const
    {
        if (!std::isfinite(thrust))
        {
            ROS_ERROR_THROTTLE(1.0,
                               "[FSM]: non-finite thrust command blocked; using hover thrust.");
            thrust = param.thr_map.hover_percentage;
        }

        const double clipped = std::max(0.0, std::min(1.0, thrust));
        if (std::abs(clipped - thrust) > 1e-9)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[FSM]: thrust command %.3f clipped to normalized range [0, 1].",
                              thrust);
        }
        return clipped;
    }

    // MAVROS 最终 body-rate 安全防火墙
    Eigen::Vector3d FSM::sanitize_bodyrates(
        const Eigen::Vector3d &bodyrates) const
    {
        Eigen::Vector3d rates =
            bodyrates;

        // 非有限保护
        if (!rates.allFinite())
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: non-finite body-rate command blocked; "
                "using zero body rates.");

            rates.setZero();

            return rates;
        }

        // 物理体速率限幅 (参数本身做最后防御)
        const Eigen::Vector3d rate_limit(
            param.kine_cons.omega_roll_max,
            param.kine_cons.omega_pitch_max,
            param.kine_cons.omega_yaw_max);

        if (!rate_limit.allFinite() ||
            (rate_limit.array() <= 0.0).any())
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: invalid body-rate safety limits; "
                "using zero body rates.");

            rates.setZero();

            return rates;
        }

        const Eigen::Vector3d clipped =
            rates
                .cwiseMax(-rate_limit)
                .cwiseMin(rate_limit);

        if ((clipped - rates).norm() > 1e-9)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "[FSM]: body-rate command saturated: "
                "raw=[%.3f %.3f %.3f] rad/s, "
                "limited=[%.3f %.3f %.3f] rad/s.",
                rates.x(),
                rates.y(),
                rates.z(),
                clipped.x(),
                clipped.y(),
                clipped.z());
        }

        return clipped;
    }

    // MAVROS 最终姿态四元数安全防火墙
    bool FSM::sanitize_attitude(
        const Eigen::Quaterniond &q_in,
        Eigen::Quaterniond &q_out) const
    {
        // 非有限
        if (!q_in.coeffs().allFinite())
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: non-finite attitude quaternion blocked.");

            return false;
        }

        // 非零四元数
        const double q_norm =
            q_in.norm();

        if (!std::isfinite(q_norm) ||
            q_norm < 1e-6)
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: invalid attitude quaternion norm=%.6e.",
                q_norm);

            return false;
        }

        // 归一化后再交 MAVROS
        q_out =
            q_in.normalized();

        if (!q_out.coeffs().allFinite())
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: quaternion normalization produced "
                "non-finite result.");

            return false;
        }

        return true;
    }

    void FSM::publish_bodyrate_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp)
    {
        mavros_msgs::AttitudeTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

        // body-rate 防火墙
        const Eigen::Vector3d safe_rates = sanitize_bodyrates(u.bodyrates);

        msg.body_rate.x = safe_rates.x();
        msg.body_rate.y = safe_rates.y();
        msg.body_rate.z = safe_rates.z();

        msg.thrust = sanitize_thrust(u.thrust);

        ctrl_aw_pub.publish(msg);
    }

    void FSM::publish_attitude_ctrl(const Controller::Controller_Output_t &u, const ros::Time &stamp)
    {
        mavros_msgs::AttitudeTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
        // 不再 IGNORE_YAW_RATE: 附带期望 yaw_rate 前馈, 消除恒速 yaw 跟踪稳态误差

        Eigen::Quaterniond safe_q;

        // 姿态防火墙: 非法时回退当前 IMU 姿态, 而非发给 PX4
        if (!sanitize_attitude(
                u.q,
                safe_q))
        {
            ROS_ERROR_THROTTLE(
                1.0,
                "[FSM]: invalid desired attitude; "
                "trying current IMU attitude.");

            if (!sanitize_attitude(
                    imu_data.q,
                    safe_q))
            {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[FSM]: desired and current IMU attitude "
                    "are both invalid; "
                    "AttitudeTarget is NOT published.");

                return;
            }
        }

        // body-rate 防火墙: attitude 模式只有 yaw rate 生效, 但统一走三轴 sanitizer
        const Eigen::Vector3d safe_rates =
            sanitize_bodyrates(
                u.bodyrates);

        msg.orientation.x =
            safe_q.x();

        msg.orientation.y =
            safe_q.y();

        msg.orientation.z =
            safe_q.z();

        msg.orientation.w =
            safe_q.w();

        msg.body_rate.z =
            safe_rates.z();

        msg.thrust =
            sanitize_thrust(
                u.thrust);

        ctrl_aw_pub.publish(msg);
    }

    SrvResult FSM::toggle_arm_disarm(bool arm)
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;

        // TransportError: 服务调用本身失败(mavros 不通)
        if (!arming_client_srv.call(arm_cmd))
        {
            ROS_ERROR_THROTTLE(1.0, "[%s] service call failed (mavros unreachable?).",
                               arm ? "ARM" : "DISARM");
            return SrvResult::TransportError;
        }

        // MAV_RESULT: 0=ACCEPTED, 1=TEMPORARILY_REJECTED, 2=DENIED, 3=NOT_SUPPORTED, 4=IN_PROGRESS
        const int result = arm_cmd.response.result;
        if (arm_cmd.response.success && result == 0) // MAV_RESULT_ACCEPTED
            return SrvResult::Accepted;

        if (result == 1 || result == 4) // TEMPORARILY_REJECTED / IN_PROGRESS → 可重试
        {
            ROS_ERROR_THROTTLE(1.0, "[%s] temporarily rejected (MAV_RESULT=%d), retryable.",
                               arm ? "ARM" : "DISARM", result);
            return SrvResult::Retryable;
        }

        // DENIED(2, 预飞检查未通过) / NOT_SUPPORTED(3) → 永久拒绝
        ROS_ERROR("[%s] DENIED by PX4 (MAV_RESULT=%d; pre-arm check / not supported).",
                  arm ? "ARM" : "DISARM", result);
        return SrvResult::Rejected;
    }

    void FSM::set_hov_with_odom()
    {
        hover_pose.head<3>() = odom_data.p;
        hover_pose(3) = Controller::q2yaw(odom_data.q); // get yaw
        std::cout << "hover_pose = " << odom_data.p.reverse() << std::endl;
    }

    void FSM::set_start_pose_for_takeoff(const Odom_Data_t &odom)
    {
        takeoff_state.start_pose.head<3>() = odom.p;
        takeoff_state.start_pose(3) = Controller::q2yaw(odom.q); // get yaw

        takeoff_state.toggle_takeoff_time = ros::Time::now();
        takeoff_state.settled_since = ros::Time(0);
    }

    Controller::Desired_State_t FSM::get_pv_speed_up_des(const ros::Time &now)
    {
        double delta_t = (now - takeoff_state.toggle_takeoff_time).toSec();
        double des_a_z = exp((delta_t - AutoTakeoff_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // 6.0/7.0 为经验值, 曲线满意即可
        if (des_a_z > 0.1)
        {
            ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
            des_a_z = 0.0;
        }

        Controller::Desired_State_t des;
        des.p = takeoff_state.start_pose.head<3>();
        des.v = Eigen::Vector3d(0, 0, 0);
        des.a = Eigen::Vector3d(0, 0, des_a_z);
        des.j = Eigen::Vector3d::Zero();
        des.yaw = takeoff_state.start_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_takeoff_des(const Odom_Data_t &odom)
    {
        ros::Time now = ros::Time::now();
        const double delta_t = std::max(
            0.0,
            (now - takeoff_state.toggle_takeoff_time).toSec() -
                AutoTakeoff_t::MOTORS_SPEEDUP_TIME);
        const double target_z =
            takeoff_state.start_pose(2) + param.takeoff_state.height;

        Controller::Desired_State_t des;
        des.p = takeoff_state.start_pose.head<3>();
        des.p(2) = std::min(
            target_z,
            takeoff_state.start_pose(2) +
                param.takeoff_state.speed * delta_t);
        des.v = Eigen::Vector3d(
            0.0, 0.0,
            des.p(2) < target_z ? param.takeoff_state.speed : 0.0);
        des.a = Eigen::Vector3d::Zero();
        des.j = Eigen::Vector3d::Zero();
        des.yaw = takeoff_state.start_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_hover_des()
    {
        Controller::Desired_State_t des;
        des.p = hover_pose.head<3>();
        des.v = Eigen::Vector3d::Zero();
        des.a = Eigen::Vector3d::Zero();
        des.j = Eigen::Vector3d::Zero();
        des.yaw = hover_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_cmd_des()
    {
        Controller::Desired_State_t des;
        des.p = cmd_data.p;
        des.v = cmd_data.v;
        des.a = cmd_data.a;
        des.j = cmd_data.j;
        // PositionCommand 仅提供 yaw_rate; roll/pitch 前馈置零交姿态环 PD
        des.omg << 0.0, 0.0, cmd_data.yaw_rate;
        des.yaw = cmd_data.yaw;
        des.yaw_rate = cmd_data.yaw_rate;

        return des;
    }

    void FSM::set_start_pose_for_landing(const Odom_Data_t &odom)
    {
        landing_state.start_pose.head<3>() = odom.p;
        landing_state.start_pose(3) = Controller::q2yaw(odom.q);
        landing_state.toggle_land_time = ros::Time::now();
        const double dz = std::fabs(odom.p(2) - param.landing_state.target_height);
        const double speed = std::max(param.landing_state.speed, 1e-6);
        landing_state.land_duration = std::max(dz / speed, kMinLandDuration);
    }

    bool FSM::validateLandingRequest(const Odom_Data_t &odom)
    {
        const double z = odom.p(2);
        const double target_h = param.landing_state.target_height;
        const double speed = param.landing_state.speed;
        const double dis_arm = param.landing_state.dis_arm_height;

        // finite: odom.z 运行时可能异常, 配置参数启动已查但此处防御性再查
        if (!std::isfinite(z) || !std::isfinite(target_h) ||
            !std::isfinite(speed) || !std::isfinite(dis_arm))
        {
            ROS_ERROR("[FSM]: LAND rejected: non-finite landing params "
                      "(z=%.3f target=%.3f speed=%.3f dis_arm=%.3f).",
                      z, target_h, speed, dis_arm);
            return false;
        }
        // speed<=0 会被 set_start_pose_for_landing 钳到 1e-6 → land_duration 极大 → 降落卡住
        if (!(speed > 0.0))
        {
            ROS_ERROR("[FSM]: LAND rejected: landing_state/speed=%.3f must be > 0.", speed);
            return false;
        }
        if (!(dis_arm > 0.0))
        {
            ROS_ERROR("[FSM]: LAND rejected: landing_state/dis_arm_height=%.3f must be > 0.", dis_arm);
            return false;
        }
        // 方向: target_height 必须低于当前高度, 否则 LAND 会生成上升轨迹
        if (!(target_h < z))
        {
            ROS_ERROR("[FSM]: LAND rejected: target_height=%.3f not below current z=%.3f "
                      "(would generate upward trajectory).",
                      target_h, z);
            return false;
        }
        // 交接点 target_height+dis_arm_height 必须低于当前高度
        if (!(target_h + dis_arm < z))
        {
            ROS_ERROR("[FSM]: LAND rejected: handover height (target+dis_arm=%.3f) "
                      "not below current z=%.3f.",
                      target_h + dis_arm, z);
            return false;
        }
        return true;
    }

    Controller::Desired_State_t FSM::get_land_des(const ros::Time &now)
    {
        const double delta_t = (now - landing_state.toggle_land_time).toSec();
        const double T = landing_state.land_duration;
        const double r = (T > 1e-6) ? clamp01(delta_t / T) : 1.0;
        const double s0 = smoothP(r);
        const double s1 = smoothV(r);
        const double s2 = smoothA(r);

        const double z_start = landing_state.start_pose(2);
        const double z_target = param.landing_state.target_height;
        const double dz = z_target - z_start; // <= 0(下降)

        Controller::Desired_State_t des;
        des.p = landing_state.start_pose.head<3>(); // xy 保持 land 起点
        des.p(2) = z_start + dz * s0;
        des.v << 0.0, 0.0, (T > 1e-6 ? dz / T * s1 : 0.0);
        des.a << 0.0, 0.0, (T > 1e-6 ? dz / (T * T) * s2 : 0.0);
        des.j = Eigen::Vector3d::Zero();
        des.yaw = landing_state.start_pose(3);
        des.yaw_rate = 0.0;
        return des;
    }

    SrvResult FSM::request_auto_land()
    {
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = "AUTO.LAND";

        // TransportError: 服务调用本身失败(mavros 不通)
        if (!set_mode_client_srv.call(set_mode_srv))
        {
            ROS_ERROR_THROTTLE(1.0, "[FSM]:/mavros/set_mode AUTO.LAND service call failed (mavros unreachable?).");
            return SrvResult::TransportError;
        }

        if (set_mode_srv.response.mode_sent)
            return SrvResult::Accepted;

        // call 成功但 mode_sent=false: PX4 未确认。SetMode 无 MAV_RESULT, 一律视为可重试。
        ROS_ERROR_THROTTLE(1.0, "[FSM]:/mavros/set_mode AUTO.LAND mode_sent=false, retrying.");
        return SrvResult::Retryable;
    }
}
