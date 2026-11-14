#include "quadrotor_msgs/encode_msgs.h"
#include <quadrotor_msgs/comm_types.h>

namespace quadrotor_msgs
{

  void encodeTRPYCommand(const quadrotor_msgs::TRPYCommand &trpy_command,
                         std::vector<uint8_t> &output)
  {
    struct TRPY_CMD trpy_cmd_input;
    trpy_cmd_input.thrust = trpy_command.thrust * 1e4;
    trpy_cmd_input.roll = trpy_command.roll * 1e4;
    trpy_cmd_input.pitch = trpy_command.pitch * 1e4;
    trpy_cmd_input.yaw = trpy_command.yaw * 1e4;
    trpy_cmd_input.current_yaw = trpy_command.aux.current_yaw * 1e4;
    trpy_cmd_input.enable_motors = trpy_command.aux.enable_motors;
    trpy_cmd_input.use_external_yaw = trpy_command.aux.use_external_yaw;

    output.resize(sizeof(trpy_cmd_input));
    memcpy(&output[0], &trpy_cmd_input, sizeof(trpy_cmd_input));
  }

  void encodePPRGains(const quadrotor_msgs::Gains &gains,
                      std::vector<uint8_t> &output)
  {
    struct PPR_GAINS ppr_gains;
    ppr_gains.Kp = gains.Kp;
    ppr_gains.Kd = gains.Kd;
    ppr_gains.Kp_yaw = gains.Kp_yaw;
    ppr_gains.Kd_yaw = gains.Kd_yaw;

    output.resize(sizeof(ppr_gains));
    memcpy(&output[0], &ppr_gains, sizeof(ppr_gains));
  }

}
