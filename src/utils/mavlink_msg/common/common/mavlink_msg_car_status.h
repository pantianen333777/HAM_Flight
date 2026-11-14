#pragma once
// MESSAGE CAR_STATUS PACKING

#define MAVLINK_MSG_ID_CAR_STATUS 204


typedef struct __mavlink_car_status_t {
 uint64_t time_ms; /*<  */
 float px; /*<  */
 float py; /*<  */
 float pz; /*<  */
 float vx; /*<  */
 float vy; /*<  */
 float vz; /*<  */
 float yaw; /*<  */
 float pitch; /*<  */
 float roll; /*<  */
 uint8_t status; /*<  */
} mavlink_car_status_t;

#define MAVLINK_MSG_ID_CAR_STATUS_LEN 45
#define MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN 45
#define MAVLINK_MSG_ID_204_LEN 45
#define MAVLINK_MSG_ID_204_MIN_LEN 45

#define MAVLINK_MSG_ID_CAR_STATUS_CRC 254
#define MAVLINK_MSG_ID_204_CRC 254



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CAR_STATUS { \
    204, \
    "CAR_STATUS", \
    11, \
    {  { "time_ms", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_car_status_t, time_ms) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_car_status_t, status) }, \
         { "px", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_car_status_t, px) }, \
         { "py", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_car_status_t, py) }, \
         { "pz", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_car_status_t, pz) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_car_status_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_car_status_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_car_status_t, vz) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_car_status_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_car_status_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_car_status_t, roll) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CAR_STATUS { \
    "CAR_STATUS", \
    11, \
    {  { "time_ms", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_car_status_t, time_ms) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_car_status_t, status) }, \
         { "px", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_car_status_t, px) }, \
         { "py", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_car_status_t, py) }, \
         { "pz", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_car_status_t, pz) }, \
         { "vx", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_car_status_t, vx) }, \
         { "vy", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_car_status_t, vy) }, \
         { "vz", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_car_status_t, vz) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_car_status_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_car_status_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_car_status_t, roll) }, \
         } \
}
#endif

/**
 * @brief Pack a car_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_ms  
 * @param status  
 * @param px  
 * @param py  
 * @param pz  
 * @param vx  
 * @param vy  
 * @param vz  
 * @param yaw  
 * @param pitch  
 * @param roll  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_car_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_ms, uint8_t status, float px, float py, float pz, float vx, float vy, float vz, float yaw, float pitch, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_ms);
    _mav_put_float(buf, 8, px);
    _mav_put_float(buf, 12, py);
    _mav_put_float(buf, 16, pz);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, pitch);
    _mav_put_float(buf, 40, roll);
    _mav_put_uint8_t(buf, 44, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAR_STATUS_LEN);
#else
    mavlink_car_status_t packet;
    packet.time_ms = time_ms;
    packet.px = px;
    packet.py = py;
    packet.pz = pz;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAR_STATUS_LEN, MAVLINK_MSG_ID_CAR_STATUS_CRC);
}

/**
 * @brief Pack a car_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_ms  
 * @param status  
 * @param px  
 * @param py  
 * @param pz  
 * @param vx  
 * @param vy  
 * @param vz  
 * @param yaw  
 * @param pitch  
 * @param roll  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_car_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_ms,uint8_t status,float px,float py,float pz,float vx,float vy,float vz,float yaw,float pitch,float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_ms);
    _mav_put_float(buf, 8, px);
    _mav_put_float(buf, 12, py);
    _mav_put_float(buf, 16, pz);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, pitch);
    _mav_put_float(buf, 40, roll);
    _mav_put_uint8_t(buf, 44, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAR_STATUS_LEN);
#else
    mavlink_car_status_t packet;
    packet.time_ms = time_ms;
    packet.px = px;
    packet.py = py;
    packet.pz = pz;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CAR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAR_STATUS_LEN, MAVLINK_MSG_ID_CAR_STATUS_CRC);
}

/**
 * @brief Encode a car_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param car_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_car_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_car_status_t* car_status)
{
    return mavlink_msg_car_status_pack(system_id, component_id, msg, car_status->time_ms, car_status->status, car_status->px, car_status->py, car_status->pz, car_status->vx, car_status->vy, car_status->vz, car_status->yaw, car_status->pitch, car_status->roll);
}

/**
 * @brief Encode a car_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param car_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_car_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_car_status_t* car_status)
{
    return mavlink_msg_car_status_pack_chan(system_id, component_id, chan, msg, car_status->time_ms, car_status->status, car_status->px, car_status->py, car_status->pz, car_status->vx, car_status->vy, car_status->vz, car_status->yaw, car_status->pitch, car_status->roll);
}

/**
 * @brief Send a car_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_ms  
 * @param status  
 * @param px  
 * @param py  
 * @param pz  
 * @param vx  
 * @param vy  
 * @param vz  
 * @param yaw  
 * @param pitch  
 * @param roll  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_car_status_send(mavlink_channel_t chan, uint64_t time_ms, uint8_t status, float px, float py, float pz, float vx, float vy, float vz, float yaw, float pitch, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CAR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_ms);
    _mav_put_float(buf, 8, px);
    _mav_put_float(buf, 12, py);
    _mav_put_float(buf, 16, pz);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, pitch);
    _mav_put_float(buf, 40, roll);
    _mav_put_uint8_t(buf, 44, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAR_STATUS, buf, MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAR_STATUS_LEN, MAVLINK_MSG_ID_CAR_STATUS_CRC);
#else
    mavlink_car_status_t packet;
    packet.time_ms = time_ms;
    packet.px = px;
    packet.py = py;
    packet.pz = pz;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.yaw = yaw;
    packet.pitch = pitch;
    packet.roll = roll;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAR_STATUS_LEN, MAVLINK_MSG_ID_CAR_STATUS_CRC);
#endif
}

/**
 * @brief Send a car_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_car_status_send_struct(mavlink_channel_t chan, const mavlink_car_status_t* car_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_car_status_send(chan, car_status->time_ms, car_status->status, car_status->px, car_status->py, car_status->pz, car_status->vx, car_status->vy, car_status->vz, car_status->yaw, car_status->pitch, car_status->roll);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAR_STATUS, (const char *)car_status, MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAR_STATUS_LEN, MAVLINK_MSG_ID_CAR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_CAR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_car_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_ms, uint8_t status, float px, float py, float pz, float vx, float vy, float vz, float yaw, float pitch, float roll)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_ms);
    _mav_put_float(buf, 8, px);
    _mav_put_float(buf, 12, py);
    _mav_put_float(buf, 16, pz);
    _mav_put_float(buf, 20, vx);
    _mav_put_float(buf, 24, vy);
    _mav_put_float(buf, 28, vz);
    _mav_put_float(buf, 32, yaw);
    _mav_put_float(buf, 36, pitch);
    _mav_put_float(buf, 40, roll);
    _mav_put_uint8_t(buf, 44, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAR_STATUS, buf, MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAR_STATUS_LEN, MAVLINK_MSG_ID_CAR_STATUS_CRC);
#else
    mavlink_car_status_t *packet = (mavlink_car_status_t *)msgbuf;
    packet->time_ms = time_ms;
    packet->px = px;
    packet->py = py;
    packet->pz = pz;
    packet->vx = vx;
    packet->vy = vy;
    packet->vz = vz;
    packet->yaw = yaw;
    packet->pitch = pitch;
    packet->roll = roll;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAR_STATUS, (const char *)packet, MAVLINK_MSG_ID_CAR_STATUS_MIN_LEN, MAVLINK_MSG_ID_CAR_STATUS_LEN, MAVLINK_MSG_ID_CAR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE CAR_STATUS UNPACKING


/**
 * @brief Get field time_ms from car_status message
 *
 * @return  
 */
static inline uint64_t mavlink_msg_car_status_get_time_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field status from car_status message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_car_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field px from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_px(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field py from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_py(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pz from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_pz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field vx from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_vx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field vy from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_vy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field vz from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_vz(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field yaw from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field pitch from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field roll from car_status message
 *
 * @return  
 */
static inline float mavlink_msg_car_status_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a car_status message into a struct
 *
 * @param msg The message to decode
 * @param car_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_car_status_decode(const mavlink_message_t* msg, mavlink_car_status_t* car_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    car_status->time_ms = mavlink_msg_car_status_get_time_ms(msg);
    car_status->px = mavlink_msg_car_status_get_px(msg);
    car_status->py = mavlink_msg_car_status_get_py(msg);
    car_status->pz = mavlink_msg_car_status_get_pz(msg);
    car_status->vx = mavlink_msg_car_status_get_vx(msg);
    car_status->vy = mavlink_msg_car_status_get_vy(msg);
    car_status->vz = mavlink_msg_car_status_get_vz(msg);
    car_status->yaw = mavlink_msg_car_status_get_yaw(msg);
    car_status->pitch = mavlink_msg_car_status_get_pitch(msg);
    car_status->roll = mavlink_msg_car_status_get_roll(msg);
    car_status->status = mavlink_msg_car_status_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CAR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_CAR_STATUS_LEN;
        memset(car_status, 0, MAVLINK_MSG_ID_CAR_STATUS_LEN);
    memcpy(car_status, _MAV_PAYLOAD(msg), len);
#endif
}
