// MESSAGE PID_SETPOINTS PACKING

#define MAVLINK_MSG_ID_PID_SETPOINTS 155

typedef struct __mavlink_pid_setpoints_t
{
 float rollRateSetpoint; ///< The roll rate PID's output.
 float pitchRateSetpoint; ///< The pitch rate PID's output.
 float yawRateSetpoint; ///< The yaw rate PID's output.
 float rollSetpoint; ///< The roll angle PID's output.
 float pitchSetpoint; ///< The pitch angle PID's output.
 float yawSetpoint; ///< The yaw angle PID's output.
} mavlink_pid_setpoints_t;

#define MAVLINK_MSG_ID_PID_SETPOINTS_LEN 24
#define MAVLINK_MSG_ID_155_LEN 24

#define MAVLINK_MSG_ID_PID_SETPOINTS_CRC 158
#define MAVLINK_MSG_ID_155_CRC 158



#define MAVLINK_MESSAGE_INFO_PID_SETPOINTS { \
	"PID_SETPOINTS", \
	6, \
	{  { "rollRateSetpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pid_setpoints_t, rollRateSetpoint) }, \
         { "pitchRateSetpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pid_setpoints_t, pitchRateSetpoint) }, \
         { "yawRateSetpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pid_setpoints_t, yawRateSetpoint) }, \
         { "rollSetpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pid_setpoints_t, rollSetpoint) }, \
         { "pitchSetpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pid_setpoints_t, pitchSetpoint) }, \
         { "yawSetpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pid_setpoints_t, yawSetpoint) }, \
         } \
}


/**
 * @brief Pack a pid_setpoints message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rollRateSetpoint The roll rate PID's output.
 * @param pitchRateSetpoint The pitch rate PID's output.
 * @param yawRateSetpoint The yaw rate PID's output.
 * @param rollSetpoint The roll angle PID's output.
 * @param pitchSetpoint The pitch angle PID's output.
 * @param yawSetpoint The yaw angle PID's output.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_setpoints_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float rollRateSetpoint, float pitchRateSetpoint, float yawRateSetpoint, float rollSetpoint, float pitchSetpoint, float yawSetpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_SETPOINTS_LEN];
	_mav_put_float(buf, 0, rollRateSetpoint);
	_mav_put_float(buf, 4, pitchRateSetpoint);
	_mav_put_float(buf, 8, yawRateSetpoint);
	_mav_put_float(buf, 12, rollSetpoint);
	_mav_put_float(buf, 16, pitchSetpoint);
	_mav_put_float(buf, 20, yawSetpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#else
	mavlink_pid_setpoints_t packet;
	packet.rollRateSetpoint = rollRateSetpoint;
	packet.pitchRateSetpoint = pitchRateSetpoint;
	packet.yawRateSetpoint = yawRateSetpoint;
	packet.rollSetpoint = rollSetpoint;
	packet.pitchSetpoint = pitchSetpoint;
	packet.yawSetpoint = yawSetpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_SETPOINTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PID_SETPOINTS_LEN, MAVLINK_MSG_ID_PID_SETPOINTS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif
}

/**
 * @brief Pack a pid_setpoints message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rollRateSetpoint The roll rate PID's output.
 * @param pitchRateSetpoint The pitch rate PID's output.
 * @param yawRateSetpoint The yaw rate PID's output.
 * @param rollSetpoint The roll angle PID's output.
 * @param pitchSetpoint The pitch angle PID's output.
 * @param yawSetpoint The yaw angle PID's output.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_setpoints_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float rollRateSetpoint,float pitchRateSetpoint,float yawRateSetpoint,float rollSetpoint,float pitchSetpoint,float yawSetpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_SETPOINTS_LEN];
	_mav_put_float(buf, 0, rollRateSetpoint);
	_mav_put_float(buf, 4, pitchRateSetpoint);
	_mav_put_float(buf, 8, yawRateSetpoint);
	_mav_put_float(buf, 12, rollSetpoint);
	_mav_put_float(buf, 16, pitchSetpoint);
	_mav_put_float(buf, 20, yawSetpoint);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#else
	mavlink_pid_setpoints_t packet;
	packet.rollRateSetpoint = rollRateSetpoint;
	packet.pitchRateSetpoint = pitchRateSetpoint;
	packet.yawRateSetpoint = yawRateSetpoint;
	packet.rollSetpoint = rollSetpoint;
	packet.pitchSetpoint = pitchSetpoint;
	packet.yawSetpoint = yawSetpoint;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_SETPOINTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PID_SETPOINTS_LEN, MAVLINK_MSG_ID_PID_SETPOINTS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif
}

/**
 * @brief Encode a pid_setpoints struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pid_setpoints C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_setpoints_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pid_setpoints_t* pid_setpoints)
{
	return mavlink_msg_pid_setpoints_pack(system_id, component_id, msg, pid_setpoints->rollRateSetpoint, pid_setpoints->pitchRateSetpoint, pid_setpoints->yawRateSetpoint, pid_setpoints->rollSetpoint, pid_setpoints->pitchSetpoint, pid_setpoints->yawSetpoint);
}

/**
 * @brief Encode a pid_setpoints struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pid_setpoints C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_setpoints_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pid_setpoints_t* pid_setpoints)
{
	return mavlink_msg_pid_setpoints_pack_chan(system_id, component_id, chan, msg, pid_setpoints->rollRateSetpoint, pid_setpoints->pitchRateSetpoint, pid_setpoints->yawRateSetpoint, pid_setpoints->rollSetpoint, pid_setpoints->pitchSetpoint, pid_setpoints->yawSetpoint);
}

/**
 * @brief Send a pid_setpoints message
 * @param chan MAVLink channel to send the message
 *
 * @param rollRateSetpoint The roll rate PID's output.
 * @param pitchRateSetpoint The pitch rate PID's output.
 * @param yawRateSetpoint The yaw rate PID's output.
 * @param rollSetpoint The roll angle PID's output.
 * @param pitchSetpoint The pitch angle PID's output.
 * @param yawSetpoint The yaw angle PID's output.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_setpoints_send(mavlink_channel_t chan, float rollRateSetpoint, float pitchRateSetpoint, float yawRateSetpoint, float rollSetpoint, float pitchSetpoint, float yawSetpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_SETPOINTS_LEN];
	_mav_put_float(buf, 0, rollRateSetpoint);
	_mav_put_float(buf, 4, pitchRateSetpoint);
	_mav_put_float(buf, 8, yawRateSetpoint);
	_mav_put_float(buf, 12, rollSetpoint);
	_mav_put_float(buf, 16, pitchSetpoint);
	_mav_put_float(buf, 20, yawSetpoint);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, buf, MAVLINK_MSG_ID_PID_SETPOINTS_LEN, MAVLINK_MSG_ID_PID_SETPOINTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, buf, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif
#else
	mavlink_pid_setpoints_t packet;
	packet.rollRateSetpoint = rollRateSetpoint;
	packet.pitchRateSetpoint = pitchRateSetpoint;
	packet.yawRateSetpoint = yawRateSetpoint;
	packet.rollSetpoint = rollSetpoint;
	packet.pitchSetpoint = pitchSetpoint;
	packet.yawSetpoint = yawSetpoint;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, (const char *)&packet, MAVLINK_MSG_ID_PID_SETPOINTS_LEN, MAVLINK_MSG_ID_PID_SETPOINTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, (const char *)&packet, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PID_SETPOINTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pid_setpoints_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rollRateSetpoint, float pitchRateSetpoint, float yawRateSetpoint, float rollSetpoint, float pitchSetpoint, float yawSetpoint)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, rollRateSetpoint);
	_mav_put_float(buf, 4, pitchRateSetpoint);
	_mav_put_float(buf, 8, yawRateSetpoint);
	_mav_put_float(buf, 12, rollSetpoint);
	_mav_put_float(buf, 16, pitchSetpoint);
	_mav_put_float(buf, 20, yawSetpoint);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, buf, MAVLINK_MSG_ID_PID_SETPOINTS_LEN, MAVLINK_MSG_ID_PID_SETPOINTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, buf, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif
#else
	mavlink_pid_setpoints_t *packet = (mavlink_pid_setpoints_t *)msgbuf;
	packet->rollRateSetpoint = rollRateSetpoint;
	packet->pitchRateSetpoint = pitchRateSetpoint;
	packet->yawRateSetpoint = yawRateSetpoint;
	packet->rollSetpoint = rollSetpoint;
	packet->pitchSetpoint = pitchSetpoint;
	packet->yawSetpoint = yawSetpoint;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, (const char *)packet, MAVLINK_MSG_ID_PID_SETPOINTS_LEN, MAVLINK_MSG_ID_PID_SETPOINTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_SETPOINTS, (const char *)packet, MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PID_SETPOINTS UNPACKING


/**
 * @brief Get field rollRateSetpoint from pid_setpoints message
 *
 * @return The roll rate PID's output.
 */
static inline float mavlink_msg_pid_setpoints_get_rollRateSetpoint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitchRateSetpoint from pid_setpoints message
 *
 * @return The pitch rate PID's output.
 */
static inline float mavlink_msg_pid_setpoints_get_pitchRateSetpoint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yawRateSetpoint from pid_setpoints message
 *
 * @return The yaw rate PID's output.
 */
static inline float mavlink_msg_pid_setpoints_get_yawRateSetpoint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rollSetpoint from pid_setpoints message
 *
 * @return The roll angle PID's output.
 */
static inline float mavlink_msg_pid_setpoints_get_rollSetpoint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitchSetpoint from pid_setpoints message
 *
 * @return The pitch angle PID's output.
 */
static inline float mavlink_msg_pid_setpoints_get_pitchSetpoint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yawSetpoint from pid_setpoints message
 *
 * @return The yaw angle PID's output.
 */
static inline float mavlink_msg_pid_setpoints_get_yawSetpoint(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a pid_setpoints message into a struct
 *
 * @param msg The message to decode
 * @param pid_setpoints C-struct to decode the message contents into
 */
static inline void mavlink_msg_pid_setpoints_decode(const mavlink_message_t* msg, mavlink_pid_setpoints_t* pid_setpoints)
{
#if MAVLINK_NEED_BYTE_SWAP
	pid_setpoints->rollRateSetpoint = mavlink_msg_pid_setpoints_get_rollRateSetpoint(msg);
	pid_setpoints->pitchRateSetpoint = mavlink_msg_pid_setpoints_get_pitchRateSetpoint(msg);
	pid_setpoints->yawRateSetpoint = mavlink_msg_pid_setpoints_get_yawRateSetpoint(msg);
	pid_setpoints->rollSetpoint = mavlink_msg_pid_setpoints_get_rollSetpoint(msg);
	pid_setpoints->pitchSetpoint = mavlink_msg_pid_setpoints_get_pitchSetpoint(msg);
	pid_setpoints->yawSetpoint = mavlink_msg_pid_setpoints_get_yawSetpoint(msg);
#else
	memcpy(pid_setpoints, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PID_SETPOINTS_LEN);
#endif
}
