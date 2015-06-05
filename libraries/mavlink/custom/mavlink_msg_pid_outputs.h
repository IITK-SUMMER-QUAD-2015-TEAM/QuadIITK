// MESSAGE PID_OUTPUTS PACKING

#define MAVLINK_MSG_ID_PID_OUTPUTS 152

typedef struct __mavlink_pid_outputs_t
{
 float rollRateMotorCommand; ///< The roll rate PID's output.
 float pitchRateMotorCommand; ///< The pitch rate PID's output.
 float yawRateMotorCommand; ///< The yaw rate PID's output.
 float rollMotorCommand; ///< The roll angle PID's output.
 float pitchMotorCommand; ///< The pitch angle PID's output.
 float yawMotorCommand; ///< The yaw angle PID's output.
 float thrustMotorCommand; ///< The thrust motor command
} mavlink_pid_outputs_t;

#define MAVLINK_MSG_ID_PID_OUTPUTS_LEN 28
#define MAVLINK_MSG_ID_152_LEN 28

#define MAVLINK_MSG_ID_PID_OUTPUTS_CRC 206
#define MAVLINK_MSG_ID_152_CRC 206



#define MAVLINK_MESSAGE_INFO_PID_OUTPUTS { \
	"PID_OUTPUTS", \
	7, \
	{  { "rollRateMotorCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pid_outputs_t, rollRateMotorCommand) }, \
         { "pitchRateMotorCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pid_outputs_t, pitchRateMotorCommand) }, \
         { "yawRateMotorCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pid_outputs_t, yawRateMotorCommand) }, \
         { "rollMotorCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pid_outputs_t, rollMotorCommand) }, \
         { "pitchMotorCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pid_outputs_t, pitchMotorCommand) }, \
         { "yawMotorCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pid_outputs_t, yawMotorCommand) }, \
         { "thrustMotorCommand", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pid_outputs_t, thrustMotorCommand) }, \
         } \
}


/**
 * @brief Pack a pid_outputs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rollRateMotorCommand The roll rate PID's output.
 * @param pitchRateMotorCommand The pitch rate PID's output.
 * @param yawRateMotorCommand The yaw rate PID's output.
 * @param rollMotorCommand The roll angle PID's output.
 * @param pitchMotorCommand The pitch angle PID's output.
 * @param yawMotorCommand The yaw angle PID's output.
 * @param thrustMotorCommand The thrust motor command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_outputs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float rollRateMotorCommand, float pitchRateMotorCommand, float yawRateMotorCommand, float rollMotorCommand, float pitchMotorCommand, float yawMotorCommand, float thrustMotorCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_OUTPUTS_LEN];
	_mav_put_float(buf, 0, rollRateMotorCommand);
	_mav_put_float(buf, 4, pitchRateMotorCommand);
	_mav_put_float(buf, 8, yawRateMotorCommand);
	_mav_put_float(buf, 12, rollMotorCommand);
	_mav_put_float(buf, 16, pitchMotorCommand);
	_mav_put_float(buf, 20, yawMotorCommand);
	_mav_put_float(buf, 24, thrustMotorCommand);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#else
	mavlink_pid_outputs_t packet;
	packet.rollRateMotorCommand = rollRateMotorCommand;
	packet.pitchRateMotorCommand = pitchRateMotorCommand;
	packet.yawRateMotorCommand = yawRateMotorCommand;
	packet.rollMotorCommand = rollMotorCommand;
	packet.pitchMotorCommand = pitchMotorCommand;
	packet.yawMotorCommand = yawMotorCommand;
	packet.thrustMotorCommand = thrustMotorCommand;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_OUTPUTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PID_OUTPUTS_LEN, MAVLINK_MSG_ID_PID_OUTPUTS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif
}

/**
 * @brief Pack a pid_outputs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rollRateMotorCommand The roll rate PID's output.
 * @param pitchRateMotorCommand The pitch rate PID's output.
 * @param yawRateMotorCommand The yaw rate PID's output.
 * @param rollMotorCommand The roll angle PID's output.
 * @param pitchMotorCommand The pitch angle PID's output.
 * @param yawMotorCommand The yaw angle PID's output.
 * @param thrustMotorCommand The thrust motor command
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_outputs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float rollRateMotorCommand,float pitchRateMotorCommand,float yawRateMotorCommand,float rollMotorCommand,float pitchMotorCommand,float yawMotorCommand,float thrustMotorCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_OUTPUTS_LEN];
	_mav_put_float(buf, 0, rollRateMotorCommand);
	_mav_put_float(buf, 4, pitchRateMotorCommand);
	_mav_put_float(buf, 8, yawRateMotorCommand);
	_mav_put_float(buf, 12, rollMotorCommand);
	_mav_put_float(buf, 16, pitchMotorCommand);
	_mav_put_float(buf, 20, yawMotorCommand);
	_mav_put_float(buf, 24, thrustMotorCommand);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#else
	mavlink_pid_outputs_t packet;
	packet.rollRateMotorCommand = rollRateMotorCommand;
	packet.pitchRateMotorCommand = pitchRateMotorCommand;
	packet.yawRateMotorCommand = yawRateMotorCommand;
	packet.rollMotorCommand = rollMotorCommand;
	packet.pitchMotorCommand = pitchMotorCommand;
	packet.yawMotorCommand = yawMotorCommand;
	packet.thrustMotorCommand = thrustMotorCommand;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_OUTPUTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PID_OUTPUTS_LEN, MAVLINK_MSG_ID_PID_OUTPUTS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif
}

/**
 * @brief Encode a pid_outputs struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pid_outputs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_outputs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pid_outputs_t* pid_outputs)
{
	return mavlink_msg_pid_outputs_pack(system_id, component_id, msg, pid_outputs->rollRateMotorCommand, pid_outputs->pitchRateMotorCommand, pid_outputs->yawRateMotorCommand, pid_outputs->rollMotorCommand, pid_outputs->pitchMotorCommand, pid_outputs->yawMotorCommand, pid_outputs->thrustMotorCommand);
}

/**
 * @brief Encode a pid_outputs struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pid_outputs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_outputs_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pid_outputs_t* pid_outputs)
{
	return mavlink_msg_pid_outputs_pack_chan(system_id, component_id, chan, msg, pid_outputs->rollRateMotorCommand, pid_outputs->pitchRateMotorCommand, pid_outputs->yawRateMotorCommand, pid_outputs->rollMotorCommand, pid_outputs->pitchMotorCommand, pid_outputs->yawMotorCommand, pid_outputs->thrustMotorCommand);
}

/**
 * @brief Send a pid_outputs message
 * @param chan MAVLink channel to send the message
 *
 * @param rollRateMotorCommand The roll rate PID's output.
 * @param pitchRateMotorCommand The pitch rate PID's output.
 * @param yawRateMotorCommand The yaw rate PID's output.
 * @param rollMotorCommand The roll angle PID's output.
 * @param pitchMotorCommand The pitch angle PID's output.
 * @param yawMotorCommand The yaw angle PID's output.
 * @param thrustMotorCommand The thrust motor command
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_outputs_send(mavlink_channel_t chan, float rollRateMotorCommand, float pitchRateMotorCommand, float yawRateMotorCommand, float rollMotorCommand, float pitchMotorCommand, float yawMotorCommand, float thrustMotorCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_OUTPUTS_LEN];
	_mav_put_float(buf, 0, rollRateMotorCommand);
	_mav_put_float(buf, 4, pitchRateMotorCommand);
	_mav_put_float(buf, 8, yawRateMotorCommand);
	_mav_put_float(buf, 12, rollMotorCommand);
	_mav_put_float(buf, 16, pitchMotorCommand);
	_mav_put_float(buf, 20, yawMotorCommand);
	_mav_put_float(buf, 24, thrustMotorCommand);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, buf, MAVLINK_MSG_ID_PID_OUTPUTS_LEN, MAVLINK_MSG_ID_PID_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, buf, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif
#else
	mavlink_pid_outputs_t packet;
	packet.rollRateMotorCommand = rollRateMotorCommand;
	packet.pitchRateMotorCommand = pitchRateMotorCommand;
	packet.yawRateMotorCommand = yawRateMotorCommand;
	packet.rollMotorCommand = rollMotorCommand;
	packet.pitchMotorCommand = pitchMotorCommand;
	packet.yawMotorCommand = yawMotorCommand;
	packet.thrustMotorCommand = thrustMotorCommand;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, (const char *)&packet, MAVLINK_MSG_ID_PID_OUTPUTS_LEN, MAVLINK_MSG_ID_PID_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, (const char *)&packet, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PID_OUTPUTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pid_outputs_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rollRateMotorCommand, float pitchRateMotorCommand, float yawRateMotorCommand, float rollMotorCommand, float pitchMotorCommand, float yawMotorCommand, float thrustMotorCommand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, rollRateMotorCommand);
	_mav_put_float(buf, 4, pitchRateMotorCommand);
	_mav_put_float(buf, 8, yawRateMotorCommand);
	_mav_put_float(buf, 12, rollMotorCommand);
	_mav_put_float(buf, 16, pitchMotorCommand);
	_mav_put_float(buf, 20, yawMotorCommand);
	_mav_put_float(buf, 24, thrustMotorCommand);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, buf, MAVLINK_MSG_ID_PID_OUTPUTS_LEN, MAVLINK_MSG_ID_PID_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, buf, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif
#else
	mavlink_pid_outputs_t *packet = (mavlink_pid_outputs_t *)msgbuf;
	packet->rollRateMotorCommand = rollRateMotorCommand;
	packet->pitchRateMotorCommand = pitchRateMotorCommand;
	packet->yawRateMotorCommand = yawRateMotorCommand;
	packet->rollMotorCommand = rollMotorCommand;
	packet->pitchMotorCommand = pitchMotorCommand;
	packet->yawMotorCommand = yawMotorCommand;
	packet->thrustMotorCommand = thrustMotorCommand;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, (const char *)packet, MAVLINK_MSG_ID_PID_OUTPUTS_LEN, MAVLINK_MSG_ID_PID_OUTPUTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_OUTPUTS, (const char *)packet, MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PID_OUTPUTS UNPACKING


/**
 * @brief Get field rollRateMotorCommand from pid_outputs message
 *
 * @return The roll rate PID's output.
 */
static inline float mavlink_msg_pid_outputs_get_rollRateMotorCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitchRateMotorCommand from pid_outputs message
 *
 * @return The pitch rate PID's output.
 */
static inline float mavlink_msg_pid_outputs_get_pitchRateMotorCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yawRateMotorCommand from pid_outputs message
 *
 * @return The yaw rate PID's output.
 */
static inline float mavlink_msg_pid_outputs_get_yawRateMotorCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rollMotorCommand from pid_outputs message
 *
 * @return The roll angle PID's output.
 */
static inline float mavlink_msg_pid_outputs_get_rollMotorCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitchMotorCommand from pid_outputs message
 *
 * @return The pitch angle PID's output.
 */
static inline float mavlink_msg_pid_outputs_get_pitchMotorCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yawMotorCommand from pid_outputs message
 *
 * @return The yaw angle PID's output.
 */
static inline float mavlink_msg_pid_outputs_get_yawMotorCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field thrustMotorCommand from pid_outputs message
 *
 * @return The thrust motor command
 */
static inline float mavlink_msg_pid_outputs_get_thrustMotorCommand(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a pid_outputs message into a struct
 *
 * @param msg The message to decode
 * @param pid_outputs C-struct to decode the message contents into
 */
static inline void mavlink_msg_pid_outputs_decode(const mavlink_message_t* msg, mavlink_pid_outputs_t* pid_outputs)
{
#if MAVLINK_NEED_BYTE_SWAP
	pid_outputs->rollRateMotorCommand = mavlink_msg_pid_outputs_get_rollRateMotorCommand(msg);
	pid_outputs->pitchRateMotorCommand = mavlink_msg_pid_outputs_get_pitchRateMotorCommand(msg);
	pid_outputs->yawRateMotorCommand = mavlink_msg_pid_outputs_get_yawRateMotorCommand(msg);
	pid_outputs->rollMotorCommand = mavlink_msg_pid_outputs_get_rollMotorCommand(msg);
	pid_outputs->pitchMotorCommand = mavlink_msg_pid_outputs_get_pitchMotorCommand(msg);
	pid_outputs->yawMotorCommand = mavlink_msg_pid_outputs_get_yawMotorCommand(msg);
	pid_outputs->thrustMotorCommand = mavlink_msg_pid_outputs_get_thrustMotorCommand(msg);
#else
	memcpy(pid_outputs, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PID_OUTPUTS_LEN);
#endif
}
